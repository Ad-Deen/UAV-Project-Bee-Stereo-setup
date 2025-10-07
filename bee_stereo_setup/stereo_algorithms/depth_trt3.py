#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import time
import open3d as o3d
from message_filters import Subscriber, ApproximateTimeSynchronizer

# ======================== o3D Viz ========================
class O3DVisualizerNonBlocking:
    def __init__(self, window_name="Test PointCloud", width=800, height=600):
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name, width=width, height=height)
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)
        self.first_update = True
        self.should_close = False

        # Set up key callback for ESC key
        self.vis.register_key_callback(256, self._on_escape)  # ESC key

        # Get render option and set point size
        render_option = self.vis.get_render_option()
        render_option.point_size = 2.0

    def _on_escape(self, vis):
        self.should_close = True
        return False

    def update(self, points: np.ndarray, colors: np.ndarray = None):
        # points: (N,3), colors: (N,3) in [0..1]
        if points is None or points.size == 0 or self.should_close:
            return False

        try:
            pts = points.astype(np.float64).reshape(-1, 3)
            self.pcd.points = o3d.utility.Vector3dVector(pts)

            if colors is not None and colors.shape[0] == pts.shape[0]:
                cols = colors.astype(np.float64).reshape(-1, 3)
                cols = np.clip(cols, 0.0, 1.0)
                self.pcd.colors = o3d.utility.Vector3dVector(cols)
            else:
                self.pcd.colors = o3d.utility.Vector3dVector(np.ones_like(pts) * 0.7)

            self.vis.update_geometry(self.pcd)

            if self.first_update:
                self.vis.reset_view_point(True)
                self.first_update = False

            # return True while running
            return self.vis.poll_events() and not self.should_close

        except Exception as e:
            print(f"Error in O3D update: {e}")
            return False

    def close(self):
        try:
            self.vis.destroy_window()
        except Exception:
            pass
# ======================== o3D Viz End ========================

ENGINE_PATH = "/home/deen/ONNX-CREStereo-Depth-Estimation/models/iter20/crestereo_iter20_240x320.plan"
BATCH = 1
IMG_H, IMG_W = 240, 320
INIT_H, INIT_W = IMG_H // 2, IMG_W // 2

def preprocess_image_cv(img, H, W):
    img = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)
    img = np.transpose(img, (2, 0, 1))[np.newaxis, ...]   # NCHW
    return np.ascontiguousarray(img)

def colorize_disparity(disp):
    disp_min, disp_max = disp.min(), disp.max()
    disp_norm = (disp - disp_min) / 50.0
    return cv2.applyColorMap((disp_norm * 255).astype(np.uint8), cv2.COLORMAP_MAGMA)

class CreStereoNode(Node):
    def __init__(self):
        super().__init__("crestereo_infer_node")

        self.bridge = CvBridge()

        # --- message_filters subscribers (synchronized callback) ---
        left_sub  = Subscriber(self, Image, "/automama/camera2")
        right_sub = Subscriber(self, Image, "/automama/camera1")
        # use Exact TimeSynchronizer if stamps are identical, else Approximate
        ats = ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=10, slop=0.01)
        ats.registerCallback(self.stereo_callback)

        # --- Load TensorRT engine ---
        TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
        runtime = trt.Runtime(TRT_LOGGER)
        with open(ENGINE_PATH, "rb") as f:
            engine_data = f.read()
        self.engine = runtime.deserialize_cuda_engine(engine_data)
        self.context_trt = self.engine.create_execution_context()
        self.stream = cuda.Stream()

        # Allocate buffers (host pointers still in your pattern)
        self.output = np.empty((BATCH, 2, IMG_H, IMG_W), dtype=np.float32)
        self.d_output = cuda.mem_alloc(self.output.nbytes)
        self.d_init_left  = cuda.mem_alloc(BATCH * 3 * INIT_H * INIT_W * 4)
        self.d_init_right = cuda.mem_alloc(BATCH * 3 * INIT_H * INIT_W * 4)
        self.d_next_left  = cuda.mem_alloc(BATCH * 3 * IMG_H * IMG_W * 4)
        self.d_next_right = cuda.mem_alloc(BATCH * 3 * IMG_H * IMG_W * 4)

        # set tensor addresses
        self.context_trt.set_tensor_address("init_left", int(self.d_init_left))
        self.context_trt.set_tensor_address("init_right", int(self.d_init_right))
        self.context_trt.set_tensor_address("next_left", int(self.d_next_left))
        self.context_trt.set_tensor_address("next_right", int(self.d_next_right))
        self.context_trt.set_tensor_address("next_output", int(self.d_output))

        # Camera intrinsics (adjust if known)
        self.cx = IMG_W / 2.0
        self.cy = IMG_H / 2.0
        self.focal_length = 125.0
        self.baseline = 0.07
        self.u, self.v = np.meshgrid(np.arange(IMG_W), np.arange(IMG_H))

        # Open3D visualizer
        self.vis = O3DVisualizerNonBlocking(window_name="CREStereo Depth PointCloud", width=800, height=600)

        self.get_logger().info("CreStereoNode initialized and waiting for synchronized stereo frames...")

    def run_inference(self, left_full, right_full, left_half, right_half):
        # left_* etc are contiguos numpy arrays NCHW float32
        cuda.memcpy_htod_async(self.d_init_left, left_half, self.stream)
        cuda.memcpy_htod_async(self.d_init_right, right_half, self.stream)
        cuda.memcpy_htod_async(self.d_next_left, left_full, self.stream)
        cuda.memcpy_htod_async(self.d_next_right, right_full, self.stream)

        self.context_trt.execute_async_v3(self.stream.handle)
        cuda.memcpy_dtoh_async(self.output, self.d_output, self.stream)
        self.stream.synchronize()
        return np.squeeze(self.output[:, 0, :, :])

    def stereo_callback(self, left_msg, right_msg):
        """Called with synchronized pair (left_msg, right_msg). Do full pipeline here."""
        start = time.perf_counter()

        # Convert sensor_msgs -> cv2 images (BGR)
        try:
            left_img  = self.bridge.imgmsg_to_cv2(left_msg,  "bgr8")
            right_img = self.bridge.imgmsg_to_cv2(right_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv bridge error: {e}")
            return

        # Preprocess for model (both full and half sizes)
        left_full  = preprocess_image_cv(left_img, IMG_H, IMG_W)
        right_full = preprocess_image_cv(right_img, IMG_H, IMG_W)
        left_half  = preprocess_image_cv(left_img, INIT_H, INIT_W)
        right_half = preprocess_image_cv(right_img, INIT_H, INIT_W)

        # Run TRT inference
        disp_map = self.run_inference(left_full, right_full, left_half, right_half)

        # Mask invalid disparity (avoid divide by zero)
        valid_mask = (disp_map > 2.0) & (disp_map < 60.0)

        # Compute depth
        depth = np.zeros_like(disp_map, dtype=np.float32)
        depth[valid_mask] = (self.focal_length * self.baseline) / disp_map[valid_mask]

        # Visualization prep
        depth_vis = np.clip(depth, 0.02, 10.0)
        # normalize for color map (avoid division by zero)
        if np.nanmax(depth_vis) > 0:
            depth_vis_norm = (depth_vis / np.nanmax(depth_vis)).astype(np.float32)
        else:
            depth_vis_norm = depth_vis.astype(np.float32)
        depth_color = cv2.applyColorMap((depth_vis_norm * 255).astype(np.uint8), cv2.COLORMAP_MAGMA)

        # Show left + depth side-by-side
        left_vis = cv2.resize(left_img, (depth_color.shape[1], depth_color.shape[0]))
        combined = np.hstack((left_vis, depth_color))
        cv2.imshow("Stereo Depth", combined)
        cv2.waitKey(1)

        # Build point cloud
        X = (self.u - self.cx) * depth / self.focal_length
        Y = (self.v - self.cy) * depth / self.focal_length
        Z = depth

        points = np.stack((X, Y, Z), axis=-1).reshape(-1, 3)
        colors = cv2.cvtColor(left_img, cv2.COLOR_BGR2RGB)
        colors = cv2.resize(colors, (IMG_W, IMG_H)).reshape(-1, 3) / 255.0
        z_flat = Z.reshape(-1)

        # Valid mask (same mask used for depth)
        mask = (z_flat > 0.02) & np.isfinite(z_flat)
        points = points[mask]
        colors = colors[mask]

        if points.shape[0] > 0:
            try:
                # update visualizer (non-blocking)
                self.vis.update(points, colors)
            except Exception as e:
                self.get_logger().warn(f"O3D update failed: {e}")
        else:
            self.get_logger().info("No valid 3D points in this frame.")

        self.get_logger().info(f"Inference loop: {time.perf_counter()-start:.4f} sec")

def main(args=None):
    rclpy.init(args=args)
    node = CreStereoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # cleanup
        node.vis.close()
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

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


map_db = {}

# ======================= SLAM UTILS =======================
class FeatureTracker:
    def __init__(self, min_features=50):
        self.min_features = min_features
        self.active_points = None   # Nx2 array of 2D feature positions
        self.prev_gray = None
        self.akaze = cv2.AKAZE_create()

    def initialize(self, gray_img):
        """Initial feature detection on the first frame."""
        keypoints = self.akaze.detect(gray_img, None)
        self.active_points = np.array([kp.pt for kp in keypoints], dtype=np.float32)

    def track_and_refresh(self, gray_img, prev_2d, prev_3d):
        """
        Track points from previous frame to current and top up new features.

        Args:
            gray_img (H x W) : Current grayscale frame
            prev_2d (N x 2) : Previous 2D points (u,v)
            prev_3d (N x 3) : Corresponding 3D landmarks

        Returns:
            all_2d_curr (M x 2)    : All 2D points in current frame (tracked + new)
            tracked_2d_curr (K x 2): Subset of current 2D points that survived from prev_2d
            tracked_3d_prev (K x 3): Matching 3D landmarks from prev_3d for PnP
        """
        tracked_2d_curr = np.empty((0, 2), dtype=np.float32)
        tracked_3d_prev = np.empty((0, 3), dtype=np.float32)

        # --- Track prev points with optical flow ---
        if prev_2d is not None and len(prev_2d) > 0 and self.prev_gray is not None:
            next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, gray_img, prev_2d.astype(np.float32), None,
                winSize=(21, 21), maxLevel=3,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
            )

            good_mask = status.flatten() == 1
            tracked_2d_curr = next_pts[good_mask]
            tracked_3d_prev = prev_3d[good_mask]

        # --- Top-up new features if total is too low ---
        all_2d_curr = tracked_2d_curr.copy()
        if all_2d_curr is None or len(all_2d_curr) < self.min_features:
            mask = np.full(gray_img.shape, 255, dtype=np.uint8)

            # Avoid placing new features too close to tracked ones
            if len(tracked_2d_curr) > 0:
                for pt in tracked_2d_curr.astype(int):
                    cv2.circle(mask, tuple(pt), 15, 0, -1)

            # Detect new AKAZE keypoints
            new_kps = self.akaze.detect(gray_img, mask)
            new_pts = np.array([kp.pt for kp in new_kps], dtype=np.float32)

            if new_pts.size > 0:
                if len(all_2d_curr) > 0:
                    all_2d_curr = np.vstack([all_2d_curr, new_pts])
                else:
                    all_2d_curr = new_pts

            # Limit to min_features
            if len(all_2d_curr) > self.min_features:
                all_2d_curr = all_2d_curr[:self.min_features]

        # --- Update memory for next round ---
        self.prev_gray = gray_img.copy()

        # Always return np.ndarray
        return (
            np.array(all_2d_curr, dtype=np.float32),
            np.array(tracked_2d_curr, dtype=np.float32),
            np.array(tracked_3d_prev, dtype=np.float32),
        )
# ========================================================
class LandmarkProjector:
    def __init__(self, K):
        """
        Args:
            K: (3x3) camera intrinsic matrix
        """
        self.K = K
        self.fx = K[0, 0]
        self.fy = K[1, 1]
        self.cx = K[0, 2]
        self.cy = K[1, 2]

    def get_valid_landmarks(self, pts_2d, depth_map):
        """
        For each 2D feature point, get its valid 3D landmark if available.
        
        Args:
            pts_2d: (N x 2) array of (u,v) pixel coords
            depth_map: (H x W) depth array, already masked/filtered

        Returns:
            valid_2d: (M x 2) array of 2D feature coords with valid depth
            valid_3d: (M x 3) array of corresponding 3D points
        """
        valid_2d = []
        valid_3d = []

        H, W = depth_map.shape

        for u, v in pts_2d:
            u_int, v_int = int(round(u)), int(round(v))

            # Skip points outside the image
            if not (0 <= u_int < W and 0 <= v_int < H):
                continue

            d = depth_map[v_int, u_int]
            if d <= 0 or np.isnan(d) or np.isinf(d):
                continue  # skip invalid depth

            # Backproject pixel (u,v,d) to 3D
            X = (u - self.cx) * d / self.fx
            Y = (v - self.cy) * d / self.fy
            Z = d

            valid_2d.append([u, v])
            valid_3d.append([X, Y, Z])

        if len(valid_2d) == 0:
            return np.empty((0, 2), dtype=np.float32), np.empty((0, 3), dtype=np.float32)

        return np.array(valid_2d, dtype=np.float32), np.array(valid_3d, dtype=np.float32)


# ==============================================================

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

        self.tracker = FeatureTracker(min_features=50)
        
        # Global state
        # self.keyframe_initialized = False
        self.prev_3d = None
        self.prev_gray = None
        self.prev_pts = None
        self.frame_count = 0
        self.track_interval = 10   # every 10 frames, do local re-matching
        self.boundary_ratio = 0.9
        
        # Camera intrinsics (adjust if known)
        self.cx = IMG_W / 2.0
        self.cy = IMG_H / 2.0
        self.focal_length = 277.12  # for 320x240
        self.baseline = 0.07
        self.u, self.v = np.meshgrid(np.arange(IMG_W), np.arange(IMG_H))

        self.K = np.array([[self.focal_length, 0, self.cx],
              [0, self.focal_length, self.cy],
              [0, 0, 1]], dtype=np.float32)
        
        self.landmark_projector = LandmarkProjector(self.K)
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
        valid_mask = (disp_map > 3.0) & (disp_map < 60.0)

        # Compute depth
        depth = np.zeros_like(disp_map, dtype=np.float32)
        depth[valid_mask] = (self.focal_length * self.baseline) / disp_map[valid_mask]

        # # =============2D Viz=================================
        # depth_vis = np.clip(depth, 0.02, 10.0)
        # # normalize for color map (avoid division by zero)
        # if np.nanmax(depth_vis) > 0:
        #     depth_vis_norm = (depth_vis / np.nanmax(depth_vis)).astype(np.float32)
        # else:
        #     depth_vis_norm = depth_vis.astype(np.float32)
        # depth_color = cv2.applyColorMap((depth_vis_norm * 255).astype(np.uint8), cv2.COLORMAP_MAGMA)

        # # Show left + depth side-by-side
        # left_vis = cv2.resize(left_img, (depth_color.shape[1], depth_color.shape[0]))
        # combined = np.hstack((left_vis, depth_color))
        # cv2.imshow("Stereo Depth", combined)
        # cv2.waitKey(1)
        # ===============================================================
        
        # ================= VO Pipeline =======================
        gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        if self.tracker.active_points is None:
            self.tracker.initialize(gray)
        # print(len(self.tracker.active_points))
        
        pts_2d, pt_2d_surv, pt_3d_prev = self.tracker.track_and_refresh(gray,self.prev_pts,self.prev_3d)  # Nx2 array of (u,v) coordinates
        
        # Inside stereo_callback:
        valid_2d, valid_3d = self.landmark_projector.get_valid_landmarks(pts_2d, depth)
        
        self.prev_3d = valid_3d
        self.prev_pts = valid_2d
        self.get_logger().info(f"active feats: {len(self.prev_pts)} survived {len(pt_2d_surv)} pts | gap {abs(len(self.prev_pts)-len(pt_2d_surv))}")
         # Optional: draw tracked points for visualization
        vis_frame = left_img.copy()
        for pt in valid_2d:
            u, v = int(pt[0]), int(pt[1])
            cv2.circle(vis_frame, (u, v), 2, (0, 0, 255), -1)
        cv2.imshow("Tracked Features", vis_frame)
        cv2.waitKey(1)
        if len(pt_3d_prev) > 0 and len(pt_2d_surv) > 0:
        # --- Estimate Camera Pose (PnP + RANSAC) ---
            # self.get_logger().info(f"3d feats: {len(self.pt_3d_prev)} survived {len(pt_2d_surv)} pts")
            if len(valid_2d) >= 10:   # need at least 4 for PnP, 10+ is safer
                success, rvec, tvec, inliers = cv2.solvePnPRansac(
                    objectPoints=pt_3d_prev.astype(np.float32),
                    imagePoints=pt_2d_surv.astype(np.float32),
                    cameraMatrix=self.K,   # your intrinsics
                    distCoeffs=None,
                    flags=cv2.SOLVEPNP_EPNP,
                    reprojectionError=8.0,
                    iterationsCount=100,
                    confidence=0.99
                )

                if success:
                    # Convert rotation vector to rotation matrix
                    R, _ = cv2.Rodrigues(rvec)

                    # Build homogeneous transformation [R | t]
                    T_cam = np.eye(4)
                    T_cam[:3, :3] = R
                    T_cam[:3, 3] = tvec.ravel()

                    # Update global pose (world_T_cam)
                    if not hasattr(self, "world_T_cam"):
                        self.world_T_cam = np.eye(4)  # initialize identity at frame 0

                    # Chain poses (odometry)
                    self.world_T_cam = self.world_T_cam @ np.linalg.inv(T_cam)

                    # Extract translation & orientation
                    tx, ty, tz = self.world_T_cam[:3, 3]
                    # Convert R -> Euler (roll, pitch, yaw)
                    roll, pitch, yaw = cv2.RQDecomp3x3(self.world_T_cam[:3, :3])[0]

                    # Print odometry nicely
                    self.get_logger().info(
                        f"Odometry | Position: x={tx:.3f}, y={ty:.3f}, z={tz:.3f} | "
                        f"Orientation (rpy): roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}"
                    )

        # #=================== Build point cloud===========================
        X = (self.u - self.cx) * depth / self.focal_length
        Y = (self.v - self.cy) * depth / self.focal_length
        Z = depth

        points = np.stack((X, Y, Z), axis=-1).reshape(-1, 3)
        colors = cv2.cvtColor(vis_frame, cv2.COLOR_BGR2RGB)
        colors = cv2.resize(colors, (IMG_W, IMG_H)).reshape(-1, 3) / 255.0
        z_flat = Z.reshape(-1)

        # Valid mask (same mask used for depth)
        mask = (z_flat > 0.02) & np.isfinite(z_flat)
        points = points[mask]
        colors = colors[mask]
        # ================================== 3D viz ======================
        if points.shape[0] > 0:
            try:
                # update visualizer (non-blocking)
                self.vis.update(points, colors)
            except Exception as e:
                self.get_logger().warn(f"O3D update failed: {e}")
        else:
            self.get_logger().info("No valid 3D points in this frame.")
        # =================================================================


        # self.get_logger().info(f"Inference loop: {time.perf_counter()-start:.4f} sec")

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

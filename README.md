# Untitled

# 📷 Custom Low-Cost Stereo Depth Camera Setup (Project Bee)

In order to enable **real-time depth perception** on a low budget, we designed and built a **custom stereo vision camera** using off-the-shelf ESP32-S3 modules and OV5640 sensors. This system serves as the backbone for our UAV’s onboard depth estimation pipeline, integrated into **Project Bee**.

---

## 🛠️ Hardware Design & Assembly

1. **PCB Design & 3D Printed Mounts**
    - Designed a custom PCB to mount and interface two **ESP32-S3-CAM modules**, each equipped with **OV5640 camera sensors**.
    - 3D-printed mechanical mounts ensured correct stereo baseline alignment and rigidity.
2. **Parallel Camera Configuration**
    - ESP32-S3 boards connected in **parallel** on the PCB.
    - UART **RX/TX lines bridged** via etched PCB traces to enable synchronized data signaling.
3. **Synchronized Capture Trigger**
    - A hardware pulse was generated to **trigger both cameras simultaneously**, ensuring **no temporal offset** between stereo frames.

---

## 📡 Firmware & Transmission

1. **ESP32 VTX Firmware**
    - Written in **C++**, each ESP32-S3 captures **VGA-resolution frames**.
    - Frames are **chunked into UDP packets** for transmission over Wi-Fi.
2. **Networking Setup**
    - Each ESP32 connects to a **router with a dedicated IP**.
    - UDP streams from both cameras are sent simultaneously to the ground station (server PC).

---

## 💻 Server-Side Reconstruction

1. **ROS 2 Frame Reconstruction Node**
    - Custom ROS 2 nodes receive fragmented UDP packets.
    - Packets reassembled into full image frames.
2. **Error Handling**
    - If chunks are missing, a **Gaussian patching algorithm** reconstructs the missing pixel regions, minimizing frame drops.
3. **Stereo Stream Output**
    - Once reconstructed, frames are published as **synchronized stereo image topics**, ready for integration with **depth estimation, point cloud generation, and SLAM pipelines**.

---

## 🚀 Applications

- **Stereo Depth Estimation**
    
    Real-time disparity computation for **dense point cloud generation**.
    
- **Perception Stack Integration**
    
    Direct pipeline into **Depth Anything V2** and other 3D reconstruction systems.
    
- **Robust UAV Autonomy**
    
    Enables operation in **GPS-denied** or cluttered environments.
    

---

✅ This system provides a **low-cost, lightweight, and Wi-Fi-based stereo vision solution**, making it ideal for UAVs where traditional stereo cameras are too expensive or power-hungry.
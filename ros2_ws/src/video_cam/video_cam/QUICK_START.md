# SIYI Camera Start Guide

**Version:** 2.0  
**Last Updated:** January 15, 2026

---

## What we got

This new system takes pictures, saves them into an SD thats mounted on the camera, then downloads them to the Jetson over Ethernet.

---
## Before setting up
```bash
# setting up the ip
ip link show
# this will show all the ports. Look for 'eth0'
# then check the jetson ip
ip addr show eth0
# Or whatever it is instead of the eth0
# Look for: inet 192.168.144.100/24

# check connection to Camera
ping 192.168.144.25
```
##  Setup

### Step 1: Connect Hardware
```bash
# Physical connections:
 Ethernet cable: Camera ↔ Jetson
 SD card inserted in camera
 Camera powered on
```

### Step 2: Configure Network
```bash
# Set Jetson IP to 192.168.144.100
sudo nmtui
# Select "Edit a connection" → Your Ethernet → Manual
# IP: 192.168.144.100, Netmask: 255.255.255.0
```

### Step 3: Test Connection
```bash
# Ping the camera
ping 192.168.144.25
# Should see replies with <5ms latency

# Test API (optional but recommended)
curl http://192.168.144.25:82/cgi-bin/media.cgi/api/v1/getdirectories?media_type=0
# Should return JSON with "success":true
```

### Step 4: Build Package
```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
colcon build
source install/setup.bash
```

### Step 5: Run Setup Check (Optional)
```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/video_cam
./siyi_setup_check.sh
# This will verify everything is configured correctly
```

---

## Taking 4K Photos

### Terminal 1: Start Camera Node
```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
source install/setup.bash
ros2 launch video_cam siyi_camera2.launch.py
```

Wait for this message:
```
✓ Camera stream connected
✓ SD card ready
```

### Terminal 2: Capture Photo
```bash
source install/setup.bash
ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"
```


### Photos Are Here:
```bash
~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/video_cam_data/
├── camera_feed/         ← For detection/processing
│   └── photo_4K_20260115-143022.jpg
└── mapping_photos/      ← For archival
    └── mapping_20260115-143022.jpg
```

---

## Common Tasks

### Change Resolution
```bash
# 4K (default - 3840x2160)
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '4K'"

# 2.7K (2704x1520)
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '2.7K'"

# 1080P (1920x1080)
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '1080P'"
```

### View Live Stream
```bash
ros2 run rqt_image_view rqt_image_view /image_raw
```

### Download All Photos from SD Card
```bash
mkdir -p ~/siyi_downloads
ros2 run video_cam siyi2 --dest ~/siyi_downloads --verbose
```

### Check System Status
```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/video_cam
./siyi_setup_check.sh
```

---

## Troubleshooting

### Problem: "Cannot connect to camera"
```bash
# 1. Check Ethernet cable
# 2. Verify IP:
ip addr | grep 192.168.144
# Should show: 192.168.144.100/24

# 3. Ping camera:
ping 192.168.144.25
```

### Problem: "SD verification timeout"
```bash
# 1. Check SD card is inserted
# 2. Format SD card in camera menu (erases all data!)
# 3. Try taking a photo manually on camera
# 4. If still fails, try a different SD card (Class 10+)
```

### Problem: "Images are not 4K"
```bash
# Check camera settings using SIYI app
# Set Photo Resolution to 4K in camera menu
# Then run:
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '4K'"
```

-------------------    -----------------    -------------   --------------

##  File Locations

### Old Code:
- `video_cam/video_cam/image_pub_siyi.py` ← Original (preserved)
- `video_cam/siyi.py` ← Original (preserved)

### New Code:
- `video_cam/video_cam/image_pub_siyi2.py` ← Combined ROS2 node
- `video_cam/video_cam/siyi2.py` ← Enhanced download utility
- `video_cam/launch/siyi_camera2.launch.py` ← Launch file
- `video_cam/SIYI_CAMERA_COMPLETE_GUIDE.md` ← Full documentation
- `video_cam/siyi_setup_check.sh` ← Setup verification

### Generated Data:
- `video_cam_data/camera_feed/` ← Photos for detection
- `video_cam_data/mapping_photos/` ← Photos for archival
- `video_cam_data/downloaded_from_sd/` ← Batch downloads

---

##  Before Flight

**Checklist:**
- Network configured (192.168.144.100)
- Camera connected and powered
- SD card inserted in camera
- Setup check script passes: `./siyi_setup_check.sh`
- Test photo capture works
- Verify photos in both save directories

---

##  Need Help?

1. **Read full guide:** `SIYI_CAMERA_COMPLETE_GUIDE.md`
2. **Run setup check:** `./siyi_setup_check.sh`

---

##  Pro Tips

### Faster Workflow
Create aliases in `~/.bashrc`:
```bash
alias siyi_start='cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws && source install/setup.bash && ros2 launch video_cam siyi_camera2.launch.py'
alias siyi_photo='ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"'
alias siyi_4k='ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: \"4K\""'
```

Then just run:
```bash
siyi_start     # Start camera
siyi_photo     # Take photo
siyi_4k        # Set 4K mode
```

### Monitor Captures
```bash
# Watch for new photos
watch -n 1 'ls -lht ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/video_cam_data/camera_feed/ | head -5'
```

### Auto-backup
```bash
# Sync photos to external drive
rsync -av ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/video_cam_data/ /mnt/backup/siyi_photos/
```

---
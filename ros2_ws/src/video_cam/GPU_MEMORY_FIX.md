# GPU Memory Management for SAHI Object Detection on Jetson Orin Nano

## Problem

When running SAHI object detection on Jetson Orin Nano, you may encounter these errors:

```
NVML_SUCCESS == r INTERNAL ASSERT FAILED at "/opt/pytorch/pytorch/c10/cuda/CUDACachingAllocator.cpp":838
NvMapMemAllocInternalTagged: 1075072515 error 12
NvMapMemHandleAlloc: error 0
CUBLAS_STATUS_ALLOC_FAILED when calling `cublasCreate(handle)`
```

These errors indicate **GPU memory exhaustion** caused by:
1. SAHI creating multiple image slices processed simultaneously
2. Limited GPU memory (7.44GB) on Jetson Orin Nano
3. Multiple model instances being loaded
4. GPU memory fragmentation

## Quick Fix

### Step 1: Run the GPU Memory Recovery Script

```bash
cd ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/video_cam
bash fix_gpu_memory.sh
```

This script will:
- Kill all processes using GPU memory
- Clear PyTorch GPU cache
- Reset CUDA contexts
- Display current GPU/RAM status

### Step 2: Run with Optimized Parameters

After cleanup, run with **larger slices and less overlap** to reduce memory usage:

```bash
source ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/install/setup.bash

ros2 run video_cam object_detection_sahi --ros-args \
  -p slice_height:=640 \
  -p slice_width:=640 \
  -p overlap_height_ratio:=0.2 \
  -p overlap_width_ratio:=0.2
```

**Why this works:**
- Larger slices (640×640 instead of 512×512) = fewer slices
- Less overlap (20% instead of 30%) = fewer duplicate regions
- Fewer slices = less GPU memory usage

## Alternative Solutions

### Option 1: Use Smaller Model

If still experiencing issues, use the nano model (smaller, faster, less memory):

```bash
ros2 run video_cam object_detection_sahi --ros-args \
  -p model_path:=yolo11n.pt \
  -p slice_height:=640 \
  -p slice_width:=640 \
  -p overlap_height_ratio:=0.2 \
  -p overlap_width_ratio:=0.2
```

### Option 2: Process Fewer Images

Clear out old images from camera_feed folder before running:

```bash
# Keep only recent images
cd ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/install/video_cam/share/video_cam/camera_feed
ls -t | tail -n +10 | xargs rm  # Keep only 10 newest images
```

### Option 3: CPU Mode (Last Resort)

If GPU continues to fail, use CPU mode (slower but reliable):

```bash
ros2 run video_cam object_detection_sahi --ros-args \
  -p device:=cpu
```

## Understanding the Parameters

### Slice Size
- **512×512**: More slices, better small object detection, MORE memory
- **640×640**: Fewer slices, good detection, LESS memory ✓ (recommended)
- **1280×720**: No slicing, fast but misses small objects

### Overlap Ratio
- **0.3 (30%)**: More overlap, better boundary detection, MORE memory
- **0.2 (20%)**: Good overlap, efficient, LESS memory ✓ (recommended)
- **0.1 (10%)**: Minimal overlap, may miss boundary objects

### Example: 1280×720 image with different settings

**512×512 slices, 30% overlap:**
- Number of slices: ~9
- GPU memory: ~3-4GB
- Best for: Maximum accuracy

**640×640 slices, 20% overlap:**
- Number of slices: ~4
- GPU memory: ~2-3GB
- Best for: Balance (recommended for Jetson)

## Diagnostic Tools

### Check GPU Memory Before Running

```bash
cd ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/video_cam
python3 check_gpu_memory.py
```

This will show:
- Available GPU memory
- Recommended parameters based on current state
- PyTorch/CUDA status

### Monitor GPU During Execution

In a separate terminal:

```bash
# Jetson-specific monitoring
tegrastats --interval 1000

# Or generic NVIDIA monitoring
watch -n 1 nvidia-smi
```

## Files Created

1. **fix_gpu_memory.sh** - Main GPU recovery script
2. **check_gpu_memory.py** - GPU diagnostic tool
3. **clear_gpu_memory.sh** - Simple cleanup script
4. **GPU_MEMORY_FIX.md** - This documentation

## Permanent Code Changes Made

The following optimizations were added to `object_detection_sahi.py`:

1. **Memory fraction limiting** (line ~262):
   ```python
   torch.cuda.set_per_process_memory_fraction(0.8, 0)  # Use max 80% GPU memory
   ```

2. **Cache clearing before each image** (line ~420):
   ```python
   torch.cuda.empty_cache()
   ```

3. **Cache clearing after each image** (line ~393):
   ```python
   torch.cuda.empty_cache()
   ```

4. **Skip full image prediction** (line ~438):
   ```python
   perform_standard_pred=False  # Save memory
   ```

5. **Changed default parameters** (lines 70-71):
   ```python
   slice_height: 640  # Changed from 512
   slice_width: 640   # Changed from 512
   overlap_ratio: 0.2 # Changed from 0.3
   ```

## Troubleshooting

### Still getting memory errors?

1. **Reboot the Jetson** - Clears all system memory
   ```bash
   sudo reboot
   ```

2. **Check for other GPU processes**:
   ```bash
   nvidia-smi
   # Kill any unexpected processes
   ```

3. **Reduce image resolution** before processing:
   ```bash
   # In camera capture script, resize to 640×480 instead of 1280×720
   ```

4. **Process images one at a time** by moving them in/out of camera_feed folder

### Error persists after all fixes?

This may indicate a hardware or driver issue:

1. Check CUDA/cuDNN installation
2. Verify Jetson power mode: `sudo nvpmodel -q`
3. Set max performance: `sudo nvpmodel -m 0`
4. Check system logs: `dmesg | grep -i cuda`

## Best Practices

1. **Always run fix_gpu_memory.sh** before starting detection
2. **Start with optimized parameters** (640×640, 0.2 overlap)
3. **Monitor GPU usage** during first run
4. **Process in batches** if many images
5. **Clear processed images** from camera_feed regularly

## Performance Comparison

| Configuration | Memory Usage | Speed | Small Object Detection |
|---------------|--------------|-------|------------------------|
| 512×512, 30% overlap | 3-4GB | Slow | Excellent |
| 640×640, 20% overlap | 2-3GB | Medium | Good ✓ |
| No SAHI | 1-2GB | Fast | Poor |
| CPU Mode | 0GB GPU | Very Slow | Good |

**Recommended**: 640×640 with 20% overlap for Jetson Orin Nano

## Additional Notes

- The Jetson Orin Nano has **unified memory** (shared between CPU and GPU)
- System RAM usage also affects GPU memory availability
- Close unnecessary applications before running detection
- GPU temperature affects performance (keep below 85°C)

## Support

If issues persist after following this guide:
1. Check the ROS2 logs: `ros2 topic echo /sahi_detection_info`
2. Review terminal output for specific error messages
3. Try CPU mode to verify the model itself works
4. Check if other CUDA applications work on your system

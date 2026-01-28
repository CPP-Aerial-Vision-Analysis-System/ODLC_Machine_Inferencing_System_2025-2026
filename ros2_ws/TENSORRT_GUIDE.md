# TensorRT Acceleration for YOLO Detection

This guide explains how to use TensorRT-accelerated YOLO models for faster inference on NVIDIA Jetson devices.

## Overview

TensorRT provides **3-5x faster inference** compared to PyTorch by optimizing the model specifically for your GPU. The first-time conversion takes 5-15 minutes, but subsequent runs are much faster.

## Quick Start

### Option 1: Auto-Convert on First Run (Easiest)
The detection node will automatically convert `.pt` to `.engine` on first run:

```bash
cd ~/astra/ros2_ws
source install/setup.bash
ros2 run detection new_od
```

**Note**: The first run will take 5-15 minutes as it builds the TensorRT engine. Be patient and don't interrupt! Subsequent runs will be instant.

### Option 2: Pre-Build Engine (Recommended)
Build the TensorRT engine offline before running the node:

```bash
cd ~/astra/ros2_ws
python3 build_tensorrt_engine.py yolo26m.pt
```

This shows progress and allows you to troubleshoot any issues before running the detection node.

## Fixing Common Issues

### Issue 1: Protobuf Version Conflicts
**Symptom**: `Descriptors cannot be created directly` or `np.object` errors

**Fix**:
```bash
pip3 install --user "numpy==1.23.5" "onnx>=1.12.0,<=1.19.1" "protobuf>=3.20.0,<4.0.0" onnxslim
```

### Issue 2: GPU Memory Errors
**Symptom**: `NvMapMemAllocInternalTagged` errors or OOM during conversion

**Fix**: Run the memory cleanup script:
```bash
cd ~/astra/ros2_ws
bash fix_gpu_memory.sh
# Answer 'y' to kill GPU processes
# Answer 'n' to skip adding swap (unless you need it)
```

Then retry the conversion with smaller workspace:
```bash
python3 build_tensorrt_engine.py yolo26m.pt --workspace 2
```

### Issue 3: Conversion Takes Too Long
**Solution**: This is normal for large models. For `yolo26x.pt` (200MB), expect 10-15 minutes. Use smaller models if time is critical:

- `yolo26n.pt` - Fastest, ~2-3 minutes to convert
- `yolo26s.pt` - Balanced, ~4-6 minutes
- `yolo26m.pt` - Good accuracy, ~6-10 minutes ✅ **Recommended**
- `yolo26x.pt` - Best accuracy, ~10-15 minutes

## Using TensorRT with ROS2

### Default Behavior
The node automatically:
1. Checks for `.engine` file
2. If not found, converts `.pt` to `.engine`
3. Uses TensorRT if available, falls back to PyTorch if conversion fails

### Manual Control
Force PyTorch mode (no TensorRT):
```bash
ros2 run detection new_od --ros-args -p model_format:=pytorch
```

Force TensorRT mode (fail if conversion fails):
```bash
ros2 run detection new_od --ros-args -p model_format:=tensorrt
```

Disable auto-conversion:
```bash
ros2 run detection new_od --ros-args -p auto_convert_tensorrt:=false
```

## Performance Comparison

| Model | Format | FPS (4K) | Latency | GPU Memory |
|-------|--------|----------|---------|------------|
| yolo26m | PyTorch | ~0.1 | 10s | 3-4 GB |
| yolo26m | TensorRT | ~0.3-0.5 | 2-3s | 2-3 GB |
| yolo26n | PyTorch | ~0.3 | 3s | 2 GB |
| yolo26n | TensorRT | ~1.0 | 1s | 1-2 GB |

*Note: FPS and latency depend on SAHI slicing configuration*

## Verifying TensorRT is Working

When the node starts, look for these log messages:

**✅ TensorRT Active:**
```
[INFO] [new_od]: ✓ TensorRT engine found: yolo26m.engine (50.2MB)
[INFO] [new_od]: Loading model: /home/astra-dev/astra/ros2_ws/yolo26m.engine (format: tensorrt)
[INFO] [new_od]: ✓ TensorRT model loaded via SAHI
```

**⚠️ PyTorch Fallback:**
```
[WARN] [new_od]: TensorRT conversion failed, using PyTorch
[INFO] [new_od]: Loading model: /home/astra-dev/astra/ros2_ws/yolo26m.pt (format: pytorch)
```

## File Locations

Models and engines are stored in the ROS2 workspace:
```
~/astra/ros2_ws/
├── yolo26n.pt          # PyTorch models
├── yolo26s.pt
├── yolo26m.pt
├── yolo26x.pt
├── yolo26n.engine      # TensorRT engines (generated)
├── yolo26s.engine
├── yolo26m.engine
└── yolo26x.engine
```

You can also specify custom paths:
```bash
ros2 run detection new_od --ros-args -p model_path:=/custom/path/model.pt
```

## Advanced Configuration

### Custom Image Size
Match your slice dimensions:
```bash
python3 build_tensorrt_engine.py yolo26m.pt --imgsz 512
```

### More GPU Memory
Increase workspace for faster conversion (if you have memory):
```bash
python3 build_tensorrt_engine.py yolo26m.pt --workspace 6
```

### CPU-Only Mode
Build engine without CUDA (much slower):
```bash
python3 build_tensorrt_engine.py yolo26m.pt --device cpu --no-half
```

## Troubleshooting Checklist

If TensorRT conversion fails:

1. ✅ Check Python dependencies:
   ```bash
   pip3 list | grep -E "numpy|onnx|protobuf|torch"
   ```
   Should show: numpy 1.23.5, onnx 1.12-1.19, protobuf 3.20.x or 6.x

2. ✅ Clear GPU memory:
   ```bash
   bash fix_gpu_memory.sh
   ```

3. ✅ Check available GPU memory:
   ```bash
   python3 -c "import torch; print(f'{torch.cuda.get_device_properties(0).total_memory/1e9:.1f}GB')"
   ```

4. ✅ Try smaller workspace:
   ```bash
   python3 build_tensorrt_engine.py yolo26m.pt --workspace 2
   ```

5. ✅ Use smaller model:
   ```bash
   python3 build_tensorrt_engine.py yolo26n.pt  # Much faster
   ```

6. ✅ Check logs for specific errors:
   ```bash
   ros2 run detection new_od 2>&1 | tee detection.log
   ```

## Getting Help

If you're still having issues:
1. Check the full error message in terminal output
2. Verify CUDA is working: `python3 -c "import torch; print(torch.cuda.is_available())"`
3. Check TensorRT version: `dpkg -l | grep tensorrt`
4. Check system info: `cat /proc/device-tree/model`

## Best Practices

1. **Pre-build engines offline** using `build_tensorrt_engine.py` before running the node
2. **Use yolo26m** for best balance of speed and accuracy
3. **Clear GPU memory** before building engines
4. **Don't interrupt** the conversion process
5. **Reuse engines** - they only need to be built once per model

## Environment Variables

The detection node automatically sets:
```bash
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
```

This fixes protobuf compatibility issues with ONNX/TensorRT.

## Summary

✅ **Fixed Issues:**
- Protobuf version conflicts (now uses pure-Python implementation)
- NumPy compatibility (downgraded to 1.23.5)
- GPU memory management (added cache clearing)
- **SAHI + TensorRT integration** (TensorRT engines now load correctly with SAHI)
- Better error messages and logging

✅ **New Features:**
- `build_tensorrt_engine.py` - Pre-build engines offline
- `test_tensorrt_sahi.py` - Verify TensorRT + SAHI integration
- Auto-detection of TensorRT vs PyTorch
- Graceful fallback if TensorRT fails
- Verbose logging for troubleshooting

🚀 **Performance Gain:**
- 3-5x faster inference with TensorRT
- Lower GPU memory usage
- Better throughput for real-time detection

## Technical Details

The detection node now properly loads TensorRT engines by:
1. Loading the `.engine` file with `YOLO(model.engine, task='detect')`
2. Wrapping it in SAHI's `Yolov8DetectionModel` for sliced inference
3. This gives us both TensorRT speed AND SAHI's slicing capabilities

The previous approach tried to use `AutoDetectionModel.from_pretrained()` which doesn't support `.engine` files.

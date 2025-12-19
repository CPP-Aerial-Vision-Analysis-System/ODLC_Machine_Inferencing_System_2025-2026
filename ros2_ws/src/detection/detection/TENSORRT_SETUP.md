# TensorRT Setup Guide

## What Was Changed

A new file `new_od.py` has been created with TensorRT support. This file includes:

1. **TensorRT Detection**: Automatically detects if TensorRT is available
2. **Model Format Selection**: Supports `pytorch`, `tensorrt`, or `auto` modes
3. **Auto-Conversion**: Can automatically convert PyTorch models to TensorRT
4. **Fallback Support**: Falls back to PyTorch if TensorRT unavailable

## What Else Needs to Be Done

### 1. Install TensorRT

#### For Jetson (NVIDIA embedded systems):
```bash
# TensorRT is usually pre-installed on Jetson
# Verify installation:
python3 -c "import tensorrt; print(tensorrt.__version__)"
```

#### For x86 systems with NVIDIA GPU:
```bash
# Install TensorRT
pip install nvidia-tensorrt

# Or download from NVIDIA website:
# https://developer.nvidia.com/tensorrt
```

### 2. Update setup.py

Add the new executable to `setup.py`:

```python
# In ros2_ws/src/detection/setup.py
entry_points={
    'console_scripts': [
        'object_detection_sahi = detection.object_detection_sahi:main',
        'object_detection_sahi_tensorrt = detection.new_od:main',  # ← ADD THIS
        # ... other entries
    ],
},
```

### 3. Rebuild the Package

```bash
cd ros2_ws
colcon build --packages-select detection
source install/setup.bash
```

### 4. Convert Your Model (Optional - Auto-conversion available)

#### Manual Conversion:
```python
from ultralytics import YOLO

# Load PyTorch model
model = YOLO('yolo11s.pt')

# Export to TensorRT
model.export(
    format='engine',
    device=0,              # GPU device
    workspace=4,           # GPU memory in GB
    simplify=True
)
# Creates: yolo11s.engine
```

#### Automatic Conversion:
The new code will automatically convert if:
- `model_format:='auto'` (default)
- `auto_convert_tensorrt:=True` (default)
- `.pt` file exists but `.engine` doesn't

### 5. Update Launch Files

Create or modify launch file to use TensorRT version:

```python
# In detection/launch/detection_tensorrt.launch.py
sahi_detection_node = Node(
    package='detection',
    executable='object_detection_sahi_tensorrt',  # ← Use new executable
    name='sahi_object_detection_node_tensorrt',
    parameters=[{
        'model_path': 'yolo11s.pt',           # Can be .pt or .engine
        'model_format': 'auto',                # 'pytorch', 'tensorrt', or 'auto'
        'auto_convert_tensorrt': True,         # Auto-convert if needed
        'tensorrt_workspace': 4,               # GPU memory for TensorRT (GB)
        'confidence_threshold': 0.15,
        # ... other parameters
    }],
)
```

### 6. Test the Setup

#### Test 1: Check TensorRT Availability
```bash
python3 -c "import tensorrt; print('TensorRT version:', tensorrt.__version__)"
```

#### Test 2: Run with PyTorch (fallback)
```bash
ros2 run detection object_detection_sahi_tensorrt \
    --ros-args \
    -p model_path:=yolo11s.pt \
    -p model_format:=pytorch
```

#### Test 3: Run with TensorRT
```bash
ros2 run detection object_detection_sahi_tensorrt \
    --ros-args \
    -p model_path:=yolo11s.engine \
    -p model_format:=tensorrt
```

#### Test 4: Run with Auto-detection
```bash
ros2 run detection object_detection_sahi_tensorrt \
    --ros-args \
    -p model_path:=yolo11s.pt \
    -p model_format:=auto \
    -p auto_convert_tensorrt:=True
```

### 7. Verify SAHI TensorRT Support

**Important**: SAHI may need updates to fully support TensorRT engines directly. 

If you encounter issues, you may need to:
- Use Ultralytics YOLO directly with TensorRT (bypass SAHI for TensorRT)
- Wait for SAHI library updates
- Use ONNX as intermediate format

### 8. Performance Testing

Compare performance:

```bash
# PyTorch baseline
time ros2 run detection object_detection_sahi

# TensorRT optimized
time ros2 run detection object_detection_sahi_tensorrt \
    --ros-args -p model_format:=tensorrt
```

Expected speedup: **2-5x faster** with TensorRT on NVIDIA GPUs

### 9. Update Documentation

Update any documentation that references the old node name:
- Change `sahi_object_detection_node` → `sahi_object_detection_node_tensorrt`
- Document new parameters: `model_format`, `auto_convert_tensorrt`, `tensorrt_workspace`

### 10. Handle Edge Cases

The new code handles:
- ✅ Missing TensorRT (falls back to PyTorch)
- ✅ Missing .engine file (auto-converts or uses .pt)
- ✅ Invalid model format (warns and uses auto-detection)
- ✅ GPU memory issues (configurable workspace)

## New Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_format` | `'auto'` | Model format: 'pytorch', 'tensorrt', or 'auto' |
| `auto_convert_tensorrt` | `True` | Automatically convert .pt to .engine if missing |
| `tensorrt_workspace` | `4` | GPU memory (GB) for TensorRT optimization |

## Troubleshooting

### Issue: "TensorRT not available"
**Solution**: Install TensorRT or use `model_format:=pytorch`

### Issue: "Conversion failed"
**Solution**: 
- Check GPU memory (increase `tensorrt_workspace`)
- Ensure CUDA is properly installed
- Try manual conversion first

### Issue: "SAHI doesn't support TensorRT"
**Solution**: 
- Use `model_format:=pytorch` for now
- Or modify code to use YOLO directly for TensorRT

### Issue: "Model loading fails"
**Solution**:
- Verify model file exists
- Check file permissions
- Ensure model format matches parameter

## Next Steps

1. ✅ Code created (`new_od.py`)
2. ⏳ Install TensorRT
3. ⏳ Update `setup.py`
4. ⏳ Rebuild package
5. ⏳ Test conversion
6. ⏳ Create launch file
7. ⏳ Performance testing
8. ⏳ Update documentation

## Notes

- TensorRT engines are GPU-specific (Jetson vs x86)
- First conversion takes several minutes
- Subsequent loads are fast
- Keep both .pt and .engine files for flexibility

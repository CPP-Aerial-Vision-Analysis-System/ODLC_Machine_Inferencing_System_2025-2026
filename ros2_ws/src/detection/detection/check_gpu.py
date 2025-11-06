#!/usr/bin/env python3
"""
GPU Detection and Diagnostics Script for Jetson/CUDA Systems
Run this to verify GPU is available and properly configured
"""

import sys
import os

print("=" * 80)
print("GPU DETECTION DIAGNOSTIC TOOL")
print("=" * 80)

# Check 1: System Information
print("\n1. SYSTEM INFORMATION:")
print("-" * 80)
try:
    import platform
    print(f"   OS: {platform.system()} {platform.release()}")
    print(f"   Architecture: {platform.machine()}")
    print(f"   Python: {platform.python_version()}")
    
    # Check if Jetson
    try:
        with open('/proc/device-tree/model', 'r') as f:
            model = f.read().strip('\x00')
            print(f"   Device Model: {model}")
            if 'jetson' in model.lower():
                print(f"   ✓ NVIDIA Jetson Device Detected!")
    except:
        print("   Device Model: Not a Jetson (or file not accessible)")
except Exception as e:
    print(f"   Error getting system info: {e}")

# Check 2: PyTorch Installation
print("\n2. PYTORCH INSTALLATION:")
print("-" * 80)
try:
    import torch
    print(f"   ✓ PyTorch Installed: {torch.__version__}")
    print(f"   Built with CUDA: {torch.version.cuda if torch.version.cuda else 'No'}")
    print(f"   cuDNN Version: {torch.backends.cudnn.version() if torch.backends.cudnn.is_available() else 'N/A'}")
except ImportError:
    print("   ✗ PyTorch NOT installed")
    print("   Install with: pip3 install torch torchvision torchaudio")
    torch = None

# Check 3: CUDA Availability
print("\n3. CUDA AVAILABILITY:")
print("-" * 80)
if torch:
    if torch.cuda.is_available():
        print(f"   ✓ CUDA Available: YES")
        print(f"   CUDA Version: {torch.version.cuda}")
        print(f"   GPU Count: {torch.cuda.device_count()}")
        
        for i in range(torch.cuda.device_count()):
            print(f"\n   GPU {i}:")
            props = torch.cuda.get_device_properties(i)
            print(f"      Name: {torch.cuda.get_device_name(i)}")
            print(f"      Compute Capability: {props.major}.{props.minor}")
            print(f"      Total Memory: {props.total_memory / 1024**3:.2f} GB")
            print(f"      Multi-Processor Count: {props.multi_processor_count}")
            
            # Memory info
            try:
                mem_allocated = torch.cuda.memory_allocated(i) / 1024**3
                mem_reserved = torch.cuda.memory_reserved(i) / 1024**3
                mem_free = (props.total_memory - torch.cuda.memory_allocated(i)) / 1024**3
                print(f"      Memory Allocated: {mem_allocated:.2f} GB")
                print(f"      Memory Reserved: {mem_reserved:.2f} GB")
                print(f"      Memory Free: {mem_free:.2f} GB")
            except:
                pass
    else:
        print(f"   ✗ CUDA Available: NO")
        print("\n   Possible reasons:")
        print("   1. No NVIDIA GPU detected")
        print("   2. CUDA drivers not installed")
        print("   3. PyTorch not built with CUDA support")
        print("   4. Incompatible CUDA version")
else:
    print("   ✗ Cannot check (PyTorch not installed)")

# Check 4: NVIDIA System Tools
print("\n4. NVIDIA SYSTEM TOOLS:")
print("-" * 80)

# Check nvidia-smi
try:
    import subprocess
    result = subprocess.run(['nvidia-smi'], capture_output=True, text=True, timeout=5)
    if result.returncode == 0:
        print("   ✓ nvidia-smi available:")
        # Parse key info
        lines = result.stdout.split('\n')
        for line in lines:
            if 'Driver Version' in line or 'CUDA Version' in line:
                print(f"      {line.strip()}")
            elif '|' in line and 'MiB' in line:
                print(f"      {line.strip()}")
    else:
        print("   ✗ nvidia-smi failed")
except FileNotFoundError:
    print("   ✗ nvidia-smi not found (NVIDIA drivers may not be installed)")
except Exception as e:
    print(f"   ✗ Error running nvidia-smi: {e}")

# Check jetson-clocks (Jetson specific)
try:
    result = subprocess.run(['jetson_clocks', '--show'], capture_output=True, text=True, timeout=5)
    if result.returncode == 0:
        print("\n   ✓ jetson_clocks available (Jetson device)")
except:
    pass

# Check 5: TensorFlow (if available)
print("\n5. TENSORFLOW (Optional):")
print("-" * 80)
try:
    import tensorflow as tf
    print(f"   ✓ TensorFlow Installed: {tf.__version__}")
    print(f"   Built with CUDA: {tf.test.is_built_with_cuda()}")
    gpus = tf.config.list_physical_devices('GPU')
    print(f"   GPU Devices: {len(gpus)}")
    for gpu in gpus:
        print(f"      {gpu}")
except ImportError:
    print("   TensorFlow not installed (optional)")
except Exception as e:
    print(f"   Error checking TensorFlow: {e}")

# Check 6: Ultralytics YOLO
print("\n6. ULTRALYTICS YOLO:")
print("-" * 80)
try:
    from ultralytics import YOLO
    print("   ✓ Ultralytics installed")
    
    # Try to get device info from YOLO
    import ultralytics
    print(f"   Version: {ultralytics.__version__}")
    
    # Test YOLO with device detection
    if torch and torch.cuda.is_available():
        print("   Testing YOLO with CUDA...")
        try:
            model = YOLO('yolo11n.pt')  # Tiny model for testing
            device_info = model.device
            print(f"   ✓ YOLO can use device: {device_info}")
        except Exception as e:
            print(f"   ✗ Error testing YOLO: {e}")
except ImportError:
    print("   ✗ Ultralytics not installed")
    print("   Install with: pip3 install ultralytics")

# Check 7: Environment Variables
print("\n7. ENVIRONMENT VARIABLES:")
print("-" * 80)
cuda_vars = ['CUDA_HOME', 'CUDA_PATH', 'LD_LIBRARY_PATH', 'PATH']
for var in cuda_vars:
    value = os.environ.get(var, 'Not set')
    if var in ['LD_LIBRARY_PATH', 'PATH'] and len(value) > 100:
        # Truncate long paths
        if 'cuda' in value.lower():
            print(f"   {var}: ...contains CUDA paths...")
        else:
            print(f"   {var}: Set (no CUDA in path)")
    else:
        print(f"   {var}: {value}")

# Check 8: Quick Benchmark
print("\n8. QUICK BENCHMARK:")
print("-" * 80)
if torch and torch.cuda.is_available():
    try:
        print("   Running simple tensor operations...")
        
        # CPU benchmark
        import time
        size = 1000
        a = torch.randn(size, size)
        b = torch.randn(size, size)
        
        start = time.time()
        c = torch.matmul(a, b)
        cpu_time = time.time() - start
        print(f"   CPU Time (1000x1000 matmul): {cpu_time*1000:.2f} ms")
        
        # GPU benchmark
        a_gpu = a.cuda()
        b_gpu = b.cuda()
        torch.cuda.synchronize()
        
        start = time.time()
        c_gpu = torch.matmul(a_gpu, b_gpu)
        torch.cuda.synchronize()
        gpu_time = time.time() - start
        print(f"   GPU Time (1000x1000 matmul): {gpu_time*1000:.2f} ms")
        
        speedup = cpu_time / gpu_time
        print(f"   Speedup: {speedup:.2f}x")
        
        if speedup > 1.5:
            print("   ✓ GPU is significantly faster!")
        elif speedup > 0.8:
            print("   ⚠ GPU performance similar to CPU (overhead may be high)")
        else:
            print("   ✗ GPU slower than CPU (check configuration)")
            
    except Exception as e:
        print(f"   Error running benchmark: {e}")

# Summary
print("\n" + "=" * 80)
print("SUMMARY:")
print("=" * 80)

if torch and torch.cuda.is_available():
    print("✓ GPU ACCELERATION AVAILABLE")
    print(f"  Device: {torch.cuda.get_device_name(0)}")
    print(f"  Recommended device string: 'cuda:0'")
else:
    print("✗ GPU ACCELERATION NOT AVAILABLE")
    print("  System will use CPU only")
    print("\n  To enable GPU:")
    print("  1. Install NVIDIA drivers")
    print("  2. Install CUDA toolkit")
    print("  3. Install PyTorch with CUDA support:")
    print("     For Jetson: Follow NVIDIA's PyTorch installation guide")
    print("     For x86: pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118")

print("=" * 80)

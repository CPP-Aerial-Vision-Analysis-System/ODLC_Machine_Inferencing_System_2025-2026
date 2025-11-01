#!/usr/bin/env python3

"""
GPU Detection Test Script

This script checks if a GPU is available and provides detailed information
about the detected hardware acceleration options.
"""

import sys

def print_header(text):
    """Print formatted header"""
    print("\n" + "="*80)
    print(f"  {text}")
    print("="*80)


def check_pytorch():
    """Check PyTorch installation and GPU availability"""
    print_header("PyTorch GPU Detection")
    
    try:
        import torch
        print(f"✓ PyTorch installed: {torch.__version__}")
    except ImportError:
        print("✗ PyTorch not installed")
        print("  Install with: pip install torch torchvision")
        return False
    
    # Check CUDA (NVIDIA GPU)
    print("\n1. NVIDIA CUDA GPU:")
    if torch.cuda.is_available():
        print("   ✓ CUDA is available")
        print(f"   CUDA Version: {torch.version.cuda}")
        print(f"   cuDNN Version: {torch.backends.cudnn.version()}")
        print(f"   GPU Count: {torch.cuda.device_count()}")
        
        for i in range(torch.cuda.device_count()):
            print(f"\n   GPU {i}:")
            print(f"      Name: {torch.cuda.get_device_name(i)}")
            props = torch.cuda.get_device_properties(i)
            print(f"      Total Memory: {props.total_memory / 1024**3:.2f} GB")
            print(f"      Compute Capability: {props.major}.{props.minor}")
            print(f"      Multi-Processor Count: {props.multi_processor_count}")
            
            # Try to allocate memory to verify GPU works
            try:
                x = torch.randn(100, 100).cuda(i)
                del x
                torch.cuda.empty_cache()
                print(f"      Status: ✓ Working")
            except Exception as e:
                print(f"      Status: ✗ Error - {e}")
        
        # Check if Jetson
        try:
            with open('/proc/device-tree/model', 'r') as f:
                model = f.read()
                if 'jetson' in model.lower():
                    print(f"\n   🤖 Platform: NVIDIA Jetson")
                    print(f"      Model: {model.strip()}")
        except:
            pass
        
        print("\n   → Recommended device: cuda:0")
        return True
    else:
        print("   ✗ CUDA not available")
        
        # Check why CUDA might not be available
        try:
            # Try to see if CUDA was compiled into PyTorch
            print("   Reason: No NVIDIA GPU detected or drivers not installed")
            print("   PyTorch CUDA support: Built with CUDA" if torch.version.cuda else "Built without CUDA")
        except:
            print("   Reason: No NVIDIA GPU detected")
    
    # Check MPS (Apple Silicon)
    print("\n2. Apple Metal Performance Shaders (MPS):")
    if hasattr(torch.backends, 'mps'):
        if torch.backends.mps.is_available():
            print("   ✓ MPS is available")
            
            if torch.backends.mps.is_built():
                print("   ✓ MPS is built")
                
                # Try to use MPS
                try:
                    x = torch.randn(100, 100).to('mps')
                    del x
                    print("   ✓ MPS is working")
                    print("\n   → Recommended device: mps")
                    return True
                except Exception as e:
                    print(f"   ✗ MPS error: {e}")
            else:
                print("   ✗ MPS not built in this PyTorch version")
        else:
            print("   ✗ MPS not available")
            print("   Reason: Not running on Apple Silicon Mac")
    else:
        print("   ✗ MPS not supported in this PyTorch version")
    
    # Fallback to CPU
    print("\n3. CPU:")
    print("   ✓ CPU is always available")
    import os
    print(f"   CPU Count: {os.cpu_count()}")
    print("\n   → Recommended device: cpu")
    print("   ⚠️  Warning: CPU will be slower than GPU for inference")
    
    return False


def check_ultralytics():
    """Check Ultralytics YOLO GPU support"""
    print_header("Ultralytics YOLO GPU Support")
    
    try:
        from ultralytics import YOLO
        import ultralytics
        print(f"✓ Ultralytics installed: {ultralytics.__version__}")
    except ImportError:
        print("✗ Ultralytics not installed")
        print("  Install with: pip install ultralytics")
        return False
    
    # Test YOLO with device detection
    try:
        print("\nTesting YOLO model loading...")
        
        # This will use a small model for testing
        model = YOLO('yolov8n.pt')
        
        # Check what device YOLO is using
        import torch
        if torch.cuda.is_available():
            print("✓ YOLO will use CUDA GPU by default")
        elif hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
            print("✓ YOLO will use MPS (Apple GPU) by default")
        else:
            print("✓ YOLO will use CPU by default")
        
        return True
    except Exception as e:
        print(f"✗ Error loading YOLO: {e}")
        return False


def check_sahi():
    """Check SAHI installation"""
    print_header("SAHI Installation")
    
    try:
        import sahi
        print(f"✓ SAHI installed: {sahi.__version__}")
        return True
    except ImportError:
        print("✗ SAHI not installed")
        print("  Install with: pip install sahi")
        return False


def check_opencv():
    """Check OpenCV installation"""
    print_header("OpenCV Installation")
    
    try:
        import cv2
        print(f"✓ OpenCV installed: {cv2.__version__}")
        
        # Check if CUDA support is built in OpenCV
        if cv2.cuda.getCudaEnabledDeviceCount() > 0:
            print(f"✓ OpenCV built with CUDA support")
            print(f"  CUDA Devices: {cv2.cuda.getCudaEnabledDeviceCount()}")
        else:
            print("  OpenCV not built with CUDA (not required for SAHI)")
        
        return True
    except ImportError:
        print("✗ OpenCV not installed")
        print("  Install with: pip install opencv-python")
        return False
    except:
        # cv2.cuda might not exist
        print("  OpenCV doesn't have CUDA module (not required)")
        return True


def print_system_info():
    """Print system information"""
    print_header("System Information")
    
    import platform
    import os
    
    print(f"Operating System: {platform.system()} {platform.release()}")
    print(f"Platform: {platform.platform()}")
    print(f"Architecture: {platform.machine()}")
    print(f"Processor: {platform.processor()}")
    print(f"Python Version: {platform.python_version()}")
    print(f"CPU Count: {os.cpu_count()}")


def print_recommendations():
    """Print recommendations based on detected hardware"""
    print_header("Recommendations for SAHI Object Detection")
    
    try:
        import torch
        
        if torch.cuda.is_available():
            print("\n✓ NVIDIA GPU Detected")
            print("\nRecommended settings:")
            print("  Device: cuda:0 (auto-detected)")
            print("  Model: yolo11s.pt or yolo11m.pt for better accuracy")
            print("  Slice size: 512x512 (good balance)")
            print("  Overlap: 0.3 (30%)")
            print("\nCommand:")
            print("  ros2 run video_cam object_detection_sahi")
            print("\nExpected performance:")
            print("  Processing time: ~0.3-0.5s per image")
            print("  Memory usage: ~2-4GB GPU RAM")
            
        elif hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
            print("\n✓ Apple Silicon GPU Detected")
            print("\nRecommended settings:")
            print("  Device: mps (auto-detected)")
            print("  Model: yolo11s.pt (recommended for MPS)")
            print("  Slice size: 512x512")
            print("  Overlap: 0.3 (30%)")
            print("\nCommand:")
            print("  ros2 run video_cam object_detection_sahi")
            print("\nExpected performance:")
            print("  Processing time: ~0.4-0.7s per image")
            print("  Memory usage: ~2-3GB unified memory")
            
        else:
            print("\n⚠️  No GPU Detected - Using CPU")
            print("\nRecommended settings:")
            print("  Device: cpu (auto-detected)")
            print("  Model: yolo11n.pt (nano - fastest on CPU)")
            print("  Slice size: 640x640 (larger slices for speed)")
            print("  Overlap: 0.2 (20%)")
            print("\nCommand:")
            print("  ros2 launch video_cam sahi_detection.launch.py \\")
            print("    model_path:=yolo11n.pt \\")
            print("    slice_height:=640 \\")
            print("    slice_width:=640 \\")
            print("    overlap_height_ratio:=0.2")
            print("\nExpected performance:")
            print("  Processing time: ~2-5s per image (slower)")
            print("\nConsider using a GPU for better performance!")
            
    except ImportError:
        print("Cannot determine hardware - PyTorch not installed")


def main():
    """Main function"""
    print("\n" + "="*80)
    print("  GPU DETECTION TEST FOR SAHI OBJECT DETECTION")
    print("="*80)
    
    # System info
    print_system_info()
    
    # Check dependencies
    has_pytorch = check_pytorch()
    has_ultralytics = check_ultralytics()
    has_sahi = check_sahi()
    has_opencv = check_opencv()
    
    # Print recommendations
    if has_pytorch and has_ultralytics and has_sahi and has_opencv:
        print_recommendations()
        
        print_header("Status: Ready to Use!")
        print("\n✓ All dependencies installed")
        print("✓ GPU detection complete")
        print("\nYou can now run:")
        print("  ros2 run video_cam object_detection_sahi")
    else:
        print_header("Status: Missing Dependencies")
        print("\n⚠️  Some dependencies are missing")
        print("\nInstall missing dependencies:")
        print("  cd /ODLC_Machine_Inferencing_System_2025-2026/ros2_ws")
        print("  pip install -r src/video_cam/requirements_sahi.txt")
    
    print("\n" + "="*80 + "\n")


if __name__ == "__main__":
    main()

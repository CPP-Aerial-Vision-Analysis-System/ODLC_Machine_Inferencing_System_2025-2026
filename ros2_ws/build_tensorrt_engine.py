#!/usr/bin/env python3
"""
Pre-build TensorRT Engine for YOLO Models
Run this script to convert .pt models to .engine files before starting the detection node.
This avoids long conversion times during node startup.

Usage:
    python3 build_tensorrt_engine.py [model_path] [--imgsz 640] [--workspace 4]
    
Example:
    python3 build_tensorrt_engine.py yolo26m.pt --imgsz 640 --workspace 4
"""

import os
import sys
import argparse
import time

# Fix protobuf compatibility
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

try:
    from ultralytics import YOLO
    import torch
except ImportError as e:
    print(f"❌ Error: Missing dependencies - {e}")
    print("Install with: pip install ultralytics torch")
    sys.exit(1)


def build_tensorrt_engine(model_path, imgsz=640, workspace=4, half=True, device='auto'):
    """
    Convert PyTorch YOLO model to TensorRT engine.
    
    Args:
        model_path: Path to .pt model file
        imgsz: Input image size (default: 640)
        workspace: GPU memory workspace in GB (default: 4)
        half: Use FP16 precision (default: True)
        device: Device to use ('auto', 'cuda:0', 'cpu')
    """
    print(f"\n{'='*60}")
    print(f"  TensorRT Engine Builder for YOLO")
    print(f"{'='*60}\n")
    
    # Validate model path
    if not os.path.exists(model_path):
        print(f"❌ Error: Model file not found: {model_path}")
        return False
    
    # Check CUDA availability
    if device == 'auto':
        device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
    
    cuda_available = torch.cuda.is_available()
    print(f"📊 System Info:")
    print(f"   CUDA Available: {cuda_available}")
    if cuda_available:
        print(f"   GPU: {torch.cuda.get_device_name(0)}")
        print(f"   CUDA Version: {torch.version.cuda}")
        total_mem = torch.cuda.get_device_properties(0).total_memory / (1024**3)
        print(f"   GPU Memory: {total_mem:.1f}GB")
    print(f"   Device: {device}")
    print()
    
    # Model info
    model_size = os.path.getsize(model_path) / (1024**2)
    engine_path = model_path.replace('.pt', '.engine')
    
    print(f"📦 Model Info:")
    print(f"   Input Model: {model_path} ({model_size:.1f}MB)")
    print(f"   Output Engine: {engine_path}")
    print(f"   Image Size: {imgsz}x{imgsz}")
    print(f"   Precision: {'FP16' if half else 'FP32'}")
    print(f"   Workspace: {workspace}GB")
    print()
    
    # Check if engine already exists
    if os.path.exists(engine_path):
        print(f"⚠️  Warning: Engine file already exists: {engine_path}")
        response = input("   Overwrite? (y/N): ")
        if response.lower() != 'y':
            print("❌ Cancelled")
            return False
        os.remove(engine_path)
        print("   Removed existing engine")
    
    try:
        # Clear GPU cache
        if cuda_available:
            torch.cuda.empty_cache()
            torch.cuda.synchronize()
            print("🧹 GPU cache cleared")
        
        print(f"\n🔨 Starting TensorRT conversion...")
        print(f"   This may take 5-15 minutes depending on your hardware...")
        print(f"   (Progress will be shown below)\n")
        
        start_time = time.time()
        
        # Load model
        print(f"📥 Loading model: {model_path}")
        model = YOLO(model_path)
        
        # Export to TensorRT
        print(f"🚀 Exporting to TensorRT...")
        model.export(
            format='engine',
            device=device if device.startswith('cuda') else 'cpu',
            imgsz=imgsz,
            half=half and cuda_available,
            workspace=workspace,
            simplify=True,
            verbose=True
        )
        
        elapsed = time.time() - start_time
        
        # Verify engine was created
        if os.path.exists(engine_path):
            engine_size = os.path.getsize(engine_path) / (1024**2)
            print(f"\n{'='*60}")
            print(f"✅ SUCCESS!")
            print(f"{'='*60}")
            print(f"   Engine: {engine_path}")
            print(f"   Size: {engine_size:.1f}MB")
            print(f"   Build Time: {elapsed:.1f}s ({elapsed/60:.1f} minutes)")
            print(f"\nYou can now use this engine with the detection node:")
            print(f"   ros2 run detection new_od")
            print()
            return True
        else:
            print(f"\n❌ Error: Engine file was not created")
            return False
            
    except KeyboardInterrupt:
        print(f"\n\n⚠️  Build interrupted by user")
        # Clean up partial engine
        if os.path.exists(engine_path):
            os.remove(engine_path)
            print(f"   Cleaned up partial engine file")
        return False
        
    except Exception as e:
        print(f"\n❌ Error during conversion:")
        print(f"   {str(e)}")
        import traceback
        print(f"\nFull traceback:")
        traceback.print_exc()
        return False


def main():
    parser = argparse.ArgumentParser(
        description='Build TensorRT engine from YOLO PyTorch model',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Build engine for yolo26m.pt with default settings
  python3 build_tensorrt_engine.py yolo26m.pt
  
  # Build with custom image size and workspace
  python3 build_tensorrt_engine.py yolo26x.pt --imgsz 640 --workspace 6
  
  # Build for CPU (no CUDA)
  python3 build_tensorrt_engine.py yolo26n.pt --device cpu --no-half
        """
    )
    
    parser.add_argument('model', help='Path to .pt model file')
    parser.add_argument('--imgsz', type=int, default=640, 
                        help='Input image size (default: 640)')
    parser.add_argument('--workspace', type=int, default=4,
                        help='GPU memory workspace in GB (default: 4)')
    parser.add_argument('--no-half', action='store_true',
                        help='Use FP32 instead of FP16 (slower but more accurate)')
    parser.add_argument('--device', default='auto',
                        help='Device to use: auto, cuda:0, or cpu (default: auto)')
    
    args = parser.parse_args()
    
    success = build_tensorrt_engine(
        args.model,
        imgsz=args.imgsz,
        workspace=args.workspace,
        half=not args.no_half,
        device=args.device
    )
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()

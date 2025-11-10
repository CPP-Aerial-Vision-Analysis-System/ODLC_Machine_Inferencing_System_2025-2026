#!/bin/bash

# GPU Memory Recovery Script for Jetson Orin Nano
# Fixes "NVML_SUCCESS == r INTERNAL ASSERT FAILED" errors
# Run this script before starting object detection if you encounter memory errors

set -e

echo "================================================================================"
echo "GPU MEMORY RECOVERY SCRIPT FOR JETSON ORIN NANO"
echo "================================================================================"
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored messages
print_info() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

# Step 1: Kill all running processes that might be using GPU
echo "Step 1: Stopping all GPU-using processes..."
echo "----------------------------------------"

# Kill Python processes
if pgrep -f "python.*video_cam" > /dev/null; then
    print_info "Stopping video_cam Python processes..."
    pkill -9 -f "python.*video_cam" || true
    sleep 1
fi

if pgrep -f "ros2 run" > /dev/null; then
    print_info "Stopping ROS2 processes..."
    pkill -9 -f "ros2 run" || true
    sleep 1
fi

if pgrep -f "object_detection" > /dev/null; then
    print_info "Stopping object detection processes..."
    pkill -9 -f "object_detection" || true
    sleep 1
fi

if pgrep -f "sahi" > /dev/null; then
    print_info "Stopping SAHI processes..."
    pkill -9 -f "sahi" || true
    sleep 1
fi

# Kill any Python process using significant GPU memory
if pgrep -f "python" > /dev/null; then
    print_warning "Stopping ALL Python processes (they may be holding GPU memory)..."
    pkill -9 python3 || true
    sleep 2
fi

print_info "All GPU-using processes stopped"
echo ""

# Step 2: Clear PyTorch GPU cache
echo "Step 2: Clearing PyTorch GPU cache..."
echo "----------------------------------------"

python3 << 'EOF'
import sys
try:
    import torch
    if torch.cuda.is_available():
        # Empty cache
        torch.cuda.empty_cache()
        
        # Synchronize to ensure all operations complete
        torch.cuda.synchronize()
        
        # Reset peak memory stats
        torch.cuda.reset_peak_memory_stats()
        torch.cuda.reset_accumulated_memory_stats()
        
        # Get memory info
        allocated = torch.cuda.memory_allocated(0) / 1024**3
        reserved = torch.cuda.memory_reserved(0) / 1024**3
        total = torch.cuda.get_device_properties(0).total_memory / 1024**3
        
        print(f"✓ PyTorch GPU cache cleared")
        print(f"  Allocated: {allocated:.2f} GB")
        print(f"  Reserved: {reserved:.2f} GB")
        print(f"  Total: {total:.2f} GB")
        print(f"  Free: {total - allocated:.2f} GB")
    else:
        print("⚠ CUDA not available in PyTorch")
        sys.exit(1)
except ImportError:
    print("⚠ PyTorch not installed")
    sys.exit(1)
except Exception as e:
    print(f"✗ Error: {e}")
    sys.exit(1)
EOF

if [ $? -eq 0 ]; then
    print_info "PyTorch cache cleared successfully"
else
    print_error "Failed to clear PyTorch cache"
fi
echo ""

# Step 3: Reset CUDA contexts (Jetson-specific)
echo "Step 3: Resetting CUDA contexts..."
echo "----------------------------------------"

# Check if nvidia-smi is available
if command -v nvidia-smi &> /dev/null; then
    print_info "Resetting GPU using nvidia-smi..."
    sudo nvidia-smi --gpu-reset || print_warning "GPU reset failed (may not be supported)"
fi

print_info "CUDA context reset attempted"
echo ""

# Step 4: Check system memory
echo "Step 4: Checking system memory..."
echo "----------------------------------------"

# Get memory info
total_mem=$(free -g | awk '/^Mem:/ {print $2}')
used_mem=$(free -g | awk '/^Mem:/ {print $3}')
free_mem=$(free -g | awk '/^Mem:/ {print $4}')

print_info "System RAM: ${free_mem}GB free / ${total_mem}GB total"

if [ "$free_mem" -lt 2 ]; then
    print_warning "Low system RAM! Consider closing other applications"
fi
echo ""

# Step 5: Check GPU status
echo "Step 5: Checking GPU status..."
echo "----------------------------------------"

if command -v tegrastats &> /dev/null; then
    print_info "Jetson GPU status:"
    timeout 3 tegrastats --interval 1000 || true
else
    if command -v nvidia-smi &> /dev/null; then
        nvidia-smi --query-gpu=name,memory.total,memory.used,memory.free --format=csv,noheader,nounits | while IFS=, read -r name total used free; do
            print_info "GPU: $name"
            print_info "  Total: ${total}MB"
            print_info "  Used: ${used}MB"
            print_info "  Free: ${free}MB"
        done
    fi
fi
echo ""

# Step 6: Recommendations
echo "Step 6: Optimization Recommendations"
echo "----------------------------------------"

print_info "To prevent future memory errors:"
echo ""
echo "1. Use larger slice sizes (less memory overhead):"
echo "   ros2 run video_cam object_detection_sahi --ros-args \\"
echo "     -p slice_height:=640 \\"
echo "     -p slice_width:=640 \\"
echo "     -p overlap_height_ratio:=0.2 \\"
echo "     -p overlap_width_ratio:=0.2"
echo ""
echo "2. Process fewer images at once by clearing camera_feed folder"
echo ""
echo "3. Use a smaller model (yolo11n.pt instead of yolo11s.pt):"
echo "   ros2 run video_cam object_detection_sahi --ros-args \\"
echo "     -p model_path:=yolo11n.pt"
echo ""
echo "4. As last resort, use CPU mode (slower but no GPU memory issues):"
echo "   ros2 run video_cam object_detection_sahi --ros-args \\"
echo "     -p device:=cpu"
echo ""

echo "================================================================================"
echo "GPU MEMORY RECOVERY COMPLETE"
echo "================================================================================"
echo ""
print_info "You can now start your object detection node:"
echo ""
echo "source ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/install/setup.bash"
echo "ros2 run video_cam object_detection_sahi --ros-args \\"
echo "  -p slice_height:=640 \\"
echo "  -p slice_width:=640 \\"
echo "  -p overlap_height_ratio:=0.2 \\"
echo "  -p overlap_width_ratio:=0.2"
echo ""

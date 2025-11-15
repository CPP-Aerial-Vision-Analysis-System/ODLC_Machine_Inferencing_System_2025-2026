#!/bin/bash

################################################################################
# GPU Memory Cleanup Script for NVIDIA Jetson
# Frees up GPU memory and system resources
################################################################################

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }
print_header() { echo -e "\n========================================"; echo "  $1"; echo "========================================\n"; }

print_header "GPU Memory Cleanup for Jetson"

# Check if on Jetson
if [ ! -f /proc/device-tree/model ]; then
    print_error "This script is designed for NVIDIA Jetson devices"
    exit 1
fi

MODEL=$(cat /proc/device-tree/model | tr -d '\0')
print_info "Device: $MODEL"

# Show current memory status
print_header "Current Memory Status"
echo "GPU Memory:"
python3 << 'EOF'
try:
    import torch
    if torch.cuda.is_available():
        for i in range(torch.cuda.device_count()):
            allocated = torch.cuda.memory_allocated(i) / 1024**3
            reserved = torch.cuda.memory_reserved(i) / 1024**3
            total = torch.cuda.get_device_properties(i).total_memory / 1024**3
            free = total - allocated
            print(f"  GPU {i}: {allocated:.2f}GB allocated, {reserved:.2f}GB reserved, {free:.2f}GB free / {total:.2f}GB total")
    else:
        print("  CUDA not available")
except ImportError:
    print("  PyTorch not installed")
EOF

echo ""
echo "System Memory:"
free -h | grep -E "Mem:|Swap:"

echo ""
echo "GPU Processes:"
if command -v fuser &> /dev/null; then
    sudo fuser -v /dev/nvhost-* 2>&1 | grep -v "Cannot stat" || echo "  No processes using GPU"
else
    print_warning "fuser not installed, skipping process check"
fi

# Kill GPU-using processes (optional)
echo ""
echo -n "Kill all processes using GPU? (y/N): "
read -r response

if [[ $response =~ ^[Yy]$ ]]; then
    print_info "Killing GPU processes..."
    
    # Kill known GPU-heavy processes
    pkill -9 python3 2>/dev/null || true
    pkill -9 ros2 2>/dev/null || true
    
    # Kill processes using nvhost devices
    if command -v fuser &> /dev/null; then
        sudo fuser -k /dev/nvhost-* 2>/dev/null || true
    fi
    
    sleep 2
    print_success "Processes killed"
else
    print_info "Skipping process kill"
fi

# Clear PyTorch cache
print_header "Clearing PyTorch GPU Cache"
python3 << 'EOF'
try:
    import torch
    if torch.cuda.is_available():
        print("Clearing CUDA cache...")
        torch.cuda.empty_cache()
        torch.cuda.synchronize()
        print("✓ CUDA cache cleared")
        
        # Show memory after clearing
        allocated = torch.cuda.memory_allocated(0) / 1024**3
        reserved = torch.cuda.memory_reserved(0) / 1024**3
        total = torch.cuda.get_device_properties(0).total_memory / 1024**3
        free = total - allocated
        print(f"After clearing: {free:.2f}GB free / {total:.2f}GB total")
    else:
        print("CUDA not available")
except Exception as e:
    print(f"Error: {e}")
EOF

# Clear system cache
print_header "Clearing System Cache"
print_info "Dropping system caches..."
sync
sudo sh -c 'echo 3 > /proc/sys/vm/drop_caches'
print_success "System cache cleared"

# Check/add swap if needed
print_header "Checking Swap Space"
SWAP_SIZE=$(free -m | grep Swap | awk '{print $2}')
print_info "Current swap: ${SWAP_SIZE}MB"

if [ "$SWAP_SIZE" -lt 4096 ]; then
    echo ""
    echo "Swap is less than 4GB. Recommended to add swap for GPU operations."
    echo -n "Add 4GB swap file? (y/N): "
    read -r response
    
    if [[ $response =~ ^[Yy]$ ]]; then
        print_info "Creating 4GB swap file (this may take a while)..."
        
        SWAPFILE="/swapfile"
        
        # Remove old swapfile if exists
        if [ -f "$SWAPFILE" ]; then
            sudo swapoff "$SWAPFILE" 2>/dev/null || true
            sudo rm -f "$SWAPFILE"
        fi
        
        # Create new swapfile
        sudo fallocate -l 4G "$SWAPFILE"
        sudo chmod 600 "$SWAPFILE"
        sudo mkswap "$SWAPFILE"
        sudo swapon "$SWAPFILE"
        
        # Make persistent
        if ! grep -q "$SWAPFILE" /etc/fstab; then
            echo "$SWAPFILE none swap sw 0 0" | sudo tee -a /etc/fstab
        fi
        
        print_success "4GB swap added"
    fi
fi

# Final memory status
print_header "Final Memory Status"
echo "GPU Memory:"
python3 << 'EOF'
try:
    import torch
    if torch.cuda.is_available():
        allocated = torch.cuda.memory_allocated(0) / 1024**3
        total = torch.cuda.get_device_properties(0).total_memory / 1024**3
        free = total - allocated
        print(f"  {free:.2f}GB free / {total:.2f}GB total")
    else:
        print("  CUDA not available")
except:
    print("  Cannot check")
EOF

echo ""
echo "System Memory:"
free -h | grep -E "Mem:|Swap:"

print_header "Cleanup Complete"
print_info "GPU memory has been freed up"
echo ""
echo "Tips to prevent OOM errors:"
echo "  1. Use smaller model: yolo11n.pt (fastest) or yolo11s.pt (balanced)"
echo "  2. Reduce slice size: --ros-args -p slice_height:=256 -p slice_width:=256"
echo "  3. Enable performance mode: sudo nvpmodel -m 0 && sudo jetson_clocks"
echo "  4. Reboot if issues persist: sudo reboot"
echo ""

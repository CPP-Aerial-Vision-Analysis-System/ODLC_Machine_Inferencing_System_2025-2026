#!/bin/bash
# Script to install PyTorch with CUDA support on NVIDIA Jetson
# For Jetson Orin Nano with JetPack/CUDA 12.6

echo "============================================================================"
echo "PyTorch CUDA Installation for Jetson Orin Nano"
echo "============================================================================"
echo ""

# Check if we're on a Jetson device
if [ ! -f /proc/device-tree/model ]; then
    echo "ERROR: This doesn't appear to be a Jetson device"
    exit 1
fi

MODEL=$(cat /proc/device-tree/model 2>/dev/null)
echo "Device: $MODEL"
echo ""

# Check CUDA version
if command -v nvidia-smi &> /dev/null; then
    echo "CUDA Status:"
    nvidia-smi --query-gpu=driver_version,cuda_version --format=csv,noheader
    echo ""
else
    echo "ERROR: nvidia-smi not found. Please install NVIDIA drivers first."
    exit 1
fi

# Check current PyTorch
echo "Current PyTorch installation:"
python3 -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA Available: {torch.cuda.is_available()}')" 2>/dev/null || echo "PyTorch not installed or import failed"
echo ""

echo "============================================================================"
echo "INSTALLATION OPTIONS"
echo "============================================================================"
echo ""
echo "For Jetson devices, you have two options:"
echo ""
echo "OPTION 1: Use NVIDIA's pre-built PyTorch wheel (RECOMMENDED)"
echo "   - Download from: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048"
echo "   - Find the wheel matching your JetPack version"
echo "   - Install with: pip3 install <downloaded_wheel>.whl"
echo ""
echo "OPTION 2: Use PyTorch from NVIDIA NGC container"
echo "   - Pull container: docker pull nvcr.io/nvidia/l4t-pytorch:r36.4.0-pth2.5-py3"
echo ""
echo "OPTION 3: Build PyTorch from source (takes several hours)"
echo ""
echo "============================================================================"
echo ""

# Try to detect JetPack version
echo "Detecting JetPack version..."
if command -v dpkg &> /dev/null; then
    JETPACK_VERSION=$(dpkg -l | grep nvidia-jetpack | awk '{print $3}' | head -1)
    if [ ! -z "$JETPACK_VERSION" ]; then
        echo "JetPack version: $JETPACK_VERSION"
    fi
fi
echo ""

# Provide download instructions
echo "============================================================================"
echo "RECOMMENDED: Install pre-built PyTorch wheel"
echo "============================================================================"
echo ""
echo "Step 1: Visit NVIDIA PyTorch for Jetson page:"
echo "   https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048"
echo ""
echo "Step 2: Download the appropriate wheel for your system"
echo "   For JetPack 6.x (CUDA 12.x):"
echo "   Look for: torch-*.whl for JetPack 6.x"
echo ""
echo "Step 3: Install dependencies:"
echo "   sudo apt-get install -y libopenblas-base libopenmpi-dev"
echo ""
echo "Step 4: Install the wheel:"
echo "   pip3 install <path_to_downloaded_wheel>.whl"
echo ""
echo "Step 5: Verify installation:"
echo "   python3 -c 'import torch; print(torch.__version__); print(torch.cuda.is_available())'"
echo ""
echo "============================================================================"
echo ""

# Ask if user wants to try automatic download
read -p "Would you like to try automatic download and installation? (y/n) " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "Attempting to download PyTorch wheel for Jetson..."
    echo ""
    
    # Create temp directory
    TEMP_DIR=$(mktemp -d)
    cd "$TEMP_DIR"
    
    # Try to download a recent wheel (this URL may need updating)
    # Note: This is an example - actual URL needs to be current
    WHEEL_URL="https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0-cp310-cp310-linux_aarch64.whl"
    
    echo "Downloading from: $WHEEL_URL"
    echo "(Note: If this fails, manually download from NVIDIA forum)"
    echo ""
    
    wget "$WHEEL_URL" -O pytorch_jetson.whl
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "Download successful! Installing..."
        
        # Install dependencies
        sudo apt-get update
        sudo apt-get install -y libopenblas-base libopenmpi-dev libjpeg-dev zlib1g-dev
        
        # Uninstall CPU version
        pip3 uninstall -y torch torchvision
        
        # Install CUDA version
        pip3 install pytorch_jetson.whl
        
        echo ""
        echo "Installation complete! Verifying..."
        python3 -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA Available: {torch.cuda.is_available()}'); print(f'CUDA Device: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else \"N/A\"}')"
        
    else
        echo ""
        echo "Automatic download failed."
        echo "Please manually download the wheel from:"
        echo "https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048"
    fi
    
    # Cleanup
    cd -
    rm -rf "$TEMP_DIR"
else
    echo ""
    echo "Skipping automatic installation."
    echo "Please follow the manual steps above."
fi

echo ""
echo "============================================================================"
echo "For more information, visit:"
echo "https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048"
echo "============================================================================"

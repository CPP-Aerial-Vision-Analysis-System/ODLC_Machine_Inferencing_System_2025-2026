# this is a janky way of doing things but it works. there might be issues in the forseeable future
# but for setup this is enough
FROM joestrada1022/suas-sim:ros2-gazebo 

# Install Python dependencies for video_cam package (SAHI, etc.)
# Install core dependencies directly in Dockerfile for better build caching
RUN pip3 install --no-cache-dir \
    torch>=2.0.0 \
    torchvision>=0.15.0 \
    sahi>=0.11.0 \
    opencv-python>=4.8.0 \
    opencv-contrib-python>=4.8.0 \
    Pillow>=10.0.0 \
    numpy>=1.24.0 \
    pandas>=2.0.0 \
    matplotlib>=3.7.0

# Verify critical installations
RUN python3 -c "import sahi; import torch; print('✓ Core dependencies (sahi, torch) installed successfully')" || \
    echo "⚠ Warning: Some dependencies may not be available - will install via requirements file on startup"


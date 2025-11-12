#!/bin/bash
set -e

OS=$(uname -s)
echo "Detected OS: $OS"

case "$OS" in
  Linux*)
    echo " Linux detected – enabling host networking & display support"
    # For Linux we just print hints or set environment vars
    echo "DISPLAY=:0" >> ~/.bashrc
    echo "export DISPLAY=:0" >> ~/.bashrc
    ;;
  Darwin*)
    echo " macOS detected – skipping network/display changes"
    ;;
  MINGW*|CYGWIN*|MSYS*|Windows*)
    echo " Windows/WSL detected – using defaults"
    ;;
  *)
    echo "Unknown OS – using safe defaults"
    ;;
esac

# Verify Python dependencies are installed (should be installed in Dockerfile)
echo "Verifying Python dependencies..."
MISSING_DEPS=0

python3 -c "import ultralytics" 2>/dev/null && echo "✓ ultralytics installed" || { echo "✗ ultralytics not found"; MISSING_DEPS=1; }
python3 -c "import sahi" 2>/dev/null && echo "✓ sahi installed" || { echo "✗ sahi not found"; MISSING_DEPS=1; }
python3 -c "import torch" 2>/dev/null && echo "✓ torch installed" || { echo "✗ torch not found"; MISSING_DEPS=1; }

# If dependencies are missing, try to install from requirements file
if [ $MISSING_DEPS -eq 1 ]; then
    echo ""
    echo "⚠ Some dependencies are missing. Attempting to install from requirements file..."
    
    # Try common workspace locations
    for WORKSPACE_DIR in "/ODLC_Machine_Inferencing_System_2025-2026" "$HOME/ODLC_Machine_Inferencing_System_2025-2026" "$(pwd)"; do
        REQUIREMENTS_FILE="$WORKSPACE_DIR/ros2_ws/src/video_cam/requirements_sahi.txt"
        if [ -f "$REQUIREMENTS_FILE" ]; then
            echo "Found requirements file at: $REQUIREMENTS_FILE"
            pip3 install --no-cache-dir -r "$REQUIREMENTS_FILE" && echo "✓ Dependencies installed from requirements file" || echo "✗ Failed to install from requirements file"
            break
        fi
    done
    
    # Re-verify after installation attempt
    echo ""
    echo "Re-verifying dependencies..."
    python3 -c "import ultralytics" 2>/dev/null && echo "✓ ultralytics installed" || echo "✗ ultralytics still not found"
    python3 -c "import sahi" 2>/dev/null && echo "✓ sahi installed" || echo "✗ sahi still not found"
    python3 -c "import torch" 2>/dev/null && echo "✓ torch installed" || echo "✗ torch still not found"
else
    echo "✓ All dependencies verified successfully!"
fi


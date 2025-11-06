#!/bin/bash

# Quick start script for YOLO SAHI ASTRA

echo "🎯 Starting YOLO SAHI ASTRA - SAHI Mode (Small Object Detection)..."
echo ""

# Activate virtual environment
source venv/bin/activate

# Check GPU
echo "🖥️  Checking GPU availability..."
python -c "import torch; device = 'MPS (Apple Silicon GPU)' if hasattr(torch.backends, 'mps') and torch.backends.mps.is_available() else ('CUDA (NVIDIA GPU)' if torch.cuda.is_available() else 'CPU'); print(f'Using: {device}')"
echo ""

# Run the detection
echo "▶️  Starting SAHI detection for small objects..."
echo "📊 Expected FPS: 5-10 (this is normal for SAHI!)"
echo "🎯 Detection: Sliced inference with overlap"
echo "Press 'q' in the detection window to quit"
echo ""
python main.py

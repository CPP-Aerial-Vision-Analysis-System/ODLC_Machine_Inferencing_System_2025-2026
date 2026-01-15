#!/usr/bin/env python3
"""Download YOLO26m model"""
from ultralytics import YOLO

print("Downloading YOLO26m model...")
model = YOLO('yolo26m.pt')
print("YOLO26m model downloaded successfully!")
print(f"Model file should be in: {model.ckpt_path if hasattr(model, 'ckpt_path') else 'current directory'}")

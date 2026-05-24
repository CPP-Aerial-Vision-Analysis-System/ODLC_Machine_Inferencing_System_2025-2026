#!/usr/bin/env python3
from ultralytics import YOLO

print("Downloading YOLO26m model...")
model = YOLO('yolo26m.pt')
print(f"Model file should be in: {model.ckpt_path if hasattr(model, 'ckpt_path') else 'current directory'}")
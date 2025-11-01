# YOLOx SAHI ASTRA - Real-Time Object Detection

GPU-accelerated object detection using YOLOv8 and **SAHI (Slicing Aided Hyper Inference)** for small object detection.

## 🚀 Features
- ✅ **GPU Acceleration** (Apple Silicon MPS / NVIDIA CUDA)
- ✅ **SAHI Mode** - Excellent small object detection using sliced inference
- ✅ **Fast Mode** - High-speed detection for regular objects (40+ FPS)
- ✅ **Webcam & Video Support**
- ✅ **Live FPS & Detection Counter**

## ⚠️ **IMPORTANT: Which Mode to Use?**

### 🎯 **SAHI Mode** (`main.py`) - **USE THIS FOR SMALL OBJECTS** ✅
- **FPS**: 5-10 (this is normal!)
- **Small Object Detection**: Excellent ✅
- **How it works**: Slices each frame into 8 pieces, detects in each slice
- **Use when**: You need to detect small/tiny objects (YOUR REQUIREMENT)

### ⚡ **Fast Mode** (`main_fast.py`) - For Regular Objects Only
- **FPS**: 40-50
- **Small Object Detection**: Poor ❌
- **How it works**: Single-pass YOLO detection
- **Use when**: Speed matters more than detecting small objects

📖 **See [SAHI_vs_FAST.md](SAHI_vs_FAST.md) for detailed comparison**

## 📋 Setup

### 1. Activate Virtual Environment
```bash
source venv/bin/activate
```

### 2. Install Dependencies (if not already installed)
```bash
pip install -r requirements.txt
```

## 🎮 Usage

### **Option 1: SAHI MODE** 🎯 (For Small Object Detection - RECOMMENDED)
```bash
python main.py
```
- **FPS**: 5-10 (this is expected for SAHI!)
- **Best for**: Detecting small and tiny objects
- **Detection**: Sliced inference with 8 slices per frame + 20% overlap
- **Window**: Shows "SAHI Mode (Small Objects)" indicator
- **Input**: Live webcam (real-time)

### **Option 2: VIDEO FILE PROCESSING** 🎬 (Process Pre-recorded Videos)
```bash
python process_video.py
```
- **FPS**: 5-10 (processing speed)
- **Best for**: Processing video files with SAHI
- **Detection**: Same SAHI slicing for small objects
- **Input**: Video file from `input_video/` folder
- **Output**: Processed video saved to `output_video/` folder
- 📖 **See [VIDEO_PROCESSING_GUIDE.md](VIDEO_PROCESSING_GUIDE.md) for details**

### **Option 3: FAST MODE** ⚡ (For Large Objects Only)
```bash
python main_fast.py
```
- **FPS**: 40+ 
- **Best for**: Real-time detection of regular-sized objects
- **Detection**: Single-pass YOLO (NO slicing)
- **⚠️ Warning**: Will NOT detect small objects well!

### **Quick Start Script**
```bash
./run.sh
```

## ⚙️ Configuration

### Fast Mode (`main_fast.py`)
```python
detector.run(
    source=0,              # 0=webcam, "path/to/video.mp4"=video file
    weights="yolov8n.pt",  # yolov8n.pt (fastest), yolov8s.pt, yolov8m.pt
    device=None,           # None=auto-detect GPU
    view_img=True,         # Show live preview
    save_img=True,         # Save output video
    conf_threshold=0.25    # Confidence threshold
)
```

### SAHI Mode (`main.py`)
```python
detector.run(
    source=0,                    # 0=webcam, "path/to/video.mp4"=video file
    weights="yolov8n.pt",        # Model selection
    device=None,                 # None=auto-detect GPU
    view_img=True,               # Show live preview
    save_img=True,               # Save output video
    slice_size=(640, 640),       # Slice size (smaller=slower but better for tiny objects)
    output_video_name="output.mp4"
)
```

## 🖥️ GPU Status
Your system: **Apple Silicon with MPS (Metal Performance Shaders) ✅**

The code automatically detects and uses:
- **MPS** for Apple Silicon (M1/M2/M3) - **~40 FPS**
- **CUDA** for NVIDIA GPUs
- **CPU** as fallback

## 🎯 Controls
- Press **'q'** to quit during detection
- FPS is displayed in the top-left corner
- Output videos are saved in `output_video/`

## 📁 Project Structure
```
YOLOxSAHI_ASTRA/
├── main.py                 ⭐ SAHI mode for webcam (small objects)
├── main_fast.py            ⚡ Fast mode for webcam (regular objects)
├── process_video.py        🎬 Process video files with SAHI
├── run.sh                  🚀 Quick start script
├── requirements.txt        📦 Dependencies
├── README.md               📖 Main documentation
├── VIDEO_PROCESSING_GUIDE.md  📄 Video processing guide
├── SAHI_vs_FAST.md         📊 Mode comparison
├── HOW_SAHI_WORKS.md       🔍 SAHI explanation
├── SETUP_COMPLETE.md       ✅ Setup status
├── QUICKSTART.md           ⚡ Quick reference
├── input_video/            📁 Place input videos here
│   └── input_video.mp4
├── output_video/           💾 Saved output videos
├── models/                 🤖 YOLO weights (auto-downloaded)
└── venv/                   🐍 Python environment
```

## 🏆 Model Comparison

| Model | Speed | Accuracy | Best For |
|-------|-------|----------|----------|
| `yolov8n.pt` | ⚡⚡⚡ Fast (40+ FPS) | Good | Real-time webcam |
| `yolov8s.pt` | ⚡⚡ Medium (20+ FPS) | Better | Balanced |
| `yolov8m.pt` | ⚡ Slower (10+ FPS) | Best | High accuracy |

## 🔧 Troubleshooting

### Camera Not Working
- Make sure you've granted camera permissions to Terminal/VS Code
- Try different camera indices (0, 1, 2)
- Check if camera is being used by another application

### Window Not Showing
- Make sure `view_img=True` is set
- Check if you have display access (especially if using SSH)

### Low FPS
- **Use Fast Mode**: `python main_fast.py` instead of `main.py`
- **Use smaller model**: `weights="yolov8n.pt"` (nano model)
- **Reduce resolution**: Modify the code to resize frames

### Video File Detection
Edit the source in the script:
```python
source="input_video/input_video.mp4"  # instead of source=0
```

## 📊 Performance Tips
- **Real-time webcam**: Use `main_fast.py` with `yolov8n.pt`
- **Small objects in video**: Use `main.py` with larger slice_size
- **Speed priority**: Fast Mode + Nano model = 40+ FPS
- **Accuracy priority**: SAHI Mode + Medium model = better detection

## ✨ What's Running Now?
Your detection is running with:
- **Mode**: Fast Mode
- **Device**: MPS (Apple Silicon GPU)
- **FPS**: 40+ frames per second
- **Model**: YOLOv8-Nano
- **Source**: Webcam (camera 0)

Press **'q'** in the detection window to stop!

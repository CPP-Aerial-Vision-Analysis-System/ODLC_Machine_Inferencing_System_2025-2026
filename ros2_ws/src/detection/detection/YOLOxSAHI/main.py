import cv2
import time 
import torch
from pathlib import Path
from sahi import AutoDetectionModel
from sahi.predict import get_sliced_prediction

def get_device():
    """Auto-detect the best available device (CUDA, MPS, or CPU)"""
    if torch.cuda.is_available():
        return "cuda:0"
    elif hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
        return "mps"
    else:
        return "cpu"

def increment_path(path, exist_ok=False):
    """Increment file or directory path, i.e. output_video -> output_video2, output_video3, etc."""
    path = Path(path)
    if not path.exists() or exist_ok:
        path.mkdir(parents=True, exist_ok=True)
        return path
    
    i = 2
    while True:
        new_path = Path(f"{path}{i}")
        if not new_path.exists():
            new_path.mkdir(parents=True, exist_ok=True)
            return new_path
        i += 1

class SAHIInterference:
    def __init__(self):
        self.detection_model = None
    
    def load_model(self, weights, device):
        """Load YOLO model with automatic download if needed"""
        print(f"Loading SAHI model {weights} for small object detection...")
        
        # AutoDetectionModel will handle model download automatically
        self.detection_model = AutoDetectionModel.from_pretrained(
            model_type="yolov8",  # Use yolov8 as model_type for YOLO models
            model_path=weights,
            device=device,
            confidence_threshold=0.15,  # Lower threshold to catch more small objects
            image_size=640,  # Standard detection size
        )
        print(f"SAHI model loaded successfully on {device}!")
    
    def run(self, source, weights, device=None, view_img=True, save_img=True,
            hide_conf=True, slice_height=640, slice_width=640, overlap_height_ratio=0.2,
            overlap_width_ratio=0.2, output_video_name="output.mp4"):
        # Auto-detect device if not specified
        if device is None:
            device = get_device()
            print(f"Using device: {device}")
        
        # Support webcam (0, 1, etc.) or video file path
        if isinstance(source, int) or source.isdigit():
            cap = cv2.VideoCapture(int(source))
        else:
            cap = cv2.VideoCapture(source)
        
        assert cap.isOpened(), f"Failed to open video: {source}"
        save_dir = increment_path("output_video", exist_ok=True)
        save_dir.mkdir(parents=True, exist_ok=True)

        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps_input = int(cap.get(cv2.CAP_PROP_FPS))

        #DEFINE OUTPUT WRITER
        video_writer = None
        if save_img:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            video_writer = cv2.VideoWriter(str(save_dir/output_video_name), fourcc, fps_input, (width, height))
        self.load_model(weights, device)

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            start_time = time.time()

            # SAHI Sliced Prediction for Small Object Detection
            results = get_sliced_prediction(
                image=frame[..., ::-1],
                detection_model=self.detection_model,
                slice_height=slice_height,
                slice_width=slice_width,
                overlap_height_ratio=overlap_height_ratio,
                overlap_width_ratio=overlap_width_ratio,
                postprocess_type="NMS",  # Non-Maximum Suppression
                postprocess_match_metric="IOS",  # Intersection over smaller area
                postprocess_match_threshold=0.5,  # Threshold for merging
                postprocess_class_agnostic=False,  # Class-aware NMS
            )

            #DRAW BOUNDING BOXES with confidence scores
            num_detections = len(results.object_prediction_list)
            for pred in results.object_prediction_list:
                x1,y1,x2,y2 = map(int , [pred.bbox.minx, pred.bbox.miny, pred.bbox.maxx, pred.bbox.maxy])
                class_name = pred.category.name
                confidence = pred.score.value
                
                # Draw rectangle
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Put class name and confidence
                label = f"{class_name} {confidence:.2f}"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36,255,12), 2)
            
            #Calculate FPS and display info
            fps = 1.0 / (time.time() - start_time)
            
            # Display FPS in top-left
            cv2.putText(frame, f"FPS: {fps:.1f}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            
            # Display detection count
            cv2.putText(frame, f"Objects: {num_detections}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            # Display SAHI mode indicator
            cv2.putText(frame, "SAHI Mode (Small Objects)", (20, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            #Display Output video Frame
            if view_img:
                cv2.imshow("ASTRA SAHI Detection - Small Objects", frame)
                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #     break
            #Save video frame
            if save_img:
                video_writer.write(frame)
            
            #Exit on persising 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        if video_writer:
            video_writer.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = SAHIInterference()
    
    # IMPROVED SAHI Mode for Better Detection:
    # - Smaller slices (512x512) = more slices = better small object detection
    # - Higher overlap (30%) = fewer missed objects on slice borders
    # - Better model (yolov8m) = higher accuracy
    # - Lower confidence (0.15) = catches more potential objects
    
    detector.run(
        source=0,  # 0 for webcam, or "input_video/my_video.mp4" for video file
        weights="yolov8m.pt",  # UPGRADED: yolov8m for better accuracy
        device=None,  # Will auto-detect GPU (CUDA/MPS) or fallback to CPU
        view_img=True,
        save_img=True,
        hide_conf=True,
        slice_height=512,  # SMALLER slices = better detection
        slice_width=512,   # SMALLER slices = better detection
        overlap_height_ratio=0.3,  # INCREASED: 30% overlap for better border detection
        overlap_width_ratio=0.3,   # INCREASED: 30% overlap for better border detection
        output_video_name="output_sahi.mp4"
    )
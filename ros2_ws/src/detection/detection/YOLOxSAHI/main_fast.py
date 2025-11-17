import cv2
import time
import torch
from ultralytics import YOLO
from pathlib import Path

def get_device():
    """Auto-detect the best available device (CUDA, MPS, or CPU)"""
    if torch.cuda.is_available():
        return "cuda:0"
    elif hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
        return "mps"
    else:
        return "cpu"

class FastYOLODetection:
    def __init__(self):
        self.model = None
    
    def run(self, source=0, weights="yolov8n.pt", device=None, view_img=True, 
            save_img=True, conf_threshold=0.25):
        # Auto-detect device if not specified
        if device is None:
            device = get_device()
            print(f"Using device: {device}")
        
        # Load YOLO model
        print(f"Loading model {weights}...")
        self.model = YOLO(weights)
        self.model.to(device)
        print(f"Model loaded successfully on {device}!")
        
        # Open video source
        if isinstance(source, int) or (isinstance(source, str) and source.isdigit()):
            cap = cv2.VideoCapture(int(source))
        else:
            cap = cv2.VideoCapture(source)
        
        assert cap.isOpened(), f"Failed to open video: {source}"
        
        # Get video properties
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps_input = int(cap.get(cv2.CAP_PROP_FPS))
        
        # Setup video writer
        video_writer = None
        if save_img:
            output_dir = Path("output_video")
            output_dir.mkdir(parents=True, exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            video_writer = cv2.VideoWriter(
                str(output_dir / "output.mp4"), 
                fourcc, 
                fps_input if fps_input > 0 else 30, 
                (width, height)
            )
        
        print("Starting detection... Press 'q' to quit")
        frame_count = 0
        
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            
            start_time = time.time()
            
            # Run detection
            results = self.model(frame, conf=conf_threshold, device=device, verbose=False)
            
            # Draw results on frame
            annotated_frame = results[0].plot()
            
            # Calculate FPS
            fps = 1.0 / (time.time() - start_time)
            
            # Add FPS text
            cv2.putText(
                annotated_frame, 
                f"FPS: {fps:.1f}", 
                (20, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                1, 
                (0, 255, 0), 
                2
            )
            
            # Display frame
            if view_img:
                cv2.imshow("ASTRA ObjectDetection - Fast Mode", annotated_frame)
            
            # Save frame
            if save_img and video_writer:
                video_writer.write(annotated_frame)
            
            # Exit on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            frame_count += 1
            if frame_count % 30 == 0:
                print(f"Processed {frame_count} frames - Current FPS: {fps:.1f}")
        
        # Cleanup
        cap.release()
        if video_writer:
            video_writer.release()
        cv2.destroyAllWindows()
        print(f"\nDone! Processed {frame_count} frames")

if __name__ == "__main__":
    detector = FastYOLODetection()
    
    detector.run(
        source=0,              # 0 for webcam, or path to video file
        weights="yolov8n.pt",  # yolov8n.pt (fastest), yolov8s.pt, yolov8m.pt
        device=None,           # Auto-detect GPU
        view_img=True,         # Show window
        save_img=True,         # Save output video
        conf_threshold=0.25    # Confidence threshold
    )

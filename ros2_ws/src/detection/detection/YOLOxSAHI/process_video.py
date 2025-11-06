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

class SAHIVideoProcessor:
    def __init__(self):
        self.detection_model = None
    
    def load_model(self, weights, device):
        """Load YOLO model with SAHI for small object detection"""
        print(f"Loading SAHI model {weights} for small object detection...")
        
        self.detection_model = AutoDetectionModel.from_pretrained(
            model_type="yolov8",
            model_path=weights,
            device=device,
            confidence_threshold=0.15,  # Lower threshold to catch more objects
            image_size=640,  # Standard detection size
        )
        print(f"SAHI model loaded successfully on {device}!")
    
    def process_video(self, input_video, output_dir="output_video", weights="yolov8s.pt", 
                     device=None, slice_height=640, slice_width=640, 
                     overlap_height_ratio=0.2, overlap_width_ratio=0.2,
                     show_preview=False):
        """
        Process a video file with SAHI + YOLO for small object detection
        
        Args:
            input_video: Path to input video file
            output_dir: Directory to save output video
            weights: YOLO model weights
            device: Device to run on (None = auto-detect)
            slice_height: Height of each slice for SAHI
            slice_width: Width of each slice for SAHI
            overlap_height_ratio: Overlap ratio for height
            overlap_width_ratio: Overlap ratio for width
            show_preview: Show processing preview window
        """
        
        # Auto-detect device if not specified
        if device is None:
            device = get_device()
            print(f"Using device: {device}")
        
        # Validate input video
        input_path = Path(input_video)
        if not input_path.exists():
            raise FileNotFoundError(f"Input video not found: {input_video}")
        
        print(f"\n{'='*60}")
        print(f"Processing video: {input_path.name}")
        print(f"{'='*60}\n")
        
        # Open video
        cap = cv2.VideoCapture(str(input_path))
        if not cap.isOpened():
            raise ValueError(f"Failed to open video: {input_video}")
        
        # Get video properties
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps_input = int(cap.get(cv2.CAP_PROP_FPS))
        
        print(f"Video Info:")
        print(f"  Resolution: {width}x{height}")
        print(f"  FPS: {fps_input}")
        print(f"  Total Frames: {total_frames}")
        print(f"  Duration: {total_frames/fps_input:.2f} seconds\n")
        
        # Create output directory
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        # Generate output filename
        output_filename = f"{input_path.stem}_sahi_detected{input_path.suffix}"
        output_video_path = output_path / output_filename
        
        print(f"Output will be saved to: {output_video_path}\n")
        
        # Setup video writer
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        video_writer = cv2.VideoWriter(
            str(output_video_path), 
            fourcc, 
            fps_input if fps_input > 0 else 30, 
            (width, height)
        )
        
        # Load model
        self.load_model(weights, device)
        
        print(f"\nSAHI Configuration:")
        print(f"  Slice size: {slice_width}x{slice_height}")
        print(f"  Overlap: {overlap_width_ratio*100}% horizontal, {overlap_height_ratio*100}% vertical")
        print(f"  Model: {weights}")
        print(f"\nProcessing frames...\n")
        
        frame_count = 0
        start_time_total = time.time()
        total_detections = 0
        
        try:
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    break
                
                frame_start = time.time()
                
                # SAHI Sliced Prediction for Small Object Detection
                results = get_sliced_prediction(
                    image=frame[..., ::-1],  # BGR to RGB
                    detection_model=self.detection_model,
                    slice_height=slice_height,
                    slice_width=slice_width,
                    overlap_height_ratio=overlap_height_ratio,
                    overlap_width_ratio=overlap_width_ratio,
                    postprocess_type="NMS",  # Non-Maximum Suppression for better filtering
                    postprocess_match_metric="IOS",  # Intersection over smaller area
                    postprocess_match_threshold=0.5,  # Threshold for merging predictions
                    postprocess_class_agnostic=False,  # Class-aware NMS
                )
                
                # Count detections
                num_detections = len(results.object_prediction_list)
                total_detections += num_detections
                
                # Draw bounding boxes with confidence scores
                for pred in results.object_prediction_list:
                    x1, y1, x2, y2 = map(int, [pred.bbox.minx, pred.bbox.miny, 
                                               pred.bbox.maxx, pred.bbox.maxy])
                    class_name = pred.category.name
                    confidence = pred.score.value
                    
                    # Draw rectangle
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Put class name and confidence
                    label = f"{class_name} {confidence:.2f}"
                    cv2.putText(frame, label, (x1, y1 - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36, 255, 12), 2)
                
                # Calculate FPS for this frame
                frame_time = time.time() - frame_start
                fps = 1.0 / frame_time if frame_time > 0 else 0
                
                # Add information overlay
                cv2.putText(frame, f"Frame: {frame_count+1}/{total_frames}", 
                           (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(frame, f"FPS: {fps:.1f}", 
                           (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(frame, f"Objects: {num_detections}", 
                           (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(frame, "SAHI Mode (Small Objects)", 
                           (20, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # Write frame to output video
                video_writer.write(frame)
                
                # Show preview if enabled
                if show_preview:
                    cv2.imshow("SAHI Video Processing", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print("\n⚠️  Processing stopped by user")
                        break
                
                frame_count += 1
                
                # Print progress every 30 frames
                if frame_count % 30 == 0:
                    progress = (frame_count / total_frames) * 100
                    elapsed = time.time() - start_time_total
                    eta = (elapsed / frame_count) * (total_frames - frame_count)
                    print(f"Progress: {progress:.1f}% ({frame_count}/{total_frames}) | "
                          f"FPS: {fps:.1f} | Objects: {num_detections} | "
                          f"ETA: {eta:.1f}s")
        
        except KeyboardInterrupt:
            print("\n⚠️  Processing interrupted by user")
        
        finally:
            # Cleanup
            cap.release()
            video_writer.release()
            if show_preview:
                cv2.destroyAllWindows()
            
            # Print summary
            total_time = time.time() - start_time_total
            avg_fps = frame_count / total_time if total_time > 0 else 0
            avg_detections = total_detections / frame_count if frame_count > 0 else 0
            
            print(f"\n{'='*60}")
            print(f"Processing Complete!")
            print(f"{'='*60}")
            print(f"Frames processed: {frame_count}/{total_frames}")
            print(f"Total time: {total_time:.2f} seconds")
            print(f"Average FPS: {avg_fps:.2f}")
            print(f"Total detections: {total_detections}")
            print(f"Average detections per frame: {avg_detections:.1f}")
            print(f"\n✅ Output saved to: {output_video_path}")
            print(f"{'='*60}\n")
            
            return str(output_video_path)


if __name__ == "__main__":
    processor = SAHIVideoProcessor()
    
    # Configuration
    INPUT_VIDEO = "input_video/my_video.mp4"  # Change to your video path
    OUTPUT_DIR = "output_video"
    
    # IMPROVED SETTINGS for better detection:
    # - Smaller slices (512x512) = more slices = better for small objects
    # - Higher overlap (0.3 = 30%) = less missed objects on borders
    # - Better model (yolov8m.pt) = higher accuracy
    # - Lower confidence threshold (0.15 in load_model) = catch more objects
    
    processor.process_video(
        input_video=INPUT_VIDEO,
        output_dir=OUTPUT_DIR,
        weights="yolov8m.pt",          # UPGRADED: yolov8m (better accuracy than yolov8s)
        device=None,                    # Auto-detect GPU
        slice_height=512,               # SMALLER slices = better small object detection
        slice_width=512,                # SMALLER slices = better small object detection
        overlap_height_ratio=0.3,       # INCREASED to 30% overlap (catches border objects)
        overlap_width_ratio=0.3,        # INCREASED to 30% overlap (catches border objects)
        show_preview=False              # Set True to see processing in real-time
    )

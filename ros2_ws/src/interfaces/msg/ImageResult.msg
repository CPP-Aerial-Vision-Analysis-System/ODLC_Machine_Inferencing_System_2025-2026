# ====== Standard header ======
std_msgs/Header header

# ====== Detection results ======
vision_msgs/Detection2DArray detections
sensor_msgs/Image[] masks

# ====== Metadata fields ======
string image_name
string timestamp
int32 num_detections
string saved_to
string method
string slice_size
string overlap
int32 waypoint_index
float32 longitude
float32 latitude

# ====== Optional object summary ======
# (if you want to explicitly include the object info)
string[] classes
float32[] confidences
float32[] areas
string[] descriptions

Header header
uint32 xmin   # X coordinate of the upper left corner, where bounding box of an object starts
uint32 ymin   # Y coordinate of the upper left corner, where bounding box of an object starts
uint32 xmax   # X coordinate of the bottom right corner, where bounding box of an object ends
uint32 ymax   # Y coordinate of the bottom right corner, where bounding box of an object ends
string Class  # Class name of an object
float32 probability # Probability that the object belongs to the class
geometry_msgs/PointStamped position # Position of the object

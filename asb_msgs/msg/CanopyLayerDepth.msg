string canopy_id
std_msgs/Header header
asb_msgs/CanopyRegionOfInterest roi  # the canopy region of interest from which the canopy depth is computed
float32 resolution  # [m], the quantization length along the x-axis of the canopy

float64 x  # [m], rounded x coordinate along the x-axis of the canopy
float64[] mean_depth  # [m], mean depth of the canopy for each layer
float64[] canopy_layer_bounds  # [m], the z coordinates separating each canopy layer
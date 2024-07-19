std_msgs/Header header
asb_msgs/CanopyRegionOfInterest roi  # the canopy region of interest for which the canopy data is computed

float32 resolution  # [m], the size along the x-axis of the slice used to compute the canopy volume

float64[] volume_x_array  # [m], x-axis of the volume data
float64[] volume_y_array  # [m^3], the volume of the canopy slice along the x-axis (between x-resolution/2 to x+resolution/2), canopy holes are ignored

float64[] depth_x_array  # [m], x-axis of the depth data
float64[] depth_y_array  # [m], the depth of the canopy at corresponding x, z coordinates, canopy holes are ignored
float64[] depth_z_array  # [m], z-axis of the depth data

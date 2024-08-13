uint8 STATUS_OK = 0
uint8 STATUS_STOPPED = 1
uint8 STATUS_FAILED = 2

std_msgs/Header header
string row_id  # identifier of the row
uint8 status  # status of the spraying regulator for the specific row_id

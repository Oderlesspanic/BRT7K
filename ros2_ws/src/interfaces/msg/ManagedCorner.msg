string corner_uid
string corner_id
string wall_id

geometry_msgs/Point center_map
geometry_msgs/Point[] corners_map

float64 map_width
float64 map_height
float64 map_area

float64 mean_h
float64 mean_s
float64 mean_v

float64 median_h
float64 median_s
float64 median_v

float64 std_h
float64 std_s
float64 std_v

float64 validation_score

uint32 seen_count
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
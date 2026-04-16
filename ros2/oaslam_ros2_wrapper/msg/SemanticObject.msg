uint8 STATUS_ONLY_2D=0
uint8 STATUS_INITIALIZED=1
uint8 STATUS_IN_MAP=2
uint8 STATUS_BAD=3

uint32 local_object_id
uint32 category_id
uint8 status
geometry_msgs/Pose pose
geometry_msgs/Vector3 axes
uint32 observation_count
uint32 keyframe_observation_count
float64 last_obs_score
int32 last_obs_frame_id

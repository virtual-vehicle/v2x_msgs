#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Patrizia Neubauer (https://github.com/patrizianeubauer)
#

# CHOICE adaption

# CHOICE! - Choose exactly of the containers
int64 lane_data_attribute_container_select 0
int64 LANE_DATA_ATTRIBUTE_NOTHING = 0
int64 LANE_DATA_ATTRIBUTE_PATH_END_POINT_ANGLE = 1
int64 LANE_DATA_ATTRIBUTE_LANE_CROWN_POINT_CENTER = 2
int64 LANE_DATA_ATTRIBUTE_LANE_CROWN_POINT_LEFT = 3
int64 LANE_DATA_ATTRIBUTE_LANE_CROWN_POINT_RIGHT = 4
int64 LANE_DATA_ATTRIBUTE_LANE_ANGLE = 5
int64 LANE_DATA_ATTRIBUTE_SPEED_LIMITS = 6
int64 LANE_DATA_ATTRIBUTE_REGIONAL = 7

# container 1
DeltaAngle path_end_point_angle

# container 2
RoadwayCrownAngle lane_crown_point_center

# container 3
RoadwayCrownAngle lane_crown_point_left

# container 4
RoadwayCrownAngle lane_crown_point_right

# container 5
MergeDivergeNodeAngle lane_angle

# container 6
SpeedLimitList speed_limits

# container 7
RegLaneAttributes[] regional #size(1..4)


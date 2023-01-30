#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 


# CHOICE! - Choose exactly of the containers
int64 intersection_access_point_container_select 0
int64 INTERSECTION_ACCESS_POINT_NOTHING = 0
int64 INTERSECTION_ACCESS_POINT_LANE = 1
int64 INTERSECTION_ACCESS_POINT_APPROACH = 2
int64 INTERSECTION_ACCESS_POINT_CONNECTION = 3

# container 1
LaneID lane

# container 2
ApproachID approach

# container 3
LaneConnectionID connection


#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 


# CHOICE! - Choose exactly of the containers
int64 zone_container_select 0
int64 ZONE_NOTHING = 0
int64 ZONE_SEGMENT = 1
int64 ZONE_AREA = 2
int64 ZONE_COMPUTED_SEGMENT = 3

# container 1
Segment segment

# container 2
PolygonalLine area

# container 3
ComputedSegment computed_segment


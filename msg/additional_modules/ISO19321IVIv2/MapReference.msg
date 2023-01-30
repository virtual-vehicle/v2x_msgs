#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 


# CHOICE! - Choose exactly of the containers
int64 map_reference_container_select 0
int64 MAP_REFERENCE_NOTHING = 0
int64 MAP_REFERENCE_ROADSEGMENT = 1
int64 MAP_REFERENCE_INTERSECTION = 2

# container 1
RoadSegmentReferenceID roadsegment

# container 2
IntersectionReferenceID intersection


#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 


# CHOICE! - Choose exactly of the containers
int64 originating_rsucontainer_container_select 0
int64 ORIGINATING_RSUCONTAINER_NOTHING = 0
int64 ORIGINATING_RSUCONTAINER_INTERSECTION_REFERENCE_ID = 1
int64 ORIGINATING_RSUCONTAINER_ROAD_SEGMENT_REFERENCE_ID = 2

# container 1
IntersectionReferenceID intersection_reference_id

# container 2
RoadSegmentReferenceID road_segment_reference_id


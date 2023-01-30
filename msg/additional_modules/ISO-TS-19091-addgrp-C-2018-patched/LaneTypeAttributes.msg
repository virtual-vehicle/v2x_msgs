#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 


# CHOICE! - Choose exactly of the containers
int64 lane_type_attributes_container_select 0
int64 LANE_TYPE_ATTRIBUTES_NOTHING = 0
int64 LANE_TYPE_ATTRIBUTES_VEHICLE = 1
int64 LANE_TYPE_ATTRIBUTES_CROSSWALK = 2
int64 LANE_TYPE_ATTRIBUTES_BIKE_LANE = 3
int64 LANE_TYPE_ATTRIBUTES_SIDEWALK = 4
int64 LANE_TYPE_ATTRIBUTES_MEDIAN = 5
int64 LANE_TYPE_ATTRIBUTES_STRIPING = 6
int64 LANE_TYPE_ATTRIBUTES_TRACKED_VEHICLE = 7
int64 LANE_TYPE_ATTRIBUTES_PARKING = 8

# container 1
LaneAttributesVehicle vehicle

# container 2
LaneAttributesCrosswalk crosswalk

# container 3
LaneAttributesBike bike_lane

# container 4
LaneAttributesSidewalk sidewalk

# container 5
LaneAttributesBarrier median

# container 6
LaneAttributesStriping striping

# container 7
LaneAttributesTrackedVehicle tracked_vehicle

# container 8
LaneAttributesParking parking


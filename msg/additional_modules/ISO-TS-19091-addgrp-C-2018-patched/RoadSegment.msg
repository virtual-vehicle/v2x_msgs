#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



# Optional Field
bool name_present 0
DescriptiveName name

RoadSegmentReferenceID id

MsgCount revision

Position3D ref_point

# Optional Field
bool lane_width_present 0
LaneWidth lane_width

# Optional Field
bool speed_limits_present 0
SpeedLimitList speed_limits

RoadLaneSetList road_lane_set

# Optional Field
bool regional_present 0
RegRoadSegment[] regional #size(1..4)

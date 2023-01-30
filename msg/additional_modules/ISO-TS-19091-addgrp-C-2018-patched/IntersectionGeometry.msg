#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



# Optional Field
bool name_present 0
DescriptiveName name

IntersectionReferenceID id

MsgCount revision

Position3D ref_point

# Optional Field
bool lane_width_present 0
LaneWidth lane_width

# Optional Field
bool speed_limits_present 0
SpeedLimitList speed_limits

LaneList lane_set

# Optional Field
bool preempt_priority_data_present 0
PreemptPriorityList preempt_priority_data

# Optional Field
bool regional_present 0
RegIntersectionGeometry[] regional #size(1..4)

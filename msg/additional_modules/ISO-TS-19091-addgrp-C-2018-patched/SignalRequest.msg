#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



IntersectionReferenceID id

RequestID request_id

PriorityRequestType request_type

IntersectionAccessPoint in_bound_lane

# Optional Field
bool out_bound_lane_present 0
IntersectionAccessPoint out_bound_lane

# Optional Field
bool regional_present 0
RegSignalRequest[] regional #size(1..4)

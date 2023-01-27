#
# Created by script of Patrizia Neubauer (https://github.com/patrizianeubauer)
# Date: Fri Jan 20 13:59:04 2023
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)



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

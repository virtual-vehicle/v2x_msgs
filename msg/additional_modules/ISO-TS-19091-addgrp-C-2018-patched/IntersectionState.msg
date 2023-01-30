#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



# Optional Field
bool name_present 0
DescriptiveName name

IntersectionReferenceID id

MsgCount revision

IntersectionStatusObject status

# Optional Field
bool moy_present 0
MinuteOfTheYear moy

# Optional Field
bool time_stamp_present 0
DSecond time_stamp

# Optional Field
bool enabled_lanes_present 0
EnabledLaneList enabled_lanes

MovementList states

# Optional Field
bool maneuver_assist_list_present 0
ManeuverAssistList maneuver_assist_list

# Optional Field
bool regional_present 0
RegIntersectionState[] regional #size(1..4)

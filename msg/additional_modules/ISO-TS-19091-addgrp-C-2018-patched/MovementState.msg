#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



# Optional Field
bool movement_name_present 0
DescriptiveName movement_name

SignalGroupID signal_group

MovementEventList state_time_speed

# Optional Field
bool maneuver_assist_list_present 0
ManeuverAssistList maneuver_assist_list

# Optional Field
bool regional_present 0
RegMovementState[] regional #size(1..4)

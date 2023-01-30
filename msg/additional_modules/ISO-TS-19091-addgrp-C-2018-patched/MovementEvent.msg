#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



MovementPhaseState event_state

# Optional Field
bool timing_present 0
TimeChangeDetails timing

# Optional Field
bool speeds_present 0
AdvisorySpeedList speeds

# Optional Field
bool regional_present 0
RegMovementEvent[] regional #size(1..4)

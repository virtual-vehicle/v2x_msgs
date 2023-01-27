#
# Created by script of Patrizia Neubauer (https://github.com/patrizianeubauer)
# Date: Thu Jan 19 09:10:59 2023
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
#

# Removed Reg in front of Connection[], because parser adds Reg in front of identifier if regional is variable name

LaneConnectionID connection_id

# Optional Field
bool queue_length_present 0
ZoneLength queue_length

# Optional Field
bool available_storage_length_present 0
ZoneLength available_storage_length

# Optional Field
bool wait_on_stop_present 0
WaitOnStopline wait_on_stop

# Optional Field
bool ped_bicycle_detect_present 0
PedestrianBicycleDetect ped_bicycle_detect

# Optional Field
bool regional_present 0
Connection[] regional #size(1..4)

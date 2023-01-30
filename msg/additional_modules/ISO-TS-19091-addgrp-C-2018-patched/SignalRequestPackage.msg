#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



SignalRequest request

# Optional Field
bool minute_present 0
MinuteOfTheYear minute

# Optional Field
bool second_present 0
DSecond second

# Optional Field
bool duration_present 0
DSecond duration

# Optional Field
bool regional_present 0
RegSignalRequestPackage[] regional #size(1..4)

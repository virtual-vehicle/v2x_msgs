#
# Created by script of Patrizia Neubauer (https://github.com/patrizianeubauer)
# Date: Fri Jan 20 13:59:04 2023
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)



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

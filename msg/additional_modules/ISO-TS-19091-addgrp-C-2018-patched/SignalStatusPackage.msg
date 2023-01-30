#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



# Optional Field
bool requester_present 0
SignalRequesterInfo requester

IntersectionAccessPoint inbound_on

# Optional Field
bool outbound_on_present 0
IntersectionAccessPoint outbound_on

# Optional Field
bool minute_present 0
MinuteOfTheYear minute

# Optional Field
bool second_present 0
DSecond second

# Optional Field
bool duration_present 0
DSecond duration

PrioritizationResponseStatus status

# Optional Field
bool regional_present 0
RegSignalStatusPackage[] regional #size(1..4)

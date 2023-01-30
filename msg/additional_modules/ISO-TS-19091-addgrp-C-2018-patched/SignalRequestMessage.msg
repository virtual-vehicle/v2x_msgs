#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



# Optional Field
bool time_stamp_present 0
MinuteOfTheYear time_stamp

DSecond second

# Optional Field
bool sequence_number_present 0
MsgCount sequence_number

# Optional Field
bool requests_present 0
SignalRequestList requests

RequestorDescription requestor

# Optional Field
bool regional_present 0
RegSignalRequestMessage[] regional #size(1..4)

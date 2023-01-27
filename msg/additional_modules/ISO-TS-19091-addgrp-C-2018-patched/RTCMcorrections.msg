#
# Created by script of Patrizia Neubauer (https://github.com/patrizianeubauer)
# Date: Fri Jan 20 13:59:04 2023
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)



MsgCount msg_cnt

RTCMRevision rev

# Optional Field
bool time_stamp_present 0
MinuteOfTheYear time_stamp

# Optional Field
bool anchor_point_present 0
FullPositionVector anchor_point

# Optional Field
bool rtcm_header_present 0
RTCMheader rtcm_header

RTCMmessageList msgs

# Optional Field
bool regional_present 0
RegRTCMcorrections[] regional #size(1..4)

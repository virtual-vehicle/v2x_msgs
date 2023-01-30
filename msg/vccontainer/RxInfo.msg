#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



TimeVal timestamp

int64 rssi
int64 RSSI_RANGE_MIN = -32768
int64 RSSI_RANGE_MAX = 32767

int64 data_rate
int64 DATA_RATE_RANGE_MIN = 0
int64 DATA_RATE_RANGE_MAX = 65535

int64 traffic_class_id
int64 TRAFFIC_CLASS_ID_RANGE_MIN = 0
int64 TRAFFIC_CLASS_ID_RANGE_MAX = 255

int64 remain_hop_limit
int64 REMAIN_HOP_LIMIT_RANGE_MIN = 0
int64 REMAIN_HOP_LIMIT_RANGE_MAX = 255

RxSecurity security

PositionInfo position_info

AreaTypeInfo area_type_info

MACAddress source_mac_address

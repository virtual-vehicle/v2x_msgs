#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



int64 dest_port
int64 DEST_PORT_RANGE_MIN = 0
int64 DEST_PORT_RANGE_MAX = 65535

int64 data_rate
int64 DATA_RATE_RANGE_MIN = 0
int64 DATA_RATE_RANGE_MAX = 255

int64 tx_power
int64 TX_POWER_RANGE_MIN = -128
int64 TX_POWER_RANGE_MAX = 127

int64 traffic_class_id
int64 TRAFFIC_CLASS_ID_RANGE_MIN = 0
int64 TRAFFIC_CLASS_ID_RANGE_MAX = 255

TxSecurity security

PositionInfo position_info

AreaTypeInfo area_type_info

bool dest_port_present

bool data_rate_present

bool tx_power_present

bool traffic_class_idpresent

bool security_present

bool position_info_present

bool area_type_info_present

#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



VcClass road_sign_class

int64 road_sign_code
int64 ROAD_SIGN_CODE_RANGE_MIN = 1
int64 ROAD_SIGN_CODE_RANGE_MAX = 64

VcOption vc_option

# Optional Field
bool validity_present 0
ValidityPeriods validity

# Optional Field
bool value_present 0
int64 value
int64 VALUE_RANGE_MIN = 0
int64 VALUE_RANGE_MAX = 65535

# Optional Field
bool unit_present 0
RSCUnit unit

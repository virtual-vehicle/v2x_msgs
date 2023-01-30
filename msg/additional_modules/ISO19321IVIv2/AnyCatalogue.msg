#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



Provider owner

int64 version
int64 VERSION_RANGE_MIN = 0
int64 VERSION_RANGE_MAX = 255

int64 pictogram_code
int64 PICTOGRAM_CODE_RANGE_MIN = 0
int64 PICTOGRAM_CODE_RANGE_MAX = 65535

# Optional Field
bool value_present 0
int64 value
int64 VALUE_RANGE_MIN = 0
int64 VALUE_RANGE_MAX = 65535

# Optional Field
bool unit_present 0
RSCUnit unit

# Optional Field
bool attributes_present 0
ISO14823Attributes attributes

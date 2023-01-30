#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



DangerousGoodsBasic dangerous_goods_type

int64 un_number
int64 UN_NUMBER_RANGE_MIN = 0
int64 UN_NUMBER_RANGE_MAX = 9999

bool elevated_temperature

bool tunnels_restricted

bool limited_quantity

# Optional Field
bool emergency_action_code_present 0
string emergency_action_code

# Optional Field
bool phone_number_present 0
PhoneNumber phone_number

# Optional Field
bool company_name_present 0
string company_name

#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



# Optional Field
bool layout_component_id_present 0
int64 layout_component_id
int64 LAYOUT_COMPONENT_ID_RANGE_MIN = 1
int64 LAYOUT_COMPONENT_ID_RANGE_MAX = 4

# CHOICE! - Choose exactly of the containers
int64 code_container_select 0
int64 CODE_NOTHING = 0
int64 CODE_VC_CODE = 1
int64 CODE_ISO14823CODE = 2
int64 CODE_ITIS_CODES = 3
int64 CODE_ANY_CATALOGUE = 4

# container 1
VcCode vienna_convention

# container 2
ISO14823Code iso14823

# container 3
int64 itis_codes

# container 4
AnyCatalogue any_catalogue


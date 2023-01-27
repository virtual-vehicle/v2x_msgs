#
# Created by script of Patrizia Neubauer (https://github.com/patrizianeubauer)
# Date: Thu Jan 19 09:10:59 2023
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
#

# CHOICE adaption

# Optional Field
bool country_code_present 0
int64[] country_code #size(1..2)


# CHOICE! - Choose exactly of the containers
int64 service_category_code_container_select 0
int64 SERVICE_CATEGORY_CODE_NOTHING = 0
int64 SERVICE_CATEGORY_CODE_TRAFFIC_SIGN_PICTOGRAM = 1
int64 SERVICE_CATEGORY_CODE_PUBLIC_FACILITIES_PICTOGRAM = 2
int64 SERVICE_CATEGORY_CODE_AMBIENT_OR_ROAD_CONDITION_PICTOGRAM = 3


# container 1
int64 traffic_sign_pictogram
int64 TRAFFIC_SIGN_PICTOGRAM_DANGER_WARNING = 0
int64 TRAFFIC_SIGN_PICTOGRAM_REGULATORY = 1
int64 TRAFFIC_SIGN_PICTOGRAM_INFORMATIVE = 2

# container 2
int64 public_facilities_pictorgram
int64 PUBLIC_FACILITIES_PICTOGRAM_PUBLIC_FACILITIES = 0

# container 3
int64 ambient_or_road_condition_pictogram
int64 AMBIENT_OR_ROAD_CONDITION_PICTOGRAM_AMBIENT_CONDITION = 0
int64 AMBIENT_OR_ROAD_CONDITION_PICTOGRAM_ROAD_CONDITION = 1


int64 nature
int64 NATURE_RANGE_MIN = 1
int64 NATURE_RANGE_MAX = 9

int64 serial_number
int64 SERIAL_NUMBER_RANGE_MIN = 0
int64 SERIAL_NUMBER_RANGE_MAX = 99

# Optional Field
bool attributes_present 0
ISO14823Attributes attributes
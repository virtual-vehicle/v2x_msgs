#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Patrizia Neubauer (https://github.com/patrizianeubauer)
#

# Ringdependency

# Optional Field
bool local_info_present 0
LocalInfo local_info

RxInfo rx_info

int64 payload_size
int64 PAYLOAD_SIZE_RANGE_MIN = 0
int64 PAYLOAD_SIZE_RANGE_MAX = 1500

# Optional Field
bool payload_string_present 0
int64[] payload_string #size(1..1500)


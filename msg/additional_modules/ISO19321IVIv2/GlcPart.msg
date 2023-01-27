#
# Created by script of Patrizia Neubauer (https://github.com/patrizianeubauer)
# Date: Fri Jan 20 13:59:04 2023
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)



Zid zone_id

# Optional Field
bool lane_number_present 0
LanePosition lane_number

# Optional Field
bool zone_extension_present 0
int64 zone_extension
int64 ZONE_EXTENSION_RANGE_MIN = 0
int64 ZONE_EXTENSION_RANGE_MAX = 255

# Optional Field
bool zone_heading_present 0
HeadingValue zone_heading

# Optional Field
bool zone_present 0
Zone zone

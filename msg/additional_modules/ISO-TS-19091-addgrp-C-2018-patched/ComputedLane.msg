#
# Created by script of Patrizia Neubauer (https://github.com/patrizianeubauer)
# Date: Thu Jan 19 09:10:59 2023
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
#

# duplicates (small, large) removed by adding char

LaneID reference_lane_id

# CHOICE! - Choose exactly of the containers
int64 offset_xaxis_container_select 0
int64 OFFSET_XAXIS_NOTHING = 0
int64 OFFSET_XAXIS_DRIVEN_LINE_OFFSET_SM = 1
int64 OFFSET_XAXIS_DRIVEN_LINE_OFFSET_LG = 2

# container 1
DrivenLineOffsetSm small_x

# container 2
DrivenLineOffsetLg large_x


# CHOICE! - Choose exactly of the containers
int64 offset_yaxis_container_select 0
int64 OFFSET_YAXIS_NOTHING = 0
int64 OFFSET_YAXIS_DRIVEN_LINE_OFFSET_SM = 1
int64 OFFSET_YAXIS_DRIVEN_LINE_OFFSET_LG = 2

# container 1
DrivenLineOffsetSm small_y

# container 2
DrivenLineOffsetLg large_y


# Optional Field
bool rotate_xy_present 0
Angle rotate_xy

# Optional Field
bool scale_xaxis_present 0
ScaleB12 scale_xaxis

# Optional Field
bool scale_yaxis_present 0
ScaleB12 scale_yaxis

# Optional Field
bool regional_present 0
AntennaOffsetSet[] regional #size(1..4)

#
# Created by script of Patrizia Neubauer (https://github.com/patrizianeubauer)
# Date: Thu Jan 19 09:10:59 2023
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
#

# CHOICE adaption

# CHOICE! - Choose exactly of the containers
int64 restriction_user_type_container_select 0
int64 RESTRICTION_USER_TYPE_NOTHING = 0
int64 RESTRICTION_USER_TYPE_BASIC_TYPE = 1
int64 RESTRICTION_USER_REGIONAL = 2

# container 1
RestrictionAppliesTo basic_type

# container 2
RegRestrictionUserType[] regional #size(1..4)




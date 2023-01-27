#
# Created by script of Patrizia Neubauer (https://github.com/patrizianeubauer)
# Date: Thu Jan 19 09:10:59 2023
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
#

# Added pos in front of long & lat

Latitude pos_lat

Longitude pos_long

# Optional Field
bool elevation_present 0
Elevation elevation

# Optional Field
bool regional_present 0
RegPosition3D[] regional #size(1..4)

#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 


# CHOICE! - Choose exactly of the containers
int64 vehicle_id_container_select 0
int64 VEHICLE_ID_NOTHING = 0
int64 VEHICLE_ID_ENTITY_ID = 1
int64 VEHICLE_ID_STATION_ID = 2

# container 1
TemporaryID entity_id

# container 2
StationID station_id


#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 


# CHOICE! - Choose exactly of the containers
int64 station_data_container_container_select 0
int64 STATION_DATA_CONTAINER_NOTHING = 0
int64 STATION_DATA_CONTAINER_ORIGINATING_VEHICLE_CONTAINER = 1
int64 STATION_DATA_CONTAINER_ORIGINATING_RSUCONTAINER = 2

# container 1
OriginatingVehicleContainer originating_vehicle_container

# container 2
OriginatingRSUContainer originating_rsucontainer


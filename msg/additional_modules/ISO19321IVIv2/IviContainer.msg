#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 


# CHOICE! - Choose exactly of the containers
int64 ivi_container_container_select 0
int64 IVI_CONTAINER_NOTHING = 0
int64 IVI_CONTAINER_GLC = 1
int64 IVI_CONTAINER_GIV = 2
int64 IVI_CONTAINER_RCC = 3
int64 IVI_CONTAINER_TC = 4
int64 IVI_CONTAINER_LAC = 5
int64 IVI_CONTAINER_AUTOMATED_VEHICLE_CONTAINER = 6
int64 IVI_CONTAINER_MAP_LOCATION_CONTAINER = 7
int64 IVI_CONTAINER_ROAD_SURFACE_CONTAINER = 8

# container 1
GeographicLocationContainer glc

# container 2
GeneralIviContainer giv

# container 3
RoadConfigurationContainer rcc

# container 4
TextContainer tc

# container 5
LayoutContainer lac

# container 6
AutomatedVehicleContainer avc

# container 7
MapLocationContainer mlc

# container 8
RoadSurfaceContainer rsc


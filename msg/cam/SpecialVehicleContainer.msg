#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 


# CHOICE! - Choose exactly of the containers
int64 special_vehicle_container_container_select 0
int64 SPECIAL_VEHICLE_CONTAINER_NOTHING = 0
int64 SPECIAL_VEHICLE_CONTAINER_PUBLIC_TRANSPORT_CONTAINER = 1
int64 SPECIAL_VEHICLE_CONTAINER_SPECIAL_TRANSPORT_CONTAINER = 2
int64 SPECIAL_VEHICLE_CONTAINER_DANGEROUS_GOODS_CONTAINER = 3
int64 SPECIAL_VEHICLE_CONTAINER_ROAD_WORKS_CONTAINER_BASIC = 4
int64 SPECIAL_VEHICLE_CONTAINER_RESCUE_CONTAINER = 5
int64 SPECIAL_VEHICLE_CONTAINER_EMERGENCY_CONTAINER = 6
int64 SPECIAL_VEHICLE_CONTAINER_SAFETY_CAR_CONTAINER = 7

# container 1
PublicTransportContainer public_transport_container

# container 2
SpecialTransportContainer special_transport_container

# container 3
DangerousGoodsContainer dangerous_goods_container

# container 4
RoadWorksContainerBasic road_works_container_basic

# container 5
RescueContainer rescue_container

# container 6
EmergencyContainer emergency_container

# container 7
SafetyCarContainer safety_car_container


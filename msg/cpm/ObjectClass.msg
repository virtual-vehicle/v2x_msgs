#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



ClassConfidence confidence

# CHOICE! - Choose exactly of the containers
int64 class_container_select 0
int64 CLASS_NOTHING = 0
int64 CLASS_VEHICLE_SUBCLASS = 1
int64 CLASS_PERSON_SUBCLASS = 2
int64 CLASS_ANIMAL_SUBCLASS = 3
int64 CLASS_OTHER_SUBCLASS = 4

# container 1
VehicleSubclass vehicle

# container 2
PersonSubclass person

# container 3
AnimalSubclass animal

# container 4
OtherSubclass other


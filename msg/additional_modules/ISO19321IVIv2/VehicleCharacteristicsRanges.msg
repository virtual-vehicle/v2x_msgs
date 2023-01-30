#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



ComparisonOperator comparison_operator

# CHOICE! - Choose exactly of the containers
int64 limits_container_select 0
int64 LIMITS_NOTHING = 0
int64 LIMITS_NUMBER_OF_AXLES = 1
int64 LIMITS_VEHICLE_DIMENSIONS = 2
int64 LIMITS_VEHICLE_WEIGHT_LIMITS = 3
int64 LIMITS_AXLE_WEIGHT_LIMITS = 4
int64 LIMITS_PASSENGER_CAPACITY = 5
int64 LIMITS_EXHAUST_EMISSION_VALUES = 6
int64 LIMITS_DIESEL_EMISSION_VALUES = 7
int64 LIMITS_SOUND_LEVEL = 8

# container 1
int64 number_of_axles

# container 2
VehicleDimensions vehicle_dimensions

# container 3
VehicleWeightLimits vehicle_weight_limits

# container 4
AxleWeightLimits axle_weight_limits

# container 5
PassengerCapacity passenger_capacity

# container 6
ExhaustEmissionValues exhaust_emission_values

# container 7
DieselEmissionValues diesel_emission_values

# container 8
SoundLevel sound_level


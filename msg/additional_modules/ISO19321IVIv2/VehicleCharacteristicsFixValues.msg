#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 


# CHOICE! - Choose exactly of the containers
int64 vehicle_characteristics_fix_values_container_select 0
int64 VEHICLE_CHARACTERISTICS_FIX_VALUES_NOTHING = 0
int64 VEHICLE_CHARACTERISTICS_FIX_VALUES_SIMPLE_VEHICLE_TYPE = 1
int64 VEHICLE_CHARACTERISTICS_FIX_VALUES_EU_VEHICLE_CATEGORY_CODE = 2
int64 VEHICLE_CHARACTERISTICS_FIX_VALUES_ISO3833VEHICLE_TYPE = 3
int64 VEHICLE_CHARACTERISTICS_FIX_VALUES_EURO_AND_CO2VALUE = 4
int64 VEHICLE_CHARACTERISTICS_FIX_VALUES_ENGINE_CHARACTERISTICS = 5
int64 VEHICLE_CHARACTERISTICS_FIX_VALUES_LOAD_TYPE = 6
int64 VEHICLE_CHARACTERISTICS_FIX_VALUES_USAGE = 7

# container 1
StationType simple_vehicle_type

# container 2
EuVehicleCategoryCode eu_vehicle_category_code

# container 3
Iso3833VehicleType iso3833vehicle_type

# container 4
EnvironmentalCharacteristics euro_and_co2value

# container 5
EngineCharacteristics engine_characteristics

# container 6
LoadType load_type

# container 7
VehicleRole usage


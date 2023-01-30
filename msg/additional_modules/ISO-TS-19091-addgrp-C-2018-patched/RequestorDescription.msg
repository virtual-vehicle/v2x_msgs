#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



VehicleID id

# Optional Field
bool type_present 0
RequestorType type

# Optional Field
bool position_present 0
RequestorPositionVector position

# Optional Field
bool name_present 0
DescriptiveName name

# Optional Field
bool route_name_present 0
DescriptiveName route_name

# Optional Field
bool transit_status_present 0
TransitVehicleStatus transit_status

# Optional Field
bool transit_occupancy_present 0
TransitVehicleOccupancy transit_occupancy

# Optional Field
bool transit_schedule_present 0
DeltaTime transit_schedule

# Optional Field
bool regional_present 0
RegRequestorDescription[] regional #size(1..4)

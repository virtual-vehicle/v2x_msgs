#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



int64 arrow_direction
int64 ARROW_DIRECTION_RANGE_MIN = 0
int64 ARROW_DIRECTION_RANGE_MAX = 7

# Optional Field
bool dest_place_present 0
DestinationPlaces dest_place

# Optional Field
bool dest_road_present 0
DestinationRoads dest_road

# Optional Field
bool road_number_identifier_present 0
int64 road_number_identifier
int64 ROAD_NUMBER_IDENTIFIER_RANGE_MIN = 1
int64 ROAD_NUMBER_IDENTIFIER_RANGE_MAX = 999

# Optional Field
bool street_name_present 0
int64 street_name
int64 STREET_NAME_RANGE_MIN = 1
int64 STREET_NAME_RANGE_MAX = 999

# Optional Field
bool street_name_text_present 0
string street_name_text

# Optional Field
bool distance_to_diverging_point_present 0
DistanceOrDuration distance_to_diverging_point

# Optional Field
bool distance_to_destination_place_present 0
DistanceOrDuration distance_to_destination_place

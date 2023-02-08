#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Patrizia Neubauer (https://github.com/patrizianeubauer)
#

# RingDependency with dest_rscode in line 6: GddStructure -> GddAttributes -> GddAtrribute -> DDDIOList -> DDDIO -> DestinationPlace -> GddStructure

DestinationType dest_type

# Optional Field
bool dest_rscode_present 0
# GddStructure dest_rscode

# Optional Field
bool dest_blob_present 0
int64[] dest_blob

# Optional Field
bool place_name_identification_present 0
int64 place_name_identification
int64 PLACE_NAME_IDENTIFICATION_RANGE_MIN = 1
int64 PLACE_NAME_IDENTIFICATION_RANGE_MAX = 999

# Optional Field
bool place_name_text_present 0
string place_name_text

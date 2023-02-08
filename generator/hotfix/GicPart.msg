#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Patrizia Neubauer (https://github.com/patrizianeubauer)
#

# Added SEQUENCE OF adaptation

# Optional Field
bool detection_zone_ids_present 0
ZoneIds detection_zone_ids

# Optional Field
bool relevance_zone_ids_present 0
ZoneIds relevance_zone_ids

# Optional Field
bool direction_present 0
Direction direction

# Optional Field
bool driver_awareness_zone_ids_present 0
ZoneIds driver_awareness_zone_ids

# Optional Field
bool minimum_awareness_time_present 0
int64 minimum_awareness_time
int64 MINIMUM_AWARENESS_TIME_RANGE_MIN = 0
int64 MINIMUM_AWARENESS_TIME_RANGE_MAX = 255

# Optional Field
bool applicable_lanes_present 0
LanePositions applicable_lanes

IviType ivi_type

# Optional Field
bool ivi_purpose_present 0
IviPurpose ivi_purpose

# Optional Field
bool lane_status_present 0
LaneStatus lane_status

# Optional Field
bool vehicle_characteristics_present 0
VehicleCharacteristicsList vehicle_characteristics

# Optional Field
bool driver_characteristics_present 0
DriverCharacteristics driver_characteristics

# Optional Field
bool layout_id_present 0
int64 layout_id
int64 LAYOUT_ID_RANGE_MIN = 1
int64 LAYOUT_ID_RANGE_MAX = 4

# Optional Field
bool pre_storedlayout_id_present 0
int64 pre_storedlayout_id
int64 PRE_STOREDLAYOUT_ID_RANGE_MIN = 1
int64 PRE_STOREDLAYOUT_ID_RANGE_MAX = 64

RoadSignCodes road_sign_codes

# Optional Field
bool extra_text_present 0
ConstraintTextLines1 extra_text

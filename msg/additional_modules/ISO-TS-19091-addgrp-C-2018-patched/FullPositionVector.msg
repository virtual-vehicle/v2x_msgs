#
# Created by script of Patrizia Neubauer (https://github.com/patrizianeubauer)
# Date: Thu Jan 19 09:10:59 2023
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
#

# Added vector in front of long & lat

# Optional Field
bool utc_time_present 0
DDateTime utc_time

Longitude vector_long

Latitude vector_lat

# Optional Field
bool elevation_present 0
Elevation elevation

# Optional Field
bool heading_present 0
HeadingDSRC heading

# Optional Field
bool speed_present 0
TransmissionAndSpeed speed

# Optional Field
bool pos_accuracy_present 0
PositionalAccuracy pos_accuracy

# Optional Field
bool time_confidence_present 0
TimeConfidence time_confidence

# Optional Field
bool pos_confidence_present 0
PositionConfidenceSet pos_confidence

# Optional Field
bool speed_confidence_present 0
SpeedandHeadingandThrottleConfidence speed_confidence

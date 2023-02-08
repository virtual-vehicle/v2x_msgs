#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Patrizia Neubauer (https://github.com/patrizianeubauer)
#

AdvisorySpeedType type

# Optional Field
bool speed_present 0
SpeedAdvice speed

# Optional Field
bool confidence_present 0
SpeedConfidenceDSRC confidence

# Optional Field
bool distance_present 0
ZoneLength distance

# Parser generates RestrictionClassID class, but class is a c++ key word, therefor not supported
# Optional Field
bool class_present 0
RestrictionClassID rest_class

# Optional Field
bool regional_present 0
RegAdvisorySpeed[] regional #size(1..4)

#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 


# CHOICE! - Choose exactly of the containers
int64 gdd_attribute_container_select 0
int64 GDD_ATTRIBUTE_NOTHING = 0
int64 GDD_ATTRIBUTE_DTM = 1
int64 GDD_ATTRIBUTE_EDT = 2
int64 GDD_ATTRIBUTE_DFL = 3
int64 GDD_ATTRIBUTE_VED = 4
int64 GDD_ATTRIBUTE_SPE = 5
int64 GDD_ATTRIBUTE_ROI = 6
int64 GDD_ATTRIBUTE_DBV = 7
int64 GDD_ATTRIBUTE_DDD = 8
int64 GDD_ATTRIBUTE_SET = 9
int64 GDD_ATTRIBUTE_NOL = 10

# container 1
InternationalSignApplicablePeriod dtm

# container 2
InternationalSignExemptedApplicablePeriod edt

# container 3
InternationalSignDirectionalFlowOfLane dfl

# container 4
InternationalSignApplicableVehicleDimensions ved

# container 5
InternationalSignSpeedLimits spe

# container 6
InternationalSignRateOfIncline roi

# container 7
InternationalSignDistanceBetweenVehicles dbv

# container 8
InternationalSignDestinationInformation ddd

# container 9
InternationalSignSection set

# container 10
InternationalSignNumberOfLane nol


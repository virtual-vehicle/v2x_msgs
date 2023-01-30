#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 


# CHOICE! - Choose exactly of the containers
int64 detection_area_container_select 0
int64 DETECTION_AREA_NOTHING = 0
int64 DETECTION_AREA_VEHICLE_SENSOR = 1
int64 DETECTION_AREA_STATIONARY_SENSOR_RADIAL = 2
int64 DETECTION_AREA_STATIONARY_SENSOR_POLYGON = 3
int64 DETECTION_AREA_STATIONARY_SENSOR_CIRCULAR = 4
int64 DETECTION_AREA_STATIONARY_SENSOR_ELLIPSE = 5
int64 DETECTION_AREA_STATIONARY_SENSOR_RECTANGLE = 6

# container 1
VehicleSensor vehicle_sensor

# container 2
AreaRadial stationary_sensor_radial

# container 3
AreaPolygon stationary_sensor_polygon

# container 4
AreaCircular stationary_sensor_circular

# container 5
AreaEllipse stationary_sensor_ellipse

# container 6
AreaRectangle stationary_sensor_rectangle


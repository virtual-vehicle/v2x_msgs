#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 


# CHOICE! - Choose exactly of the containers
int64 free_space_area_container_select 0
int64 FREE_SPACE_AREA_NOTHING = 0
int64 FREE_SPACE_AREA_FREE_SPACE_POLYGON = 1
int64 FREE_SPACE_AREA_FREE_SPACE_CIRCULAR = 2
int64 FREE_SPACE_AREA_FREE_SPACE_ELLIPSE = 3
int64 FREE_SPACE_AREA_FREE_SPACE_RECTANGLE = 4

# container 1
AreaPolygon free_space_polygon

# container 2
AreaCircular free_space_circular

# container 3
AreaEllipse free_space_ellipse

# container 4
AreaRectangle free_space_rectangle


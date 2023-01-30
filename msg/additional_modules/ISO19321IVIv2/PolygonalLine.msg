#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 


# CHOICE! - Choose exactly of the containers
int64 polygonal_line_container_select 0
int64 POLYGONAL_LINE_NOTHING = 0
int64 POLYGONAL_LINE_DELTA_POSITIONS = 1
int64 POLYGONAL_LINE_DELTA_POSITIONS_WITH_ALTITUDE = 2
int64 POLYGONAL_LINE_ABSOLUTE_POSITIONS = 3
int64 POLYGONAL_LINE_ABSOLUTE_POSITIONS_WITH_ALTITUDE = 4

# container 1
DeltaPositions delta_positions

# container 2
DeltaReferencePositions delta_positions_with_altitude

# container 3
AbsolutePositions absolute_positions

# container 4
AbsolutePositionsWAltitude absolute_positions_with_altitude


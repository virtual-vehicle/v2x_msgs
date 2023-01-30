#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



LaneID lane_id

# Optional Field
bool name_present 0
DescriptiveName name

# Optional Field
bool ingress_approach_present 0
ApproachID ingress_approach

# Optional Field
bool egress_approach_present 0
ApproachID egress_approach

LaneAttributes lane_attributes

# Optional Field
bool maneuvers_present 0
AllowedManeuvers maneuvers

NodeListXY node_list

# Optional Field
bool connects_to_present 0
ConnectsToList connects_to

# Optional Field
bool overlays_present 0
OverlayLaneList overlays

# Optional Field
bool regional_present 0
RegGenericLane[] regional #size(1..4)

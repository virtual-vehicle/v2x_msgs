#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Patrizia Neubauer (https://github.com/patrizianeubauer)
#

bool mode

DsrcEID eid

ActionType action_type

# Optional Field
bool access_credentials_present 0
int64[] access_credentials #size(0..127)

# container-reference not parsed
# Optional Field
bool action_parameter_present 0
int64 action_parameter

# Optional Field
bool iid_present 0
DsrcEID iid

#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 


# CHOICE! - Choose exactly of the containers
int64 tapdusgeneric_container_select 0
int64 TAPDUSGENERIC_NOTHING = 0
int64 TAPDUSGENERIC_ACTION_REQUEST = 1
int64 TAPDUSGENERIC_ACTION_RESPONSE = 2
int64 TAPDUSGENERIC_EVENT_REPORT_REQUEST = 3
int64 TAPDUSGENERIC_EVENT_REPORT_RESPONSE = 4
int64 TAPDUSGENERIC_SET_REQUEST = 5
int64 TAPDUSGENERIC_SET_RESPONSE = 6
int64 TAPDUSGENERIC_GET_REQUEST = 7
int64 TAPDUSGENERIC_GET_RESPONSE = 8
int64 TAPDUSGENERIC_INITIALISATION_REQUEST = 9
int64 TAPDUSGENERIC_INITIALISATION_RESPONSE = 10

# container 1
ActionRequest actionrequest

# container 2
ActionResponse actionresponse

# container 3
EventReportRequest eventreportrequest

# container 4
EventReportResponse eventreportresponse

# container 5
SetRequest setrequest

# container 6
SetResponse setresponse

# container 7
GetRequest getrequest

# container 8
GetResponse getresponse

# container 9
InitialisationRequest initialisationrequest

# container 10
InitialisationResponse initialisationresponse


#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Copyright: Patrizia Neubauer (https://github.com/patrizianeubauer)
# 



DateAndTime session_time

Provider session_service_provider

Int2 location_of_station

SessionLocation session_location

Int1 session_type

ResultOp session_result

Int1 session_tariff_class

Int1 session_claimed_class

PaymentFee session_fee

Provider session_contract_provider

int64[] session_type_of_contract #size(1..2)


int64 session_context_version
int64 SESSION_CONTEXT_VERSION_RANGE_MIN = 0
int64 SESSION_CONTEXT_VERSION_RANGE_MAX = 127

int64[] receipt_data_authenticator #size(1..4)


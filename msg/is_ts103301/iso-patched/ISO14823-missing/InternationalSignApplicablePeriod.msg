#
# Created by script of Patrizia Neubauer (https://github.com/patrizianeubauer)
# Date: Thu Jan 19 09:10:59 2023
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
#

# Added SEQUENCE OF adaptation

int64 year_range_start_year
int64 YEAR_RANGE_START_YEAR_RANGE_MIN = 2000
int64 YEAR_RANGE_START_YEAR_RANGE_MAX = 2127

int64 year_range_end_year
int64 YEAR_RANGE_END_YEAR_RANGE_MIN = 2000
int64 YEAR_RANGE_END_YEAR_RANGE_MAX = 2127
MonthDay date_range_start_month_day
MonthDay date_range_end_month_day

# Optional Field
bool repeating_period_day_types_present 0
RepeatingPeriodDayTypes repeating_period_day_types
HoursMinutes time_range_start_time
HoursMinutes time_range_end_time

# Optional Field
bool date_range_of_week_present 0
DayOfWeek date_range_of_week

# Optional Field
bool duration_hour_minute_present 0
HoursMinutes duration_hour_minute

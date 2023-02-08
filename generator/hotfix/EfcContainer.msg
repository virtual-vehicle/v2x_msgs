#
# Generated with ROS-Parser from vehicleCAPTAIN Toolbox (https://github.com/virtual-vehicle/vehicle_captain_asn1_parser)
# Patrizia Neubauer (https://github.com/patrizianeubauer)
#

# Added some choices & SEQUENCE OF adaptation

# CHOICE! - Choose exactly of the containers
int64 efc_container_container_select 0
int64 EFC_CONTAINER_NOTHING = 0
int64 EFC_CONTAINER_INTEGER = 1
int64 EFC_CONTAINER_BITSTRING = 2
int64 EFC_CONTAINER_OCTETSTRING = 3
int64 EFC_CONTAINER_UNIVERSAL_STRING = 4
int64 EFC_CONTAINER_BEACON_ID = 5
int64 EFC_CONTAINER_T_APDU = 6
int64 EFC_CONTAINER_DSRC_APPLICATION_ENTITY_ID = 7
int64 EFC_CONTAINER_DSRC_ASE_ID = 8
int64 EFC_CONTAINER_ATTR_ID_LIST = 9
int64 EFC_CONTAINER_ATTR_LIST = 10
int64 EFC_CONTAINER_BROADCAST_POOL = 11
int64 EFC_CONTAINER_DIRECTORY = 12
int64 EFC_CONTAINER_FILE = 13
int64 EFC_CONTAINER_FILE_TYPE = 14
int64 EFC_CONTAINER_RECORD = 15
int64 EFC_CONTAINER_TIME = 16
int64 EFC_CONTAINER_VECTOR = 17
int64 EFC_CONTAINER_GSTRQ = 18
int64 EFC_CONTAINER_GSTRS = 19
int64 EFC_CONTAINER_SSTRQ = 20
int64 EFC_CONTAINER_GINRQ = 21
int64 EFC_CONTAINER_GINRS = 22
int64 EFC_CONTAINER_SINRQ = 23
int64 EFC_CONTAINER_CHARQ = 24
int64 EFC_CONTAINER_CHARS = 25
int64 EFC_CONTAINER_CPPRQ = 26
int64 EFC_CONTAINER_SUBRQ = 27
int64 EFC_CONTAINER_ADDRQ = 28
int64 EFC_CONTAINER_DEBRQ = 29
int64 EFC_CONTAINER_DEBRS = 30
int64 EFC_CONTAINER_CRERQ = 31
int64 EFC_CONTAINER_CRERS = 32
int64 EFC_CONTAINER_EFCCONTEXT = 33
int64 EFC_CONTAINER_CONTSER = 34
int64 EFC_CONTAINER_CONTVAL = 35
int64 EFC_CONTAINER_CONTVEH = 36
int64 EFC_CONTAINER_CONTAUTH = 37
int64 EFC_CONTAINER_RECSPT = 38
int64 EFC_CONTAINER_SESSIONCLS = 39
int64 EFC_CONTAINER_RECSERVSERIALNO = 40
int64 EFC_CONTAINER_RECCONT = 41
int64 EFC_CONTAINER_REC_OBUID = 42
int64 EFC_CONTAINER_REC_ICCID = 43
int64 EFC_CONTAINER_RECTEXT = 44
int64 EFC_CONTAINER_RECAUTH = 45
int64 EFC_CONTAINER_RECDIST = 46
int64 EFC_CONTAINER_VEHLPN = 47
int64 EFC_CONTAINER_VEHID = 48
int64 EFC_CONTAINER_VEHCLASS = 49
int64 EFC_CONTAINER_VEHDIMS = 50
int64 EFC_CONTAINER_VEHAXLES = 51
int64 EFC_CONTAINER_VEHWTLIMS = 52
int64 EFC_CONTAINER_VEHWTLADEN = 53
int64 EFC_CONTAINER_VEHSPCHARS = 54
int64 EFC_CONTAINER_VEHAUTH = 55
int64 EFC_CONTAINER_EQU_OBUID = 56
int64 EFC_CONTAINER_EQU_ICCID = 57
int64 EFC_CONTAINER_EQUSTAT = 58
int64 EFC_CONTAINER_DVRCHARS = 59
int64 EFC_CONTAINER_PAYMBAL = 60
int64 EFC_CONTAINER_PAYMUNIT = 61
int64 EFC_CONTAINER_PAYSECDATA = 62
int64 EFC_CONTAINER_PAYMEANS = 63
int64 EFC_CONTAINER_RECDATA1 = 64
int64 EFC_CONTAINER_RECDATA2 = 65
int64 EFC_CONTAINER_VALOFCON = 66
int64 EFC_CONTAINER_RECFINPT = 67
int64 EFC_CONTAINER_SETMMIRQ = 68
int64 EFC_CONTAINER_AWL = 69
int64 EFC_CONTAINER_PACA = 70
int64 EFC_CONTAINER_ENG = 71
int64 EFC_CONTAINER_SL = 72
int64 EFC_CONTAINER_EEV = 73
int64 EFC_CONTAINER_DEV = 74
int64 EFC_CONTAINER_CO2EV = 75
int64 EFC_CONTAINER_VTD = 76
int64 EFC_CONTAINER_TLPN = 77
int64 EFC_CONTAINER_TCH = 78
int64 EFC_CONTAINER_ANP = 79

# container 1
int64 integer

# container 2
int64 bitstring

# container 3
int64[] octetstring

# container 4
string universal_string

# container 5
BeaconID beacon_id

# container 6
#TAPDUs t_apdu

# container 7
DSRCApplicationEntityID dsrc_application_entity_id

# container 8
DsrcEID dsrc_ase_id

# container 9
AttributeIdList attr_id_list

# container 10
AttributeList attr_list

# container 11
BroadcastPool broadcast_pool

# container 12
Directory directory

# container 13
File file

# container 14
FileType file_type

# container 15
Record record

# container 16
Time time

# container 17
int64[] vector #size(0..255)

# container 18
GetStampedRq gstrq

# container 19
GetStampedRs gstrs

# container 20
SetStampedRq sstrq

# container 21
GetInstanceRq ginrq

# container 22
GetInstanceRs ginrs

# container 23
SetInstanceRq sinrq

# container 24
ChannelRq charq

# container 25
ChannelRs chars

# container 26
CopyRq cpprq

# container 27
SubRq subrq

# container 28
AddRq addrq

# container 29
DebitRq debrq

# container 30
DebitRs debrs

# container 31
CreditRq crerq

# container 32
CreditRs crers

# container 33
EFCContextMark efccontext

# container 34
ContractSerialNumber contser

# container 35
ContractValidity contval

# container 36
ContractVehicle contveh

# container 37
ContractAuthenticator contauth

# container 38
ReceiptServicePart recspt

# container 39
SessionClass sessioncls

# container 40
ReceiptServiceSerialNumber recservserialno

# container 41
ReceiptContract reccont

# container 42
ReceiptOBUId rec_obuid

# container 43
ReceiptICCId rec_iccid

# container 44
ReceiptText rectext

# container 45
ReceiptAuthenticator recauth

# container 46
ReceiptDistance recdist

# container 47
LPN vehlpn

# container 48
CS5 vehid

# container 49
VehicleClass vehclass

# container 50
VehicleDimensions vehdims

# container 51
VehicleAxles vehaxles

# container 52
VehicleWeightLimits vehwtlims

# container 53
VehicleWeightLaden vehwtladen

# container 54
VehicleSpecificCharacteristics vehspchars

# container 55
VehicleAuthenticator vehauth

# container 56
EquipmentOBUId equ_obuid

# container 57
ICCId equ_iccid

# container 58
EquipmentStatus equstat

# container 59
DriverCharacteristics dvrchars

# container 60
PaymentMeansBalance paymbal

# container 61
PaymentMeansUnit paymunit

# container 62
PaymentSecurityData paysecdata

# container 63
PaymentMeans paymeans

# container 64
ReceiptData1 recdata1

# container 65
ReceiptData2 recdata2

# container 66
ValidityOfContract valofcon

# container 67
ReceiptFinancialPart recfinpt

# container 68
SetMMIRq setmmirq

# container 69
AxleWeightLimits awl

# container 70
PassengerCapacity paca

# container 71
Engine eng

# container 72
SoundLevel sl

# container 73
ExhaustEmissionValues eev

# container 74
DieselEmissionValues dev

# container 75
CO2EmissionValue co2ev

# container 76
VehicleTotalDistance vtd

# container 77
TrailerLicencePlateNumber tlpn

# container 78
TrailerCharacteristics tch

# container 79
ActualNumberOfPassengers anp
#include "Arduino.h"
#include "time.h"

#ifdef ONE_HOUR
#undef ONE_HOUR
#endif
#define ONE_HOUR 3600L //it is preferred to use const like
//const long one_hour = 3600;

#ifdef ONE_DAY
#undef ONE_DAY
#endif
#define ONE_DAY 86400L

#ifdef UNIX_OFFSET
#undef UNIX_OFFSET
#endif
#define UNIX_OFFSET 946684800L

#define GPS_OFFSET 630720000L // = UNIX_OFFSET - 315964800 (DIFF BETWEEN GPS and UNIX timestamps)

const uint32_t maxWait = 3000L;

const uint8_t SYNC_CHARS[] = {0xB5, 0x62};
const char NMEA_START = '$', NMEA_END = '*';

enum DebugLevel : uint8_t {
	DEBUG_LEVEL_ERROR,
	DEBUG_LEVEL_WARNING,
	DEBUG_LEVEL_NOTICE,
	DEBUG_LEVEL_TEST,
	DEBUG_LEVEL_DEBUG
};

//#define DEBUG_LEVEL_UBX DEBUG_LEVEL_ERROR //set debug level for UBX protocol; select the number from enum DebugLevel (see above)
//#define DEBUG_LEVEL_NMEA DEBUG_LEVEL_ERROR //set debug level for NMEA protocol; select the number from enum DebugLevel (see above)

#define MON_VER_EXTENSION_NUMBER 4

//Message Classes
#define UBX_NAV 1
#define UBX_RXM 2
#define UBX_INF 4
#define UBX_ACK 5
#define UBX_CFG 6
#define UBX_UPD 9
#define UBX_MON 0xA
#define UBX_AID 0xB
#define UBX_TIM 0xD
#define UBX_MGA 0x13
#define UBX_LOG 0x21
#define UBX_NMEA 0xF0
#define UBX_PUBX 0xF1


//Message IDs
#define ACK_NAK 0
#define ACK_ACK 1

#define CFG_PRT 0
#define CFG_MSG 1
#define CFG_INF 2
#define CFG_RST 4
#define CFG_RATE 8
#define CFG_CFG 9
#define CFG_RXM 0x11
#define CFG_SBAS 0x16
#define CFG_NMEA 0x17
#define CFG_ODO 0x1E
#define CFG_NAVX5 0x23
#define CFG_NAV5 0x24
#define CFG_TP5 0x31
#define CFG_PM2 0x3B
#define CFG_GNSS 0x3E
#define CFG_LOGFILTER 0x47
#define CFG_PWR 0x57
#define CFG_GEOFENCE 0x69
#define CFG_PMS 0x86
#define CFG_SLAS 0x8D
//#define CFG_BATCH 0x93 //not supported in protocol version < 23.01

#define MON_VER 0x4
#define MON_PATCH 0x27
#define MON_GNSS 0x28 

#define NAV_POSLLH 2
#define NAV_DOP 4
#define NAV_PVT 7
#define NAV_ODO 9
#define NAV_RESETODO 0x10
#define NAV_TIMEGPS 0x20
#define NAV_TIMEUTC 0x21
#define NAV_CLOCK 0x22
#define NAV_TIMEGLO 0x23
#define NAV_TIMEBDS 0x24
#define NAV_TIMEGAL 0x25
#define NAV_TIMELS 0x26
#define NAV_DGPS 0x31
#define NAV_SBAS 0x32
#define NAV_ORB 0x34
#define NAV_SAT 0x35
#define NAV_GEOFENCE 0x39
#define NAV_SLAS 0x42
#define NAV_EOE 0x61

#define TIM_TP 1
#define TIM_TM2 3

#define LOG_ERASE 3
#define LOG_STRING 4
#define LOG_CREATE 7
#define LOG_INFO 8
#define LOG_RETRIEVE 9
#define LOG_RETRIEVEPOS 0xB
#define LOG_RETRIEVESTRING 0xD
#define LOG_FINDTIME 0xE
#define LOG_RETRIEVEPOSEXTRA 0xF

#define INF_ERROR 0
#define INF_WARNING 1
#define INF_NOTICE 2
#define INF_TEST 3
#define INF_DEBUG 4

#define NMEA_GGA 0
#define NMEA_GLL 1
#define NMEA_GSA 2
#define NMEA_GSV 3
#define NMEA_RMC 4
#define NMEA_VTG 5
#define NMEA_GRS 6
#define NMEA_GST 7
#define NMEA_ZDA 8
#define NMEA_GBS 9
#define NMEA_DTM 0xA
#define NMEA_GNS 0xD
#define NMEA_VLW 0xF
#define NMEA_GPQ 0x40
#define NMEA_TXT 0x41
#define NMEA_GNQ 0x42
#define NMEA_GLQ 0x43
#define NMEA_GBQ 0x44

#define PUBX_POSITION 0
#define PUBX_SVSTATUS 3 
#define PUBX_TIME 4
#define PUBX_RATE 0x40
#define PUBX_CONFIG 0x41

enum Port : uint8_t {
	DDC,
	COM1,
	COM2,
	USB,
	SPI,
	RES
};

enum Protocol : uint8_t {
	UBX,
	NMEA
};

struct UbxAck {
	uint8_t classId;
	uint8_t messageId;
};

struct ValidTime {
	uint8_t validDate : 1;
	uint8_t validTime : 1;
	uint8_t fullyResolved : 1;
	uint8_t validMag : 1;
};

enum UtcSource : uint8_t {
	NO_INFO,
	CRL, //Communications Research Labratory
	NIST, //National Institute of Standards and Technology
	USNO, //U.S. Naval Observatory
	BIPM, //International Bureau of Weights and Measures
	EUROLAB, //European Laboratory
	SU, //Former Soviet Union
	NTSC, //National Time Service Center, China
	UNKNOWN_SOURCE = 15
};

struct ValidUtc {
	uint8_t validTOW : 1;
	uint8_t validWKN : 1;
	uint8_t validUTC : 1;
	uint8_t reserved : 1;
	uint8_t utcSource : 4; //enum UtcSource
};

struct Time {
	uint8_t hour; //0..23
	uint8_t minute; //0..59
	uint8_t second; //0..60
};

struct DateTime {
	uint16_t year;
	uint8_t month; //1..12
	uint8_t day; //1..31
	uint8_t hour; //0..23
	uint8_t minute; //0..59
	uint8_t second; //0..60
}; //7 bytes

struct NavTimeUtc {
	uint32_t iTOW;
	uint32_t tAcc;
	int32_t nano;
	DateTime dateTime;
	ValidUtc valid;
}; //20 bytes

enum FixType : uint8_t {
	NONE,
	DR,
	TWO_D,
	THREE_D,
	GNSS_DR,
	TIME_ONLY
};

enum PsmState : uint8_t {
	NOT_ACTIVE,
	ENABLED,
	ACQUISITION,
	TRACKING,
	POWER_OPTIMIZED_TRACKING,
	INACTIVE
};

enum CarrierSolution : uint8_t {
	NO_SOLUTION,
	FLOAT,
	FIXED
};

struct PvtFlags {
	uint8_t gnssFixOk : 1;
	uint8_t diffSoln : 1;
	uint8_t psmState : 3;
	uint8_t headingValid : 1;
	uint8_t carrSoln : 2;
};

//these flags should be ignored in protocol version <= 18
struct PvtFlags2 {
	uint8_t reserved : 5;
	uint8_t confirmedAvai : 1; //only supported in protocol version >= 19
	uint8_t confirmedDate : 1; //only valid if confirmedAvai is set
	uint8_t confirmedTime : 1; //only valid if confirmedAvai is set
};

struct SatInfoFlags {
	uint32_t signalQuality : 3;
	uint32_t svUsed : 1;
	uint32_t health : 2;
	uint32_t diffCorr : 1;
	uint32_t smoothed : 1;
	uint32_t orbitSource : 3;
	uint32_t ephAvail : 1;
	uint32_t almAvail : 1;
	uint32_t anoAvail : 1;
	uint32_t aopAvail : 1;
	uint32_t reserved : 1;
	uint32_t sbasCorrUsed : 1;
	uint32_t rtcmCorrUsed : 1;
	uint32_t slasCorrUsed : 1;
	uint32_t reserved2 : 1;
	uint32_t prCorrUsed : 1;
	uint32_t crCorrUsed : 1;
	uint32_t doCorrUsed : 1;
	uint32_t reserved3 : 9;
};

struct SatInfo {
  uint8_t gnssId;
  uint8_t svId;
  uint8_t cno;
  int8_t elev;
  int16_t azim;
  int16_t prRes;
  SatInfoFlags flags;
};

struct NavSat {
	uint32_t iTOW;
	uint8_t version;
	uint8_t numSvs;
	uint16_t reserved;
//	SatInfo * satInfo; //repeated structure x numSvs
};

struct MonVer {
	char swVersion[30];
	char hwVersion[10];
	char extensions[MON_VER_EXTENSION_NUMBER][30];
};

enum SignalQuality : uint8_t {
	NO_SIGNAL,
	SEARCHING_SIGNAL,
	SIGNAL_ACQUIRED,
	SIGNAL_DETECTED_BUT_UNUSABLE,
	CODE_LOCKED_TIME_SYNCED,
	CODE_CARRIER_LOCKED_TIME_SYNCED,
	CODE_CARRIER_LOCKED_TIME_SYNCED2,
	CODE_CARRIER_LOCKED_TIME_SYNCED3
};

enum Health : uint8_t {
	UNKNOWN,
	HEALTHY,
	UNHEALTHY
};

enum OrbitSource : uint8_t {
	NO_SOURCE,
	EPHEMERIS,
	ALMANAC,
	ASSIST_NOW_OFFLINE,
	ASSIST_NOW_AUTONOMOUS,
	OTHER,
	OTHER2,
	OTHER3
};

struct NavPvt
{
	uint32_t iTOW; //ms
	DateTime dateTime;
	ValidTime validTime;
	uint32_t tAcc; //ns
	int32_t nano; //ns
	FixType fixType;
	PvtFlags flags;
	PvtFlags2 flags2;
	uint8_t numSV;
	int32_t lon; //10^-7 deg
	int32_t lat; //10^-7 deg
	int32_t alt; //mm
	int32_t altMSL; //mm
	uint32_t hAcc; //mm
	uint32_t vAcc; //mm
	int32_t velN; //mm/s, when 3D low-pass filter is activated, velocity in North direction (see UBX-CFG-ODO, velLpGain)
	int32_t velE; //mm/s, when 3D low-pass filter is activated, velocity in East direction (see UBX-CFG-ODO, velLpGain)
	int32_t velD; //mm/s, when 3D low-pass filter is activated, velocity in Down direction (see UBX-CFG-ODO, velLpGain)
	int32_t SOG; //mm/s
	int32_t motionHeading; //10^-5 deg, when 2D low-pass filter is activated, speed over ground (< 8m/s) - (see UBX-CFG-ODO, cogLpGain)
	uint32_t sAcc; //mm/s
	uint32_t headAcc; //10^-5
	uint16_t pDOP; //10^-2
	uint32_t reserved1;
	uint16_t reserved2;
	int32_t heading; //10^-5 deg
	int16_t magDec; //10^-2 deg
	uint16_t magDecAcc; //10^-2 deg
}; //92 bytes

enum DynModel : uint8_t {
	PORTABLE,
	STATIONARY = 2,
	PEDESTRIAN,
	AUTOMOTIVE,
	SEA,
	AIRBORNE_1G,
	AIRBORNE_2G,
	AIRBORNE_4G,
	WATCH,
	MODEL_ERROR = 255
};

enum FixMode : uint8_t {
	TWO_D_ONLY = 1,
	THREE_D_ONLY,
	AUTO,
	FIX_MODE_ERROR = 255
};

enum UtcStandard : uint8_t {
	AUTOMATIC,
	GPS = 3,
	GLONASS = 6,
	BEIDOU = 7,
	UTC_ERROR = 255
};

struct CfgNavMask {
	uint16_t dyn : 1;
	uint16_t minElev : 1;
	uint16_t posFixMode : 1;
	uint16_t drLim : 1; //reserved
	uint16_t posMask : 1;
	uint16_t timeMask : 1;
	uint16_t staticHoldMask : 1;
	uint16_t dgpsMask : 1;
	uint16_t cnoThreshold : 1;
	uint16_t reserved : 1;
	uint16_t utcMask : 1;
	uint16_t unused : 5;
};

struct InfoMsgMask {
	uint8_t error : 1;
	uint8_t warning : 1;
	uint8_t notice : 1;
	uint8_t test : 1;
	uint8_t debug : 1;
	uint8_t reserved : 3;
};

struct CfgInfo {
	uint8_t protocolId;
	uint16_t reserved;
	uint8_t reserved2;
	InfoMsgMask mask[6];
};

struct CfgNav {
	CfgNavMask mask;	
	DynModel dynModel;
	FixMode fixMode;
	int32_t fixedAlt; //m for 2D mode
	uint32_t fixedAltVar; //m^2 for 2D mode, must be > 0
	int8_t minElev; //deg
	uint8_t drLimit; //reserved
	uint16_t pDOP; //10^-1
	uint16_t tDOP; //10^-1
	uint16_t pAcc; //m
	uint16_t tAcc; //ns or us?
	uint8_t staticHoldThreshold; //cm/s - the speed, below which the static mode is activated
	uint8_t dgnssTimeout; //s
	uint8_t cnoThresholdNumSVs; //Number of satellites required to have cno above cnoThreshold for a fix to be attempted
	uint8_t cnoThreshold; //dBHz
	uint16_t reserved;
	uint16_t staticHoldMaxDistance; //m - if this distance exceeded, the static mode gets deactivated
	UtcStandard utcStandard;
	uint32_t reserved2;
	uint8_t reserved3;
};//36 bytes

struct FixedAlt {
	int32_t fixedAlt; //m for 2D mode
	uint32_t fixedAltVar; //m^2 for 2D mode
};

struct DOP {
	uint16_t pDOP;
	uint16_t tDOP;
};

struct Accuracy {
	uint16_t pAcc; //m
	uint16_t tAcc; //ns or us?
};

struct CnoThreshold {
	uint8_t cnoThreshold; //dBHz
	uint8_t cnoThresholdNumSVs; //Number of satellites required to have cno above cnoThresh for a fix to be attempted
};

struct StaticHoldThresholds {
	uint8_t staticHoldThreshold; //cm/s - the speed, below which the static mode is activated
	uint16_t staticHoldMaxDistance; //m - if this distance exceeded, the static mode gets deactivated
};

struct CfgLogFilterFlags {
	uint8_t recordingEnabled : 1;
	uint8_t psmOncePerWakeUpEnabled : 1; //enable recording only one single position per PSM on/off mode wake-up period
	uint8_t applyAllFilterSettings : 1;
};

struct CfgLogFilter {
	uint8_t version;
	CfgLogFilterFlags flags;
	uint16_t minInterval; //s, 0 - not set (ignored). This is only applied in combination with the speed and/or position thresholds. 
						  //If both minInterval and timeThreshold are set, minInterval must be less than or equal to timeThreshold.
	uint16_t timeThreshold; //s, 0 - not set (ignored)
	uint16_t speedThreshold; //m/s, 0 - not set (ignored)
	uint16_t positionThreshold; //m, 0 - not set (ignored)
};

enum LogSize : uint8_t {
	SAFE_SIZE,
	MIN_SIZE,
	USER_DEFINED_SIZE
};

struct LogCreate {
	uint8_t version;
	uint8_t circular; // 1 - circular, 0 - not
	uint8_t reserved;
	LogSize logSize; //0 (maximum safe size): Ensures that logging will not be interrupted and enough space will be left
					 //available for all other uses of the filestore 
					 //1 (minimum size): 
					 //2 (user defined): See 'userDefinedSize' below
	uint32_t userDefinedSize; //bytes
};

struct LogFindTimeRequest {
	uint8_t version; // 0
	uint8_t type; // 0 - for request
	uint16_t reserved;
	DateTime dateTime;
	uint8_t reserved2;
};

struct LogFindTimeResponse {
	uint8_t version; // 1
	uint8_t type; // 1 - for response
	uint16_t reserved;
	uint32_t index; //0-based index, If 0xFFFFFFFF, no log entry found with time <= given time
};

struct LogInfoFlags {
	uint8_t reserved : 3;
	uint8_t enabled : 1;
	uint8_t inactive : 1;
	uint8_t circular : 1;
	uint8_t reserved2 : 2;
};

struct LogInfo {
	uint8_t version; //1
	uint16_t reserved;
	uint8_t reserved2;
	uint32_t fileStoreCapacity; //bytes
	uint32_t reserved3;
	uint32_t reserved4;
	uint32_t currentMaxLogSize; //bytes
	uint32_t currentLogSize; //bytes
	uint32_t entryCount;
	DateTime oldestDateTime;
	uint8_t reserved5;
	DateTime newestDateTime;
	uint8_t reserved6;
	LogInfoFlags flags;
	uint16_t reserved7;
	uint8_t reserved8;
};

struct LogRetrievePos {
	uint32_t index;
	int32_t lon; // 10^-7 deg
	int32_t lat; // 10^-7 deg
	int32_t altMSL; //mm
	uint32_t hAcc; //mm
	uint32_t gSpeed; //mm/s - ground speed 2-D
	uint32_t heading; //10^-5 deg
	uint8_t version; //0
	FixType fixType; 
	DateTime dateTime;
	uint8_t reserved;
	uint8_t numSV;
	uint8_t reserved2;
}; //40 bytes

struct LogRetrievePosExtra {
	uint32_t index;
	uint8_t version; //0
	uint8_t reserved;
	DateTime dateTime;
	uint16_t reserved2;
	uint8_t reserved3;
	uint32_t distance;
	uint32_t reserved4;
	uint32_t reserved5;
	uint32_t reserved6;
}; // 32 bytes

struct LogRetrieveString {
	uint32_t index;
	uint8_t version; //0
	uint8_t reserved;
	DateTime dateTime;
	uint8_t reserved2;
	uint16_t byteCount; // < 256
	//uint8_t * bytes; 
}; //16 bytes

struct TxReady {
	uint16_t en : 1;
	uint16_t pol : 1;
	uint16_t pin : 5;
	uint16_t thres : 9;
};

enum BaudRate : uint32_t {
	BAUD_RATE_4800 = 4800,
	BAUD_RATE_9600 = 9600,
	BAUD_RATE_14400 = 14400,
	BAUD_RATE_19200 = 19200,
	BAUD_RATE_38400 = 38400,
	BAUD_RATE_57600 = 57600,
	BAUD_RATE_115200 = 115200,
	BAUD_RATE_230400 = 230400,
	BAUD_RATE_460800 = 460800
};

struct InProtoMask {
	uint16_t inUbx : 1;
	uint16_t inNmea : 1;
	uint16_t inRtcm : 1;
	uint16_t reserved : 13;
};

struct OutProtoMask {
	uint16_t outUbx : 1;
	uint16_t outNmea : 1;
	uint16_t reserved : 14;
};

struct PrtFlags {
	uint16_t reserverd : 1;
	uint16_t extendedTimeout : 1;
	uint16_t reserved2 : 14;
};

enum CharLen : uint8_t {
	FIVE_BIT,
	SIX_BIT, 
	SEVEN_BIT,
	EIGHT_BIT
};

enum Parity :uint8_t {
	PARITY_EVEN,
	PARITY_ODD,
	NO_PARITY = 4,
	NO_PARITY2 = 5
};

enum StopBits : uint8_t {
	ONE_STOP_BIT,
	ONE_AND_HALF_STOP_BIT,
	TWO_STOP_BIT,
	HALF_STOP_BIT
};

struct PrtMode {
	uint32_t reserved : 6;
	uint32_t charLen : 2;
	uint32_t reserved2 : 1;
	uint32_t parity : 3;
	uint32_t nStopBits : 2;
	uint32_t reserved3 : 18;
};

struct CfgPrt {
	Port portId;
	uint8_t reserved1;
	TxReady txReady;
	PrtMode mode;
	BaudRate baudRate;
	InProtoMask inProtoMask;
	OutProtoMask outProtoMask;
	PrtFlags flags;
	uint16_t reserved2;
};

enum Error : uint8_t {
	NO_ERROR,
	CHECKSUM_ERROR,
	OUT_OF_MEMORY
};

struct ConfMask {
	uint32_t ioPort : 1; //Communications port settings. Modifying this sub-section results in an IO system reset. 
	                     //Because of this undefined data may be output for a short period of time after receiving the message.
	uint32_t msgConf : 1; //Message configuration
	uint32_t infMsg : 1; //INF message configuration
	uint32_t navConf : 1; //Navigation configuration
	uint32_t rxmConf : 1; //Receiver Manager configuration
	uint32_t reserved : 3; //must be set to 0!
	uint32_t senConf : 1; //Sensor interface configuration (not supported in protocol versions less than 19)
	uint32_t rinvConf : 1; //Remote inventory configuration
	uint32_t antConf : 1; //Antenna configuration
	uint32_t logConf : 1; //Logging configuration
	uint32_t ftsConf : 1; //FTS configuration. Only applicable to the FTS product variant.
	uint32_t reserved2 : 19; //must be set to 0!
};

struct Device {
	uint8_t devBBR : 1; //Battery backed RAM
	uint8_t devFlash : 1; //Flash
	uint8_t devEEPROM : 1; //EEPROM
	uint8_t reserved : 1;
	uint8_t devSpiFlash : 1; //SPI Flash
	uint8_t reserved2 : 3;
};

struct ConfigAllDevs {
	ConfMask clearMask;
	ConfMask saveMask;
	ConfMask loadMask;
};

struct ConfigDev {
	ConfMask clearMask;
	ConfMask saveMask;
	ConfMask loadMask;
	Device device;
};

enum ConfigType : uint8_t {
	DEFAULT_CONFIG,
	SAVE_CONFIG,
	LOAD_CONFIG
};

struct CfgMsg {
	uint8_t msgClass;
	uint8_t msgId;
	uint8_t rate[6]; //array index is Port enum (see above)
};

struct CfgMsgCOM1 {
	uint8_t msgClass;
	uint8_t msgId;
	uint8_t rate; //sets rate for the current port (COM1)
};
 
 enum PowerModes : uint8_t {
	FULL_POWER_MODE = 0,
	BALANCED_POWER_MODE,
	INTERVAL_POWER_MODE,
	AGGRESSIVE_1HZ_POWER_MODE,
	AGGRESSIVE_2HZ_POWER_MODE,
	AGGRESSIVE_4HZ_POWER_MODE,
	INVALID_POWER_MODE = 0xFF
};

struct PowerMode {
	uint8_t version;
	PowerModes powerMode;
	uint16_t period; // must be > 5s, recommeded min 10s - only for INTERVAL_POWER_MODE, otherwise must be 0!
	uint16_t onTime; // must be smaller period
	uint16_t reserved;
};

enum OdoProfile : uint8_t {
	RUNNING,
	CYCLING,
	SWIMMING,
	DRIVING,
	CUSTOM
};

struct OdoFlags {
	uint8_t ODOenabled : 1;
	uint8_t COGenabled : 1; //low speed COG filter
	uint8_t outputLPvelocity : 1; //output low pass filtered velocity for speed < 5m/s;
	uint8_t outputLPcog : 1; //output low pass filtered heading, aka 2D heading of motion in UBX-NAV-PVT for speed < 8m/s
};

struct ODOCfg {
	uint8_t version;
	uint16_t reserved1;
	uint8_t reserved2;
	OdoFlags flags;
	OdoProfile odoProfile;
	uint32_t reserverd3;
	uint16_t reserverd4;
	uint8_t cogMaxSpeed; //m/s
	uint8_t cogMaxPosAccuracy; //m
	uint16_t reserved5;
	uint8_t velLPgain; //velocity low pass filter levels for speed < 5m/s; 0..255 where 0 is heavy filtering and 255 - weak
	uint8_t cogLPgain; //COG low pass filter levels at speed < 8m/s: 0..255
	uint16_t reserved6;
}; //20 bytes

struct GnssFlags {
	uint32_t enabled : 1;
	uint32_t reserved : 15;
	uint32_t sigCfgMask : 8; //When gnssId is 0 (GPS): 0x01 = GPS L1C/A, 0x10 = GPS L2C
							 //When gnssId is 1 (SBAS): 0x01 = SBAS L1C/A
							 //When gnssId is 2 (Galileo): 0x01 = Galileo E1 (not supported in protocol versions less than 18), 0x20 = Galileo E5b
							 //When gnssId is 3 (BeiDou): 0x01 = BeiDou B1I, 0x10 = BeiDou B2I
							 //When gnssId is 4 (IMES): 0x01 = IMES L1
							 //When gnssId is 5 (QZSS): 0x01 = QZSS L1C/A, 0x04 = QZSS L1S, 0x10 = QZSS L2C
							 //When gnssId is 6 (GLONASS): 0x01 = GLONASS L1, 0x10 = GLONASS L2
	uint32_t reserved2 : 8;
};

enum GnssId : uint8_t {
	GPS_ID,
	SBAS_ID,
	Galileo_ID,
	BeiDou_ID,
	IMES_ID,
	QZSS_ID,
	GLONASS_ID
};

struct GnssCfg {
	GnssId gnssId;
	uint8_t minTrkCh;
	uint8_t maxTrkCh;
	uint8_t reserved;
	GnssFlags flags;
};

struct GnssConf {
	uint8_t version;//0
	uint8_t numTrkChHw;
	uint8_t numTrkChUse;
	uint8_t numConfigBlocks;
	//followed by repeated blocks of GnssCfg (times numConfigBlocks)
};

struct MajorGnss {
	uint8_t Gps : 1;
	uint8_t Glonass : 1;
	uint8_t BeiDou : 1;
	uint8_t Galileo : 1;
};

struct GnssSupport {
	uint8_t version; //1
	MajorGnss supportedGnss;
	MajorGnss defaultGnss;
	MajorGnss enabledGnss;
	uint8_t simultaneous;
	uint16_t reserved;
	uint8_t reserved2;
};

enum TimeRef : uint16_t {
	UTC_TIME,
	GPS_TIME,
	GLONASS_TIME,
	BEIDOU_TIME,
	GALILEO_TIME
};

enum TimeRefGnss : uint8_t {
	GPS_TP_TIME,
	GLONASS_TP_TIME,
	BEIDOU_TP_TIME,
	UNKNOWN_TP_TIME = 15
};

struct NavRate {
	uint16_t rate; //ms - measurement rate (time between measurements: 1000ms = 1Hz)
	uint16_t navSolRate; //cycles - ratio: number of measurements to number of navigation solutions: max 127
	TimeRef timeRef;
};

struct TimePulseFlags {
	uint32_t active : 1;
	uint32_t lockGnssFreq : 1;
	uint32_t lockedOtherSet : 1;
	uint32_t isFreq : 1;
	uint32_t isLength : 1;
	uint32_t alignToTOW : 1; //Align pulse to top of a second (period time must be integer fraction of 1s). Also set 'lockGnssFreq' to use this feature.
	uint32_t polarity : 1; //pulse polarity: 0 - falling edge at top of second, 1 - rising edge at top of second
	uint32_t gridUtcGnss : 4; //timeGrid to use - same as TimeRef enum - This flag is only relevant if 'lockGnssFreq' and 'alignToTOW' are set.
							  //ensure that time source is the same as configured in UBX-CFG-GNSS
							  //ensure that UBX-CFG-NAV5 sets the UTC variant to Auto (default) if this bit is set to UTC!
							  //or match it with relevant UTC variant if it is set to GNSS time
	uint32_t syncMode : 3; //only relevant for FTS chips
	uint32_t reserved : 18;
};

struct TimePulse {
	uint8_t pulseId;
	uint8_t version; //1
	uint16_t reserved;
	int16_t antCableDelay; //ns
	int16_t rfGroupDelay; //ns
	uint32_t pulsePeriod; //Hz or us depending on isFreq bit
	uint32_t pulsePeriodLocked; //Hz or us when locked to GPS time, only used when lockedOtherSet is set
	uint32_t pulseLen; //us or 2^-32 - pulse length or duty cycles depending on isLength bit
	uint32_t pulseLenLocked; //us or 2^-32 - pulse length or duty cycles when locked to GPS time, only used when lockedOtherSet is set
	int32_t userConfigDelay; //ns - user-configurable time pulse delay. If they don't match the time error can be as much as 100s ns
	TimePulseFlags flags;
}; //32 bytes

//Receiver Autonomous Integrity Monitoring Algorithm (RAIM)
enum RAIM : uint8_t {
	RAIM_INFO_NOT_AVAILABLE,
	RAIM_NOT_ACTIVE,
	RAIM_ACTIVE
};

enum TimeBase : uint8_t {
	GNSS_TIME_BASE,
	UTC_TIME_BASE
};

struct TimeBaseFlags {
	uint8_t utcBase : 1; // TimeBase enum
	uint8_t utc : 1; //UTC available if true
	uint8_t raim : 2; // RAIM enum - Receiver Autonomous Integrity Monitoring Algorithm (RAIM)
};

struct TimeRefInfo {
	uint8_t timeRefGnss : 4; //TimeRefGnss enum, only valid when timeBase is GNSS (not UTC!)
	uint8_t utcSource : 4; //UtcSource enum, only valid when timeBase is UTC (not GNSS!)
};

struct TimeTP {
	uint32_t tow; //ms
	uint32_t subTOW; //ms 2^-32
	int32_t quantErr; //ps
	uint16_t weeks; //weeks since 6 Jan 1980
	TimeBaseFlags timeBase;
	TimeRefInfo timeRef;
};

struct Latitude {
	uint8_t deg;
	uint8_t min;
	float sec;
	char NS;
};

struct Longitude {
	uint8_t deg;
	uint8_t min;
	float sec;
	char EW;
};

struct GNRMC {
	DateTime dateTime;
	Latitude lat;
	Longitude lon;
	float SOG;
	float COG;
	float magVar;
	char magVarEW;
	char fixType; //Possible values for posMode: N = No fix, E = Estimated/Dead reckoning fix, 
				  //A = Autonomous GNSS fix, D = Differential GNSS fix, F = RTK float, R = RTK fixed
	char status; //V = Navigation receiver warning, A = Data valid
//	char navStatus; //V = Equipment is not providing navigational status information
};

enum FixTypeNMEA {
	NO_FIX_TYPE,
	GNSS_FIX_TYPE,
	DIFF_GNSS_FIX_TYPE,
	RTK_FIX_TYPE = 4,
	RTK_FLOAT_FIX_TYPE,
	DR_FIX_TYPE
};

struct GNGGA {
	DateTime dateTime;
	Latitude lat;
	Longitude lon;
	uint8_t fixType; //FixTypeNMEA enum
	uint8_t numSV;
	float hDOP;
	float alt; //m above mean sea level (MSL)
	float geoidEllipsoidDiff; //m, geoid is MSL
	uint32_t ageDiffCorr; //s, blank (0) when DGPS is not in use
	uint8_t diffCorrStationId; //id of DGPS station, blank (0) when not in use
};

struct GNGLL {
	DateTime dateTime;
	Latitude lat;
	Longitude lon;
	char fixType; //Possible values for posMode: N = No fix, E = Estimated/Dead reckoning fix, 
				  //A = Autonomous GNSS fix, D = Differential GNSS fix, F = RTK float, R = RTK fixed
	char status; //V = Data Invalid, A = Data Valid
};

struct GNVTG {
	float COGt;//true course
	float COGm;//magnetic course, supported in ADR 4.10 and above
	float SOGkt; //kt
	float SOGkmh; //km/h
	char fixType; //Possible values for posMode: N = No fix, E = Estimated/Dead reckoning fix, 
				  //A = Autonomous GNSS fix, D = Differential GNSS fix, F = RTK float, R = RTK fixed
};

enum FixTypeNMEA2 {
	FIX_NOT_AVAILABLE = 1,
	TWO_D_FIX,
	THREE_D_FIX
};

struct GNGSA {
	char opMode; //A - automatically set to operate in 2D or 3D mode, M - manually set
	uint8_t fixType; //FixTypeNMEA2
	uint8_t svId[12]; //No more than 12 SVs are reported in each message. The SV numbers (fields 'sv') are in the range of 1 to 32 for GPS satellites, 
					//and 33 to 64 for SBAS satellites (33 = SBAS PRN 120, 34 = SBAS PRN 121, and so on)
	float pDOP;
	float hDOP;
	float vDOP;
//	uint8_t gnssId; //NMEA defined GNSS System ID NMEA v4.1 and above only
};

struct SvAttr {
	uint8_t svId;
	uint8_t elev; //0..90
	uint16_t azim; //0..359
	uint8_t cno; //signal strength 0..99
};

struct GNGSV {
	String gnss;
	uint8_t totalMsg; //total number of these messages
	uint8_t msgNum; //this message number out of total
	uint8_t numSV; //number of SV in view
	SvAttr svAttr[4];
//	uint8_t gnssId; //NMEA defined GNSS System ID NMEA v4.1 and above only	
};

struct GNGST {
	DateTime dateTime;
	float rangeRms; //m, RMS value of the standard deviation of the ranges
	float stdMajor; //m, Standard deviation of semi-major axis - only supported in ADR >= 4.10
	float stdMinor; //m, Standard deviation of semi-minor axis - only supported in ADR >= 4.10
	float orient; //deg, Orientation of semi-major axis - only supported in ADR >= 4.10
	float stdLat; //m, Standard deviation of latitude error
	float stdLon; //m, Standard deviation of longitude error
	float stdAlt; //m, Standard deviation of altitude error
};

struct GNVLW {
	float twd; //nm, Total cumulative water distance, not output
	char twdUnit; //Fixed field: nautical miles ('N')
	float wd; //nm, Water distance since reset, not output
	char wdUnit; //Fixed field: nautical miles ('N')
	float tgd; //nm, Total cumulative ground distance
	char tgdUnit; //Fixed field: nautical miles ('N')
	float gd; //nm, Ground distance since reset
	char gdUnit; //Fixed field: nautical miles ('N')
};

struct GNGNS {
	DateTime dateTime;
	Latitude lat;
	Longitude lon;
	char fixType[2]; //fix type, First character for GPS, second character for GLONASS
	uint8_t numSV; //Number of satellites used (range: 0-99)
	float hDOP; //Horizontal Dilution of Precision
	float alt; //m, Altitude above mean sea level
	float geoidEllipsoidDiff; //m, Geoid separation: difference between ellipsoid and mean sea level
	uint32_t ageDiffCorr; //s, Age of differential corrections (blank when DGPS is not used)
	uint8_t diffCorrStationId; //ID of station providing differential corrections (blank when DGPS is not used)
//	char navStatus; //Navigational status indicator (V = Equipment is not providing navigational status information) NMEA v4.1 and above only
};

struct GNZDA {
	DateTime dateTime;
	int8_t utcOffsetHours; //Local time zone hours (fixed to 00)
	uint8_t utcOffsetMinutes; //Local time zone minutes (fixed to 00)
};

struct GNTXT {
	uint8_t numMsg; //Total number of messages in this transmission, 01..99
	uint8_t msgNum; //Message number in this transmission, range 01..xx
	uint8_t msgType; //Text identifier, u-blox receivers specify the type of the message with this number.
					 //00: Error, 01 : Warning, 02 : Notice, 07 : User
	String text; //Any ASCII text
};

struct GBS {
	DateTime dateTime; //UTC time only
	float errLat; //m, Expected error in latitude
	float errLon; //m, Expected error in longitude
	float errAlt; //m, Expected error in altitude
	uint8_t svId; //Satellite ID of most likely failed satellite
	float prob; //Probability of missed detection, not supported (empty)
	float bias; //m, Estimate on most likely failed satellite (a priori residual)
	float stddev; //m, Standard deviation of estimated bias
	uint8_t systemId; //NMEA defined GNSS System ID, NMEA v4.1 and above only, see GnssId enum
	uint8_t signalId; //NMEA defined GNSS Signal ID (0 = All signals) NMEA v4.1 and above only
};

struct DTM {
	String datum; //Local datum code: W84 = WGS84, 999 = user defined
	String subDatum; //A null field
	float lat; //minutes, offset in Latitude
	char NS; //North-South
	float lon; //minutes, offset in Longitude
	char EW; //East-West
	float alt; //m, offset in altitude
	String refDatum; //Reference datum code (always W84 = WGS 84)
};

struct GRS {
	DateTime dateTime; //UTC time only
	uint8_t mode; //Mode (see table below), u-blox receivers will always output Mode 1 residuals
	float residual[12]; //m, Range residuals for SVs used in navigation. The SV order matches the order from the GSA sentence.
	uint8_t systemId; //NMEA defined GNSS System ID, NMEA v4.1 and above only, see GnssId enum
	uint8_t signalId; //NMEA defined GNSS Signal ID (0 = All signals) NMEA v4.1 and above only
};

struct PubxPosition {
	DateTime dateTime;
	Latitude lat;
	Longitude lon;
	float alt; //Altitude above user datum ellipsoid.
	char fixType[3];
	float hAcc;
	float vAcc;
	float sog; //km/h
	uint16_t cog;
	float vVel; //m/s - Vertical velocity (positive downwards)
	uint32_t ageDiffCorr;
	float hDOP;
	float vDOP;
	float tDOP;
	uint8_t numSV;
	bool drUsed;
};

struct PubxTime {
	DateTime dateTime;
	uint32_t utcTow; //s
	uint32_t utcWeek;
	uint8_t leapSec; //s
	char leapSecSrc; //D - firmware, S - satellite
	uint32_t clkBias; //ns, Receiver clock bias
	uint32_t clkDrift; //ns/s, Receiver clock drift
	uint32_t tpGran; //ns, Time Pulse Granularity, The quantization error of the TIMEPULSE pin
};

struct PubxSvStatus {
	uint16_t svId;
	char svStatus; //- Not used, U - Used in solution, e - Ephemeris available, but not used for navigation
	uint16_t azim;
	uint8_t elev;
	uint8_t cno;
	uint8_t lockTime; //Satellite carrier lock time (range: 0-64)
					  //0: code lock only, 64: lock for 64 seconds or more
};

enum PosOdoStr : uint8_t {
	POSITION_LOG_RECORD,
	ODO_LOG_RECORD,
	STRING_LOG_RECORD,
	LOG_RECORD_TIMEOUT = 255
};

//gnss chip clock metrics
struct NavClock {
	uint32_t iTOW; //ms
	int32_t clkB; //gnss chip clock bias, ns
	int32_t clkD; //gnss chip clock drift, ns/s
	uint32_t tAcc; //gnss chip time accuracy, ns
	uint32_t fAcc; //gnss chip frequency accuracy, ps/s
};

struct NavDGPS {
	uint32_t iTOW;
	int32_t age; //ms
	int16_t baseId; //DGPS base station identifier
	int16_t baseHealth;
	uint8_t numCh; //number of channels for which correction data is following
	uint8_t status; //0x00: none, 0x01: PR+PRR correction
	uint16_t reserved;
	//repeated (numCh times) block of DGPSCorrData
};

struct DGPSCorrDataFlags {
	uint8_t channel : 4; //channels greater than 15 are displayed as 15
	uint8_t used : 1;
	uint8_t reserved : 3;
};

struct DGPSCorrData {
	uint8_t svid;
	DGPSCorrDataFlags flags;
	uint16_t ageC; //ms
	float prc; //Pseudorange correction, m
	float prrc; //Pseudorange rate correction, m/s
};

struct NavDOP {
	uint32_t iTOW;
	uint16_t gDOP; //Geometric DOP
	uint16_t pDOP; //Position DOP
	uint16_t tDOP; //time DOP
	uint16_t vDOP; //vertical DOP
	uint16_t hDOP; //horizontal DOP
	uint16_t nDOP; //northing DOP
	uint16_t eDOP; //easting DOP
};

enum GeofenceState : uint8_t {
	UKNOWN_GEOFENCE_STATE,
	INSIDE_GEOFENCE_STATE,
	OUTSIDE_GEOFENCE_STATE
};

struct NavGeofence {
	uint32_t iTOW;
	uint8_t version; //0
	uint8_t status; //0 - n/a or not reliable, 1 - active
	uint8_t numFences; //number of fences
	GeofenceState combState; //combined state
	//followed by repeated block (times numFences) of GeofenceState enum + uint8_t reserved byte
};

struct NavODO {
	uint8_t version;//0
	uint16_t reserved;
	uint8_t reserved2;
	uint32_t iTOW;
	uint32_t distance; //ground distance since last reset, m
	uint32_t totalDistance; //total ground distance, m
	uint32_t distanceStd; //m - ground distance accuracy (1-sigma)
};

struct GeoFences {
	uint8_t version; //0
	uint8_t numFences; //number of fences
	uint8_t confidenceLevel; //Required confidence level for state evaluation. This value times the position's standard
							 //deviation (sigma) defines the confidence band. 0=no confidence required, 1=68%, 2=95%, 3=99.7% etc.
	uint8_t reserved;
	uint8_t pioEnabled; //1 = Enable PIO combined fence state output, 0 = disable
	uint8_t pinPolarity; //PIO pin polarity. 0 = Low means inside, 1 = Low means outside. Unknown state is always high.
	uint8_t pin; //PIO pin number
	uint8_t reserved2;
	//followed by repeated block of GeoFence (times numFences)
};

struct GeoFence {
	int32_t lat; //deg x 10^-7
	int32_t lon; //deg x 10^-7
	uint32_t radius; //cm
};

struct CfgGeofences {
	GeoFences geoFences;
	GeoFence geoFence;
};

struct CfgNmeaFilter {
	uint8_t failedFix : 1; //Enable position output for failed or invalid fixes
	uint8_t invalidFix : 1; //Enable position output for invalid fixes
	uint8_t invalidTime : 1; //Enable time output for invalid times
	uint8_t invalidDate : 1; //Enable date output for invalid dates
	uint8_t gpsOnly : 1; //Restrict output to GPS satellites only
	uint8_t invalidCog : 1; //Enable COG output even if COG is frozen
	uint8_t reserved : 2;
};

struct CfgNmeaFlags {
	uint8_t compat : 1; //enable compatibility mode. 
	                    //This might be needed for certain applications when customer's NMEA parser expects 
						// a fixed number of digits in position coordinates
	uint8_t consider : 1; //enable considering mode
	uint8_t limit82 : 1; //enable strict limit to 82 characters maximum
	uint8_t highPrecision : 1; //enable high precision mode.
							   //This flag cannot be set in conjunction with either Compatibility Mode or Limit82 Mode.
							   //(not supported in protocol versions less than 20.01)
	uint8_t reserved : 4;
};

struct CfgNmeaGnss {
	uint32_t disableGps : 1;
	uint32_t disableSbas : 1;
	uint32_t reserved : 2;
	uint32_t disableQzss : 1;
	uint32_t disableGlonass : 1;
	uint32_t disableBeidou : 1;
	uint32_t reserved2 : 25;
};

enum CfgNmeaTalkerId : uint8_t {
	DEFAULT_TALKER_ID, //Uses GNSS assignment of the receiver's channels (see UBX-CFG-GNSS)
	GP_TALKER_ID, //GPS, SBAS, QZSS
	GL_TALKER_ID, //GLONASS
	GN_TALKER_ID, //Any combination of GNSS
	GA_TALKER_ID, //Galileo
	GB_TALKER_ID //BeiDou
};

struct CfgNmea {
	CfgNmeaFilter filter;
	uint8_t nmeaVersion; //0x41: NMEA version 4.1, 0x40: NMEA version 4.0, 0x23: NMEA version 2.3, 0x21: NMEA version 2.1
	uint8_t maxSV; //Maximum Number of SVs to report per TalkerId. 0 - unlimited, 8 SVs, 12 SVs, 16 SVs
	CfgNmeaFlags flags;
	CfgNmeaGnss gnssFilter;
	uint8_t displayNonNmeaSVs; //Configures the display of satellites that do not have an NMEA-defined value.
							   //Note: this does not apply to satellites with an unknown ID.
							   //0: do not display, 1: Use proprietary numbering
	CfgNmeaTalkerId mainTalkerId;
	uint8_t gsvTalkerId; //0 - Use GNSS specific Talker ID (as defined by NMEA), 1 - Use the main Talker ID
	uint8_t version; //1
	char dbsTalkerId[3]; //Sets the two characters that should be used for the BeiDou Talker ID
						 //If these are set to zero, the default BeiDou TalkerId will be used
	uint32_t reserved;
	uint16_t reserved2;
};

enum Visibility : uint8_t {
	UNKNOWN_VISIBILITY,
	BELOW_HIRIZON,
	ABOVE_HORIZON,
	ABOVE_ELEVATION_THRESHOLD
};

struct SvFlags {
	uint8_t health : 2; // HEALTHY enum
	uint8_t visibility : 2; // Visibility enum
	uint8_t reserved : 4;
};

struct OrbFlags {
	uint8_t Usability : 5; //31 - usability period is unknown, 
	                         //30 - usability period is > 450 (min for ephFlags, days for almFlags and otherFlags), 
							 //30 > n > 0 - between (n - 1)*15 and n*15, 
							 //0 - cannot be used
	uint8_t Source : 3; //0 - N/A, 1 - gnss (assist now offline for otherFlags), 2 - external aiding (assist now automomous for otherFlags), 3-7 - other 
};

struct OrbData {
	uint8_t gnssId;
	uint8_t svId;
	SvFlags svFlags;
	OrbFlags ephFlags;
	OrbFlags almFlags;
	OrbFlags otherFlags;
};

struct NavOrb {
	uint32_t iTOW;
	uint8_t version; //1
	uint8_t numSv;
	uint16_t reserved;
	//followed by repeated block of OrbData (x numSv)
};

struct NavPosLlh {
	uint32_t iTOW;
	int32_t lon;
	int32_t lat;
	int32_t alt;
	int32_t altMSL;
	uint32_t hAcc;
	uint32_t vAcc;
};

struct SbasServices {
	uint8_t ranging : 1;
	uint8_t corrections : 1;
	uint8_t integrity : 1;
	uint8_t testMode : 1;
	uint8_t reserved : 4;
};

enum SbasSystems : uint8_t {
	WAAS,
	EGNOS,
	MSAS,
	GAGAN,
	GPS_SYS = 16
};

struct SbasSv {
	uint8_t svId;
	uint8_t flags;
	uint8_t status;
	SbasSystems sys;
	SbasServices services;
	uint8_t reserved;
	int16_t prc; //pseudo range correction in cm
	uint16_t reserved2;
	int16_t ic; //ionospheric correction in cm
};

struct NavSbas {
	uint32_t iTOW;
	uint8_t geo; //PRN Number of the GEO where correction and integrity data is used from
	uint8_t mode; // 0 - disabled, 1 - enabled integrity, 3 - enabled test mode
	int8_t sys; //-1 -unknown, 0 - WAAS, 1 - EGNOS, 2 - MSAS, 3 - GAGAN, 16 - GPS
	SbasServices services;
	uint8_t cnt;//number of repeated blocks following
	uint16_t reserved;
	uint8_t reserved2;
	//repeated blocks of SbasSv (x cnt)
};

struct NavSlasFlags {
	uint8_t gmsAvailable : 1; //ground station available
	uint8_t qzssSvAvailable : 1; //SV available
	uint8_t testMode : 1;
};

struct SlasSv {
	uint8_t gnssId;
	uint8_t svId;
	uint32_t reserved;
	int16_t prc; //pseudo range correction in cm
};

struct NavSlas {
	uint32_t iTOW;
	uint8_t version;//00
	int32_t gmsLon;//10^-3 deg, Longitude of the used ground monitoring station
	int32_t gmsLat;//10^-3 deg, Latitude of the used ground monitoring station
	uint8_t gmsCode; // http://qzss.go.jp/en/
	uint8_t gzssSvId;
	NavSlasFlags flags;
	uint8_t cnt; //number of repeated blocks
	//repeated SlasSv blocks (x cnt)
};

struct TimeFlags {
	uint8_t towValid : 1;
	uint8_t weekValid : 1;
	uint8_t leapSecValid : 1;
	uint8_t reserved : 5;
};

struct NavTimeBdsGal {
	uint32_t iTOW; //ms, GPS time of the week
	uint32_t tow; //s, BeiDou or Galileo time of the week
	int32_t fTow; //ns, fractional seconds
	int16_t weeks; // BeiDou or Galileo week number of nav epoch
	int8_t leapSec; //s, leap seconds
	TimeFlags flags;
	uint32_t tAcc; //ns
};

struct NavTimeGps {
	uint32_t iTOW; //ms, GPS time of the week
	int32_t fTow; //ns, fractional seconds
	int16_t weeks; // BeiDou or Galileo week number of nav epoch
	int8_t leapSec; //s, leap seconds
	TimeFlags flags;
	uint32_t tAcc; //ns
};

struct TimeGloFlags {
	uint8_t todValid : 1; //both tod and fTod are valid
	uint8_t dateValid : 1; //both days and year are valid
	uint8_t reserved : 6;
};

struct NavTimeGlo {
	uint32_t iTOW; //ms, GPS time of the week
	uint32_t tod; //s, Glonass time of the day
	int32_t fTod; //ns, fractional seconds
	uint16_t day;//Current date (range: 1-1461), starting at 1 from the 1st Jan of the year indicated by year and
				  //ending at 1461 at the 31st Dec of the third year after that indicated by year
	uint8_t year;//1=1996, 2=2000, 3=2004, etc
	TimeGloFlags flags;
	uint32_t tAcc; //ns
};

enum TimeLsSource : uint8_t {
	DEFAULT_LSS,
	GPS_GLONASS_DIFF,
	GPS_LSS,
	SBAS_LSS,
	BEIDOU_LSS,
	GALILEO_LSS,
	AIDED_DATA,
	CONFIGURED_LSS,
	UNKNOWN_LSS = 255
};

enum TimeLsChangeSource : uint8_t {
	NO_SOURCE_LSS,
	GPS_CHANGE_LSS,
	SBAS_CHANGE_LSS,
	BEIDOU_CHANGE_LSS,
	GALILEO_CHANGE_LSS,
	GLONAS_CHANGE_LSS
};

struct NavTimeLsFlags {
	uint8_t validCurrLs : 1;
	uint8_t validTimeToLs : 1;
};

struct NavTimeLs {
	uint32_t iTOW;
	uint8_t version; //0
	uint16_t reserved;
	uint8_t reserved2;
	TimeLsSource source;
	int8_t leapSec; //Current number of leap seconds since start of GPS time (Jan 6, 1980). It reflects how much
					//GPS time is ahead of UTC time. Galileo number of leap seconds is the same as GPS. BeiDou
					//number of leap seconds is 14 less than GPS. GLONASS follows UTC time, so no leap seconds
	TimeLsChangeSource changeSource;
	int8_t lsChange; //s, Future leap second change if one is scheduled. +1 = positive leap second, -1 = negative leap
					 //second, 0 = no future leap second event scheduled or no information available.
	int32_t timeToLs; //s, Number of seconds until the next leap second event, or from the last leap second event if no
						   //future event scheduled. If > 0 event is in the future, = 0 event is now, < 0 event is in the
						   //past. Valid only if validTimeToLsEvent = 1
	uint16_t gpsWeek; //GPS week number (WN) of the next leap second event or the last one if no future event
					//scheduled. Valid only if validTimeToLsEvent = 1
	uint16_t gpsDay; //GPS day of week number (DN) for the next leap second event or the last one if no future event
					 //scheduled. Valid only if validTimeToLsEvent = 1. (GPS and Galileo DN: from 1 = Sun to 7 = Sat.
					 //BeiDou DN: from 0 = Sun to 6 = Sat.)
	uint16_t reserved3;
	uint8_t reserved4;
	NavTimeLsFlags flags;
};

struct CfgPmFlags {
	uint32_t reserved : 4;
	uint32_t extintSel : 1; //EXTINT pin select: 0 - EXTINT0, 1 - EXTINT1
	uint32_t extintWake : 1; //EXTINT pin control: 0 - disabled, 1 - enabled, keep receiver awake as long as selected EXTINT pin is 'high'
	uint32_t extintBackup : 1; //EXTINT pin control: EXTINT Pin Control: 0 - disabled, 1 - enabled, force receiver into BACKUP mode when selected EXTINT pin is 'low'
	uint32_t extintInactivity : 1; //EXTINT Pin control: 0 - disabled, 1 - enabled, force backup in case EXTINT Pin is inactive for time longer than extintInactivity
	uint32_t limitPeakCurr : 2; //Limit Peak Current: 00 - disabled, 01 - enabled, peak current is limited, 10 - reserved, 11 - reserved
	uint32_t waitTimeFix : 1; //Wait for Timefix (see waitTimeFix): 0 - wait for normal fix ok before starting on time, 1 - wait for time fix ok before starting on time
	uint32_t updateRTC : 1; //Update Real Time Clock (see updateRTC): 
							//0 - Do not wake up to update RTC. RTC is updated during normal on-time.
							//1 - Update RTC. The receiver adds extra wake-up cycles to update the RTC.
	uint32_t updateEph : 1; //Update Ephemeris: 0 - Do not wake up to update Ephemeris data
	                        //1 - Update Ephemeris. The receiver adds extra wake-up cycles to update the Ephemeris data
	uint32_t reserved2 : 3;
	uint32_t doNotEnterOff : 1; //Behavior of receiver in case of no fix
								//0 - receiver enters (Inactive) Awaiting Next Search state
								//1 - receiver does not enter (Inactive) Awaiting Next Search state but keeps trying to acquire a fix instead
	uint32_t mode : 2; //Mode of operation: 00 ON/OFF operation (PSMOO), 01 - Cyclic tracking operation (PSMCT), 10 - reserved, 11 - reserved
	uint32_t reserved3 : 13;
};

struct CfgPm {
	uint8_t version;//2
	uint8_t reserved;
	uint8_t maxStartupStateDuration;//s, 0 - disabled
	uint8_t reserved2;
	CfgPmFlags flags;
	uint32_t updatePeriod; //ms, Position update period. If 0 - the receiver will never retry a fix and wait for external event
	uint32_t searchPeriod; //ms, Acquisition retry period if previously failed. If set to 0, the receiver will never retry a startup
	uint32_t gridOffset; //ms, Grid offset relative to GPS start of week
	uint16_t onTime; //s, Time to stay in Tracking state
	uint16_t minAcqTime; //s, minimal search time
	uint32_t reserved3[5];
	uint32_t extintInactivity; //ms, inactivity time out on EXTINT pin if enabled
};

enum CfgRxmLpMode : uint8_t {
	CONTINUOUS_POWER,
	POWER_SAVE_MODE,
	CONTINUOUS_MODE = 4
};

struct CfgRxm {
	uint8_t reserved;
	CfgRxmLpMode lpMode;
};

struct CfgSbasMode {
	uint8_t enabled : 1; //SBAS Enabled (1) / Disabled (0) - This field is deprecated; use UBX-CFG-GNSS to enable/disable SBAS operation
	uint8_t testMode : 1; //SBAS Testbed: Use data anyhow (1) / Ignore data when in Test Mode (SBAS Msg 0)
	uint8_t reserved : 6;
};

struct CfgSbasUsage {
	uint8_t range : 1; //Use SBAS GEOs as a ranging source (for navigation)
	uint8_t diffCorr : 1; //Use SBAS Differential Corrections
	uint8_t integrity : 6; //Use SBAS Integrity Information
};

struct CfgSbas {
	CfgSbasMode mode;
	CfgSbasUsage usage;
	uint8_t maxSbas; //Maximum Number of SBAS prioritized tracking channels (valid range: 0 - 3) to use (obsolete
					 //and superseded by UBX-CFG-GNSS in protocol versions 14+)
	uint8_t scanMode2; //Continuation of scanMode1 bitmask below
	uint32_t scanMode1; //Which SBAS PRN numbers to search for (Bitmask) If all Bits are set to zero, auto-scan (i.e. all valid
						//PRNs) are searched. Every bit corresponds to a PRN number from PRN120 (bit0) to PRN158 (bit6 in scanMode2)
};

enum PatchLocation : uint8_t {
	EFUSE,
	ROM,
	BBR,
	FILE_SYSTEM
};

struct PatchInfo {
	uint32_t activated : 1;
	uint32_t location : 2; //Indicates where the patch is stored. 0: eFuse, 1: ROM, 2: BBR, 3: file system.
	uint32_t reserved : 29;
};

struct Patch {
	PatchInfo patchInfo;
	uint32_t comparatorNumber;
	uint32_t patchAddress;
	uint32_t patchData;
};

struct MonPatches {
	uint16_t version;//1
	uint16_t numPatches;
	//repeated block of Patch (x numPatches)
};

struct TimTmFlags {
	uint8_t mode : 1; //0 - single, 1 - running
	uint8_t stopped : 1; //0 - armed, 1 - stopped
	uint8_t newFallingEdge : 1; //new falling edge detected
	uint8_t timeBase : 2; //0=Time base is Receiver Time
						  //1=Time base is GNSS Time (the system according to the configuration in UBX-CFG-TP5 for tpIdx=0)
						  //2=Time base is UTC (the variant according to the configuration in UBX-CFG-NAV5)
	uint8_t utcAvailable : 1;
	uint8_t timeValid : 1;
	uint8_t newRisingEdge : 1; //new rising edge detected
};

struct TimTm {
	uint8_t channel; //Channel (i.e. EXTINT) upon which the pulse was measured
	TimTmFlags flags;
	uint16_t count;
	uint16_t weekRising;
	uint16_t weekFalling;
	uint32_t towRising;
	uint32_t towSubRising;
	uint32_t towFalling;
	uint32_t towSubFalling;
	uint32_t accEst;
};

//On chip BBR(battery backed RAM) sections to clear. The following Special Sets apply:
//0x0000 Hot start
//0x0001 Warm start
//0xFFFF Cold start
struct NavBbrMask {
	uint16_t eph : 1; //Ephemeris
	uint16_t alm : 1; //Almanac
	uint16_t health : 1; //Health
	uint16_t klob : 1; //Klobuchar parameters
	uint16_t pos : 1; //Position
	uint16_t clkd : 1; //Clock Drift
	uint16_t osc : 1; //Oscillator Parameter
	uint16_t utc : 1; //UTC Correction + GPS Leap Seconds Parameters
	uint16_t rtc : 1; //RTC
	uint16_t reserved : 6;
	uint16_t aop : 1; //Autonomous Orbit Parameters
};

enum StartTypes : uint16_t {
	HOT_START,
	WARM_START,
	COLD_START = 0xFFFF
};

union StartType {
	NavBbrMask mask;
	StartTypes start;
};

enum ResetMode : uint8_t {
	HARDWARE_RESET,
	SOFTWARE_RESET,
	SOFTWARE_RESET_GNSS_ONLY,
	HARDWARE_RESET_AFTER_SHUTDOWN = 4,
	GNSS_STOP = 8,
	GNSS_START = 9
};

struct CfgRst {
	NavBbrMask navBbrMask;
	ResetMode resetMode;
	uint8_t reserved;
};

class null {};

template <class T, class D = null> class Gnss;

template <class T>
class Gnss<T, null> {
public:
  T &serial; //2-byte pointer
  uint8_t checksum[2];
  uint16_t offset;
  uint8_t messageClass;
  uint8_t pollMessageClass;
  uint8_t messageId;
  uint8_t pollMessageId;
  uint16_t payloadLength;
  uint16_t pollPayloadLength;
  uint8_t * payload;
  uint8_t * pollPayload;
  uint32_t iTOW; //set when UBX-NAV-EOE marker is received (end of Nav Epoch - after all NAV and NMEA enabled messages)
				 //endOfNavEpoch field is set to true at this time. It is reset on the next message
  Error error;
  bool nmea, nmeaChecksumNext, nmeaValid, endOfNavEpoch;
  volatile bool pps, ttp;
  String nmeaPayload, nmeaChecksum;
  time_t utcTime;
  String nmeaDate;
  Gnss(T &serial);
  void tp();
  void begin(uint32_t gpsSpeed);
  void end();
  bool ready();
  void get();
  void navClock();
  void navDgps();
  void navDop();
  void navGeoFence();
  void navOdo();
  void navOrb();
  void navPosLlh();
  void navPvt();
  void navSat();
  void navSbas();
  void navSlas();
  void navTimeBds();
  void navTimeGal();
  void navTimeGlo();
  void navTimeGps();
  void navTimeLs();
  void navTimeUtc();
  void cfgGnss();
  void cfgInf();
  void cfgLogFilter();
  void cfgMsg();
  void cfgNav();
  void cfgOdo();
  void cfgPm();
  void cfgPms();
  void cfgPrt();
  void cfgPwr();
  void cfgRate();
  void cfgRxm();
  void cfgSbas();
  void cfgSlas();
  void cfgTp();
  void cfgGeoFence();
  void cfgNmea();
  void monVer(uint8_t extensionsNumber);
  void monGnss();
  void monPatch();
  void timTm();
  void timTp();
  void logInfo();
  void logRetrievePos();
  void logRetrievePosExtra();
  void logRetrieveString();
  void logFindTime();
  void ackAck();
  void ackNak();
  void nmeaRmc();
  void nmeaVtg();
  void nmeaGga();
  void nmeaGsa();
  void nmeaGsv();
  void nmeaGll();
  void nmeaGst();
  void nmeaVlw();
  void nmeaGns();
  void nmeaZda();
  void nmeaTxt();
  void nmeaGbs();
  void nmeaDtm();
  void nmeaGrs();
  void pubxPosition();
  void pubxSvStatus();
  void pubxTime();
  void poll(uint8_t msgClass, uint8_t msgId, uint16_t payload_length = 0, uint8_t * pload = NULL);
  MonVer * getVersion(uint32_t timeout = maxWait); //UBX-MON-VER
  MonPatches * getPatches(uint32_t timeout = maxWait); //UBX-MON-PATCH
  //UBX-CFG-NAV5 settings
  CfgNav * getCfgNav(uint32_t timeout = maxWait);
  DynModel getDynamicModel(uint32_t timeout = maxWait);
  bool setDynamicModel(DynModel model, uint32_t timeout = maxWait);
  UtcStandard getUtcStandard(uint32_t timeout = maxWait);
  bool setUtcStandard(UtcStandard standard, uint32_t timeout = maxWait);
  FixMode getFixMode(uint32_t timeout = maxWait);
  bool setFixMode(FixMode fixMode, uint32_t timeout = maxWait);
  FixedAlt * getFixedAlt(FixedAlt * fixedAlt, uint32_t timeout = maxWait);
  bool setFixedAlt(int32_t fixedAlt, uint32_t fixedAltVar, uint32_t timeout = maxWait);
  DOP * getDop(DOP * dop, uint32_t timeout = maxWait);
  bool setDop(uint16_t posDOP, uint16_t timeDOP, uint32_t timeout = maxWait);
  Accuracy * getAccuracy(Accuracy * acc, uint32_t timeout = maxWait);
  bool setAccuracy(uint16_t posAccuracy, uint16_t timeAccuracy, uint32_t timeout = maxWait);
  CnoThreshold * getCnoThreshold(CnoThreshold * cno, uint32_t timeout = maxWait);
  bool setCnoThreshold(uint8_t cnoThreshold, uint8_t cnoThresholdNumSVs, uint32_t timeout = maxWait);
  StaticHoldThresholds * getStaticHoldThresholds(StaticHoldThresholds * thresholds, uint32_t timeout = maxWait);
  bool setStaticHoldThresholds(uint8_t staticHoldThreshold, uint8_t staticHoldMaxDistance, uint32_t timeout = maxWait);
  int8_t getMinElev(int8_t timeout = maxWait);
  bool setMinElev(int8_t minElev, uint32_t timeout = maxWait);
  int8_t getDgnssTimeout(int8_t timeout = maxWait);
  bool setDgnssTimeout(int8_t dgnssTimeout, uint32_t timeout = maxWait);
  //End of UBX-CFG-NAV5 settings
  InfoMsgMask * getCfgInf(Protocol protocolId, Port portId = COM1, uint32_t timeout = maxWait); //UBX-CFG-INF
  bool setCfgInf(InfoMsgMask mask, Protocol protocolId, Port portId = COM1, uint32_t timeout = maxWait); //UBX-CFG-INF
  String getErrMsg(DebugLevel debugLevel, uint32_t timeout = maxWait); //UBX-INF-* debug messages
  CfgPrt * getCfgPrt(Port portId, uint32_t timeout = maxWait); //UBX-CFG-PRT
  bool setCfgPrt(Port portId, BaudRate rate, PrtMode mode, InProtoMask inMask, OutProtoMask outMask, bool extendedTxTimeout, uint32_t timeout = maxWait); //UBX-CFG-PRT
  bool config(ConfMask mask, ConfigType type, uint32_t timeout = maxWait); //UBX-CFG-CFG
  bool config(ConfMask mask, ConfigType type, Device dev, uint32_t timeout = maxWait); //UBX-CFG-CFG
  bool defaultConfig(bool ioPort = true, bool msgConf = true, bool infMsg = true, bool navConf = true, bool rxmConf = true, bool senConf = true, bool rinvConf = true, bool antConf = true, bool logConf = true, bool ftsConf = true, uint32_t timeout = maxWait);
  bool saveConfig(bool ioPort = true, bool msgConf = true, bool infMsg = true, bool navConf = true, bool rxmConf = true, bool senConf = true, bool rinvConf = true, bool antConf = true, bool logConf = true, bool ftsConf = true, uint32_t timeout = maxWait);
  bool loadConfig(bool ioPort = true, bool msgConf = true, bool infMsg = true, bool navConf = true, bool rxmConf = true, bool senConf = true, bool rinvConf = true, bool antConf = true, bool logConf = true, bool ftsConf = true, uint32_t timeout = maxWait);
  void cfgRst(StartType type, ResetMode mode); //UBX-CFG-RST
  void stopGnss(StartTypes startType = HOT_START); //The receiver will not be restarted, but will stop any GNSS related processing
  void startGnss(StartTypes startType = HOT_START); //Starts all GNSS tasks
  void resetGnss(StartTypes startType = HOT_START); //only restarts the GNSS tasks, without reinitializing the full system or 
												//reloading any stored configuration.
  void reset(bool soft = true, bool afterShutdown = true, StartTypes startType = HOT_START); //hardware or software reset. Reset afterShutdown applies to hardware reset only

  
  CfgMsg * getCfgMsg(uint8_t msgClass, uint8_t msgId, uint32_t timeout = maxWait); //UBX-CFG-MSG
  bool setCfgMsg(uint8_t msgClass, uint8_t msgId, uint8_t rate, uint32_t timeout = maxWait); //UBX-CFG-MSG
  PowerMode * getCfgPms(uint32_t timeout = maxWait); //UBX-CFG-PMS
  bool setCfgPms(PowerModes mode, uint8_t period = 0, uint8_t onTime = 0, uint32_t timeout = maxWait); //UBX-CFG-PMS
  CfgPm * getCfgPm(uint32_t timeout = maxWait); //UBX-CFG-PM2
  bool setCfgPm(uint8_t maxStartupStateDuration, uint32_t udpatePeriod, uint32_t searchPeriod, uint32_t gridOffset, uint16_t onTime, uint16_t minAcqTime, uint32_t extintInactivity, CfgPmFlags flags, uint32_t timeout = maxWait); //UBX-CFG-PM2
  CfgRxm * getCfgRxm(uint32_t timeout = maxWait); //UBX-CFG-RXM
  bool setCfgRxm(CfgRxmLpMode mode, int32_t timeout = maxWait); //UBX-CFG-RXM
  CfgSbas * getCfgSbas(uint32_t timeout = maxWait); //UBX-CFG-SBAS
  bool setCfgSbas(CfgSbasUsage usage, uint32_t scanMode1, uint8_t scanMode2, uint32_t timeout = maxWait); //UBX-CFG-SBAS
  CfgNmea * getCfgNmea(uint32_t timeout = maxWait); //UBX-CFG-NMEA
  bool setCfgNmea(CfgNmeaFilter filter, uint8_t nmeaVersion, uint8_t maxSVs, CfgNmeaFlags flags, CfgNmeaGnss gnssFilter, bool displayNonNmeaSVs, CfgNmeaTalkerId mainTalkerId, bool gsvTalkerIdIsMain, char * dbsTalkerId, uint32_t timeout = maxWait); //UBX-CFG-NMEA

  ODOCfg * getCfgOdo(uint32_t timeout = maxWait); //UBX-CFG-ODO
  bool setCfgOdo(OdoFlags flags, OdoProfile profile, uint8_t maxSpeed, uint8_t maxPosAccuracy, uint8_t velGain, uint8_t COGgain, uint32_t timeout = maxWait); //UBX-CFG-ODO
  CfgLogFilter * getCfgLogFilter(uint32_t timeout = maxWait); //UBX-CFG-LOGFILTER
  bool setCfgLogFilter(uint8_t minInterval, uint8_t timeThreshold, uint8_t speedThreshold, uint8_t positionThreshold, CfgLogFilterFlags flags, uint32_t timeout = maxWait); //UBX-CFG-LOGFILTER
  LogInfo * getLogInfo(uint32_t timeout = maxWait); //UBX-LOG-INFO
  bool logRetrieve(uint32_t index, uint32_t count); //UBX-LOG-RETRIEVE; count <=256. Use logFind to get the index
  bool createLog(LogSize logSize, uint32_t userDefinedSize = 0, bool circular = true, uint32_t timeout = maxWait); //UBX-LOG-CREATE
  bool eraseLog(uint32_t timeout = maxWait); //UBX-LOG-ERASE
  uint32_t logFind(DateTime dateTime, uint32_t timeout = maxWait); //UBX-LOG-FINDTIME
  bool logMsg(char * msg, uint8_t len, uint32_t timeout = maxWait); //UBX-LOG-STRING
  
  bool enableLogging(uint8_t interval, uint32_t timeout = maxWait);
  bool disableLogging(uint32_t timeout = maxWait);

  
  void infMsg(const char * infLevel); //UBX-INF-XXX
  GnssConf * getGnss(uint32_t timeout = maxWait); //UBX-CFG-GNSS
  bool setGnss(MajorGnss gnss, bool enableSBAS, bool enableIMES, uint32_t timeout = maxWait); //UBX-CFG-GNSS
  GnssSupport * getSupportedGnss(uint32_t timeout = maxWait); //UBX-MON-GNSS
  NavRate * getNavRate(uint32_t timeout = maxWait); //UBX-CFG-RATE
  bool setNavRate(uint16_t measurementRate, uint16_t navSolRate, TimeRef timeRef, uint32_t timeout = maxWait); //measurementRate in ms, navSolRate is number of measurements for each NavSol
  TimePulse * getTimePulse(uint32_t timeout = maxWait); //UBX-CFG-TP5 - for use with UBX-TIM-TP, which gives the time for the next time pulse. UBX-NAV-PVT gives time for the previous time pulse!
														//don't forget to account for antenna cable delay - 50ns in my chip + any delays in arduino and app!
  bool setTimePulse(uint32_t pulse_period, uint32_t pulse_len, uint32_t pulse_period_locked, uint32_t pulse_len_locked, int32_t delay, TimePulseFlags flags, uint32_t timeout = maxWait); //for best time pulse performance it is recommended to disable SBAS - see setGNSS()
  																														 //also recommended to set measurement rate (setRate()) and time pulse to 1Hz
  NavPvt * getNavPvt(uint32_t timeout = maxWait); //UBX-NAV-PVT
  NavSat * getNavSat(uint32_t timeout = maxWait); //UBX-NAV-SAT
  NavTimeUtc * getNavTimeUtc(uint32_t timeout = maxWait); //UBX-NAV-TIMEUTC
  TimeTP * getTimeTp(uint32_t timeout = maxWait); //UBX-TIM-TP - use gpsToUtc() function to convert GPS time(weeks, ms) to UTC(DateTime: year, month, day, hour, minute, second)
  TimTm * getTimTm(uint32_t timeout = maxWait); //UBX-TIM-TM2
  NavClock * getNavClock(uint32_t timeout = maxWait); //UBX-NAV-CLOCK
  NavDGPS * getNavDgps(uint32_t timeout = maxWait); //UBX-NAV-DGPS
  NavDOP * getNavDop(uint32_t timeout = maxWait); //UBX-NAV-DOP
  GeoFences * getCfgGeofences(uint32_t timeout = maxWait); //UBX-CFG-GEOFENCE
  bool setCfgGeofence(GeoFence * geofence, uint8_t confidenceLevel, uint32_t timeout = maxWait); //UBX-CFG-GEOFENCE
  NavGeofence * getNavGeofence(uint32_t timeout = maxWait); //UBX-NAV-GEOFENCE
  NavODO * getNavOdo(uint32_t timeout = maxWait); //UBX-NAV-ODO
  bool resetOdo(uint32_t timeout = maxWait); //UBX-NAV-RESETODO
  NavOrb * getNavOrb(uint32_t timeout = maxWait); //UBX-NAV-ORB
  NavPosLlh * getNavPosLlh(uint32_t timeout = maxWait); //UBX-NAV-POSLLH
  NavSbas * getNavSbas(uint32_t timeout = maxWait); //UBX-NAV-SBAS
  NavSlas * getNavSlas(uint32_t timeout = maxWait); //UBX-NAV-SLAS
  NavTimeBdsGal * getNavTimeBds(uint32_t timeout = maxWait); //UBX-NAV-TIMEBDS
  NavTimeBdsGal * getNavTimeGal(uint32_t timeout = maxWait); //UBX-NAV-TIMEGAL
  NavTimeGps * getNavTimeGps(uint32_t timeout = maxWait); //UBX-NAV-TIMEGPS
  NavTimeGlo * getNavTimeGlo(uint32_t timeout = maxWait); //UBX-NAV-TIMEGLO
  NavTimeLs * getNavTimeLs(uint32_t timeout = maxWait); //UBX-NAV-TIMELS

  void getGNRMC(GNRMC * data); //Recommended Minimum data. The recommended minimum sentence defined by NMEA for GNSS system data.
  void getGNGGA(GNGGA * data); //Global positioning system fix data for GN (multiple GNSS). In case of GN, it is recommened to use GNS messages instead
  void getGNGLL(GNGLL * data); //Latitude and longitude, with time of position fix and status for GN (multiple GNSS)
  void getGNVTG(GNVTG * data); //Course over ground and Ground speed
  void getGNGSA(GNGSA * data); //GNSS DOP and Active Satellites. In a multi-GNSS system this message will be output multiple times, once for each GNSS
  void getGNGSV(GNGSV * data); //GNSS Satellites in View for GP (GPS) or GL (GLONALL). Only four satellite details are transmitted in one message. 
							   //In a multi-GNSS system sets of GSV messages will be output multiple times, one set for each GNSS.
  void getGNGST(GNGST * data); //GNSS Pseudo Range Error Statistics 
  void getGNVLW(GNVLW * data); //Dual ground/water distance
  void getGNGNS(GNGNS * data); //GNSS fix data 
  void getGNZDA(GNZDA * data); //Time and Date
  void getGNTXT(GNTXT* data); //This message outputs various information on the receiver, such as power-up screen, software version etc
  void getGbs(GBS* data); //GNSS Satellite Fault Detection
  void getDtm(DTM* data); //Datum Reference
  void getGrs(GRS* data); //GNSS Range Residuals
  void nmeaGpq(const char* talkerId, const char* msgId); //Poll a standard message (if the current Talker ID is GP)
  void nmeaGnq(const char* talkerId, const char* msgId); //Poll a standard message (if the current Talker ID is GN)
  void nmeaGlq(const char* talkerId, const char* msgId); //Poll a standard message (if the current Talker ID is GL)
  void nmeaGbq(const char* talkerId, const char* msgId); //Poll a standard message (if the current Talker ID is GB)
  void getPubxPosition(PubxPosition * data); //PUBX-POSITION
  void getPubxTime(PubxTime * data); //PUBX-TIME
  void pubxConfig(BaudRate rate, InProtoMask inMask, OutProtoMask outMask, Port portId = COM1, bool autoBauding = false); //PUBX-CONFIG
  void pubxRate(char * msgId, uint8_t rateCom1, uint8_t rateCom2 = 0, uint8_t rateDDC = 0, uint8_t rateUsb = 0, uint8_t rateSpi = 0); //PUBX-RATE

private:
  void calculateChecksum(uint8_t msgClass, uint8_t msgId, uint16_t len, uint8_t * pload);
  void nmeaVerifyChecksum();
  uint16_t available();
  uint8_t read();
  bool pollNoError(uint32_t timeout);
  bool ackNoError(uint32_t timeout);
};


template <class T, class D>
class Gnss {
public:
  T &serial; //2-byte pointer
  D &debug; //for debugging output
  uint8_t checksum[2];
  uint16_t offset;
  uint8_t messageClass;
  uint8_t pollMessageClass;
  uint8_t messageId;
  uint8_t pollMessageId;
  uint16_t payloadLength;
  uint16_t pollPayloadLength;
  uint8_t * payload;
  uint8_t * pollPayload;
  uint32_t iTOW; //set when UBX-NAV-EOE marker is received (end of Nav Epoch - after all NAV and NMEA enabled messages)
				 //endOfNavEpoch field is set to true at this time. It is reset on the next message
  Error error;
  bool nmea, nmeaChecksumNext, nmeaValid, endOfNavEpoch;
  volatile bool pps, ttp;
  String nmeaPayload, nmeaChecksum;
  time_t utcTime;
  String nmeaDate;
  Gnss(T &serial, D &debug);
  void tp();
  void begin(uint32_t gpsSpeed, uint32_t debugSpeed);
  void end();
  bool ready();
  void get();
  void navClock();
  void navDgps();
  void navDop();
  void navGeoFence();
  void navOdo();
  void navOrb();
  void navPosLlh();
  void navPvt();
  void navSat();
  void navSbas();
  void navSlas();
  void navTimeBds();
  void navTimeGal();
  void navTimeGlo();
  void navTimeGps();
  void navTimeLs();
  void navTimeUtc();
  void cfgGnss();
  void cfgInf();
  void cfgLogFilter();
  void cfgMsg();
  void cfgNav();
  void cfgOdo();
  void cfgPm();
  void cfgPms();
  void cfgPrt();
  void cfgPwr();
  void cfgRate();
  void cfgRxm();
  void cfgSbas();
  void cfgSlas();
  void cfgTp();
  void cfgGeoFence();
  void cfgNmea();
  void monVer(uint8_t extensionsNumber);
  void monGnss();
  void monPatch();
  void timTm();
  void timTp();
  void logInfo();
  void logRetrievePos();
  void logRetrievePosExtra();
  void logRetrieveString();
  void logFindTime();
  void ackAck();
  void ackNak();
  void nmeaRmc();
  void nmeaVtg();
  void nmeaGga();
  void nmeaGsa();
  void nmeaGsv();
  void nmeaGll();
  void nmeaGst();
  void nmeaVlw();
  void nmeaGns();
  void nmeaZda();
  void nmeaTxt();
  void nmeaGbs();
  void nmeaDtm();
  void nmeaGrs();
  void pubxPosition();
  void pubxSvStatus();
  void pubxTime();
  void poll(uint8_t msgClass, uint8_t msgId, uint16_t payload_length = 0, uint8_t * pload = NULL);
  MonVer * getVersion(uint32_t timeout = maxWait); //UBX-MON-VER
  MonPatches * getPatches(uint32_t timeout = maxWait); //UBX-MON-PATCH
  //UBX-CFG-NAV5 settings
  CfgNav * getCfgNav(uint32_t timeout = maxWait);
  DynModel getDynamicModel(uint32_t timeout = maxWait);
  bool setDynamicModel(DynModel model, uint32_t timeout = maxWait);
  UtcStandard getUtcStandard(uint32_t timeout = maxWait);
  bool setUtcStandard(UtcStandard standard, uint32_t timeout = maxWait);
  FixMode getFixMode(uint32_t timeout = maxWait);
  bool setFixMode(FixMode fixMode, uint32_t timeout = maxWait);
  FixedAlt * getFixedAlt(FixedAlt * fixedAlt, uint32_t timeout = maxWait);
  bool setFixedAlt(int32_t fixedAlt, uint32_t fixedAltVar, uint32_t timeout = maxWait);
  DOP * getDop(DOP * dop, uint32_t timeout = maxWait);
  bool setDop(uint16_t posDOP, uint16_t timeDOP, uint32_t timeout = maxWait);
  Accuracy * getAccuracy(Accuracy * acc, uint32_t timeout = maxWait);
  bool setAccuracy(uint16_t posAccuracy, uint16_t timeAccuracy, uint32_t timeout = maxWait);
  CnoThreshold * getCnoThreshold(CnoThreshold * cno, uint32_t timeout = maxWait);
  bool setCnoThreshold(uint8_t cnoThreshold, uint8_t cnoThresholdNumSVs, uint32_t timeout = maxWait);
  StaticHoldThresholds * getStaticHoldThresholds(StaticHoldThresholds * thresholds, uint32_t timeout = maxWait);
  bool setStaticHoldThresholds(uint8_t staticHoldThreshold, uint8_t staticHoldMaxDistance, uint32_t timeout = maxWait);
  int8_t getMinElev(int8_t timeout = maxWait);
  bool setMinElev(int8_t minElev, uint32_t timeout = maxWait);
  int8_t getDgnssTimeout(int8_t timeout = maxWait);
  bool setDgnssTimeout(int8_t dgnssTimeout, uint32_t timeout = maxWait);
  //End of UBX-CFG-NAV5 settings
  InfoMsgMask * getCfgInf(Protocol protocolId, Port portId = COM1, uint32_t timeout = maxWait); //UBX-CFG-INF
  bool setCfgInf(InfoMsgMask mask, Protocol protocolId, Port portId = COM1, uint32_t timeout = maxWait); //UBX-CFG-INF
  String getErrMsg(DebugLevel debugLevel, uint32_t timeout = maxWait); //UBX-INF-* debug messages
  CfgPrt * getCfgPrt(Port portId, uint32_t timeout = maxWait); //UBX-CFG-PRT
  bool setCfgPrt(Port portId, BaudRate rate, PrtMode mode, InProtoMask inMask, OutProtoMask outMask, bool extendedTxTimeout, uint32_t timeout = maxWait); //UBX-CFG-PRT
  bool config(ConfMask mask, ConfigType type, uint32_t timeout = maxWait); //UBX-CFG-CFG
  bool config(ConfMask mask, ConfigType type, Device dev, uint32_t timeout = maxWait); //UBX-CFG-CFG
  bool defaultConfig(bool ioPort = true, bool msgConf = true, bool infMsg = true, bool navConf = true, bool rxmConf = true, bool senConf = true, bool rinvConf = true, bool antConf = true, bool logConf = true, bool ftsConf = true, uint32_t timeout = maxWait);
  bool saveConfig(bool ioPort = true, bool msgConf = true, bool infMsg = true, bool navConf = true, bool rxmConf = true, bool senConf = true, bool rinvConf = true, bool antConf = true, bool logConf = true, bool ftsConf = true, uint32_t timeout = maxWait);
  bool loadConfig(bool ioPort = true, bool msgConf = true, bool infMsg = true, bool navConf = true, bool rxmConf = true, bool senConf = true, bool rinvConf = true, bool antConf = true, bool logConf = true, bool ftsConf = true, uint32_t timeout = maxWait);
  void cfgRst(StartType type, ResetMode mode); //UBX-CFG-RST
  void stopGnss(StartTypes startType = HOT_START); //The receiver will not be restarted, but will stop any GNSS related processing
  void startGnss(StartTypes startType = HOT_START); //Starts all GNSS tasks
  void resetGnss(StartTypes startType = HOT_START); //only restarts the GNSS tasks, without reinitializing the full system or 
												//reloading any stored configuration.
  void reset(bool soft = true, bool afterShutdown = true, StartTypes startType = HOT_START); //hardware or software reset. Reset afterShutdown applies to hardware reset only


  CfgMsg * getCfgMsg(uint8_t msgClass, uint8_t msgId, uint32_t timeout = maxWait); //UBX-CFG-MSG
  bool setCfgMsg(uint8_t msgClass, uint8_t msgId, uint8_t rate, uint32_t timeout = maxWait); //UBX-CFG-MSG
  PowerMode * getCfgPms(uint32_t timeout = maxWait); //UBX-CFG-PMS
  bool setCfgPms(PowerModes mode, uint8_t period = 0, uint8_t onTime = 0, uint32_t timeout = maxWait); //UBX-CFG-PMS
  CfgPm * getCfgPm(uint32_t timeout = maxWait); //UBX-CFG-PM2
  bool setCfgPm(uint8_t maxStartupStateDuration, uint32_t udpatePeriod, uint32_t searchPeriod, uint32_t gridOffset, uint16_t onTime, uint16_t minAcqTime, uint32_t extintInactivity, CfgPmFlags flags, uint32_t timeout = maxWait); //UBX-CFG-PM2
  CfgRxm * getCfgRxm(uint32_t timeout = maxWait); //UBX-CFG-RXM
  bool setCfgRxm(CfgRxmLpMode mode, int32_t timeout = maxWait); //UBX-CFG-RXM
  CfgSbas * getCfgSbas(uint32_t timeout = maxWait); //UBX-CFG-SBAS
  bool setCfgSbas(CfgSbasUsage usage, uint32_t scanMode1, uint8_t scanMode2, uint32_t timeout = maxWait); //UBX-CFG-SBAS
  CfgNmea * getCfgNmea(uint32_t timeout = maxWait); //UBX-CFG-NMEA
  bool setCfgNmea(CfgNmeaFilter filter, uint8_t nmeaVersion, uint8_t maxSVs, CfgNmeaFlags flags, CfgNmeaGnss gnssFilter, bool displayNonNmeaSVs, CfgNmeaTalkerId mainTalkerId, bool gsvTalkerIdIsMain, char * dbsTalkerId, uint32_t timeout = maxWait); //UBX-CFG-NMEA
  ODOCfg * getCfgOdo(uint32_t timeout = maxWait); //UBX-CFG-ODO
  bool setCfgOdo(OdoFlags flags, OdoProfile profile, uint8_t maxSpeed, uint8_t maxPosAccuracy, uint8_t velGain, uint8_t COGgain, uint32_t timeout = maxWait); //UBX-CFG-ODO
  CfgLogFilter * getCfgLogFilter(uint32_t timeout = maxWait); //UBX-CFG-LOGFILTER
  bool setCfgLogFilter(uint8_t minInterval, uint8_t timeThreshold, uint8_t speedThreshold, uint8_t positionThreshold, CfgLogFilterFlags flags, uint32_t timeout = maxWait); //UBX-CFG-LOGFILTER
  LogInfo * getLogInfo(uint32_t timeout = maxWait); //UBX-LOG-INFO
  bool logRetrieve(uint32_t index, uint32_t count); //UBX-LOG-RETRIEVE; count <=256. Use logFind to get the index
  bool createLog(LogSize logSize, uint32_t userDefinedSize = 0, bool circular = true, uint32_t timeout = maxWait); //UBX-LOG-CREATE
  bool eraseLog(uint32_t timeout = maxWait); //UBX-LOG-ERASE
  uint32_t logFind(DateTime dateTime, uint32_t timeout = maxWait); //UBX-LOG-FINDTIME
  bool logMsg(char * msg, uint8_t len, uint32_t timeout = maxWait); //UBX-LOG-STRING
  
  bool enableLogging(uint8_t interval, uint32_t timeout = maxWait);
  bool disableLogging(uint32_t timeout = maxWait);

  void infMsg(const char * infLevel); //UBX-INF-XXX
  GnssConf * getGnss(uint32_t timeout = maxWait); //UBX-CFG-GNSS
  bool setGnss(MajorGnss gnss, bool enableSBAS, bool enableIMES, uint32_t timeout = maxWait); //UBX-CFG-GNSS
  GnssSupport * getSupportedGnss(uint32_t timeout = maxWait); //UBX-MON-GNSS
  NavRate * getNavRate(uint32_t timeout = maxWait); //UBX-CFG-RATE
  bool setNavRate(uint16_t measurementRate, uint16_t navSolRate, TimeRef timeRef, uint32_t timeout = maxWait); //measurementRate in ms, navSolRate is number of measurements for each NavSol
  TimePulse * getTimePulse(uint32_t timeout = maxWait); //UBX-CFG-TP5 - for use with UBX-TIM-TP, which gives the time for the next time pulse. UBX-NAV-PVT gives time for the previous time pulse!
														//don't forget to account for antenna cable delay - 50ns in my chip + any delays in arduino and app!
  bool setTimePulse(uint32_t pulse_period, uint32_t pulse_len, uint32_t pulse_period_locked, uint32_t pulse_len_locked, int32_t delay, TimePulseFlags flags, uint32_t timeout = maxWait); //for best time pulse performance it is recommended to disable SBAS - see setGNSS()
  																														 //also recommended to set measurement rate (setRate()) and time pulse to 1Hz
  NavPvt * getNavPvt(uint32_t timeout = maxWait); //UBX-NAV-PVT
  NavSat * getNavSat(uint32_t timeout = maxWait); //UBX-NAV-SAT
  NavTimeUtc * getNavTimeUtc(uint32_t timeout = maxWait); //UBX-NAV-TIMEUTC
  TimeTP * getTimeTp(uint32_t timeout = maxWait); //UBX-TIM-TP - use gpsToUtc() function to convert GPS time(weeks, ms) to UTC(DateTime: year, month, day, hour, minute, second)
  TimTm * getTimTm(uint32_t timeout = maxWait); //UBX-TIM-TM2
  NavClock * getNavClock(uint32_t timeout = maxWait); //UBX-NAV-CLOCK
  NavDGPS * getNavDgps(uint32_t timeout = maxWait); //UBX-NAV-DGPS
  NavDOP * getNavDop(uint32_t timeout = maxWait); //UBX-NAV-DOP
  NavGeofence * getNavGeofence(uint32_t timeout = maxWait); //UBX-NAV-GEOFENCE
  GeoFences * getCfgGeofences(uint32_t timeout = maxWait); //UBX-CFG-GEOFENCE
  bool setCfgGeofence(GeoFence * geofence, uint8_t confidenceLevel, uint32_t timeout = maxWait); //UBX-CFG-GEOFENCE
  NavODO * getNavOdo(uint32_t timeout = maxWait); //UBX-NAV-ODO
  bool resetOdo(uint32_t timeout = maxWait); //UBX-NAV-RESETODO
  NavOrb * getNavOrb(uint32_t timeout = maxWait); //UBX-NAV-ORB
  NavPosLlh * getNavPosLlh(uint32_t timeout = maxWait); //UBX-NAV-POSLLH
  NavSbas * getNavSbas(uint32_t timeout = maxWait); //UBX-NAV-SBAS
  NavSlas * getNavSlas(uint32_t timeout = maxWait); //UBX-NAV-SLAS
  NavTimeBdsGal * getNavTimeBds(uint32_t timeout = maxWait); //UBX-NAV-TIMEBDS
  NavTimeBdsGal * getNavTimeGal(uint32_t timeout = maxWait); //UBX-NAV-TIMEGAL
  NavTimeGps * getNavTimeGps(uint32_t timeout = maxWait); //UBX-NAV-TIMEGPS
  NavTimeGlo * getNavTimeGlo(uint32_t timeout = maxWait); //UBX-NAV-TIMEGLO
  NavTimeLs * getNavTimeLs(uint32_t timeout = maxWait); //UBX-NAV-TIMELS

  void getGNRMC(GNRMC * data); //Recommended Minimum data. The recommended minimum sentence defined by NMEA for GNSS system data.
  void getGNGGA(GNGGA * data); //Global positioning system fix data for GN (multiple GNSS). In case of GN, it is recommened to use GNS messages instead
  void getGNGLL(GNGLL * data); //Latitude and longitude, with time of position fix and status for GN (multiple GNSS)
  void getGNVTG(GNVTG * data); //Course over ground and Ground speed
  void getGNGSA(GNGSA * data); //GNSS DOP and Active Satellites. In a multi-GNSS system this message will be output multiple times, once for each GNSS
  void getGNGSV(GNGSV * data); //GNSS Satellites in View for GP (GPS) or GL (GLONALL). Only four satellite details are transmitted in one message. 
							   //In a multi-GNSS system sets of GSV messages will be output multiple times, one set for each GNSS.
  void getGNGST(GNGST * data); //GNSS Pseudo Range Error Statistics 
  void getGNVLW(GNVLW * data); //Dual ground/water distance
  void getGNGNS(GNGNS * data); //GNSS fix data 
  void getGNZDA(GNZDA * data); //Time and Date
  void getGNTXT(GNTXT* data); //This message outputs various information on the receiver, such as power-up screen, software version etc
  void getGbs(GBS* data); //GNSS Satellite Fault Detection
  void getDtm(DTM* data); //Datum Reference
  void getGrs(GRS* data); //GNSS Range Residuals
  void nmeaGpq(const char* talkerId, const char* msgId); //Poll a standard message (if the current Talker ID is GP)
  void nmeaGnq(const char* talkerId, const char* msgId); //Poll a standard message (if the current Talker ID is GN)
  void nmeaGlq(const char* talkerId, const char* msgId); //Poll a standard message (if the current Talker ID is GL)
  void nmeaGbq(const char* talkerId, const char* msgId); //Poll a standard message (if the current Talker ID is GB)
  void getPubxPosition(PubxPosition * data); //PUBX-POSITION
  void getPubxTime(PubxTime * data); //PUBX-TIME
  void pubxConfig(BaudRate rate, InProtoMask inMask, OutProtoMask outMask, Port portId = COM1, bool autoBauding = false); //PUBX-CONFIG
  void pubxRate(char * msgId, uint8_t rateCom1, uint8_t rateCom2 = 0, uint8_t rateDDC = 0, uint8_t rateUsb = 0, uint8_t rateSpi = 0); //PUBX-RATE

private:
  void calculateChecksum(uint8_t msgClass, uint8_t msgId, uint16_t len, uint8_t * pload);
  void nmeaVerifyChecksum();
  uint16_t available();
  uint8_t read();
  bool pollNoError(uint32_t timeout);
  bool ackNoError(uint32_t timeout);
};

//helper functions
void nmeaToUtc(DateTime * dt, String date, String time);
void longLatToDMS(Latitude * latitude, int32_t lat);
void longLonToDMS(Longitude * longitude, int32_t lon);
int32_t dmsToLongLat(Latitude * latitude);
int32_t dmsToLongLon(Longitude * longitude);
void nmeaLatToDMS(Latitude * latitude, String lat, char NS);
void nmeaLonToDMS(Longitude * longitude, String lon, char EW);
String dmsLatToStr(Latitude * lat);
String dmsLonToStr(Longitude * lon);
void split(String msg[], uint8_t array_size, String pload);
tm * gps2tm(DateTime * dt, tm * time_tm);

//compile with g++ -O -o gps gps.cpp -lreadline

/*#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <error.h>
#include <signal.h>
#include <unistd.h>*/
#include <sys/types.h>
#include <readline/readline.h>
#include <readline/history.h>
#include "gnss.cpp"

using namespace std;
#include <regex>
#include <string>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <csignal>

Port gnssPort = COM1;
uint32_t gnssPortRate = 9600;
const uint8_t navRate = 1;


typedef int command_function_type(const char *);

typedef struct {
	const char *name;
	const char *arg1;
	const char *arg2;
	command_function_type *func;
} COMMAND;

void nmeaStandardMessages(void);
void ubxStandardMessages(void);
void getArguments(void);
int close(const char *);
int quit(const char *);
int help(const char *);
int open(const char *);
int version(const char *);
int patches(const char *);
int nmeaGnss(const char *);
int nmeaGps(const char *);
int nmeaGlonass(const char *);
int nmeaBeidou(const char *);
int nmeaRate(const char *);
int ubxRate(const char *);
int get(const char *);
int debug(const char *);
int set(const char *);
int gnss(const char *);
int config(const char *);
int poll(const char *);
int display(const char *);

COMMAND commands[] = {
	{ "close", NULL, NULL, close },
	{ "config", "default load safe", NULL, config },
	{ "debug", "nmea ubx", "on off", debug },
	{ "display", "all cfg err nav nmea pubx tim", "on off", display },
	{ "exit", NULL, NULL, quit },
	{ "get", "accuracy cnoThreshold dgnssTimeout debug delay dop dynamicModel fixedAlt fixMode gnss logFilter logInfo minElev navRate nmea odo port pms pm rxm sbas staticHoldThresholds supportedGNSS timePulse utcStandard", NULL, get },
	{ "gnss", "start stop reset", NULL, gnss },
	{ "help", "close debug display exit get open nmeaGnss nmeaGps nmeaGlonass nmeaBeidou nmeaRate quit ubxRate", NULL, help },
	{ "open", "com1 com2 usb ddc spi", NULL, open },
	{ "nmeaGnss", "dtm gbs gga ggl gns grs gsa gst gsv rmc vlw vtg zda", NULL, nmeaGnss },
	{ "nmeaGps", "dtm gbs gga ggl gns grs gsa gst gsv rmc vlw vtg zda", NULL, nmeaGps },
	{ "nmeaGlonass", "dtm gbs gga ggl gns grs gsa gst gsv rmc vlw vtg zda", NULL, nmeaGlonass },
	{ "nmeaBeidou", "dtm gbs gga ggl gns grs gsa gst gsv rmc vlw vtg zda", NULL, nmeaBeidou },
	{ "nmeaRate", "dtm gbs gga ggl gns grs gsa gst gsv rmc vlw vtg zda", NULL, nmeaRate },
	{ "poll", "clock dgps dop geofence odo orb posecef posllh pvt sat sbas slas status timebds timegal timeglo timegps timels timeutc tm tp velecef velned", NULL, poll },
	{ "set", "accuracy cnoThreshold dgnssTimeout debug delay dop dynamicModel fixedAlt fixMode gnss logFilter minElev navRate nmea odo port pms pm rxm sbas staticHoldThresholds utcStandard", NULL, set },
	{ "ubxRate", "clock dgps dop eoe geofence odo orb posecef posllh pvt pubx_position pubx_time sat sbas slas status timebds timegal timeglo timegps timels timeutc tm tp velecef velned", NULL, ubxRate },
	{ "quit", NULL, NULL, quit },
	{ "version", NULL, NULL, version },
	{ "patches", NULL, NULL, patches },
	{ NULL, NULL, NULL, NULL }
};

Gnss gps;

uint32_t tRateToI(speed_t r) {
	switch (r) {
	case B4800: return 4800;
	case B9600: return 9600;
	case B19200: return 19200;
	case B38400: return 38400;
	case B57600: return 57600;
	case B115200: return 115200;
	case B230400: return 230400;
	}
}

int8_t setup(const char * dev, speed_t rate, bool reset, bool soft) {
	speed_t rates[7] = { B4800, B9600, B19200, B38400, B57600, B115200, B230400 };
	time_t utc, t1;

	gps = Gnss();
	InProtoMask inMask;
	OutProtoMask outMask;
	inMask.inNmea = 1;
	inMask.inUbx = 1;
	inMask.inRtcm = 1;
	inMask.reserved = 0;
	outMask.outNmea = 1;
	outMask.outUbx = 1;
	outMask.reserved = 0;

	printf("TOPGNSS\n");
	printf("Connecting to GNSS on %s at %u baud\n", dev, gnssPortRate);

	gps.begin(dev, rate);

	utc = gps.utcTime;
	gps.nmeaGnq("RMC");
	delay(2);
	if (utc != gps.utcTime) printf(" connected.\n");
	else {
		for (int i = 0; i < 7; i++) {
			if (rate != rates[i]) {
				printf(" failed. Re-trying with %d baud rate...\n", tRateToI(rates[i]));
				gps.end();
				gps.begin(dev, rates[i]);
				utc = gps.utcTime;
				gps.nmeaGnq("RMC");
				delay(2);
				if (utc != gps.utcTime) { printf(" connected.\n"); break; }
				else if (i == 6) { printf(" sorry, giving up...\n"); return -1; }
			}
		}
	}
	if (reset) {
		printf("Reseting...\n");
		gps.reset(soft);
		delay(3);
	}

	printf("gps.baudRate = %u; rate = %u\n", (uint32_t)(gps.gnssPort.baudRate), gnssPortRate);
	if (gps.gnssPort.baudRate != (BaudRate)gnssPortRate) {
		printf("Setting desired rate %u...\n", gnssPortRate);
		//gps.setCfgPrt(gps.gnssPort.portId, (BaudRate)gnssPortRate, gps.gnssPort.mode, gps.gnssPort.inProtoMask, gps.gnssPort.outProtoMask);
		gps.pubxConfig((BaudRate)gnssPortRate, inMask, outMask, gnssPort);
		gps.end();
		printf("Connecting with desired rate %u...\n", gnssPortRate);
		gps.begin(dev, rate);
		utc = gps.utcTime;
		gps.nmeaGnq("RMC");
		delay(2);
		if (utc != gps.utcTime) {
			printf("Connected\n");
			gps.saveConfig();
		}
		else {
			printf("Failed\n"); return -1;
		}
	}
	//gps.pubxRate("RMC", 1);
	const char * nmea_sentences[] = { "RMC", "DTM", "GBS", "GGA", "GLL", "GNS", "GRS", "GSA", "GST", "GSV", "VLW", "VTG", "ZDA", NULL };
	for (int i = 0; nmea_sentences[i]; i++) gps.pubxRate(nmea_sentences[i], 0);
	gps.pubxConfig((BaudRate)gnssPortRate, inMask, outMask, gnssPort);
	return 0;
}

static void sigHandler(int sig) {
	//printf("sigio caught\n");
	if (gps.isReady) {
		//printf("calling loop...\n");
		gps.get();
	}
}

COMMAND * find_command(const char * name)
{
	for (int i = 0; commands[i].name; i++)
		if (strncmp(name, commands[i].name, strlen(name)) == 0) return (&commands[i]);
	return NULL;
}

int execute_command(char * cmd) {
	char * space = NULL;
	char command[32], args[255];
	COMMAND * c;
	size_t len;

	memset(command, 0, sizeof(command));
	memset(args, 0, sizeof(args));

	space = strchr(cmd, ' ');
	if (space) {
		len = strlen(cmd) - strlen(space);
		strncpy(command, cmd, len);
		command[len] = '\0';
		strncpy(args, space + 1, strlen(space));
	} else strncpy(command, cmd, sizeof(command));

	c = find_command(command);
	if (c != NULL) return c->func(args);
	else { printf("no such command \'%s\'\n", command);   return -1; }
}

int help(const char * cmd) {
	if (cmd) {
		if (!*cmd) {
			printf("GNSS commands:\n");
			printf("close\n");
			printf("config <default|load|save>\n");
			printf("debug <nmea|ubx> <on|off>\n");
			printf("display <all|cfg|err|nav|nmea|pubx|tim> <on|off>\n");
			printf("get <accuracy|cnoThreshold|dgnssTimeout|debug|delay|dop|dynamicModel|fixedAlt|fixMode|gnss|logFilter|logInfo|minElev|navRate|nmea|odo|port|pms|pm|rxm|sbas|staticHoldThresholds|supportedGNSS|timePulse|utcStandard>\n");
			printf("gnss <start|stop|reset> [hot|warm|cold]\n");
			printf("nmeaGnss <dtm|gbs|gga|ggl|gns|grs|gsa|gst|gsv|rmc|vlw|vtg|zda>\n");
			printf("nmeaGps <dtm|gbs|gga|ggl|gns|grs|gsa|gst|gsv|rmc|vlw|vtg|zda>\n");
			printf("nmeaGlonass <dtm|gbs|gga|ggl|gns|grs|gsa|gst|gsv|rmc|vlw|vtg|zda>\n");
			printf("nmeaBeidou <dtm|gbs|gga|ggl|gns|grs|gsa|gst|gsv|rmc|vlw|vtg|zda>\n");
			printf("nmeaRate <dtm|gbs|gga|ggl|gns|grs|gsa|gst|gsv|rmc|vlw|vtg|zda> [rate]\n");
			printf("open <com1|com2|usb|spi|ddc> <dev> [4800|9600|19200|38400|57600|115200|230400] [reset] [hard]\n");
			printf("poll <clock|dgps|dop|geofence|odo|orb|posecef|posllh|pvt|sat|sbas|slas|status|timebds|timegal|timeglo|timegps|timels|timeutc|tm|tp|velecef|velned> [rate]\n");
			printf("set <accuracy|cnoThreshold|dgnssTimeout|debug|delay|dop|dynamicModel|fixedAlt|fixMode|gnss|logFilter|minElev|navRate|nmea|odo|port|pms|pm|rxm|sbas|staticHoldThresholds|utcStandard> ...\n");
			printf("ubxRate <clock|dgps|dop|eoe|geofence|odo|orb|posecef|posllh|pvt|pubx_position|pubx_time|sat|sbas|slas|status|timebds|timegal|timeglo|timegps|timels|timeutc|tm|tp|velecef|velned> [rate]\n");
			printf("patches\n");
			printf("quit\n");
			printf("version\n");
		}
		else if (strncmp(cmd, "config", 5) == 0) {
			printf("config <default|load|save>\n");
			printf("  loads the default config, saves the current one or loads the saved config\n");
		}
		else if (strncmp(cmd,"debug",5) == 0) {
			printf("debug <nmea|ubx> <on|off>\n");
			printf("  turn NMEA or U-BLOX wire message debugging on or off\n");
		}
		else if (strncmp(cmd, "display", 7) == 0) {
			printf("display <all|ack|cfg|err|inf|log|mon|nav|nmea|pubx|tim> <on|off>\n");
			printf("  display GNSS messages on STDOUT (on) or not (off)\n");
		}
		else if (strncmp(cmd, "get", 3) == 0) {
			printf("get <accuracy|cnoThreshold|dgnssTimeout|debug|dop|dynamicModel|fixedAlt|fixMode|gnss|logFilter|logInfo|minElev|navRate|nmea|odo|port|pms|pm|rxm|sbas|staticHoldThresholds|supportedGNSS|timePulse|utcStandard>\n");
			getArguments();
		}
		else if (strncmp(cmd, "gnss", 4) == 0) {
			printf("gnss <start|stop|reset> [hot|warm|cold]\n");
			printf("  controls GNSS tasks only, not the entire receiver\n");
			printf("  default action is hot\n");
		}
		else if (strncmp(cmd, "set", 3) == 0) {
			printf("set <accuracy|cnoThreshold|dgnssTimeout|debug|dop|dynamicModel|fixedAlt|fixMode|gnss|logFilter|logInfo|minElev|navRate|nmea|odo|port|pms|pm|rxm|sbas|staticHoldThresholds|supportedGNSS|timePulse|utcStandard> ...\n");
			getArguments();
		}
		else if (strncmp(cmd, "open", 4) == 0) {
			printf("open <com1|com2|usb|spi|ddc> <dev> [4800|9600|19200|38400|57600|115200|230400] [reset] [hard]\n");
			printf("  Opens a GNSS connection on the specified port at the given speed\n");
			printf(" - dev - hardware port type");
			printf(" - baudRate - optional connection speed in bits per second\n");
			printf(" - reset - controlled software reset of GNSS receiver\n");
			printf(" - hard - in a combination with reset performs harware reset of GNSS receiver after a shutdown\n");
		}
		else if (strncmp(cmd, "nmeaGnss", 8) == 0) {
			printf("nmeaGnss <dtm|gbs|gga|ggl|gns|grs|gsa|gst|gsv|rmc|vlw|vtg|zda>\n");
			printf("  polls GN talker ID for an NMEA standard message:\n");
			nmeaStandardMessages();
		}
		else if (strncmp(cmd, "nmeaGps", 7) == 0) {
			printf("nmeaGps <dtm|gbs|gga|ggl|gns|grs|gsa|gst|gsv|rmc|vlw|vtg|zda>\n");
			printf("  polls GP talker ID for an NMEA standard message:\n");
			nmeaStandardMessages();
		}
		else if (strncmp(cmd, "nmeaGlonass", 11) == 0) {
			printf("nmeaGlonass <dtm|gbs|gga|ggl|gns|grs|gsa|gst|gsv|rmc|vlw|vtg|zda>\n");
			printf("  polls GL talker ID for an NMEA standard message:\n");
			nmeaStandardMessages();
		}
		else if (strncmp(cmd, "nmeaBeidou", 10) == 0) {
			printf("nmeaBeidou <dtm|gbs|gga|ggl|gns|grs|gsa|gst|gsv|rmc|vlw|vtg|zda>\n");
			printf("  polls GB talker ID for an NMEA standard message:\n");
			nmeaStandardMessages();
		}
		else if (strncmp(cmd, "nmeaRate", 8) == 0) {
			printf("nmeaRate <dtm|gbs|gga|ggl|gns|grs|gsa|gst|gsv|rmc|vlw|vtg|zda> [rate]\n");
			printf("  sets the rate for a given NMEA standard message on a given GNSS port\n");
			printf("  - 1 means send a message every navigation solution\n");
			printf("  - 2 means - every second navigation solution, etc\n");
			printf("  rate, default is 1\n");
			printf("  NMEA standard messages are:\n");
			nmeaStandardMessages();
		}
		else if (strncmp(cmd, "ubxRate", 7) == 0) {
			printf("ubxRate <clock|dgps|dop|eoe|geofence|odo|orb|posecef|posllh|pvt|pubx_position|pubx_time|sat|sbas|slas|status|timebds|timegal|timeglo|timegps|timels|timeutc|tm|tp|velecef|velned> [rate]\n");
			printf("  sets the rate for a given U-BLOX standard message\n");
			printf("  - 1 means send a message every navigation solution\n");
			printf("  - 2 means - every second navigation solution, etc\n");
			printf("  rate, default is 1\n");
			printf("  U-BLOX standard messages are:\n");
			ubxStandardMessages();
		} 
		else if (strncmp(cmd, "poll", 4) == 0) {
			printf("poll <clock|dgps|dop|geofence|odo|orb|posecef|posllh|pvt|pubx_position|pubx_time|sat|sbas|slas|status|timedbs|timegal|timeglo|timegps|timels|timeutc|tm|tp|velecef|velned>\n");
			printf("  polls for a given U-BLOX standard message on a given GNSS port\n");
			printf("  U-BLOX standard messages are:\n");
			ubxStandardMessages();
		}
		else if (strncmp(cmd, "close", 5) == 0) {
			printf("close\n");
			printf("  closes the existing gps connection\n");
		}
		else if (strncmp(cmd, "exit", 4) == 0) {
			printf("exit\n");
			printf("  exits the application\n");
		}
		else if (strncmp(cmd, "quit", 4) == 0) {
			printf("quit\n");
			printf("  quits the application\n");
		}
		else if (strncmp(cmd, "version", 7) == 0) {
			printf("version\n");
			printf("  displays GPS hardware and software versions\n");
		}
		else if (strncmp(cmd, "patches", 7) == 0) {
			printf("patches\n");
			printf("  displays GPS patches\n");
		}
	}
	return 0;
}

int poll(const char * args) {
	char a[16];
	int argc = 0;
	char * argv;
	uint8_t msgClass, msgId;

	memset(a, 0, sizeof(a));
	strncpy(a, args, sizeof(a));
	argv = strtok(a, " ");
	while (argv && argc < 2) {
		switch (argc) {
		case 0:
			msgClass = UBX_NAV;
			if (strncmp(argv, "clock", 5) == 0) msgId = NAV_CLOCK;
			else if (strncmp(argv, "dgps", 4) == 0) msgId = NAV_DGPS;
			else if (strncmp(argv, "dop", 3) == 0) msgId = NAV_DOP;
			else if (strncmp(argv, "geofence", 8) == 0) msgId = NAV_GEOFENCE;
			else if (strncmp(argv, "odo", 3) == 0) msgId = NAV_ODO;
			else if (strncmp(argv, "orb", 3) == 0) msgId = NAV_ORB;
			else if (strncmp(argv, "posecef", 7) == 0) msgId = NAV_POSECEF;
			else if (strncmp(argv, "posllh", 6) == 0) msgId = NAV_POSLLH;
			else if (strncmp(argv, "pvt", 3) == 0) msgId = NAV_PVT;
			else if (strncmp(argv, "sat", 3) == 0) msgId = NAV_SAT;
			else if (strncmp(argv, "sbas", 4) == 0) msgId = NAV_SBAS;
			else if (strncmp(argv, "slas", 4) == 0) msgId = NAV_SLAS;
			else if (strncmp(argv, "status", 6) == 0) msgId = NAV_STATUS;
			else if (strncmp(argv, "timebds", 7) == 0) msgId = NAV_TIMEBDS;
			else if (strncmp(argv, "timegal", 7) == 0) msgId = NAV_TIMEGAL;
			else if (strncmp(argv, "timeglo", 7) == 0) msgId = NAV_TIMEGLO;
			else if (strncmp(argv, "timegps", 7) == 0) msgId = NAV_TIMEGPS;
			else if (strncmp(argv, "timels", 6) == 0) msgId = NAV_TIMELS;
			else if (strncmp(argv, "timeutc", 7) == 0) msgId = NAV_TIMEUTC;
			else if (strncmp(argv, "tm", 2) == 0) {
				msgClass = UBX_TIM; msgId = TIM_TM2;
			}
			else if (strncmp(argv, "tp", 2) == 0) {
				msgClass = UBX_TIM; msgId = TIM_TP;
			}
			else if (strncmp(argv, "velecef", 7) == 0) msgId = NAV_VELECEF;
			else if (strncmp(argv, "velned", 6) == 0) msgId = NAV_VELNED;
			break;
		}
		argc++;
		argv = strtok(NULL, " ");
	}
	if (argc < 1) {
		printf("poll <clock|dgps|dop|geofence|odo|orb|posecef|posllh|pvt|sat|sbas|slas|status|timebds|timegal|timeglo|timegps|timels|timeutc|tm|tp|velecef|velned>\n");
		printf("  polls for a given U-BLOX standard message\n");
		printf("  U-BLOX standard messages are:\n");
		ubxStandardMessages();
		return -1;
	}
	//printf("msgClass %u, msgId %u, gnssPort %u, rate %u\n", msgClass, msgId, gnssPort, rate);
	if (msgClass == UBX_NAV)
		switch (msgId) {
		case NAV_CLOCK:	gps.getNavClock(); break;
		case NAV_DGPS:	gps.getNavDgps(); break;
		case NAV_DOP:	gps.getNavDop(); break;
		case NAV_GEOFENCE:	gps.getNavGeofence(); break;
		case NAV_ODO:	gps.getNavOdo(); break;
		case NAV_ORB:	gps.getNavOrb(); break;
		case NAV_POSECEF:	gps.getNavPosEcef(); break;
		case NAV_POSLLH:	gps.getNavPosLlh(); break;
		case NAV_PVT:	gps.getNavPvt(); break;
		case NAV_SBAS:	gps.getNavSbas(); break;
		case NAV_SAT:	gps.getNavSat(); break;
		case NAV_SLAS:	gps.getNavSlas(); break;
		case NAV_STATUS:	gps.getNavStatus(); break;
		case NAV_TIMEBDS:	gps.getNavTimeBds(); break;
		case NAV_TIMEGAL:	gps.getNavTimeGal(); break;
		case NAV_TIMEGLO:	gps.getNavTimeGlo(); break;
		case NAV_TIMEGPS:	gps.getNavTimeGps(); break;
		case NAV_TIMELS:	gps.getNavTimeLs(); break;
		case NAV_TIMEUTC:	gps.getNavTimeUtc(); break;
		case NAV_VELECEF:	gps.getNavVelEcef(); break;
		case NAV_VELNED:	gps.getNavVelNed(); break;
	}
	else if (msgClass = UBX_TIM)
		switch (msgId) {
		case TIM_TM2:	gps.getTimTm(); break;
		case TIM_TP:	gps.getTimTp(); break;
		}
	return 0;
}

int config(const char * args) {
	char a[16];
	int argc = 0;
	char * argv;
	bool def = false, load = false, save = false;

	memset(a, 0, sizeof(a));
	strncpy(a, args, sizeof(a));
	argv = strtok(a, " ");

	while (argv && argc < 2) {
		switch (argc) {
		case 0:
			if (strncmp(argv, "default", 7) == 0) {
				def = true;
			}
			else if (strncmp(argv, "load", 4) == 0) {
				load = true;
			}
			else if (strncmp(argv, "save", 4) == 0) {
				save = true;
			}
			break;
		}
		argc++;
		argv = strtok(NULL, " ");
	}
	if (argc < 1) {
		printf("config <default|load|save>\n");
		printf("  loads the default config, saves the current one or loads the saved config\n");
		return -1;
	}
	if (def) gps.defaultConfig();
	else if (load) gps.loadConfig();
	else if (save) gps.saveConfig();
	return 0;
}

int gnss(const char * args) {
	char a[16];
	int argc = 0;
	char * argv;
	StartTypes type = HOT_START;
	bool start = false, stop = false, reset = false;

	memset(a, 0, sizeof(a));
	strncpy(a, args, sizeof(a));
	argv = strtok(a, " ");

	while (argv && argc < 3) {
		switch (argc) {
		case 0:
			if (strncmp(argv, "start", 5) == 0) {
				start = true;
			}
			else if (strncmp(argv, "stop", 4) == 0) {
				stop = true;
			} 
			else if (strncmp(argv, "reset", 5) == 0) {
				reset = true;
			}
			break;
		case 1:
			if (strncmp(argv, "hot", 2) == 0) {
				type = HOT_START;
			}
			else if (strncmp(argv, "warm", 4) == 0) {
				type = WARM_START;
			}
			else if (strncmp(argv, "cold", 4) == 0) {
				type = COLD_START;
			}
			break;
		}
		argc++;
		argv = strtok(NULL, " ");
	}
	if (argc < 1) {
		printf("gnss <start|stop|reset> [hot|warm|cold]\n");
		printf("  controls GNSS tasks only, not the entire receiver\n");
		printf("  default action is hot\n");
		return -1;
	}
	if (start) gps.startGnss(type);
	else if (stop) gps.stopGnss(type);
	else if (reset) gps.resetGnss(type);
	return 0;
}

int set(const char * args) {
	char a[128];
	int argc = 0;
	char * argv;
	enum COMMAND : uint8_t {
		ACCURACY, CNOTRESHOLD, DGNSSTIMEOUT, DELAY, DEBUG_COMMAND, DOP_COMMAND, DYNAMICMODEL, FIXEDALT, FIXMODE,
		GNSS, LOGFILTER, MINELEV, NAVRATE, NMEA_COMMAND, ODO, PORT, PMS, PM, RXM, SBAS,
		STATICHOLDTHRESHOLDS, TIMEPULSE, UTCSTANDARD
	} command;
	uint8_t dgnssTimeout, minElev;
	CnoThreshold threshold;
	Accuracy accuracy; 
	DOP dop;
	CfgInfo cfgInfo[2];
	InfoMsgMask debugLevel; 
	Protocol proto_nmea = PROTOCOL_NONE, proto_ubx = PROTOCOL_NONE;
	DynModel model;
	FixedAlt fixedAlt;
	FixMode fixMode;
	MajorGnss gnss;
	bool enableSBAS = false, enableIMES = false;
	CfgLogFilter cfgLogFilter;
	cfgLogFilter.minInterval = 0; cfgLogFilter.positionThreshold = 0; 
	cfgLogFilter.speedThreshold = 0; cfgLogFilter.timeThreshold = 0; cfgLogFilter.version = 1;
	cfgLogFilter.flags.applyAllFilterSettings = 0; cfgLogFilter.flags.psmOncePerWakeUpEnabled = 0;
	cfgLogFilter.flags.recordingEnabled = 0; cfgLogFilter.flags.reserved = 0;

	NavRate navRate;
	CfgNmea cfgNmea;
	cfgNmea.mainTalkerId = DEFAULT_TALKER_ID; cfgNmea.nmeaVersion = 0x40; cfgNmea.maxSV = 0;
	cfgNmea.displayNonNmeaSVs = 1; cfgNmea.gsvTalkerId = 0;
	cfgNmea.bdsTalkerId[0] = 'G'; cfgNmea.bdsTalkerId[1] = 'B';
	cfgNmea.filter.failedFix = 0; cfgNmea.filter.gpsOnly = 0; cfgNmea.filter.invalidCog = 0;
	cfgNmea.filter.invalidDate = 0; cfgNmea.filter.invalidFix = 0; cfgNmea.filter.invalidTime = 0;
	cfgNmea.flags.compat = 0; cfgNmea.flags.consider = 0; cfgNmea.flags.limit82 = 0;
	cfgNmea.flags.highPrecision = 0;
	cfgNmea.gnssFilter.disableBeidou = 0; cfgNmea.gnssFilter.disableGlonass = 0;
	cfgNmea.gnssFilter.disableGps = 0; cfgNmea.gnssFilter.disableQzss = 0; 
	cfgNmea.gnssFilter.disableSbas = 0;
	ODOCfg cfgOdo;
	cfgOdo.version = 0;
	cfgOdo.cogLPgain = 0; cfgOdo.velLPgain = 0;
	cfgOdo.cogMaxPosAccuracy = 10; cfgOdo.cogMaxSpeed = 40; cfgOdo.odoProfile = SWIMMING;
	CfgPrt cfgPrt;
	cfgPrt.inProtoMask.inNmea = 1; cfgPrt.inProtoMask.inUbx = 1; cfgPrt.inProtoMask.inRtcm = 1;
	cfgPrt.outProtoMask.outNmea = 1; cfgPrt.outProtoMask.outUbx = 1;
	cfgPrt.mode.charLen = EIGHT_BIT;
	cfgPrt.mode.nStopBits = ONE_STOP_BIT;
	cfgPrt.mode.parity = NO_PARITY; 
	cfgPrt.flags.extendedTimeout = 0;
	cfgPrt.txReady.en = 0; cfgPrt.txReady.pin = 0; cfgPrt.txReady.pol = 0; cfgPrt.txReady.thres = 0;
	PowerMode powerMode;
	powerMode.period = 0; powerMode.onTime = 0; powerMode.version = 0;
	CfgPm cfgPm;
	CfgRxmLpMode cfgRxmMode;
	CfgSbas cfgSbas;
	cfgSbas.mode.testMode = 0; 
	cfgSbas.usage.diffCorr = 0; cfgSbas.usage.integrity = 0; cfgSbas.usage.range = 0;
	StaticHoldThresholds thresholds;
	debugLevel.debug = 0; debugLevel.error = 0; debugLevel.notice = 0;
	debugLevel.reserved = 0; debugLevel.test = 0; debugLevel.warning = 0;
	UtcStandard utc;

	memset(a, 0, sizeof(a)); 
	strncpy(a, args, sizeof(a));
	argv = strtok(a, " ");
	while (argv && argc < 8) {
		switch (argc) {
		case 0:
			if (strncmp(argv, "accuracy", 8) == 0) command = ACCURACY;
			else if (strncmp(argv, "cnoThreshold", 12) == 0) command = CNOTRESHOLD;
			else if (strncmp(argv, "dgnssTimeout", 12) == 0) command = DGNSSTIMEOUT;
			else if (strncmp(argv, "debug", 5) == 0) command = DEBUG_COMMAND;
			else if (strncmp(argv, "delay", 3) == 0) command = DELAY;
			else if (strncmp(argv, "dop", 3) == 0) command = DOP_COMMAND;
			else if (strncmp(argv, "dynamicModel", 12) == 0) command = DYNAMICMODEL;
			else if (strncmp(argv, "fixedAlt", 8) == 0) command = FIXEDALT;
			else if (strncmp(argv, "fixMode", 7) == 0) command = FIXMODE;
			else if (strncmp(argv, "gnss", 4) == 0) command = GNSS;
			else if (strncmp(argv, "logFilter", 9) == 0) command = LOGFILTER;
			else if (strncmp(argv, "minElev", 7) == 0) command = MINELEV;
			else if (strncmp(argv, "navRate", 7) == 0) command = NAVRATE;
			else if (strncmp(argv, "nmea", 4) == 0) command = NMEA_COMMAND;
			else if (strncmp(argv, "odo", 3) == 0) command = ODO;
			else if (strncmp(argv, "port", 4) == 0) command = PORT;
			else if (strncmp(argv, "pms", 3) == 0) command = PMS;
			else if (strncmp(argv, "pm", 2) == 0) command = PM;
			else if (strncmp(argv, "rxm", 3) == 0) command = RXM;
			else if (strncmp(argv, "sbas", 4) == 0) command = SBAS;
			else if (strncmp(argv, "staticHoldThresholds", 20) == 0) command = STATICHOLDTHRESHOLDS;
			//else if (strncmp(argv, "timePulse", 9) == 0) command = TIMEPULSE;
			else if (strncmp(argv, "utcStandard", 11) == 0) command = UTCSTANDARD;
			break;
		case 1:
			switch (command) {
			case ACCURACY: 
				accuracy.pAcc = atoi(argv); break;
				if (accuracy.pAcc == 0) {
					printf("set accuracy <position accuracy in m> <time accuracy in ms>\n");
					return -1;
				}
			case CNOTRESHOLD:
				threshold.cnoThreshold = atoi(argv); break;
				if (threshold.cnoThreshold == 0) {
					printf("set cnoThreshold <threshold in dBHz> <min number of satellites exceeding threshold>\n");
					return -1;
				}
			case DGNSSTIMEOUT:
				dgnssTimeout = atoi(argv); break;
				if (dgnssTimeout == 0) {
					printf("set dgnssTimeout <timeout in s>\n");
					return -1;
				}
			case DEBUG_COMMAND:
				if (strstr(argv, "error")) debugLevel.error = 1;
				else debugLevel.error = 0;
				if (strstr(argv, "warning")) debugLevel.warning = 1;
				else debugLevel.warning = 0;
				if (strstr(argv, "notice")) debugLevel.notice = 1;
				else debugLevel.notice = 0;
				if (strstr(argv, "test")) debugLevel.test = 1;
				else debugLevel.test = 0;
				if (strstr(argv, "debug")) debugLevel.debug = 1;
				else debugLevel.debug = 0;
				break;
			case DELAY: 
				defaultDelay = atoi(argv); 
				if (defaultDelay == 0) {
					printf("set delay <delay in s>\n");
					return -1;
				}
				break;
			case DOP_COMMAND: 
				dop.pDOP = atoi(argv) * 10;
				if (dop.pDOP == 0) {
					printf("set dop <position DOP> <time DOP>\n");
					return -1;
				}
				break;
			case DYNAMICMODEL:
				if (strncmp(argv, "portable", 8) == 0) model = PORTABLE; 
				else if (strncmp(argv, "stationary", 10) == 0) model = STATIONARY;
				else if (strncmp(argv, "pedestrian", 10) == 0) model = PEDESTRIAN;
				else if (strncmp(argv, "car", 3) == 0) model = AUTOMOTIVE;
				else if (strncmp(argv, "boat", 4) == 0) model = SEA;
				else if (strncmp(argv, "airplane_1G", 11) == 0) model = AIRBORNE_1G;
				else if (strncmp(argv, "airplane_2G", 11) == 0) model = AIRBORNE_2G;
				else if (strncmp(argv, "airplane_4G", 11) == 0) model = AIRBORNE_4G;
				else if (strncmp(argv, "watch", 5) == 0) model = WATCH;
				else {
					printf("set dynamicModel <portable|stationary|pedestrian|car|boat|airplane_1G|airplane_2G|airplane_4G|watch>\n");
					return -1;
				}
				break;
			case FIXEDALT:
				if (!isdigit(argv[0])) {
					printf("set fixedAlt <fixed altitude in m> <fixed altitude variance in sq. m>\n");
					return -1;
				}
				fixedAlt.fixedAlt = atoi(argv) * 100; 
				break;
			case FIXMODE:
				if (strncmp(argv, "2d", 2) == 0) fixMode = TWO_D_ONLY;
				else if (strncmp(argv, "3d", 2) == 0) fixMode = THREE_D_ONLY;
				else if (strncmp(argv, "auto", 4) == 0) fixMode = AUTO;
				else {
					printf("set fixMode <2d|3d|auto>\n");
					return -1;
				}
				break;
			case GNSS:
				if (strstr(argv, "gps")) gnss.Gps = 1;
				else gnss.Gps = 0;
				if (strstr(argv, "glonass")) gnss.Glonass = 1;
				else gnss.Glonass = 0;
				if (strstr(argv, "beidou")) gnss.BeiDou = 1;
				else gnss.BeiDou = 0;
				if (strstr(argv, "galileo")) gnss.Galileo = 1;
				else gnss.Galileo = 0;
				break;
			case LOGFILTER: 
				if (strncmp(argv, "enable", 6) == 0) cfgLogFilter.flags.recordingEnabled = 1;
				else if (strncmp(argv, "disable", 7) == 0) cfgLogFilter.flags.recordingEnabled = 0;
				else {
					printf("set logFilter <enable|disable> [timeThreshold in s] [minInterval in s] [speedThreshold in m/s [positionThreshold in m] [enablePSM|disablePSM]\n");
					return -1;
				}
				break;
			case MINELEV: 
				if (!isdigit(argv[0])) {
					printf("set minElev <degrees>\n");
					return -1;
				}
				minElev = atoi(argv); 
				break;
			case NAVRATE: 
				navRate.rate = atoi(argv); 
				if (navRate.rate == 0) {
					printf("set navRate <rate in ms> <navRate in cycles> <utc|gps|glonass|beidou|galileo>\n");
					return -1;
				}
				break;
			case NMEA_COMMAND: 
				if (strncmp(argv, "2.1", 3) == 0) cfgNmea.nmeaVersion = 0x21; 
				else if (strncmp(argv, "2.3", 3) == 0) cfgNmea.nmeaVersion = 0x23;
				else if (strncmp(argv, "4.0", 3) == 0) cfgNmea.nmeaVersion = 0x40;
				else if (strncmp(argv, "4.11", 4) == 0) cfgNmea.nmeaVersion = 0x4B;
				else if (strncmp(argv, "4.1", 3) == 0) cfgNmea.nmeaVersion = 0x41;
				else {
					printf("set nmea <2.1|2.3|4.0|4.1|4.11> [0|8|12|16] [gp|gl|gn|ga|gb] [useMainTalkerIdForGSV] [onlyGps] [considering,compat,limit82] [noGps,noGlonass,noBeidou,noSbas,noQzss]\n");
					return -1;
				}
				break;
			case ODO:
				if (strncmp(argv, "off", 3) == 0) cfgOdo.flags.ODOenabled = 0;
				else {
					if (strstr(argv, "on")) cfgOdo.flags.ODOenabled = 1;
					else {
						printf("set odo <off|on,useCog,outLpVel,outLpCog> [running|cycling|swimming|car|custom] [cogMaxSpeed in m/s] [cogMaxPosAcc in m] [velLpGain] [cogLpGain]\n");
						return -1;
					}
					if (strstr(argv, "useCog")) cfgOdo.flags.COGenabled = 1;
					if (strstr(argv, "outLpVel")) cfgOdo.flags.outputLPvelocity = 1;
					if (strstr(argv, "outLpCog")) cfgOdo.flags.outputLPcog = 1;
				}
				break;
			case PORT: 
				if (strncmp(argv, "4800", 4) == 0) cfgPrt.baudRate = BAUD_RATE_4800;
				else if (strncmp(argv, "9600", 4) == 0) cfgPrt.baudRate = BAUD_RATE_9600;
				else if (strncmp(argv, "14400", 5) == 0) cfgPrt.baudRate = BAUD_RATE_14400;
				else if (strncmp(argv, "19200", 5) == 0) cfgPrt.baudRate = BAUD_RATE_19200;
				else if (strncmp(argv, "38400", 5) == 0) cfgPrt.baudRate = BAUD_RATE_38400;
				else if (strncmp(argv, "57600", 5) == 0) cfgPrt.baudRate = BAUD_RATE_57600;
				else if (strncmp(argv, "115200", 6) == 0) cfgPrt.baudRate = BAUD_RATE_115200;
				else if (strncmp(argv, "230400", 6) == 0) cfgPrt.baudRate = BAUD_RATE_230400;
				else {
					printf("set port <4800|9600|14400|19200|38400|57600|115200|230400> [nmeaIn,ubxIn,rtcmIn] [nmeaOut,ubxOut]\n");
					return -1;
				}
				break;
			case PMS:
				if (strncmp(argv, "full", 4) == 0) powerMode.powerMode = FULL_POWER_MODE;
				else if (strncmp(argv, "balanced", 8) == 0) powerMode.powerMode = BALANCED_POWER_MODE;
				else if (strncmp(argv, "interval", 8) == 0) powerMode.powerMode = INTERVAL_POWER_MODE;
				else if (strncmp(argv, "1hz", 3) == 0) powerMode.powerMode = AGGRESSIVE_1HZ_POWER_MODE;
				else if (strncmp(argv, "2hz", 3) == 0) powerMode.powerMode = AGGRESSIVE_2HZ_POWER_MODE;
				else if (strncmp(argv, "4hz", 3) == 0) powerMode.powerMode = AGGRESSIVE_4HZ_POWER_MODE;
				else {
					printf("set pms <full|balanced|interval|1hz|2hz|4hz> [period in s] [on time in s]\n");
					return -1;
				}
				break;
			case PM: 
				if (!isdigit(argv[0])) {
					printf("set pm <maxStartupStateDuration in s> <updatePeriod in ms> <searchPeriod in ms> <onTime in s> <minAcqTime in s>\n");
					return -1;
				}
				cfgPm.maxStartupStateDuration = atoi(argv); 
				break;
			case RXM:
				if (strncmp(argv, "continuous", 10) == 0) cfgRxmMode = CONTINUOUS_POWER;
				else if (strncmp(argv, "powerSave", 9) == 0) cfgRxmMode = POWER_SAVE_MODE;
				else {
					printf("set rxm <continuous|powerSave>\n");
					return -1;
				}
				break;
			case SBAS: 
				if (strncmp(argv, "enable", 6) == 0) cfgSbas.mode.enabled = 1; 
				else if (strncmp(argv, "disable", 7) == 0) cfgSbas.mode.enabled = 0;
				else {
					printf("set sbas <enable|disable> [range,diffCorr,integrity] [test]\n");
					return -1;
				}
				break;
			case STATICHOLDTHRESHOLDS: 
				if (!isdigit(argv[0])) {
					printf("set staticHoldThresholds <staticHoldThreshold in cm/s> <staticHoldMaxDist in m> \n");
					return -1;
				}
				thresholds.staticHoldThreshold = atoi(argv); 
				break;
			//case TIMEPULSE: break;
			case UTCSTANDARD: 
				if (strncmp(argv, "auto", 4) == 0) utc = AUTOMATIC;
				else if (strncmp(argv, "gps", 3) == 0) utc = GPS;
				else if (strncmp(argv, "glonass", 7) == 0) utc = GLONASS;
				else if (strncmp(argv, "beidou", 6) == 0) utc = BEIDOU;
				else {
					printf("set utcStandard <auto|gps|glonass|beidou> \n");
					return -1;
				}
				break;
			} break;
		case 2:
			switch (command) {
			case ACCURACY: 
				accuracy.tAcc = atoi(argv); 
				if (accuracy.tAcc == 0) {
					printf("set accuracy <position accuracy in m> <time accuracy in ms>\n");
					return -1;
				}
				break;
			case CNOTRESHOLD: 
				if (!isdigit(argv[0])) {
					printf("set cnoThreshold <threshold in dBHz> <min number of satellites exceeding threshold>\n");
					return -1;
				}
				threshold.cnoThresholdNumSVs = atoi(argv);
				break;
			case DEBUG_COMMAND: 
				if (strstr(argv, "nmea")) proto_nmea = NMEA;
				if (strstr(argv, "ubx")) proto_ubx = UBX;
				break;
			case DOP_COMMAND: 
				dop.tDOP = atoi(argv) * 10; 
				if (dop.pDOP == 0) {
					printf("set dop <position DOP> <time DOP>\n");
					return -1;
				}
				break;
			case FIXEDALT: 
				if (!isdigit(argv[0])) {
					printf("set fixedAlt <fixed altitude in m> <fixed altitude variance in sq. m>\n");
					return -1;
				}
				fixedAlt.fixedAltVar = atoi(argv) * 10000;
				break;
			case GNSS: 
				if (strncmp(argv, "enableSBAS", 10) == 0) enableSBAS = true; 
				if (strncmp(argv, "enableIMES", 10) == 0) enableIMES = true;
				break;
			case LOGFILTER:
				if (!isdigit(argv[0])) {
					printf("set logFilter <enable|disable> [timeThreshold in s] [minInterval in s] [speedThreshold in m/s [positionThreshold in m] [enablePSM|disablePSM]\n");
					return -1;
				}
				cfgLogFilter.timeThreshold = atoi(argv); 
				break;
			case NAVRATE: 
				navRate.navSolRate = atoi(argv); 
				if (navRate.navSolRate == 0) {
					printf("set navRate <rate in ms> <navRate in cycles> <utc|gps|glonass|beidou|galileo>\n");
					return -1;
				}
				break;
			case NMEA_COMMAND: 
				if (isalpha(argv[0])) cfgNmea.maxSV = atoi(argv); 
				else if (strncmp(argv, "gp", 2) == 0) cfgNmea.mainTalkerId = GP_TALKER_ID;
				else if (strncmp(argv, "gl", 2) == 0) cfgNmea.mainTalkerId = GL_TALKER_ID;
				else if (strncmp(argv, "gn", 2) == 0) cfgNmea.mainTalkerId = GN_TALKER_ID;
				else if (strncmp(argv, "ga", 2) == 0) cfgNmea.mainTalkerId = GA_TALKER_ID;
				else if (strncmp(argv, "gb", 2) == 0) cfgNmea.mainTalkerId = GB_TALKER_ID;
				if (strstr(argv, "useMainTalkerIdForGSV")) cfgNmea.gsvTalkerId = 1;
				if (strstr(argv, "onlyGps")) cfgNmea.filter.gpsOnly = 1;
				if (strstr(argv, "considering")) cfgNmea.flags.consider = 1;
				if (strstr(argv, "compat")) cfgNmea.flags.compat = 1;
				if (strstr(argv, "limit82")) cfgNmea.flags.limit82 = 1;
				if (strstr(argv, "noGps")) cfgNmea.gnssFilter.disableGps = 1;
				if (strstr(argv, "noGlonass")) cfgNmea.gnssFilter.disableGlonass = 1;
				if (strstr(argv, "noBeidou")) cfgNmea.gnssFilter.disableBeidou = 1;
				if (strstr(argv, "noQzss")) cfgNmea.gnssFilter.disableQzss = 1;
				if (strstr(argv, "noSbas")) cfgNmea.gnssFilter.disableSbas = 1;
				break;
			case ODO: 
				if (strncmp(argv, "running", 7) == 0) cfgOdo.odoProfile = RUNNING;
				else if (strncmp(argv, "cycling", 7) == 0) cfgOdo.odoProfile = CYCLING;
				else if (strncmp(argv, "swimming", 8) == 0) cfgOdo.odoProfile = SWIMMING;
				else if (strncmp(argv, "driving", 7) == 0) cfgOdo.odoProfile = DRIVING;
				else if (strncmp(argv, "custom", 6) == 0) cfgOdo.odoProfile = CUSTOM;
				else {
					printf("set odo <off|on,useCog,outLpVel,outLpCog> [running|cycling|swimming|car|custom] [cogMaxSpeed in m/s] [cogMaxPosAcc in m] [velLpGain] [cogLpGain]\n");
					return -1;
				}
				break;
			case PORT: 
				if (strstr(argv, "nmeaIn")) cfgPrt.inProtoMask.inNmea = 1;
				if (strstr(argv, "ubxIn")) cfgPrt.inProtoMask.inUbx = 1;
				if (strstr(argv, "rtcmIn")) cfgPrt.inProtoMask.inRtcm = 1;
				if (strstr(argv, "nmeaOut")) cfgPrt.outProtoMask.outNmea = 1;
				if (strstr(argv, "ubxOut")) cfgPrt.outProtoMask.outUbx = 1;
				break;
			case PMS: 
				powerMode.period = atoi(argv); 
				if (powerMode.period == 0) {
					printf("set pms <full|balanced|interval|1hz|2hz|4hz> [period in s] [on time in s]\n");
					return -1;
				}
				break;
			case PM: 
				if (!isdigit(argv[0])) {
					printf("set pm <maxStartupStateDuration in s> <updatePeriod in ms> <searchPeriod in ms> <onTime in s> <minAcqTime in s>\n");
					return -1;
				}
				cfgPm.updatePeriod = atoi(argv);
				break;
			case SBAS: 
				if (strstr(argv, "range")) cfgSbas.usage.range = 1;
				else cfgSbas.usage.range = 0;
				if (strstr(argv, "diffCorr")) cfgSbas.usage.diffCorr = 1;
				else cfgSbas.usage.diffCorr = 0;
				if (strstr(argv, "integrity")) cfgSbas.usage.integrity = 1;
				else cfgSbas.usage.integrity = 0;
				break;
			case STATICHOLDTHRESHOLDS: 
				if (!isdigit(argv[0])) {
					printf("set staticHoldThresholds <staticHoldThreshold in cm/s> <staticHoldMaxDist in m> \n");
					return -1;
				}
				thresholds.staticHoldMaxDistance = atoi(argv);
				break;
			//case TIMEPULSE: break;
			} break;
		case 3:
			switch (command) {
			case GNSS: 
				if (strncmp(argv, "enableSBAS", 10) == 0) enableSBAS = true;
				if (strncmp(argv, "enableIMES", 10) == 0) enableIMES = true;
				break;
			case LOGFILTER: 
				if (!isdigit(argv[0])) {
					printf("set logFilter <enable|disable> [timeThreshold in s] [minInterval in s] [speedThreshold in m/s [positionThreshold in m] [enablePSM|disablePSM]\n");
					return -1;
				}
				cfgLogFilter.minInterval = atoi(argv);
				break;
			case NAVRATE: 
				if (strncmp(argv, "utc", 3) == 0) navRate.timeRef = UTC_TIME; 
				else if (strncmp(argv, "gps", 3) == 0) navRate.timeRef = GPS_TIME;
				else if (strncmp(argv, "glonass", 7) == 0) navRate.timeRef = GLONASS_TIME;
				else if (strncmp(argv, "beidou", 6) == 0) navRate.timeRef = BEIDOU_TIME;
				else if (strncmp(argv, "galileo", 7) == 0) navRate.timeRef = GALILEO_TIME;
				else {
					printf("set navRate <rate in ms> <navRate in cycles> <utc|gps|glonass|beidou|galileo>\n");
					return -1;
				}
				break;
			case NMEA_COMMAND:
				if (strncmp(argv, "gp", 2) == 0) cfgNmea.mainTalkerId = GP_TALKER_ID;
				else if (strncmp(argv, "gl", 2) == 0) cfgNmea.mainTalkerId = GL_TALKER_ID;
				else if (strncmp(argv, "gn", 2) == 0) cfgNmea.mainTalkerId = GN_TALKER_ID;
				else if (strncmp(argv, "ga", 2) == 0) cfgNmea.mainTalkerId = GA_TALKER_ID;
				else if (strncmp(argv, "gb", 2) == 0) cfgNmea.mainTalkerId = GB_TALKER_ID;
				if (strstr(argv, "useMainTalkerIdForGSV")) cfgNmea.gsvTalkerId = 1;
				if (strstr(argv, "onlyGps")) cfgNmea.filter.gpsOnly = 1;
				if (strstr(argv, "considering")) cfgNmea.flags.consider = 1;
				if (strstr(argv, "compat")) cfgNmea.flags.compat = 1;
				if (strstr(argv, "limit82")) cfgNmea.flags.limit82 = 1;
				if (strstr(argv, "noGps")) cfgNmea.gnssFilter.disableGps = 1;
				if (strstr(argv, "noGlonass")) cfgNmea.gnssFilter.disableGlonass = 1;
				if (strstr(argv, "noBeidou")) cfgNmea.gnssFilter.disableBeidou = 1;
				if (strstr(argv, "noQzss")) cfgNmea.gnssFilter.disableQzss = 1;
				if (strstr(argv, "noSbas")) cfgNmea.gnssFilter.disableSbas = 1;
				break;
			case ODO:
				if (!isdigit(argv[0])) {
					printf("set odo <off|on,useCog,outLpVel,outLpCog> [running|cycling|swimming|car|custom] [cogMaxSpeed in m/s] [cogMaxPosAcc in m] [velLpGain] [cogLpGain]\n");
					return -1;
				}
				cfgOdo.cogMaxSpeed = atoi(argv) * 10;
				break;
			case PMS:
				if (!isdigit(argv[0])) {
					printf("set pms <full|balanced|interval|1hz|2hz|4hz> [period in s] [on time in s]\n");
					return -1;
				}
				powerMode.onTime = atoi(argv); 
				break;
			case PM: 
				if (!isdigit(argv[0])) {
					printf("set pm <maxStartupStateDuration in s> <updatePeriod in ms> <searchPeriod in ms> <onTime in s> <minAcqTime in s>\n");
					return -1;
				}
				cfgPm.searchPeriod = atoi(argv);
				break;
			case SBAS:
				if (strncmp(argv, "test", 4) == 0) cfgSbas.mode.testMode = 1;
				else cfgSbas.mode.testMode = 0;
				break;
			} break;
		case 4:
			switch (command) {
			case LOGFILTER: 
				if (!isdigit(argv[0])) {
					printf("set logFilter <enable|disable> [timeThreshold in s] [minInterval in s] [speedThreshold in m/s [positionThreshold in m] [enablePSM|disablePSM]\n");
					return -1;
				}
				cfgLogFilter.speedThreshold = atoi(argv);
				break;
			case NMEA_COMMAND:
				if (strstr(argv, "useMainTalkerIdForGSV")) cfgNmea.gsvTalkerId = 1; 
				if (strstr(argv, "onlyGps")) cfgNmea.filter.gpsOnly = 1;
				if (strstr(argv, "considering")) cfgNmea.flags.consider = 1;
				if (strstr(argv, "compat")) cfgNmea.flags.compat = 1;
				if (strstr(argv, "limit82")) cfgNmea.flags.limit82 = 1;
				if (strstr(argv, "noGps")) cfgNmea.gnssFilter.disableGps = 1;
				if (strstr(argv, "noGlonass")) cfgNmea.gnssFilter.disableGlonass = 1;
				if (strstr(argv, "noBeidou")) cfgNmea.gnssFilter.disableBeidou = 1;
				if (strstr(argv, "noQzss")) cfgNmea.gnssFilter.disableQzss = 1;
				if (strstr(argv, "noSbas")) cfgNmea.gnssFilter.disableSbas = 1;
				break;
			case ODO:
				if (!isdigit(argv[0])) {
					printf("set odo <off|on,useCog,outLpVel,outLpCog> [running|cycling|swimming|car|custom] [cogMaxSpeed in m/s] [cogMaxPosAcc in m] [velLpGain] [cogLpGain]\n");
					return -1;
				}
				cfgOdo.cogMaxPosAccuracy = atoi(argv);
				break;
			case PM: 
				if (!isdigit(argv[0])) {
					printf("set pm <maxStartupStateDuration in s> <updatePeriod in ms> <searchPeriod in ms> <onTime in s> <minAcqTime in s>\n");
					return -1;
				}
				cfgPm.onTime = atoi(argv);
				break;
			} break;
		case 5:
			switch (command) {
			case LOGFILTER: 
				if (!isdigit(argv[0])) {
					printf("set logFilter <enable|disable> [timeThreshold in s] [minInterval in s] [speedThreshold in m/s [positionThreshold in m] [enablePSM|disablePSM]\n");
					return -1;
				}
				cfgLogFilter.positionThreshold = atoi(argv);
				break;
			case NMEA_COMMAND:
				if (strstr(argv, "useMainTalkerIdForGSV")) cfgNmea.gsvTalkerId = 1;
				if (strstr(argv, "onlyGps")) cfgNmea.filter.gpsOnly = 1;
				if (strstr(argv, "considering")) cfgNmea.flags.consider = 1;
				if (strstr(argv, "compat")) cfgNmea.flags.compat = 1;
				if (strstr(argv, "limit82")) cfgNmea.flags.limit82 = 1;
				if (strstr(argv, "noGps")) cfgNmea.gnssFilter.disableGps = 1;
				if (strstr(argv, "noGlonass")) cfgNmea.gnssFilter.disableGlonass = 1;
				if (strstr(argv, "noBeidou")) cfgNmea.gnssFilter.disableBeidou = 1;
				if (strstr(argv, "noQzss")) cfgNmea.gnssFilter.disableQzss = 1;
				if (strstr(argv, "noSbas")) cfgNmea.gnssFilter.disableSbas = 1;
				break;
			case ODO:
				if (!isdigit(argv[0])) {
					printf("set odo <off|on,useCog,outLpVel,outLpCog> [running|cycling|swimming|car|custom] [cogMaxSpeed in m/s] [cogMaxPosAcc in m] [velLpGain] [cogLpGain]\n");
					return -1;
				}
				cfgOdo.velLPgain = atoi(argv);
				break;
			case PM: cfgPm.minAcqTime = atoi(argv); break;
			} break;
		case 6:
			switch (command) {
			case LOGFILTER: 
				if (strncmp(argv, "enablePSM", 9) == 0) cfgLogFilter.flags.psmOncePerWakeUpEnabled = 1;
				else if (strncmp(argv, "disablePSM", 10) == 0) cfgLogFilter.flags.psmOncePerWakeUpEnabled = 0;
				else {
						printf("set logFilter <enable|disable> [timeThreshold in s] [minInterval in s] [speedThreshold in m/s [positionThreshold in m] [enablePSM|disablePSM]\n");
						return -1;
				}
			case NMEA_COMMAND:
				if (strstr(argv, "considering")) cfgNmea.flags.consider = 1;
				if (strstr(argv, "compat")) cfgNmea.flags.compat = 1;
				if (strstr(argv, "limit82")) cfgNmea.flags.limit82 = 1;
				if (strstr(argv, "noGps")) cfgNmea.gnssFilter.disableGps = 1;
				if (strstr(argv, "noGlonass")) cfgNmea.gnssFilter.disableGlonass = 1;
				if (strstr(argv, "noBeidou")) cfgNmea.gnssFilter.disableBeidou = 1;
				if (strstr(argv, "noQzss")) cfgNmea.gnssFilter.disableQzss = 1;
				if (strstr(argv, "noSbas")) cfgNmea.gnssFilter.disableSbas = 1;
				break;
			case ODO:
				if (!isdigit(argv[0])) {
					printf("set odo <off|on,useCog,outLpVel,outLpCog> [running|cycling|swimming|car|custom] [cogMaxSpeed in m/s] [cogMaxPosAcc in m] [velLpGain] [cogLpGain]\n");
					return -1;
				}
				cfgOdo.cogLPgain = atoi(argv);
				break;
			} break;
		case 7:
			switch (command) {
			case NMEA_COMMAND:
				if (strstr(argv, "noGps")) cfgNmea.gnssFilter.disableGps = 1;
				if (strstr(argv, "noGlonass")) cfgNmea.gnssFilter.disableGlonass = 1;
				if (strstr(argv, "noBeidou")) cfgNmea.gnssFilter.disableBeidou = 1;
				if (strstr(argv, "noQzss")) cfgNmea.gnssFilter.disableQzss = 1;
				if (strstr(argv, "noSbas")) cfgNmea.gnssFilter.disableSbas = 1;
				break;
			} break;
		}
		argc++;
		argv = strtok(NULL, " ");
	}
	switch (command) {
		case ACCURACY:
			if (argc < 3) {
				printf("set accuracy <position accuracy in m> <time accuracy in ms>\n");
				printf("  sets position and time accuracy masks\n");
				return -1;
			} else gps.setAccuracy(&accuracy); 
			break;
		case CNOTRESHOLD:
			if (argc < 3) {
				printf("set cnoThreshold <threshold in dBHz> <min number of satellites exceeding threshold>\n");
				printf("  sets signal-to-noise threshold and a minimum number of satellites exceeding the theshold \n");
				return -1;
			} else gps.setCnoThreshold(&threshold); 
			break;
		case DEBUG_COMMAND:
			if (argc < 3) {
				printf("set debug <error,warning,notice,test,debug> <nmea,ubx>\n");
				printf("  sets debug level for NMEA and/or U-BLOX information messages\n");
				return -1;
			}
			else {
				if (proto_nmea == NMEA) {
					cfgInfo[1].protocolId = NMEA;
					cfgInfo[1].mask[gnssPort].debug = debugLevel.debug;
					cfgInfo[1].mask[gnssPort].error = debugLevel.error;
					cfgInfo[1].mask[gnssPort].notice = debugLevel.notice;
					cfgInfo[1].mask[gnssPort].test = debugLevel.test;
					cfgInfo[1].mask[gnssPort].warning = debugLevel.warning;
					gps.setCfgInf(&cfgInfo[1], gnssPort);
				}
				if (proto_ubx == UBX) {
					cfgInfo[0].protocolId = UBX;
					cfgInfo[0].mask[gnssPort].debug = debugLevel.debug;
					cfgInfo[0].mask[gnssPort].error = debugLevel.error;
					cfgInfo[0].mask[gnssPort].notice = debugLevel.notice;
					cfgInfo[0].mask[gnssPort].test = debugLevel.test;
					cfgInfo[0].mask[gnssPort].warning = debugLevel.warning;
					gps.setCfgInf(&cfgInfo[0], gnssPort);
				}
				if (proto_nmea == PROTOCOL_NONE && proto_ubx == PROTOCOL_NONE) {
					printf("set debug <error,warning,notice,test,debug> <nmea,ubx>\n");
					return -1;
				}
			}
			break;
		case DGNSSTIMEOUT:
			if (argc < 2) {
				printf("set dgnssTimeout <timeout in s>\n");
				printf("  sets DGNSS timeout in seconds\n");
				return -1;
			}
			else gps.setDgnssTimeout(dgnssTimeout);
			break;
		case DELAY:
			if (argc < 2) {
				printf("set delay <delay in s>\n");
				printf("  sets a timeout waiting for response from a GNSS receiver, default is 1s\n");
				return -1;
			}
			break;
		case DOP_COMMAND:
			if (argc < 3) {
				printf("set dop <position DOP> <time DOP>\n");
				printf("  sets position and time dilution of precision masks to use\n");
				return -1;
			}
			else gps.setDop(&dop);
			break;
		case DYNAMICMODEL:
			if (argc < 2) {
				printf("set dynamicModel <portable|stationary|pedestrian|car|boat|airplane_1G|airplane_2G|airplane_4G|watch>\n");
				printf("  sets dynamic platform model\n");
				return -1;
			}
			else gps.setDynamicModel(model);
			break;
		case FIXEDALT:
			if (argc < 3) {
				printf("set fixedAlt <fixed altitude in m> <fixed altitude variance in sq. m>\n");
				printf("  sets fixed altitude (or mean sea level) and its variance for 2D fix mode\n");
				return -1;
			}
			else gps.setFixedAlt(&fixedAlt);
			break;
		case FIXMODE:
			if (argc < 2) {
				printf("set fixMode <2d|3d|auto>\n");
				printf("  sets position fixing mode\n");
				return -1;
			}
			else gps.setFixMode(fixMode);
			break;
		case GNSS:
			if (argc < 2) {
				printf("set gnss <gps,glonass,beidou,galileo> [enableSBAS] [enableIMES]\n");
				printf("  sets major GNSS, not all combinations are possible\n");
				return -1;
			}
			else gps.setGnss(gnss, enableSBAS, enableIMES);
			break;
		case LOGFILTER:
			if (argc < 2) {
				printf("set logFilter <enable|disable> [timeThreshold in s] [minInterval in s] [speedThreshold in m/s [positionThreshold in m] [enablePSM|disablePSM]\n");
				printf("  sets data logger configuration\n");
				printf("  - minInterval only applies in combination with speed and/or position threshold\n");
				printf("  If both minInterval and timeThreshold are set, minInterval must be less than or equal to timeThreshold\n");
				printf("  - enable[disable]PSM - records only one position per wake up, disabled by default\n");
				return -1;
			}
			else gps.setCfgLogFilter(&cfgLogFilter);
			break;
		case MINELEV:
			if (argc < 2) {
				printf("set minElev <degrees>\n");
				printf("  sets minimum elevation for a GNSS satellite to be used\n");
				return -1;
			}
			else gps.setMinElev(minElev);
			break;
		case NAVRATE:
			if (argc < 4) {
				printf("set navRate <rate in ms> <navRate in cycles> <utc|gps|glonass|beidou|galileo>\n");
				printf("  sets navigation/measurement rate settings\n");
				printf("  - rate is a measurement rate, it must be >= 50ms\n");
				printf("  - navRate value defines that every n-th measurement triggers a navigation epoch;\n");
				printf("    it is an integer multiple of the measurement period\n");
				printf("  - last parameter sets the time system to which measurements are aligned\n");
				return -1;
			}
			else gps.setNavRate(&navRate);
			break;
		case NMEA_COMMAND:
			if (argc < 2) {
				printf("set nmea <2.1|2.3|4.0|4.1|4.11> [0|8|12|16] [gp|gl|gn|ga|gb] [useMainTalkerIdForGSV] [onlyGps] [considering,compat,limit82] [noGps,noGlonass,noBeidou,noSbas,noQzss]\n");
				printf("  configures NMEA protocol \n");
				printf("  - NMEA protocol version, default is 4.0\n");
				printf("  - max number of satellite to show, default is 0 - unlimited\n");
				printf("  - overwrite main talker ID\n");
				printf("  - onlyGps - restricts output to GPS satellites only\n");
				printf("  - considering - shows all satellites considered for navigation; otherwise, just those in use\n");
				printf("  - compat - enable when parser expects a fixed number of digits in position coordinates\n");
				printf("  - limit82 - limits the size of the output line to 82 chars\n");
				printf("  - last set of options excludes certain GNSS systems from the reporting\n");
				return -1;
			}
			else gps.setCfgNmea(&cfgNmea);
			break;
		case ODO:
			if (argc < 2) {
				printf("set odo <off|on,useCog,outLpVel,outLpCog> [running|cycling|swimming|car|custom] [cogMaxSpeed in m/s] [cogMaxPosAcc in m] [velLpGain] [cogLpGain]\n");
				printf("  configures odometer \n");
				printf("  - the first set of options enables the odometer and low-pass filters\n");
				printf("  - the second argument optionally configures odometer profile\n");
				printf("  - cogMaxSpeed - speed below which COG is computed with the low-speed COG filter\n");
				printf("  - cogMaxPosAcc - maximum acceptable position accuracy for computing COG with the low-speed COG filter\n");
				printf("  - velLpGain - velocity low-pass filter level, range 0..255\n");
				printf("  - cogLpGain - COG low-pass filter level (at speed < 8m/s), range 0..255\n");
				return -1;
			}
			else gps.setCfgOdo(&cfgOdo);
			break;
		case PORT:
			if (argc < 2) {
				printf("set port <4800|9600|14400|19200|38400|57600|115200|230400> [nmeaIn,ubxIn,rtcmIn,nmeaOut,ubxOut]\n");
				printf("  configures the COM port\n");
				printf("  - first argument is a baud rate\n");
				printf("  Other communication parameters are read-only in this version:\n");
				printf("  8 bits per char, 1 stop bit, no parity and no flow control\n");
				printf("  - second argument enables inbound and outbound protocols\n");
				return -1;
			}
			else {
				cfgPrt.portId = gnssPort;
				gps.setCfgPrt(&cfgPrt);
			}
			break;
		case PMS:
			if (argc < 2) {
				printf("set pms <full|balanced|interval|1hz|2hz|4hz> [period in s] [on time in s]\n");
				printf("  configures power mode\n");
				printf("  - first argument is a power mode\n");
				printf("  - second argument is position update and search period\n");
				printf("    Recommended minimum period is 10s, although the receiver accepts any value bigger than 5s\n");
				printf("    Only valid when power mode is set to interval\n");
				printf("  - third argument is duration of the ON phase, must be smaller than the period\n");
				printf("    Only valid when power mode is set to interval\n");
				return -1;
			}
			else gps.setCfgPms(&powerMode);
			break;
		case PM:
			if (argc < 2) {
				printf("set pm <maxStartupStateDuration in s> <updatePeriod in ms> <searchPeriod in ms> <onTime in s> <minAcqTime in s>\n");
				printf("  configures extended power management features\n");
				printf("  - first argument is the maximum time to spend in Acquisition state. 0 means forever\n");
				printf("  - second argument is position update period\n");
				printf("    If set to 0, the receiver will never retry a fix and it will wait for external events\n");
				printf("  - third argument is Acquisition retry period if previously failed\n");
				printf("    If set to 0, the receiver will never retry a startup\n");
				printf("  - the fourth argument is the time to stay in Tracking state\n");
				printf("  - the last argument is the minimal search time\n");
				return -1;
			}
			else gps.setCfgPm(&cfgPm);
			break;
		case RXM:
			if (argc < 2) {
				printf("set rxm <continuous|powerSave>\n");
				printf("  sets the receiver power mode\n");
				return -1;
			}
			else gps.setCfgRxm(cfgRxmMode);
			break;
		case SBAS:
			if (argc < 2) {
				printf("set sbas <enable|disable> [range,diffCorr,integrity] [test]\n");
				printf("  configures SBAS\n");
				printf("  - first argument enables or disables SBAS\n");
				printf("  - second argument sets the usage:\n");
				printf("  - range - use SBAS GEOs as a ranging source (for navigation)\n");
				printf("  - diffCorr - use SBAS differential corrections\n");
				printf("  - integrity - use SBAS integrity information\n");
				printf("  - test - uses test data if set\n");
				return -1;
			}
			else gps.setCfgSbas(&cfgSbas);
			break;
		case STATICHOLDTHRESHOLDS:
			if (argc < 2) {
				printf("set staticHoldThresholds <staticHoldThreshold in cm/s> <staticHoldMaxDist in m> \n");
				printf("  configures static hold threshold and static hold distance threshold\n");
				return -1;
			}
			else gps.setStaticHoldThresholds(&thresholds);
			break;
		//case TIMEPULSE: break;
		case UTCSTANDARD: 
			if (argc < 2) {
				printf("set utcStandard <auto|gps|glonass|beidou> \n");
				printf("  sets UTC standard\n");
				return -1;
			}
			else gps.setUtcStandard(utc);
			break;
	}
		
	return 0;
}

int debug(const char * args) {
	char a[32];
	int argc = 0;
	char * argv;
	bool nmea = false, ubx = false;

	memset(a, 0, sizeof(a));
	strncpy(a, args, sizeof(a));
	argv = strtok(a, " ");

	while (argv && argc < 2) {
		switch (argc) {
		case 0:
			if (strncmp(argv, "nmea", 4) == 0) {
				nmea = true;
			}
			else if (strncmp(argv, "ubx", 3) == 0) {
				ubx = true;
			}
			else {
				printf("debug <nmea|ubx> <on|off>\n");
				return -1;
			}
			break;
		case 1:
			if (strncmp(argv, "on", 2) == 0) {
				if (nmea) DEBUG_NMEA = true;
				else if (ubx) DEBUG_UBX = true;
			}
			else if (strncmp(argv, "off", 3) == 0) {
				if (nmea) DEBUG_NMEA = false;
				else if (ubx) DEBUG_UBX = false;
			} 
			else {
				printf("debug <nmea|ubx> <on|off>\n");
				return -1;
			}
			break;
		}

		argc++;
		argv = strtok(NULL, " ");
	}
	if (argc < 2) {
		printf("debug <nmea|ubx> <on|off>\n");
		printf("  turn NMEA or U-BLOX wire messages debugging on or off\n");
		return -1;
	}
	return 0;
}

int display(const char * args) {
	char a[32];
	int argc = 0;
	char * argv;
	enum MSG_CLASS : uint8_t {
		MSG_CLASS_ALL,
		MSG_CLASS_ACK,
		MSG_CLASS_CFG,
		MSG_CLASS_ERR,
		MSG_CLASS_INF,
		MSG_CLASS_LOG,
		MSG_CLASS_MON,
		MSG_CLASS_NAV,
		MSG_CLASS_NMEA,
		MSG_CLASS_PUBX,
		MSG_CLASS_TIM
	} msg_class;
	bool disp_all = true, disp_nav = true, disp_cfg = true, disp_tim = true, disp_nmea = true, disp_pubx = true, disp_err = true;
	bool on_off = true;

	memset(a, 0, sizeof(a));
	strncpy(a, args, sizeof(a));
	argv = strtok(a, " ");

	while (argv && argc < 2) {
		switch (argc) {
		case 0:
			if (strncmp(argv, "all", 3) == 0) {
				msg_class = MSG_CLASS_ALL;
			}
			else if (strncmp(argv, "cfg", 3) == 0) {
				msg_class = MSG_CLASS_CFG;
			}
			else if (strncmp(argv, "err", 3) == 0) {
				msg_class = MSG_CLASS_ERR;
			}
			else if (strncmp(argv, "nav", 3) == 0) {
				msg_class = MSG_CLASS_NAV;
			}
			else if (strncmp(argv, "nmea", 3) == 0) {
				msg_class = MSG_CLASS_NMEA;
			}
			else if (strncmp(argv, "pubx", 3) == 0) {
				msg_class = MSG_CLASS_PUBX;
			}
			else if (strncmp(argv, "tim", 3) == 0) {
				msg_class = MSG_CLASS_TIM;
			}
			else {
				printf("display <all|cfg|err|nav|nmea|pubx|tim> <on|off>\n");
				return -1;
			}
			break;
		case 1:
			if (strncmp(argv, "on", 2) == 0) {
				on_off = true;
			}
			else if (strncmp(argv, "off", 3) == 0) {
				on_off = false;
			}
			else {
				printf("display <all|cfg|err|nav|nmea|pubx|tim> <on|off>\n");
				return -1;
			}
			break;
		}
		argc++;
		argv = strtok(NULL, " ");
	}
	if (argc < 2) {
		printf("display <all|cfg|err|nav|nmea|pubx|tim> <on|off>\n");
		printf("  display GNSS messages on STDOUT (on) or not (off)\n");
		return -1;
	}
	switch (msg_class) {
	case MSG_CLASS_ALL:
		gps.disp_cfg = on_off;
		gps.disp_err = on_off;
		gps.disp_nav = on_off;
		gps.disp_nmea = on_off;
		gps.disp_pubx = on_off;
		gps.disp_tim = on_off;
		break;
	case MSG_CLASS_CFG:
		gps.disp_cfg = on_off;
		break;
	case MSG_CLASS_ERR:
		gps.disp_err = on_off;
		break;
	case MSG_CLASS_NAV:
		gps.disp_nav = on_off;
		break;
	case MSG_CLASS_NMEA:
		gps.disp_nmea = on_off;
		break;
	case MSG_CLASS_PUBX:
		gps.disp_pubx = on_off;
		break;
	case MSG_CLASS_TIM:
		gps.disp_tim = on_off;
		break;
	}
	return 0;
}

int get(const char * args) {
	char a[32];
	int argc = 0;
	char * argv;

	memset(a, 0, sizeof(a));
	strncpy(a, args, sizeof(a));
	argv = strtok(a, " ");
	while (argv && argc < 1) {
		if (strncmp(argv, "accuracy", 8) == 0) {
			Accuracy acc;
			if (gps.getAccuracy(&acc))
				printf("Position accuracy mask %um, time accuracy mask %ums\n", acc.pAcc, acc.tAcc);
			else printf("Failed to get accuracy masks\n");
		}
		else if (strncmp(argv, "cnoThreshold", 12) == 0) {
			CnoThreshold cno;
			if (gps.getCnoThreshold(&cno)) {
				printf("Signal-to-noise threshold %udBHz\n", cno.cnoThreshold);
				printf("Number of satellites required to have Signal-Noise ratio above cnoThreshold for a fix to be attempted %u\n", cno.cnoThresholdNumSVs);
			}
			else printf("Failed to get signal-to-noise threshold info\n");
		}
		else if (strncmp(argv, "dgnssTimeout", 12) == 0) {
			printf("DgnssTimeout %u\n", gps.getDgnssTimeout());
		}
		else if (strncmp(argv, "delay", 5) == 0) {
			printf("delay %u\n", defaultDelay);
		}
		else if (strncmp(argv, "dop", 3) == 0) {
			DOP dop;
			if (gps.getDop(&dop)) printf("Position DOP mask to use %u, Time DOP mask to use %u\n", dop.pDOP / 10, dop.tDOP / 10);
			else printf("Failed to get DOP masks\n");
		}
		else if (strncmp(argv, "debug", 5) == 0) {
			if (!gps.getCfgInf(UBX)) printf("Failed to get UBX protocol debug level\n");
			if (!gps.getCfgInf(NMEA)) printf("Failed to get NMEA protocol debug level\n");
		}
		else if (strncmp(argv, "odo", 3) == 0) {
			if (!gps.getCfgOdo()) printf("Failed to get odometer configuration\n");
		}
		else if (strncmp(argv, "dynamicModel", 12) == 0) {
			char model[32]; memset(model, 0, sizeof(model));
			switch (gps.getDynamicModel()) {
			case PORTABLE: sprintf(model, "Portable"); break;
			case STATIONARY: sprintf(model, "Stationary"); break;
			case PEDESTRIAN: sprintf(model, "Pedestrian"); break;
			case AUTOMOTIVE: sprintf(model, "Car"); break;
			case SEA: sprintf(model, "Boat"); break;
			case AIRBORNE_1G: sprintf(model, "Airplane 1G"); break;
			case AIRBORNE_2G: sprintf(model, "Airplane 2G"); break;
			case AIRBORNE_4G: sprintf(model, "Airplane 4G"); break;
			case WATCH: sprintf(model, "Watch"); break;
			case MODEL_ERROR: sprintf(model, "Error"); break;
			default: sprintf(model, "unknown"); break;
			}
			printf("Current dynamic model is %s\n", model);
		}
		else if (strncmp(argv, "fixedAlt", 8) == 0) {
			FixedAlt fixedAlt;
			if (gps.getFixedAlt(&fixedAlt)) printf("Fixed altitude (mean sea level) for 2D fix mode %dm, variance %usq.m\n", fixedAlt.fixedAlt / 100, fixedAlt.fixedAltVar / 10000);
			else printf("Failed to get fixed altitude\n");
		}
		else if (strncmp(argv, "fixMode", 7) == 0) {
			char mode[16]; memset(mode, 0, sizeof(mode));
			switch (gps.getFixMode()) {
			case TWO_D_ONLY: sprintf(mode, "2D"); break;
			case THREE_D_ONLY: sprintf(mode, "3D"); break;
			case AUTO: sprintf(mode, "Auto"); break;
			case FIX_MODE_ERROR: sprintf(mode, "Error"); break;
			default:  sprintf(mode, "unknown"); break;
			}
			printf("Position fix mode is set to %s\n", mode);
		}
		else if (strncmp(argv, "gnss", 4) == 0) {
			if (!gps.getGnss()) printf("Failed to get GNSS\n");
		}
		else if (strncmp(argv, "logFilter", 9) == 0) {
			if (!gps.getCfgLogFilter()) printf("Failed to get LogFilter\n");
		}
		else if (strncmp(argv, "logInfo", 7) == 0) {
			gps.getLogInfo();
		}
		else if (strncmp(argv, "minElev", 7) == 0) {
			printf("Minimum elevation for a GNSS satellite to be used is %d degrees\n", gps.getMinElev());
		}
		else if (strncmp(argv, "navRate", 7) == 0) {
			if (!gps.getNavRate()) printf("Failed to get NavRate\n");
		}
		else if (strncmp(argv, "nmea", 4) == 0) {
			if (!gps.getCfgNmea()) printf("Failed to get NMEA configuration\n");
		}
		else if (strncmp(argv, "odo", 3) == 0) {
			if (!gps.getCfgOdo()) printf("Failed to get odometer configuration\n");
		}
		else if (strncmp(argv, "port", 4) == 0) {
			if (!gps.getCfgPrt(gnssPort)) printf("Failed to get configuration of an I/O port\n");
		}
		else if (strncmp(argv, "pms", 3) == 0) {
			if (!gps.getCfgPms()) printf("Failed to get power management system configuration\n");
		}
		else if (strncmp(argv, "pm", 2) == 0) {
			if (!gps.getCfgPm()) printf("Failed to extended power management configuration\n");
		}
		else if (strncmp(argv, "rxm", 3) == 0) {
			if (!gps.getCfgRxm()) printf("Failed to get power saving configuration\n");
		}
		else if (strncmp(argv, "sbas", 4) == 0) {
			if (!gps.getCfgSbas()) printf("Failed to get SBAS configuration\n");
		}
		else if (strncmp(argv, "staticHoldThresholds", 20) == 0) {
			StaticHoldThresholds thresholds;
			if (gps.getStaticHoldThresholds(&thresholds)) {
				printf("Static hold threshold %u cm/s\n", thresholds.staticHoldThreshold);
				printf("Static hold distance threshold (before quitting static hold) %um\n", thresholds.staticHoldMaxDistance);
			}
			else printf("Failed to get StaticHoldThresholds\n");
		}
		else if (strncmp(argv, "supportedGNSS", 13) == 0) {
			gps.getSupportedGnss();
		}
		else if (strncmp(argv, "timePulse", 2) == 0) {
			if (!gps.getTimePulse()) printf("Failed to get time pulse settings\n");
		}
		else if (strncmp(argv, "utcStandard", 6) == 0) {
			char utc[16]; memset(utc, 0, sizeof(utc));
			switch (gps.getUtcStandard()) {
			case AUTOMATIC: sprintf(utc, "Auto"); break;
			case GPS: sprintf(utc, "GPS (USNO"); break;
			case GLONASS: sprintf(utc, "GLONASS (SU)"); break;
			case BEIDOU: sprintf(utc, "BeiDou (NTSC)"); break;
			case UTC_ERROR: sprintf(utc, "Error"); break;
			default: sprintf(utc, "unknown"); break;
			}
			printf("UTC standard in use is %s\n", utc);
		}

		argc++;
		argv = strtok(NULL, " ");
	}
	if (argc < 1) {
		printf("get <accuracy|cnoThreshold|dgnssTimeout|debug|delay|dop|dynamicModel|fixedAlt|fixMode|gnss|logFilter|logInfo|minElev|navRate|nmea|odo|port|pms|pm|rxm|sbas|staticHoldThresholds|supportedGNSS|timePulse|utcStandard>\n");
		getArguments();
		return -1;
	}
	return 0;
}

int open(const char * args) {
	char a[128];
	int argc = 0;
	char * argv, * dev;
	speed_t rate;
	bool reset = false, soft = true;

	memset(a, 0, sizeof(a));
	strncpy(a, args, sizeof(a));
	argv = strtok(a, " ");
	while (argv && argc < 5) {
		switch (argc) {
			case 0: 
				if (strncmp(argv, "ddc", 3) == 0) gnssPort = DDC;
				else if (strncmp(argv, "com1", 4) == 0) gnssPort = COM1;
				else if (strncmp(argv, "com2", 4) == 0) gnssPort = COM2;
				else if (strncmp(argv, "usb", 3) == 0) gnssPort = USB;
				else if (strncmp(argv, "spi", 3) == 0) gnssPort = SPI;
				else gnssPort = COM1;
				break;
			case 1: dev = argv; break;
			case 2:
				if (isdigit(argv[0])) gnssPortRate = (speed_t)atoi(argv); 
				else if (strncmp(argv, "reset", 5) == 0) reset = true;
				break;
			case 3: 
				if (strncmp(argv, "reset", 5) == 0) reset = true; 
				else if (strncmp(argv, "hard", 4) == 0) soft = false;
				break;
			case 4: 
				if (strncmp(argv, "hard", 4) == 0) soft = false; 
				break;
		}
		argc++;
		argv = strtok(NULL, " ");
	}
	if (argc < 2) {
		printf("open <com1|com2|usb|spi|ddc> <dev> [4800|9600|19200|38400|57600|115200|230400] [reset] [hard]\n");
		printf("  Opens a GNSS connection on the specified port at the given speed\n");
		printf(" - dev - hardware port type");
		printf(" - baudRate - optional connection speed in bits per second\n");
		printf(" - reset - controlled software reset of GNSS receiver\n");
		printf(" - hard - in a combination with reset performs harware reset of GNSS receiver after a shutdown\n");
		return -1;
	}
	switch (gnssPortRate) {
		case 4800: rate = B4800; break;
		case 9600: rate = B9600; break;
		case 19200: rate = B19200; break;
		case 38400: rate = B38400; break;
		case 57600: rate = B57600; break;
		case 115200: rate = B115200; break;
		case 230400: rate = B230400; break;
	}
	setup(dev, rate, reset, soft);
	return 0;
}

int nmeaGnss(const char * args) {
	char a[16];
	int argc = 0;
	char * argv, *arg;

	memset(a, 0, sizeof(a));
	strncpy(a, args, sizeof(a));
	argv = strtok(a, " ");
	while (argv && argc < 1) {
		if (argc == 0) arg = argv;
		argc++;
		argv = strtok(NULL, " ");
	}
	if (argc < 1) {
		printf("nmeaGnss <dtm|gbs|gga|ggl|gns|grs|gsa|gst|gsv|rmc|vlw|vtg|zda>\n");
		printf("  polls GN talker ID for an NMEA standard message:\n");
		nmeaStandardMessages();
		return -1;
	}
	int i;
	while (arg[i]) { arg[i] = toupper(arg[i]); i++; }
	gps.nmeaGnq(arg);
	return 0;
}

int nmeaGps(const char * args) {
	char a[16];
	int argc = 0;
	char * argv, *arg;

	memset(a, 0, sizeof(a));
	strncpy(a, args, sizeof(a));
	argv = strtok(a, " ");
	while (argv && argc < 1) {
		if (argc == 0) arg = argv;
		argc++;
		argv = strtok(NULL, " ");
	}
	if (argc < 1) {
		printf("nmeaGps <dtm|gbs|gga|ggl|gns|grs|gsa|gst|gsv|rmc|vlw|vtg|zda>\n");
		printf("  polls GP talker ID for an NMEA standard message:\n");
		nmeaStandardMessages();
		return -1;
	}
	int i;
	while (arg[i]) { arg[i] = toupper(arg[i]); i++; }
	gps.nmeaGpq(arg);
	return 0;
}

int nmeaGlonass(const char * args) {
	char a[16];
	int argc = 0;
	char * argv, *arg;

	memset(a, 0, sizeof(a));
	strncpy(a, args, sizeof(a));
	argv = strtok(a, " ");
	while (argv && argc < 1) {
		if (argc == 0) arg = argv;
		argc++;
		argv = strtok(NULL, " ");
	}
	if (argc < 1) {
		printf("nmeaGlonass <dtm|gbs|gga|ggl|gns|grs|gsa|gst|gsv|rmc|vlw|vtg|zda>\n");
		printf("  polls GL talker ID for an NMEA standard message:\n");
		nmeaStandardMessages();
		return -1;
	}
	int i;
	while (arg[i]) { arg[i] = toupper(arg[i]); i++; }
	gps.nmeaGlq(arg);
	return 0;
}

int nmeaBeidou(const char * args) {
	char a[16];
	int argc = 0;
	char * argv, *arg;

	memset(a, 0, sizeof(a));
	strncpy(a, args, sizeof(a));
	argv = strtok(a, " ");
	while (argv && argc < 1) {
		if (argc == 0) arg = argv;
		argc++;
		argv = strtok(NULL, " ");
	}
	if (argc < 1) {
		printf("nmeaBeidou <dtm|gbs|gga|ggl|gns|grs|gsa|gst|gsv|rmc|vlw|vtg|zda>\n");
		printf("  polls GB talker ID for an NMEA standard message:\n");
		nmeaStandardMessages();
		return -1;
	}
	int i;
	while (arg[i]) { arg[i] = toupper(arg[i]); i++; }
	gps.nmeaGbq(arg);
	return 0;
}

int nmeaRate(const char * args) {
	char a[16];
	int argc = 0;
	char * argv, *msg;
	uint8_t rate = 1, rateCom1 = 0, rateCom2 = 0, rateUsb = 0, rateDdc = 0, rateSpi = 0;

	memset(a, 0, sizeof(a));
	strncpy(a, args, sizeof(a));
	argv = strtok(a, " ");
	while (argv && argc < 2) {
		switch (argc) {
		case 0:
			msg = argv;
			break;
		case 1: rate = (uint8_t)atoi(argv); break;
		}
		argc++;
		argv = strtok(NULL, " ");
	}
	if (argc < 1) {
		printf("nmeaRate <DTM|GBS|GGA|GLL|GNS|GRS|GSA|GST|GSV|RMC|VLW|VTG|ZDA> [rate]\n");
		printf("  sets the rate for a given NMEA standard message on a given GNSS port\n");
		printf("  - 1 means send a message every navigation solution\n");
		printf("  - 2 means - every second navigation solution, etc\n");
		printf("  rate, default is 1\n");
		printf("  NMEA standard messages are:\n");
		nmeaStandardMessages();
		return -1;
	}
	switch (gnssPort) {
	case COM1:
		rateCom1 = rate; break;
	case COM2:
		rateCom2 = rate; break;
	case USB:
		rateUsb = rate; break;
	case DDC:
		rateDdc = rate; break;
	case SPI:
		rateSpi = rate; break;
	}
	int i;
	while (msg[i]) { msg[i] = toupper(msg[i]); i++; }
	gps.pubxRate(msg, rateCom1, rateCom2, rateUsb, rateDdc, rateSpi);
	return 0;
}

int ubxRate(const char * args) {
	char a[16];
	int argc = 0;
	char * argv;
	uint8_t msgClass, msgId;
	uint8_t rate = 1, rateCom1 = 0, rateCom2 = 0, rateUsb = 0, rateDdc = 0, rateSpi = 0;

	memset(a, 0, sizeof(a));
	strncpy(a, args, sizeof(a));
	argv = strtok(a, " ");
	while (argv && argc < 2) {
		switch (argc) {
		case 0:
			msgClass = UBX_NAV;
			if (strncmp(argv, "clock", 5) == 0) msgId = NAV_CLOCK;
			else if (strncmp(argv, "dgps", 4) == 0) msgId = NAV_DGPS;
			else if (strncmp(argv, "dop", 3) == 0) msgId = NAV_DOP;
			else if (strncmp(argv, "geofence", 8) == 0) msgId = NAV_GEOFENCE;
			else if (strncmp(argv, "eoe", 3) == 0) msgId = NAV_EOE;
			else if (strncmp(argv, "odo", 3) == 0) msgId = NAV_ODO;
			else if (strncmp(argv, "orb", 3) == 0) msgId = NAV_ORB;
			else if (strncmp(argv, "posecef", 7) == 0) msgId = NAV_POSECEF;
			else if (strncmp(argv, "posllh", 6) == 0) msgId = NAV_POSLLH;
			else if (strncmp(argv, "pvt", 3) == 0) msgId = NAV_PVT;
			else if (strncmp(argv, "pubx_position", 13) == 0) {
				msgClass = UBX_PUBX; msgId = PUBX_POSITION;
			}
			else if (strncmp(argv, "pubx_time", 9) == 0) {
				msgClass = UBX_PUBX; msgId = PUBX_TIME;
			}
			else if (strncmp(argv, "sat", 3) == 0) msgId = NAV_SAT;
			else if (strncmp(argv, "sbas", 4) == 0) msgId = NAV_SBAS;
			else if (strncmp(argv, "slas", 4) == 0) msgId = NAV_SLAS;
			else if (strncmp(argv, "status", 6) == 0) msgId = NAV_STATUS;
			else if (strncmp(argv, "timebds", 7) == 0) msgId = NAV_TIMEBDS;
			else if (strncmp(argv, "timegal", 7) == 0) msgId = NAV_TIMEGAL;
			else if (strncmp(argv, "timeglo", 7) == 0) msgId = NAV_TIMEGLO;
			else if (strncmp(argv, "timegps", 7) == 0) msgId = NAV_TIMEGPS;
			else if (strncmp(argv, "timels", 6) == 0) msgId = NAV_TIMELS;
			else if (strncmp(argv, "timeutc", 7) == 0) msgId = NAV_TIMEUTC;
			else if (strncmp(argv, "tm", 2) == 0) {
				msgClass = UBX_TIM; msgId = TIM_TM2;
			}
			else if (strncmp(argv, "tp", 2) == 0) {
				msgClass = UBX_TIM; msgId = TIM_TP;
			}
			else if (strncmp(argv, "velecef", 7) == 0) msgId = NAV_VELECEF;
			else if (strncmp(argv, "velned", 6) == 0) msgId = NAV_VELNED;
			break;
		case 1: rate = (uint8_t)atoi(argv); break;
		}
		argc++;
		argv = strtok(NULL, " ");
	}
	if (argc < 1) {
		printf("ubxRate <clock|dgps|dop|eoe|geofence|odo|orb|posecef|posllh|pvt|pubx_position|pubx_time|sat|sbas|slas|status|timedbs|timegal|timeglo|timegps|timels|timeutc|tm|tp|velecef|velned> [rate]\n");
		printf("  sets the rate for a given U-BLOX standard message on a given GNSS port\n");
		printf("  - 1 means send a message every navigation solution\n");
		printf("  - 2 means - every second navigation solution, etc\n");
		printf("  rate, default is 1\n");
		printf("  U-BLOX standard messages are:\n");
		ubxStandardMessages();
		return -1;
	}
	//printf("msgClass %u, msgId %u, gnssPort %u, rate %u\n", msgClass, msgId, gnssPort, rate);
	gps.setCfgMsg(msgClass, msgId, gnssPort, rate);
	return 0;
}

void nmeaStandardMessages() {
	printf("  - dtm - Datum reference\n");
	printf("  - gbs - GNSS satellite fault detection\n");
	printf("  - gga - Global positioning system fix data\n");
	printf("  - gll - Latitude and longitude, with time of position fix and status\n");
	printf("  - gns - GNSS fix data\n");
	printf("  - grs - GNSS range residuals\n");
	printf("  - gsa - GNSS DOP and active satellites\n");
	printf("  - gst - GNSS pseudorange error statistics\n");
	printf("  - gsv - GNSS satellites in view\n");
	printf("  - rmc - Recommended minimum data\n");
	printf("  - vlw - Dual ground/water distance\n");
	printf("  - vtg - Course over ground and ground speed\n");
	printf("  - zda - Time and date\n");
}

void ubxStandardMessages() {	 
	printf("  - clock - Clock solution\n");
	printf("  - dgps - DGPS Data Used for NAV\n");
	printf("  - dop - Dilution of precision\n");
	printf("  - geofence - Geofencing status\n");
	printf("  - odo - Odometer solution\n");
	printf("  - orb - GNSS orbit database info\n");
	printf("  - posecef - Position solution in ECEF\n");
	printf("  - posllh - Geodetic position solution\n");
	printf("  - pvt - Navigation position velocity time solution\n");
	printf("  - pubx_posiiton - PUBX NMEA Lat/Long position data (cannot be polled)\n");
	printf("  - pubx_time - PUBX NMEA time of day and clock information (cannot be polled)\n");
	printf("  - sat - Satellite information\n");
	printf("  - sbas - SBAS Status Data\n");
	printf("  - slas - QZSS L1S SLAS Status Data\n");
	printf("  - status - Receiver navigation status\n");
	printf("  - timebds - BeiDou time solution\n");
	printf("  - timegal - Galileo time solution\n");
	printf("  - timeglo - GLONASS time solution\n");
	printf("  - timegps - GPS time solution\n");
	printf("  - timels - Leap second event information\n");
	printf("  - timeutc - UTC time solution\n");
	printf("  - tm - Time mark data\n");
	printf("  - tp - Time pulse time data\n");
	printf("  - velecef - Velocity solution in ECEF (earth-centered, earth-fixed xyz coordinates)\n");
	printf("  - velned - Velocity solution in NED (North-East-Down) frame\n");
}

void getArguments() {
	printf("  - accuracy - Position and time accuracy masks\n");
	printf("  - cnoThreshold - Signal to noise threshold for attempting a fix\n");
	printf("  - dgnssTimeout - DGNSS timeout\n");
	printf("  - debug - Information messages debug level\n");
	printf("  - delay - default delay in sec waiting for a response for the GNSS receiver\n");
	printf("  - dop - Position and time dilution of precision masks\n");
	printf("  - fixedAlt - Fixed altitude (mean sea level) for 2D fix mode\n");
	printf("  - fixMode - Position fixing mode (2D, 3D, auto)\n");
	printf("  - gnss - GNSS system configuration\n");
	printf("  - logFilter - Data logger configuration\n");
	printf("  - logInfo - Logging system information\n");
	printf("  - minElev - Minimum elevation for a GNSS satellite to be used\n");
	printf("  - nmea - Extended NMEA protocol configuration V1\n");
	printf("  - odo - Odometer, low-speed COG engine settings\n");
	printf("  - port - Configuration of I/O ports\n");
	printf("  - pms - Power mode setup\n");
	printf("  - pm - Extended power management configuration\n");
	printf("  - rxm - RXM (Power saving) configuration\n");
	printf("  - sbas - SBAS (Satellite Based Augmentation Systems) configuration\n");
	printf("  - staticHoldThresholds - Static hold threshold\n");
	printf("  - supportedGNSS - Information message about major GNSS selections\n");
	printf("  - timePulse - Time Pulse parameters for Time Pulse 0\n");
	printf("  - utcStandard - UTC standard to be used (auto, USNO, NTSC, SU, European)\n");
}

int close(const char * args) {
	if (args && !*args) {
		if (gps.isReady) printf("Closing GNSS device %s...\n", gps.device); 
		gps.end();
	}
	else { printf("usage: close\n");  return -1; }
	return 0;
}

int quit(const char * args) {
	if (args && !*args) { close(""); exit(0); }
	else { printf("usage: quit\n");  return -1; }
	return 0;
}

int version(const char * args) {
	if (args && !*args) gps.getVersion();
	else { printf("usage: version\n");  return -1; }
	return 0;
}

int patches(const char * args) {
	if (args && !*args) gps.getPatches();
	else { printf("usage: patches\n");  return -1; }
	return 0;
}

//https://tiswww.case.edu/php/chet/readline/readline.html#SEC23 paragraph 2.6 Custom Completers


/*This function is repeatedly called from readline internal rl_completion_matches() function
returning a string each time. The arguments to the generator function are text and state. 
text is the partial word to be completed. 
state is zero the first time the function is called, allowing the generator to perform 
any necessary initialization, and a positive non-zero integer for each subsequent call. 
The generator function returns (char *)NULL to inform rl_completion_matches() that there are 
no more possibilities left. Usually the generator function computes the list of possible 
completions when state is zero, and returns them one at a time on subsequent calls. 
Each string the generator function returns as a match must be allocated with malloc(); 
Readline frees the strings when it has finished with them. Such a generator function is 
referred to as an application-specific completion function. 
*/
char * command_completion_generator(const char *text, int state) {
	static int list_index, arg_index;
	char t[255];
	char * w = NULL, * s = NULL;
	int l, x, len;
	if (!state) {
		list_index = 0; arg_index = 0;
	}
	while (commands[list_index].name) {
		len = rl_end > strlen(commands[list_index].name) ? strlen(commands[list_index].name) : rl_end;
		if (strncmp(commands[list_index].name, rl_line_buffer, len) == 0) {
			l = 0;
			w = strchr(rl_line_buffer, ' ');
			while (w) {
				l++;
				w = strchr(w + 1, ' ');
			}
			x = 1;
			//printf("l %d\n", l);
			switch (l) {
			case 0:
				if (!commands[list_index].name) break;
				s = (char *)malloc(strlen(commands[list_index].name) + 1);
				strcpy(s, commands[list_index].name);
				list_index++;
				return s;
				break;
			case 1: 
				if (!commands[list_index].arg1) break;
				memset(t, 0, sizeof(t));
				strncpy(t, commands[list_index].arg1, sizeof(t));
				w = strtok(t, " ");
				while (w) {
					if (x > arg_index) {
						if (text) {
							if (strncmp(w, text, strlen(text)) == 0) {
								s = (char *)malloc(strlen(w) + 1);
								strcpy(s, w);
								arg_index = x;
								return s;
							}
						}
						else {
							s = (char *)malloc(strlen(w) + 1);
							strcpy(s, w);
							arg_index = x;
							return s;
						}
					}
					x++;
					w = strtok(NULL, " ");
				}
				break;
			case 2: 
				if (!commands[list_index].arg2) break;
				memset(t, 0, sizeof(t));
				strncpy(t, commands[list_index].arg2, sizeof(t));
				w = strtok(t, " ");
				while (w) {
					if (x > arg_index) {
						if (text) {
							if (strncmp(w, text, strlen(text)) == 0) {
								s = (char *)malloc(strlen(w) + 1);
								strcpy(s, w);
								arg_index = x;
								return s;
							}
						}
						else {
							s = (char *)malloc(strlen(w) + 1);
							strcpy(s, w);
							arg_index = x;
							return s;
						}
					}
					x++;
					w = strtok(NULL, " ");
				}
				break;
			}
		}
		list_index++;
	}
	return NULL;
}

char ** completion_function(const char *text, int start, int end) {
	rl_attempted_completion_over = 0;
	return rl_completion_matches(text, command_completion_generator);
}

int main(int argc, char ** argv) {
	char prompt[] = "gps> ";
	char *instr;

	if (argc > 1) {
		printf("Usage: gps\n"); 
		return -1;
	}

	struct sigaction sa;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = SA_RESTART;
	sa.sa_handler = sigHandler;
	if (sigaction(SIGIO, &sa, NULL) == -1) {
		printf("sigaction error: %s\n", strerror(errno)); 
		return -1;
	}
	
	rl_readline_name = "gps";
	//rl_completion_entry_function = command_completion_generator;
	rl_attempted_completion_function = completion_function;

	while (1) {
		instr = readline(prompt);
		if (instr && * instr) {
			add_history(instr);
			execute_command(instr);
			delay(1);
		}
		free((char *)instr);
	}
	
	return 0;

}

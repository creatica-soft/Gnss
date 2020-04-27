//compile with g++ -O -o gnss gnss_app.cpp
#include <error.h>
#include <signal.h>
#include <unistd.h>
#include "gnss.cpp"

using namespace std;

volatile bool getLog;
const uint8_t navRate = 1;
Port gnssPort = COM1;
uint32_t gnssPortRate = 9600;

Gnss gps;

int8_t startLogging(uint8_t interval, LogSize ls_enum, uint32_t customSize, bool circular) {
	gps.getLogInfo();
	if (gps.gnssLogInfo.flags.enabled == 0) {
		if (!gps.enableLogging(interval)) {
			printf("enableLogging() failed\n"); return -1;
		}
		else printf("enableLogging() ok\n");
	}
	if (gps.gnssLogInfo.flags.inactive == 1) {
		if (!(gps.createLog(ls_enum, customSize, circular))) {
			printf("createLog() failed\n"); return -1;
		} else printf("createLog() ok\n");
	}
	char msg[16] = "Logging started"; msg[15] = 0;
	gps.logMsg(msg, 16);
	return 0;
}

int8_t stopLogging() {
	gps.getLogInfo();
	if (gps.gnssLogInfo.flags.inactive == 0 && gps.gnssLogInfo.flags.enabled == 1) {
		char msg[16] = "Logging stopped"; msg[15] = 0;
		gps.logMsg(msg, 16);
	}
	if (gps.gnssLogInfo.flags.enabled == 1) {
		if (!(gps.disableLogging())) {
			printf("disableLogging() failed\n"); return -1;
		} else printf("disableLogging() ok\n");
	}
	return 0;
}

int8_t eraseLog() {
	gps.getLogInfo();
	if (gps.gnssLogInfo.flags.enabled == 1) stopLogging();
	if (!gps.eraseLog()) { printf("eraseLog() failed\n"); return -1; }
	else printf("eraseLog() ok\n");
	return 0;
}

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

	getLog = false;
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
	delay();
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
	const char * nmea_sentences[] = { "RMC", "DTM", "GBS", "GGA", "GLL", "GNS", "GRS", "GSA", "GST", "GSV", "VLW", "VTG", "ZDA", NULL };
	for (int i = 0; nmea_sentences[i]; i++) gps.pubxRate(nmea_sentences[i], 0);
	gps.pubxConfig((BaudRate)gnssPortRate, inMask, outMask, gnssPort);
    return 0;
}

void loop() {
	if (getLog) {
		if (stopLogging() == 0) {
			gps.getLogInfo();
			uint32_t num = gps.gnssLogInfo.entryCount;
			printf("Retrieving %u log records\n", num);
			uint32_t j = 0, k = num / 256;
			for (uint32_t i = 0; i < k; i++) {
				gps.logRetrieve(j, 256);
				j += 256;
			}
			gps.logRetrieve(j, num % 256);
		}
		getLog = false;
	}

	gps.get();
}

static void sigHandler(int sig) {
	//printf("sigio caught\n");
	if (gps.isReady) {
		//printf("calling loop...\n");
		loop();
	}
}

int main(int argc, char ** argv) {
	char* dev;
	speed_t rate;
	bool reset = false, soft = true;

	if (argc <= 3 || argc > 6) {
		printf("Usage: gnss <dev> <DDC|COM1|COM2|USB|SPI> [baudRate] [reset] [hard]\n");
		printf("Supported baud rates are: 4800, 9600, 19200, 38400, 57600, 115200, 230400\n");
		return -1;
	}
	dev = argv[1];

	if (argv[2] == "DDC") gnssPort = DDC;
	else if (argv[2] == "COM1") gnssPort = COM1;
	else if (argv[2] == "COM2") gnssPort = COM2;
	else if (argv[2] == "USB") gnssPort = USB;
	else if (argv[2] == "SPI") gnssPort = SPI;
	else gnssPort = COM1;	

	if (argc > 3) gnssPortRate = (speed_t)atoi(argv[3]);

	switch (gnssPortRate) {
		case 4800: rate = B4800; break;
		case 9600: rate = B9600; break;
		case 19200: rate = B19200; break;
		case 38400: rate = B38400; break;
		case 57600: rate = B57600; break;
		case 115200: rate = B115200; break;
		case 230400: rate = B230400; break;
	}
	if (argc > 4) {
		if (strcmp(argv[4],"reset") == 0) reset = true;
	}

	if (argc > 5) {
		if (strcmp(argv[5], "hard") == 0) soft = false;
	}
	/* use sigaction instead for portability
	if (signal(SIGIO, sigHandler) == SIG_ERR) {
		printf("signal: %s\n", strerror(errno));
		return -1;
	}*/
	struct sigaction sa;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = SA_RESTART;
	sa.sa_handler = sigHandler;
	if (sigaction(SIGIO, &sa, NULL) == -1) {
		printf("sigaction error: %s\n", strerror(errno));
		return -1;
	}

	if (setup(dev, rate, reset, soft) != 0) exit(-1);

	delay(3);
	gps.getVersion();
	//gps.getCfgPrt();
	//gps.getCfgPm();
	//gps.getCfgRxm();
    //gps.getCfgInf(UBX, gnssPort);
	//gps.getCfgInf(NMEA, gnssPort);
	/*gps.getCfgNav();
	if (gps.getDynamicModel() != STATIONARY) gps.setDynamicModel(STATIONARY); //PORTABLE - default
	if (gps.getUtcStandard() != GLONASS) {
		gps.setUtcStandard(GLONASS); //AUTOMATIC - default
		gps.getUtcStandard();
	}*/
	/*gps.getCfgPms();
	if (gps.powerMode.powerMode != AGGRESSIVE_1HZ_POWER_MODE) {
		gps.setCfgPms(AGGRESSIVE_1HZ_POWER_MODE); //BALANCED - default
		gps.getCfgPms();
	}*/
	//gps.getCfgOdo();
	//gps.setCfgOdo(gps.gnssCfgOdo.flags, CYCLING, gps.gnssCfgOdo.cogMaxSpeed, gps.gnssCfgOdo.cogMaxPosAccuracy, gps.gnssCfgOdo.velLPgain, gps.gnssCfgOdo.cogLPgain);
	//gps.getCfgLogFilter();
	//gps.getLogInfo();
	//gps.getSupportedGnss();
	//gps.getGnss();
	/*MajorGnss gnss;
	gnss.Gps = 1;
	gnss.Glonass = 1;
	gnss.BeiDou = 0;
	gnss.Galileo = 0;
	bool enableSBAS = true, enableIMES = false;
	gps.setGnss(gnss, enableSBAS, enableIMES);
	gps.getGnss();*/
	//gps.getNavRate();
	//gps.getTimePulse();
	//gps.setCfgMsg(UBX_NAV, NAV_GEOFENCE, gnssPort, 0);
	/*
	//Geofencing
	gps.setCfgMsg(UBX_NAV, NAV_GEOFENCE, gnssPort, navRate);
	GeoFence geofence;
	Latitude lat;// , lat2;
	lat.deg = 36; lat.min = 48, lat.sec = 41; lat.NS = 'S';
	Longitude lon;// , lon2;
	lon.deg = 174; lon.min = 38; lon.sec = 46; lon.EW = 'E';
	geofence.lat = dmsToLongLat(&lat);
	//longLatToDMS(&lat2, geofence.lat);//to verify lat conversion from string to int32 and back
	geofence.lon = dmsToLongLon(&lon);
	//longLonToDMS(&lon2, geofence.lon); //to verify lon conversion from string to int32 and back
	//printf("%s %s\n", dmsLatToStr(&lat2).c_str(), dmsLonToStr(&lon2).c_str());
	geofence.radius = 1000000;//10km
	uint8_t confidenceLevel = 3; //0=no confidence required, 1=68%, 2=95%, 3=99.7%, 4=99.99% etc.
	gps.setCfgGeofence(&geofence, confidenceLevel);
	gps.getCfgGeofences();
	*/
	//gps.setCfgMsg(UBX_NAV, NAV_CLOCK, gnssPort, navRate);
	//gps.getCfgMsg(UBX_NAV, NAV_CLOCK);
	//gps.setCfgMsg(UBX_TIM, TIM_TP, gnssPort, navRate);
	//gps.getCfgMsg(UBX_TIM, TIM_TP);
	//gps.setCfgMsg(UBX_NAV, NAV_PVT, gnssPort, navRate);
	//gps.getCfgMsg(UBX_NAV, NAV_PVT);
	//gps.setCfgMsg(UBX_NAV, NAV_DOP, gnssPort, navRate);
	//gps.setCfgMsg(UBX_NAV, NAV_ORB, gnssPort, navRate);
	//gps.setCfgMsg(UBX_NAV, NAV_POSLLH, gnssPort, navRate);
	//gps.setCfgMsg(UBX_NAV, NAV_SAT, gnssPort, navRate);
	//gps.setCfgMsg(UBX_NAV, NAV_TIMEGPS, gnssPort, navRate);
	//gps.setCfgMsg(UBX_NAV, NAV_TIMEUTC, gnssPort, navRate);
	//gps.setCfgMsg(UBX_NAV, NAV_TIMELS, gnssPort, navRate);
	//gps.setCfgMsg(UBX_NAV, NAV_DGPS, gnssPort, navRate);
	//gps.setCfgMsg(UBX_NAV, NAV_ODO, gnssPort, navRate);
	//gps.setCfgMsg(UBX_NAV, NAV_SBAS, gnssPort, navRate);
	//gps.setCfgMsg(UBX_NAV, NAV_SLAS, gnssPort, navRate);
	//gps.setCfgMsg(UBX_NMEA, NMEA_GNS, gnssPort, navRate);//uses U-BLOX protocol
	//gps.pubxRate("GNS", navRate); //uses NMEA protocol
	//gps.setCfgMsg(UBX_NMEA, NMEA_GST, gnssPort, navRate);
	//gps.setCfgMsg(UBX_NMEA, NMEA_VLW, gnssPort, navRate);
	//gps.setCfgMsg(UBX_NMEA, NMEA_ZDA, gnssPort, navRate);
	//gps.setCfgMsg(UBX_PUBX, PUBX_POSITION, gnssPort, navRate);
	//gps.setCfgMsg(UBX_PUBX, PUBX_TIME, gnssPort, navRate);
	//gps.setCfgMsg(UBX_PUBX, PUBX_SVSTATUS, gnssPort, navRate); //does not work - u-blox will stop its output
	//gps.getCfgNmea();


	//poll NMEA message IDs (GBS, GRS, DTM) using "VR" as its own talker ID
	//gps.nmeaGnq("GBS");
	//gps.nmeaGnq("GRS");
	//gps.nmeaGnq("DTM");

	/*eraseLog();
	if (startLogging(5, SAFE_SIZE, 0, true) == 0) {
		time_t t = time(NULL);
		while (time(NULL) - t < 20)
			sleep(1);
		getLog = true;
	}*/



	while (1) {
		if (pause() == -1 && errno == EINTR) continue; //printf("sigHandler returned\n");
		else printf("error processing signal\n");
	}

	return 0;
}


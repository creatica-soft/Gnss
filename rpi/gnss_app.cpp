//compile with gcc -lsupc++ -lstdc++ -lwiringPi -O -o gnss gnss_app.cpp
#include "gnss.cpp"
#include <wiringPi.h>

#define INTERRUPT_PIN 25

using namespace std;

volatile bool getLog, gpsDataAvailable;
const uint8_t navRate = 1;
const Port gnssPort = COM1;
int gnssPortRate = 9600;

//const char* dev = "/dev/ttyAMA3";

Gnss gps;

void lg() {
	getLog = true;
}

void tp() {
	gps.tp();
}

int8_t startLogging(uint8_t interval, LogSize ls_enum, uint32_t customSize, bool circular) {
	LogInfo* log = gps.getLogInfo();
	if (log == NULL) { printf(" logInfo NULL\n"); return -1; }
	if (!(gps.logFileExists)) {
		printf("Creating log file...\n");
		if (!(gps.createLog(ls_enum, customSize, circular))) {
			printf(" failed\n"); gps.logInfo();	return -1;
		}
		sleep(1);
		log = gps.getLogInfo();
		if (log == NULL) { printf(" logInfo NULL\n"); return -1; }
		if (!(gps.logFileExists)) { 
			printf(" failed\n"); gps.logInfo(); return -1; 
		}
		printf(" OK\n");
		gps.logInfo();
	} else printf("Log file already exists\n");
	if (gps.loggingEnabled) printf("Logging already enabled\n");
	else {
		printf("Enabling logging...\n");
		if (!gps.enableLogging(interval)) {
			printf(" failed\n");  gps.logInfo(); return -1;
		}
		sleep(1);
		log = gps.getLogInfo();
		if (log == NULL) { printf(" logInfo NULL\n"); return -1; }
		if (!gps.loggingEnabled) {
			printf(" failed\n"); gps.logInfo(); return -1;
		}
		printf(" OK\n");
		gps.logInfo();
	}
	char msg[16] = "Logging started"; msg[15] = 0;
	if (gps.logMsg(msg, 16)) printf(" logMsg %s OK\n", msg); else {
		printf(" logMsg %s failed\n", msg);
	}
	return 0;
}

int8_t stopLogging() {
	if (!gps.loggingEnabled) return 0;
	char msg[16] = "Logging stopped"; msg[15] = 0;
	if (gps.logMsg(msg, 16)) printf(" logMsg %s OK\n", msg); else {
		printf(" logMsg %s failed\n", msg);
	}
	printf("Disabling logging...\n");
	if (!(gps.disableLogging())) { printf(" failed\n"); return -1; }
	LogInfo* log = gps.getLogInfo();
	if (log == NULL) { printf(" logInfo is NULL\n"); return -1; }
	if (gps.loggingEnabled) {
		printf(" failed\n"); gps.logInfo(); return -1;
	}
	printf(" OK\n");
	gps.logInfo();
	return 0;
}

int8_t eraseLog() {
	LogInfo* log = gps.getLogInfo();
	if (log == NULL) { printf(" logInfo is NULL\n"); return -1; }
	gps.logInfo();
	if (gps.loggingEnabled) {
		if (stopLogging() != 0) return -1;
		sleep(1);
	}
	log = gps.getLogInfo();
	if (log == NULL) { printf(" logInfo is NULL\n"); return -1; }
	gps.logInfo();
	//if (gps.logFileExists) {
		printf("Erasing log...\n");
		if (!gps.eraseLog()) { printf(" failed\n"); return -1;	}
		sleep(1);
		log = gps.getLogInfo();
		if (log == NULL) { printf(" logInfo is NULL\n"); return -1; }
		if (gps.logFileExists) { printf(" failed\n"); gps.logInfo(); return -1; }
		printf(" OK\n");
		gps.logInfo();
	//}
	return 0;
}

void retrieveLogs(uint32_t idx, uint32_t count) {
	if (!gps.logRetrieve(idx, count)) { printf("logRetrieve failed\n"); return; }
}

int tRateToI(speed_t r) {
	switch (r) {
		case B4800: gnssPortRate = 4800; break;
		case B9600: gnssPortRate = 9600; break;
		case B19200: gnssPortRate = 19200; break;
		case B38400: gnssPortRate = 38400; break;
		case B57600: gnssPortRate = 57600; break;
		case B115200: gnssPortRate = 115200; break;
		case B230400: gnssPortRate = 230400; break;
	}
	return gnssPortRate;
}


int8_t setup(const char * dev, speed_t rate, bool reset, bool soft) {
	speed_t rates[7] = { B4800, B9600, B19200, B38400, B57600, B115200, B230400 };
	CfgMsg* cfg = NULL;
	CfgPrt* cfgPrt = NULL;
	getLog = false;
	gps = Gnss();
	printf("TOPGNSS\n");
	printf("Connecting to GNSS on %s at %d baud\n", dev, gnssPortRate);
	gps.begin(dev, gnssPortRate);
	cfgPrt = gps.getCfgPrt();
	if (cfgPrt != NULL) printf(" connected\n");
	else {
		for (int i = 0; i < 7; i++) {
			if (rate != rates[i]) {
				printf(" failed. Re-trying with %d baud rate...\n", tRateToI(rates[i]));
				gps.end();
				gps.begin(dev, rates[i]);
				//writing at wrong baud rate causes Gnss receiver to get disabled after 100 or frame errors
				//instead of calling getCfgPrt(), we should passively listen for something meaninful
				cfgPrt = gps.getCfgPrt();
				
				if (cfgPrt != NULL) {
					printf(" connected.\n"); break;
				} else if (i == 6) { printf(" sorry, giving up...\n"); return -1; }
			}
		}
	}
	if (reset) {
		printf("Reseting...\n");
		gps.reset(soft);
		sleep(3);
	}
	if (gps.baudRate != rate) {
		printf("Setting desired rate %d...\n", tRateToI(rate));
		switch (rate) {
			case B4800: cfgPrt->baudRate = BAUD_RATE_4800; break;
			case B9600: cfgPrt->baudRate = BAUD_RATE_9600; break;
			case B19200: cfgPrt->baudRate = BAUD_RATE_19200; break;
			case B38400: cfgPrt->baudRate = BAUD_RATE_38400; break;
			case B57600: cfgPrt->baudRate = BAUD_RATE_57600; break;
			case B115200: cfgPrt->baudRate = BAUD_RATE_115200; break;
			case B230400: cfgPrt->baudRate = BAUD_RATE_230400; break;
		}
		if (gps.setCfgPrt(cfgPrt->portId, cfgPrt->baudRate, cfgPrt->mode, cfgPrt->inProtoMask, cfgPrt->outProtoMask))
		{
			printf(" success\n");
			gps.end();
			sleep(10);
			gps.begin(dev, rate);
			cfgPrt = gps.getCfgPrt();
			if (cfgPrt != NULL) {
				printf("Connected at new rate %d\n", gnssPortRate);
				if (gps.saveConfig()) printf("Config is saved\n");
				else printf("Saving config failed\n");
			}
			else {
				printf("Failed to connect at new rate %d\n", gnssPortRate); return -1;
			}
		} else {
			printf(" failed.\n"); return -1;
		}
	}

	wiringPiSetup();
	wiringPiISR(INTERRUPT_PIN, INT_EDGE_RISING, tp);

	if (gps.getVersion() != NULL) gps.monVer(MON_VER_EXTENSION_NUMBER);
	else printf("getVersion() is NULL\n");
	sleep(1);
	/*
	if (gps.getCfgPrt() != NULL) gps.cfgPrt();
	else printf("getCfgPrt() is NULL\n");
	sleep(1);
	if (gps.getCfgPm() != NULL) gps.cfgPm();
	else printf("getCfgPm() is NULL\n");
	sleep(1);
	if (gps.getCfgRxm() != NULL) gps.cfgRxm();
	else printf("getCfgRxm() is NULL\n");
	sleep(1);
	if (gps.getSupportedGnss() != NULL) gps.monGnss();
	else printf("getSupportedGnss() is NULL\n");
	sleep(1);
	if (gps.getGnss() != NULL) gps.cfgGnss();
	else printf("getGnss() is NULL\n");
	sleep(1);

	printf("getCfgInf(UBX, gnssPort)");
	if (gps.getCfgInf(UBX, gnssPort) != NULL) gps.cfgInf();
	else printf(" failed\n");
	sleep(1);
	printf("getCfgInf(NMEA, gnssPort)");
	if (gps.getCfgInf(NMEA, gnssPort) != NULL) gps.cfgInf();
	else printf(" failed\n");
	sleep(1);
	printf("getCfgNav()");
	if (gps.getCfgNav() != NULL) gps.cfgNav();
	else printf(" failed\n");
	sleep(1);

	//setting dynModel to STATIONARY
	printf("setDynamicModel");
	if (gps.setDynamicModel(STATIONARY)) printf(" OK\n"); else printf(" failed\n");
	sleep(1);

	//set UTC to GLONASS
	printf("setUtcStandard to GLONASS");
	if (gps.setUtcStandard(GLONASS)) printf(" OK\n"); else printf(" failed\n");
	sleep(1);

	if (gps.getCfgPms() != NULL) gps.cfgPms();
	sleep(1);
	//set power mode to AGGRESSIVE_1HZ_POWER_MODE
	printf("setPowerMode(AGGRESSIVE_1HZ): ");
	if (gps.setCfgPms(AGGRESSIVE_1HZ_POWER_MODE)) printf(" OK\n");
	else printf(" failed\n");
	sleep(1);

	//Turn on or off major GNSS
	MajorGnss gnss;
	gnss.Gps = 1;
	gnss.Glonass = 1;
	gnss.BeiDou = 0;
	gnss.Galileo = 0;
	bool enableSBAS = true, enableIMES = false;
	printf("setGnss");
	if (gps.setGnss(gnss, enableSBAS, enableIMES)) printf(" OK\n");
	else printf(" failed\n");
	sleep(1);

	printf("getNavRate()");
	if (gps.getNavRate() != NULL) gps.cfgRate();
	else printf(" failed\n");
	sleep(1);
	printf("getTimePulse()");
	if (gps.getTimePulse() != NULL) gps.cfgTp();
	else printf(" failed\n");
	sleep(1);

	printf("getCfgOdo\n");
	ODOCfg * odoCfg = gps.getCfgOdo();
	if (odoCfg != NULL) {
	  gps.cfgOdo();
		  sleep(1);
		  //set odomter profile to CYCLING
	  printf("setOdo CYCLING: ");
	  if (gps.setCfgOdo(odoCfg->flags, CYCLING, odoCfg->cogMaxSpeed, odoCfg->cogMaxPosAccuracy, odoCfg->velLPgain, odoCfg->cogLPgain)) printf(" OK\n");
	  else printf(" failed\n");
	} else printf(" failed\n");
	sleep(1);

	printf("getCfgLogFilter()");
	if (gps.getCfgLogFilter() != NULL) gps.cfgLogFilter();
	else printf(" failed\n");
	sleep(1);

	printf("getLogInfo()");
	if (gps.getLogInfo() != NULL) gps.logInfo();
	else printf(" failed\n");
	sleep(1);

	printf("getCfgNmea()");
	if (gps.getCfgNmea() != NULL) gps.cfgNmea();
	else printf(" failed\n");
	sleep(1);

	GeoFence geofence;
	Latitude lat;
	lat.deg = 36; lat.min = 48, lat.sec = 41; lat.NS = 'S';
	Longitude lon;
	lon.deg = 174; lon.min = 38; lon.sec = 46; lon.EW = 'E';
	geofence.lat = dmsToLongLat(&lat);
	geofence.lon = dmsToLongLon(&lon);
	geofence.radius = 1000000;//10km
	uint8_t confidenceLevel = 3; //0=no confidence required, 1=68%, 2=95%, 3=99.7% etc.
	if (gps.setCfgGeofence(&geofence, confidenceLevel)) printf("setCfgGeofence OK\n"); else printf("setCfgGeofence failed\n");
	sleep(1);
	if (gps.getCfgGeofences(3000) != NULL) gps.cfgGeoFence(); else printf("cfgGeofence NULL\n");
	sleep(1);

	printf("getCfgMsg(UBX_NAV, NAV_GEOFENCE)");
	cfg = gps.getCfgMsg(UBX_NAV, NAV_GEOFENCE);
	sleep(1);
	if (cfg != NULL) {
			if (cfg->rate[gnssPort] != navRate) {
					if (gps.setCfgMsg(UBX_NAV, NAV_GEOFENCE, navRate)) printf("UBX-NAV-GEOFENCE OK\n");
					else printf("UBX-NAV-GEOFENCE failed\n");
					sleep(1);
			}
			printf(" OK\n");
	} else printf(" failed\n");
	*/
	/*
	  printf("getCfgMsg(UBX_NAV, NAV_CLOCK)");
	  cfg = gps.getCfgMsg(UBX_NAV, NAV_CLOCK);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_NAV, NAV_CLOCK, navRate)) printf("UBX-NAV-CLOCK OK\n");
		  else printf("UBX-NAV-CLOCK failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_TIM, TIM_TP)");
	  cfg = gps.getCfgMsg(UBX_TIM, TIM_TP);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_TIM, TIM_TP, navRate)) printf("UBX-TIM-TP OK\n");
		  else printf("UBX-TIM-TP failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_NAV, NAV_DOP)");
	  cfg = gps.getCfgMsg(UBX_NAV, NAV_DOP);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_NAV, NAV_DOP, navRate)) printf("UBX-NAV-DOP OK\n");
		  else printf("UBX-NAV-DOP failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_NAV, NAV_ORB)");
	  cfg = gps.getCfgMsg(UBX_NAV, NAV_ORB);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_NAV, NAV_ORB, navRate)) printf("UBX-NAV-ORB OK\n");
		  else printf("UBX-NAV-ORB failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_NAV, NAV_POSLLH)");
	  cfg = gps.getCfgMsg(UBX_NAV, NAV_POSLLH);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_NAV, NAV_POSLLH, navRate)) printf("UBX-NAV-POSLLH OK\n");
		  else printf("UBX-NAV-POSLLH failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_NAV, NAV_PVT)");
	  cfg = gps.getCfgMsg(UBX_NAV, NAV_PVT);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_NAV, NAV_PVT, navRate)) printf("UBX-NAV-PVT OK\n");
		  else printf("UBX-NAV-PVT failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_NAV, NAV_SAT)");
	  cfg = gps.getCfgMsg(UBX_NAV, NAV_SAT);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_NAV, NAV_SAT, navRate)) printf("UBX-NAV-SAT OK\n");
		  else printf("UBX-NAV-SAT failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_NAV, NAV_TIMEGPS)");
	  cfg = gps.getCfgMsg(UBX_NAV, NAV_TIMEGPS);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_NAV, NAV_TIMEGPS, navRate)) printf("UBX-NAV-TIMEGPS OK\n");
		  else printf("UBX-NAV-TIMEGPS failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_NAV, NAV_TIMEUTC)");
	  cfg = gps.getCfgMsg(UBX_NAV, NAV_TIMEUTC);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_NAV, NAV_TIMEUTC, navRate)) printf("UBX-NAV-TIMEUTC OK\n");
		  else printf("UBX-NAV-TIMEUTC failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_NAV, NAV_TIMELS)");
	  cfg = gps.getCfgMsg(UBX_NAV, NAV_TIMELS);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_NAV, NAV_TIMELS, navRate)) printf("UBX-NAV-TIMELS OK\n");
		  else printf("UBX-NAV-TIMELS failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_NAV, NAV_DGPS)");
	  cfg = gps.getCfgMsg(UBX_NAV, NAV_DGPS);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_NAV, NAV_DGPS, navRate)) printf("UBX-NAV-DGPS OK\n");
		  else printf("UBX-NAV-DGPS failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_NAV, NAV_ODO)");
	  cfg = gps.getCfgMsg(UBX_NAV, NAV_ODO);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_NAV, NAV_ODO, navRate)) printf("UBX-NAV-ODO OK\n");
		  else printf("UBX-NAV-ODO failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_NAV, NAV_SBAS)");
	  cfg = gps.getCfgMsg(UBX_NAV, NAV_SBAS);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_NAV, NAV_SBAS, navRate)) printf("UBX-NAV-SBAS OK\n");
		  else printf("UBX-NAV-SBAS failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_NAV, NAV_SLAS)");
	  cfg = gps.getCfgMsg(UBX_NAV, NAV_SLAS);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_NAV, NAV_SLAS, navRate)) printf("UBX-NAV-SLAS OK\n");
		  else printf("UBX-NAV-SLAS failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_NMEA, NMEA_GNS)");
	  cfg = gps.getCfgMsg(UBX_NMEA, NMEA_GNS);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_NMEA, NMEA_GNS, navRate)) printf("UBX-NMEA-GNS OK\n");
		  else printf("UBX-NMEA-GNS failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_NMEA, NMEA_GST)");
	  cfg = gps.getCfgMsg(UBX_NMEA, NMEA_GST);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_NMEA, NMEA_GST, navRate)) printf("UBX-NMEA-GST OK\n");
		  else printf("UBX-NMEA-GST failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_NMEA, NMEA_VLW)");
	  cfg = gps.getCfgMsg(UBX_NMEA, NMEA_VLW);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_NMEA, NMEA_VLW, navRate)) printf("UBX-NMEA-VLW OK\n");
		  else printf("UBX-NMEA-VLW failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_NMEA, NMEA_ZDA)");
	  cfg = gps.getCfgMsg(UBX_NMEA, NMEA_ZDA);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_NMEA, NMEA_ZDA, navRate)) printf("UBX-NMEA-ZDA OK\n");
		  else printf("UBX-NMEA-ZDA failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_PUBX, PUBX_POSITION)");
	  cfg = gps.getCfgMsg(UBX_PUBX, PUBX_POSITION);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_PUBX, PUBX_POSITION, navRate)) printf("UBX-PUBX-POSITION OK\n");
		  else printf("UBX-PUBX-POSITION failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  printf("getCfgMsg(UBX_PUBX, PUBX_TIME)");
	  cfg = gps.getCfgMsg(UBX_PUBX, PUBX_TIME);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != navRate) {
		  if (gps.setCfgMsg(UBX_PUBX, PUBX_TIME, navRate)) printf("UBX-PUBX-TIME OK\n");
		  else printf("UBX-PUBX-TIME failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");

	  //somehow UBX-PUBX-SVSTATUS doesn't work on my arduino mega - use at your own risk!
	  printf("getCfgMsg(UBX_PUBX, PUBX_SVSTATUS)");
	  cfg = gps.getCfgMsg(UBX_PUBX, PUBX_SVSTATUS);
	  sleep(1);
	  if (cfg != NULL) {
			  printf(" OK\n");
			  if (cfg->rate[gnssPort] != 0) {
		  if (gps.setCfgMsg(UBX_PUBX, PUBX_SVSTATUS, 0)) printf("UBX-PUBX-SVSTATUS OK\n");
		  else printf("UBX-PUBX-SVSTATUS failed\n");
			  sleep(1);
			}
	  }
	  else printf(" failed\n");
	*/

	InProtoMask inMask;
	OutProtoMask outMask;
	inMask.inUbx = 1; inMask.inNmea = 1; inMask.inRtcm = 1;
	outMask.outUbx = 1; outMask.outNmea = 1;
	gps.pubxConfig(BAUD_RATE_57600, inMask, outMask);
	gps.end();
	gps.begin(dev, B57600);

	//turn off (0), on (1) msgId on specific interface
	/*const char* msgId = "GLL";
	uint8_t rateCOM1 = 1;
	gps.pubxRate(msgId, rateCOM1);*/

	/*eraseLog();
	if (startLogging(5, USER_DEFINED_SIZE, 8192, false) == 0) {
		sleep(20);
		getLog = true;
	}*/
	return 0;
}

void loop() {
	if (getLog) {
		if (stopLogging() == 0) {
			LogInfo* log = gps.getLogInfo();
			if (log != NULL) {
				uint32_t num = log->entryCount;
				printf("Retrieving %lu log records\n", num);
				uint32_t j = 0, k = num / 256;
				for (uint32_t i = 0; i < k; i++) {
					retrieveLogs(j, 256);
					j += 256;
				}
				retrieveLogs(j, num % 256);
			}
			else printf(" LogInfo NULL\n");
		}
		getLog = false;
	}

	gps.get();
}

int main(int argc, char ** argv) {
	char* dev;
	speed_t rate;
	bool reset = false, soft = true;
	if (argc <= 2 || argc >= 5) {
		printf("Usage: gnss <dev> [baudRate] [reset] [hard]\n");
		printf("Supported baud rates are: 4800, 9600, 19200, 38400, 57600, 115200, 230400\n");
		return -1;
	}
	dev = argv[1];
	if (argc > 2) gnssPortRate = atoi(argv[2]);

	switch (gnssPortRate) {
		case 4800: rate = B4800; break;
		case 9600: rate = B9600; break;
		case 19200: rate = B19200; break;
		case 38400: rate = B38400; break;
		case 57600: rate = B57600; break;
		case 115200: rate = B115200; break;
		case 230400: rate = B230400; break;
	}
	if (argc > 3) {
		if (strcmp(argv[3],"reset") == 0) reset = true;
	}

	if (argc > 4) {
		if (strcmp(argv[4], "hard") == 0) soft = false;
	}

	if (setup(dev, rate, reset, soft) != 0) exit(-1);
	while (1) {
		loop();
	}
	return 0;
}

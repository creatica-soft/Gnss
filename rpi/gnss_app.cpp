//compile with gcc -lsupc++ -lstdc++ -lwiringPi -O -o gnss gnss_app.cpp
#include "gnss.cpp"
#include <wiringPi.h>

#define INTERRUPT_PIN 25

using namespace std;

volatile bool getLog, gpsDataAvailable;
const Port gnssPort = COM1;
const uint8_t navRate = 1;

const char* dev = "/dev/ttyAMA3";

Gnss gps;

void lg() {
  getLog = true;
}

void tp() {
  gps.tp();
}

void startLogging(LogSize ls_enum, uint32_t customSize, bool circular) {
	printf("createLog");
	LogInfo* log = gps.getLogInfo();
	if (log == NULL) { printf(" logInfo NULL\n"); return; }
	if (log->flags.inactive) {
		if (gps.createLog(ls_enum, customSize, circular)) printf(" OK\n"); else printf(" logCreate failed\n");
	}
	if (log->flags.enabled) return;
	CfgLogFilterFlags flags;
	flags.recordingEnabled = 1;
	flags.psmOncePerWakeUpEnabled = 0;
	flags.applyAllFilterSettings = 1;
	printf(" startLogging");
	if (gps.setCfgLogFilter(0, 5, 0, 0, flags)) {
		printf(" OK\n");
		char msg[16] = "Logging started"; msg[15] = 0;
		if (gps.logMsg(msg, 16)) printf(" logMsg %s OK\n", msg); else printf(" logMsg %s failed\n", msg);
	}
	else printf(" failed\n");
}

void stopLogging() {
	char msg[16] = "Logging stopped"; msg[15] = 0;
	if (gps.logMsg(msg, 16)) printf(" logMsg %s OK\n", msg); else printf(" logMsg %s failed\n", msg);
	CfgLogFilterFlags flags;
	flags.recordingEnabled = 0;
	flags.psmOncePerWakeUpEnabled = 0;
	flags.applyAllFilterSettings = 0;
	printf("stopLogging");
	if (gps.setCfgLogFilter(0, 0, 0, 0, flags)) printf(" OK\n"); else printf(" failed\n");
}

void eraseLog() {
	printf("eraseLog");
	LogInfo* log = gps.getLogInfo();
	if (log == NULL) { printf(" logInfo NULL\n"); return; }
	if (log->flags.enabled) {
		CfgLogFilterFlags flags;
		flags.recordingEnabled = 0;
		flags.psmOncePerWakeUpEnabled = 0;
		flags.applyAllFilterSettings = 0;
		if (!gps.setCfgLogFilter(0, 0, 0, 0, flags)) { printf(" setLogFilter failed\n"); return; }
	}
	if (!log->flags.inactive) {
		if (!gps.eraseLog()) printf(" failed\n");
	}
	printf(" OK\n");
}

void retrieveLogs(uint32_t idx, uint32_t count) {
	if (!gps.logRetrieve(idx, count)) { printf("logRetrieve failed\n"); return; }
}

void setup() {
  CfgMsg* cfg = NULL;
  CfgPrt* cfgPrt = NULL;
  gps = Gnss();
  getLog = false;
  printf("TOPGNSS\n");
  gps.begin(dev, B115200);
  cfgPrt = gps.getCfgPrt(gnssPort);
  if (cfgPrt != NULL) {
	  gps.cfgPrt();
	  printf("GPS at 115200 baud\n");
  }
  else {
	  printf("getCfgPrt NULL\n");
	  gps.end();
	  gps.begin(dev, B9600);
	  cfgPrt = gps.getCfgPrt(gnssPort);
	  if (cfgPrt != NULL) {
		  gps.cfgPrt();
		  printf("GPS at 9600 baud\n");
	  }
	  else printf("error: unable to communicate to GPS\n");
  }
  wiringPiSetup();
  wiringPiISR(INTERRUPT_PIN, INT_EDGE_RISING, tp);

  /*printf("Resetting...\n");
  gps.factoryReset();
  gps.end();
  sleep(3);
  printf("Connecting to GPS at 9600 baud...\n");
  gps.begin(dev, B9600);
  printf("Done. Loading config with 115200 baud...\n");
  gps.loadConfig();
  gps.end();
  sleep(3);
  gps.begin(dev, B115200);
  printf("Done. Now connected at 115200 baud\n");*/
  cfgPrt = gps.getCfgPrt(gnssPort);
  if (cfgPrt != NULL) {
    gps.cfgPrt(); 
    //cfgPrt->outProtoMask.outNmea = 1;
    cfgPrt->baudRate = BAUD_RATE_115200;
    if (gps.setCfgPrt(cfgPrt->portId, cfgPrt->baudRate, cfgPrt->mode, cfgPrt->inProtoMask, cfgPrt->outProtoMask)) printf("setCfgPrt OK\n");
    else printf("setCfgPrt failed\n");
    gps.end();
	sleep(3);
    gps.begin(dev, B115200);
    gps.saveConfig();
  } else printf("getCfgPrt NULL\n");
  if (gps.getVersion() != NULL) gps.monVer(MON_VER_EXTENSION_NUMBER);
  else printf("getVersion() is NULL\n");
  sleep(1);
  /*
  if (gps.getCfgPrt(gnssPort) != NULL) gps.cfgPrt();
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

  printf("getCfgInf(UBX, COM1)");
  if (gps.getCfgInf(UBX, COM1) != NULL) gps.cfgInf();
  else printf(" failed\n");
  sleep(1);
  printf("getCfgInf(NMEA, COM1)");
  if (gps.getCfgInf(NMEA, COM1) != NULL) gps.cfgInf();
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


  /*InProtoMask inMask;
  OutProtoMask outMask;
  inMask.inUbx = 1; inMask.inNmea = 1; inMask.inRtcm = 1;
  outMask.outUbx = 1; outMask.outNmea = 1;
  gps.pubxConfig(BAUD_RATE_9600, inMask, outMask);
  //gps.end();
  //gps.begin(dev, B14400);*/
  
  /*char msgId[4] = "ZDA";
  uint8_t rateDDC = 0, rateCOM1 = 0, rateCOM2 = 0, rateUSB = 0, rateSPI = 0;
  gps.pubxRate(msgId, rateDDC, rateCOM1, rateCOM2, rateUSB, rateSPI);*/  

  eraseLog();
  /*startLogging(MIN_SIZE, 0, true);
  sleep(30);
  getLog = true;*/
}

void loop() {
  if (getLog) {
    stopLogging();
    LogInfo * log = gps.getLogInfo();
    if (log != NULL) {
      uint32_t num = log->entryCount;
      printf("Retrieving %lu log records\n", num);
      uint32_t j = 0, k = num / 256;
      for (uint32_t i = 0; i < k; i++) {
        retrieveLogs(j, 256);
        j += 256;
      }
      retrieveLogs(j, num % 256);        
    } else printf(" LogInfo NULL\n");
    getLog = false;
  }
  
  gps.get();
}

int main() {
	setup();
	while (1) {
		loop();
	}
	return 0;
}

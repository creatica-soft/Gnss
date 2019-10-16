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

void setup() {
  gps = Gnss();
  getLog = false;
  gps.begin(dev, B9600);
  /*gps.loadConfig();
   * gps.end();
   * gps.begin(dev,B38400);
   */
  wiringPiSetup();
  wiringPiISR(INTERRUPT_PIN, INT_EDGE_RISING, tp);
  printf("TOPGNSS\n");
  printf("GPS at 9600 baud\n");
  //gps.factoryReset();
  /*CfgPrt * cfgPrt = gps.getCfgPrt(gnssPort);
  if (cfgPrt != NULL) {
    gps.cfgPrt(); 
    cfgPrt->outProtoMask.outNmea = 1;
    cfgPrt->baudRate = BAUD_RATE_38400;
    if (gps.setCfgPrt(cfgPrt->portId, cfgPrt->baudRate, cfgPrt->mode, cfgPrt->inProtoMask, cfgPrt->outProtoMask)) printf("setCfgPrt OK\n");
    else printf("setCfgPrt failed\n");
    gps.end();
    gps.begin(dev, B38400);
    gps.cfgPrt();
    gps.saveConfig();
  } else printf("cfgPrt NULL\n");*/
  if (gps.getVersion() != NULL) gps.monVer(MON_VER_EXTENSION_NUMBER);
  else printf("getVersion() is NULL\n");
  if (gps.getCfgPrt(gnssPort) != NULL) gps.cfgPrt();
  else printf("getCfgPrt() is NULL\n");
  if (gps.getCfgPm() != NULL) gps.cfgPm();
  else printf("getCfgPm() is NULL\n");
  if (gps.getCfgRxm() != NULL) gps.cfgRxm();
  else printf("getCfgRxm() is NULL\n");
  if (gps.getSupportedGnss() != NULL) gps.monGnss();
  else printf("getSupportedGnss() is NULL\n");
  if (gps.getGnss() != NULL) gps.cfgGnss();
  /*
  if (gps.getCfgInf(UBX, COM1) != NULL) gps.cfgInf();
  if (gps.getCfgInf(NMEA, COM1) != NULL) gps.cfgInf();
  if (gps.getCfgNav() != NULL) gps.cfgNav();

  //setting dynModel to STATIONARY
  printf("setDynModel");
  if (gps.setDynModel(STATIONARY)) printf(" OK\n"); else printf(" failed\n");

  //set UTC to GLONASS
  printf("setUtcStandard to GLONASS");
  if (gps.setUtcStandard(GLONASS)) printf(" OK\n"); else printf(" failed\n");*/
  
  //if (gps.getCfgPms() != NULL) gps.cfgPms();
  //set power mode to AGGRESSIVE_1HZ_POWER_MODE
  /*printf("setPowerMode(AGGRESSIVE_1HZ): ");
  if (gps.setCfgPms(AGGRESSIVE_1HZ_POWER_MODE)) printf(" OK\n");
  else printf(" failed\n");*/
/*  
  //Turn on or off major GNSS
  MajorGnss gnss;
  gnss.Gps = 1;
  gnss.Glonass = 1;
  gnss.BeiDou = 0;
  gnss.Galileo = 0;
  bool enableSBAS = true, enableIMES = false;
  printf("setGNSS");
  if (gps.setGNSS(gnss, enableSBAS, enableIMES)) printf(" OK\n");
  else printf(" failed\n");
  
  if (gps.getNavRate() != NULL) gps.cfgRate();
  if (gps.getTimePulse() != NULL) gps.cfgTp();

  printf("getCfgOdo");
  ODOCfg * odoCfg = gps.getCfgOdo();
  if (odoCfg != NULL) { 
    gps.cfgOdo();
    //set odomter profile to CYCLING
    printf("setOdo CYCLING: ");  
    if (gps.setCfgOdo(odoCfg->flags, CYCLING, odoCfg->cogMaxSpeed, odoCfg->cogMaxPosAccuracy, odoCfg->velLPgain, odoCfg->cogLPgain)) printf(" OK\n");
    else printf(" failed\n");
  } else printf(" failed\n");
 
  if (gps.getCfgLogFilter() != NULL) gps.cfgLogFilter();
  if (gps.getLogInfo() != NULL) gps.logInfo();
  if (gps.getCfgNmea() != NULL) gps.cfgNmea();

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
  if (gps.getCfgGeofences(3000) != NULL) gps.cfgGeoFence(); else printf("cfgGeofence NULL\n");
  CfgMsg * cfg = gps.getCfgMsg(UBX_NAV, NAV_GEOFENCE);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != navRate) {
      if (gps.setCfgMsg(UBX_NAV, NAV_GEOFENCE, navRate)) printf("UBX-NAV-GEOFENCE OK\n"); 
      else printf("UBX-NAV-GEOFENCE failed\n");
    }
    
  CfgMsg * cfg = gps.getCfgMsg(UBX_NAV, NAV_CLOCK);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_CLOCK, 0)) printf("UBX-NAV-CLOCK OK\n"); 
      else printf("UBX-NAV-CLOCK failed\n");
    }
  cfg = gps.getCfgMsg(UBX_TIM, TIM_TP);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != navRate) {
      if (gps.setCfgMsg(UBX_TIM, TIM_TP, navRate)) printf("UBX-TIM-TP OK\n"); 
      else printf("UBX-TIM-TP failed\n");
    }
  cfg = gps.getCfgMsg(UBX_NAV, NAV_DOP);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_DOP, 0)) printf("UBX-NAV-DOP OK\n"); 
      else printf("UBX-NAV-DOP failed\n");
    }
  cfg = gps.getCfgMsg(UBX_NAV, NAV_ORB);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_ORB, 0)) printf("UBX-NAV-ORB OK\n"); 
      else printf("UBX-NAV-ORB failed\n");
    }
  cfg = gps.getCfgMsg(UBX_NAV, NAV_POSLLH);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_POSLLH, 0)) printf("UBX-NAV-POSLLH OK\n"); 
      else printf("UBX-NAV-POSLLH failed\n");
    }    
  cfg = gps.getCfgMsg(UBX_NAV, NAV_PVT);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_PVT, 0)) printf("UBX-NAV-PVT OK\n"); 
      else printf("UBX-NAV-PVT failed\n");
    }
  cfg = gps.getCfgMsg(UBX_NAV, NAV_SAT);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_SAT, 0)) printf("UBX-NAV-SAT OK\n"); 
      else printf("UBX-NAV-SAT failed\n");
    }
  cfg = gps.getCfgMsg(UBX_NAV, NAV_TIMEGPS);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_TIMEGPS, 0)) printf("UBX-NAV-TIMEGPS OK\n"); 
      else printf("UBX-NAV-TIMEGPS failed\n");
    }        
  cfg = gps.getCfgMsg(UBX_NAV, NAV_TIMEUTC);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_TIMEUTC, 0)) printf("UBX-NAV-TIMEUTC OK\n"); 
      else printf("UBX-NAV-TIMEUTC failed\n");
    }        
  cfg = gps.getCfgMsg(UBX_NAV, NAV_TIMELS);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_TIMELS, 0)) printf("UBX-NAV-TIMELS OK\n"); 
      else printf("UBX-NAV-TIMELS failed\n");
    }            
  cfg = gps.getCfgMsg(UBX_NAV, NAV_DGPS);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != navRate) {
      if (gps.setCfgMsg(UBX_NAV, NAV_DGPS, navRate)) printf("UBX-NAV-DGPS OK\n"); 
      else printf("UBX-NAV-DGPS failed\n");
    }
  cfg = gps.getCfgMsg(UBX_NAV, NAV_ODO);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != navRate) {
      if (gps.setCfgMsg(UBX_NAV, NAV_ODO, navRate)) printf("UBX-NAV-ODO OK\n"); 
      else printf("UBX-NAV-ODO failed\n");
    }
  cfg = gps.getCfgMsg(UBX_NAV, NAV_SBAS);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != navRate) {
      if (gps.setCfgMsg(UBX_NAV, NAV_SBAS, navRate)) printf("UBX-NAV-SBAS OK\n"); 
      else printf("UBX-NAV-SBAS failed\n");
    }
  cfg = gps.getCfgMsg(UBX_NAV, NAV_SLAS);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != navRate) {
      if (gps.setCfgMsg(UBX_NAV, NAV_SLAS, navRate)) printf("UBX-NAV-SLAS OK\n"); 
      else printf("UBX-NAV-SLAS failed\n");
    }
  cfg = gps.getCfgMsg(UBX_NMEA, NMEA_GNS);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NMEA, NMEA_GNS, 0)) printf("UBX-NMEA-GNS OK\n"); 
      else printf("UBX-NMEA-GNS failed\n");
    }  
  cfg = gps.getCfgMsg(UBX_NMEA, NMEA_GST);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NMEA, NMEA_GST, 0)) printf("UBX-NMEA-GST OK\n"); 
      else printf("UBX-NMEA-GST failed\n");
    }
  cfg = gps.getCfgMsg(UBX_NMEA, NMEA_VLW);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NMEA, NMEA_VLW, 0)) printf("UBX-NMEA-VLW OK\n"); 
      else printf("UBX-NMEA-VLW failed\n");
    }  
  cfg = gps.getCfgMsg(UBX_NMEA, NMEA_ZDA);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NMEA, NMEA_ZDA, 0)) printf("UBX-NMEA-ZDA OK\n"); 
      else printf("UBX-NMEA-ZDA failed\n");
    }  
  cfg = gps.getCfgMsg(UBX_PUBX, PUBX_POSITION);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_PUBX, PUBX_POSITION, 0)) printf("UBX-PUBX-POSITION OK\n"); 
      else printf("UBX-PUBX-POSITION failed\n");
    }  
  cfg = gps.getCfgMsg(UBX_PUBX, PUBX_TIME);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_PUBX, PUBX_TIME, 0)) printf("UBX-PUBX-TIME OK\n"); 
      else printf("UBX-PUBX-TIME failed\n");
    } 
  //somehow UBX-PUBX-SVSTATUS doesn't work on my arduino mega - use at your own risk!  
  cfg = gps.getCfgMsg(UBX_PUBX, PUBX_SVSTATUS);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_PUBX, PUBX_SVSTATUS, 0)) printf("UBX-PUBX-SVSTATUS OK\n"); 
      else printf("UBX-PUBX-SVSTATUS failed\n");
    }    
*/

  /*InProtoMask inMask;
  OutProtoMask outMask;
  inMask.inUbx = 1; inMask.inNmea = 1; inMask.inRtcm = 1;
  outMask.outUbx = 1; outMask.outNmea = 1;
  gps.pubxConfig(BAUD_RATE_14400, inMask, outMask);
  gps.end();
  gps.begin(dev, B14400);*/
  
  /*char msgId[4] = "ZDA";
  uint8_t rateDDC = 0, rateCOM1 = 0, rateCOM2 = 0, rateUSB = 0, rateSPI = 0;
  gps.pubxRate(msgId, rateDDC, rateCOM1, rateCOM2, rateUSB, rateSPI);*/  

  //eraseLog();
  //startLogging(MIN_SIZE, 0, true);
  
}

void startLogging(LogSize ls_enum, uint32_t customSize, bool circular) {
  printf("createLog");
  LogInfo * log = gps.getLogInfo();
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
  if (gps.setCfgLogFilter(0, 15, 0, 0, flags)) {
    printf(" OK\n");
    char msg[16] = "Logging started"; 
    if (gps.logMsg(msg, 16)) printf(" logMsg %s OK\n", msg); else printf(" logMsg failed\n");
  } else printf(" failed\n");
}

void stopLogging() {
  char msg[16] = "Logging stopped";
  if (gps.logMsg(msg, 16)) printf(" logMsg %s OK\n", msg); else printf(" logMsg failed\n");
  CfgLogFilterFlags flags;
  flags.recordingEnabled = 0;
  flags.psmOncePerWakeUpEnabled = 0;
  flags.applyAllFilterSettings = 0;
  printf("stopLogging");
  if (gps.setCfgLogFilter(0, 0, 0, 0, flags)) printf(" OK\n"); else printf(" failed\n");
}

void eraseLog() {
  printf("eraseLog");
  LogInfo * log = gps.getLogInfo();
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

void loop() {
  if (getLog) {
    stopLogging();
    LogInfo * log = gps.getLogInfo();
    if (log != NULL){
      uint32_t num = log->entryCount;
      printf("Retrieving %d log records\n", num);
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
  //usleep(1000);
}

int main() {
	setup();
	while (1) {
		loop();
	}
	return 0;
}

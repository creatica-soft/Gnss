#include "gnss.cpp"
#include <nz_dst.h>

#define Gps Serial1
#define console Serial2 

volatile bool getLog, gpsDataAvailable;
const Port gnssPort = COM1;
const uint8_t navRate = 1;
Gnss<HardwareSerial, HardwareSerial> gps(Gps, console);
//Gnss<HardwareSerial> gps(Gps);

void lg() {
  getLog = true;
}

void tp() {
  gps.tp();
}

void setup() {
  //pinMode(3, INPUT);
  //digitalWrite(3, LOW);
  //attachInterrupt(digitalPinToInterrupt(21), tp, RISING);
  //attachInterrupt(digitalPinToInterrupt(3), lg, HIGH);
  set_zone(12 * ONE_HOUR);
  set_dst(nz_dst);
  gps.pps = false;
  getLog = false;
  gpsDataAvailable = false;
  //console.begin(115200);
  //while (!console); //Wait for user to open terminal, needed for the USB port (Serial)
  //console.println(F("TOPGNSS"));
  //console.read();
  gps.begin(9600, 115200);
  /*gps.loadConfig();
   * gps.end();
   * gps.begin(115200,115200);
   */
  //Gps.write("\r\n");
  console.println(F("TOPGNSS"));
  //gps.begin(9600);
  //console.println("GPS at 9600 baud");
  //gps.factoryReset();
  /*CfgPrt * cfgPrt = gps.getCfgPrt(gnssPort);
  if (cfgPrt != NULL) {
    gps.cfgPrt(); 
    cfgPrt->outProtoMask.outNmea = 1;
    bool extendedTxTimeout = true;
    cfgPrt->baudRate = BAUD_RATE_115200;
    if (gps.setCfgPrt(cfgPrt->portId, cfgPrt->baudRate, cfgPrt->mode, cfgPrt->inProtoMask, cfgPrt->outProtoMask, extendedTxTimeout)) console.println(F("setCfgPrt OK"));
    else console.println(F("setCfgPrt failed"));
    gps.end();
    gps.begin(115200,115200);
    gps.cfgPrt();
    gps.saveConfig();
  } else console.println(F("cfgPrt NULL"));*/
/*  
  if (gps.getVersion() != NULL) gps.monVer(MON_VER_EXTENSION_NUMBER);
  if (gps.getCfgInf(UBX, COM1) != NULL) gps.cfgInf();
  if (gps.getCfgInf(NMEA, COM1) != NULL) gps.cfgInf();
  if (gps.getCfgNav() != NULL) gps.cfgNav();

  //setting dynModel to STATIONARY
  console.print(F("setDynModel"));
  if (gps.setDynModel(STATIONARY)) console.println(F(" OK")); else console.println(F(" failed"));

  //set UTC to GLONASS
  console.print(F("setUtcStandard to GLONASS"));
  if (gps.setUtcStandard(GLONASS)) console.println(F(" OK")); else console.println(F(" failed"));*/

  if (gps.getCfgPrt(gnssPort) != NULL) gps.cfgPrt();
  
  if (gps.getCfgPms() != NULL) gps.cfgPms();
  //set power mode to AGGRESSIVE_1HZ_POWER_MODE
  /*console.print("setPowerMode(AGGRESSIVE_1HZ): ");
  if (gps.setCfgPms(AGGRESSIVE_1HZ_POWER_MODE)) console.println(" OK");
  else console.println(" failed");*/
/*  
  if (gps.getCfgPm() != NULL) gps.cfgPm();
  if (gps.getCfgRxm() != NULL) gps.cfgRxm();
  if (gps.getSupportedGnss() != NULL) gps.monGnss();
  if (gps.getGnss() != NULL) gps.cfgGnss();
  //Turn on or off major GNSS
  MajorGnss gnss;
  gnss.Gps = 1;
  gnss.Glonass = 1;
  gnss.BeiDou = 0;
  gnss.Galileo = 0;
  bool enableSBAS = true, enableIMES = false;
  console.print("setGNSS");
  if (gps.setGNSS(gnss, enableSBAS, enableIMES)) console.println(" OK");
  else console.println(" failed");
  
  if (gps.getNavRate() != NULL) gps.cfgRate();
  if (gps.getTimePulse() != NULL) gps.cfgTp();

  console.print("getCfgOdo");
  ODOCfg * odoCfg = gps.getCfgOdo();
  if (odoCfg != NULL) { 
    gps.cfgOdo();
    //set odomter profile to CYCLING
    console.print("setOdo CYCLING: ");  
    if (gps.setCfgOdo(odoCfg->flags, CYCLING, odoCfg->cogMaxSpeed, odoCfg->cogMaxPosAccuracy, odoCfg->velLPgain, odoCfg->cogLPgain)) console.println(" OK");
    else console.println(" failed");
  } else console.println(" failed");
 
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
  if (gps.setCfgGeofence(&geofence, confidenceLevel)) console.println(F("setCfgGeofence OK")); else console.println(F("setCfgGeofence failed"));
  if (gps.getCfgGeofences(3000) != NULL) gps.cfgGeoFence(); else console.println(F("cfgGeofence NULL"));
  CfgMsg * cfg = gps.getCfgMsg(UBX_NAV, NAV_GEOFENCE);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != navRate) {
      if (gps.setCfgMsg(UBX_NAV, NAV_GEOFENCE, navRate)) console.println(F("UBX-NAV-GEOFENCE OK")); 
      else console.println(F("UBX-NAV-GEOFENCE failed"));
    }
    
  CfgMsg * cfg = gps.getCfgMsg(UBX_NAV, NAV_CLOCK);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_CLOCK, 0)) console.println(F("UBX-NAV-CLOCK OK")); 
      else console.println(F("UBX-NAV-CLOCK failed"));
    }
  cfg = gps.getCfgMsg(UBX_TIM, TIM_TP);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != navRate) {
      if (gps.setCfgMsg(UBX_TIM, TIM_TP, navRate)) console.println(F("UBX-TIM-TP OK")); 
      else console.println(F("UBX-TIM-TP failed"));
    }
  cfg = gps.getCfgMsg(UBX_NAV, NAV_DOP);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_DOP, 0)) console.println(F("UBX-NAV-DOP OK")); 
      else console.println(F("UBX-NAV-DOP failed"));
    }
  cfg = gps.getCfgMsg(UBX_NAV, NAV_ORB);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_ORB, 0)) console.println(F("UBX-NAV-ORB OK")); 
      else console.println(F("UBX-NAV-ORB failed"));
    }
  cfg = gps.getCfgMsg(UBX_NAV, NAV_POSLLH);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_POSLLH, 0)) console.println(F("UBX-NAV-POSLLH OK")); 
      else console.println(F("UBX-NAV-POSLLH failed"));
    }    
  cfg = gps.getCfgMsg(UBX_NAV, NAV_PVT);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_PVT, 0)) console.println(F("UBX-NAV-PVT OK")); 
      else console.println(F("UBX-NAV-PVT failed"));
    }
  cfg = gps.getCfgMsg(UBX_NAV, NAV_SAT);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_SAT, 0)) console.println(F("UBX-NAV-SAT OK")); 
      else console.println(F("UBX-NAV-SAT failed"));
    }
  cfg = gps.getCfgMsg(UBX_NAV, NAV_TIMEGPS);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_TIMEGPS, 0)) console.println(F("UBX-NAV-TIMEGPS OK")); 
      else console.println(F("UBX-NAV-TIMEGPS failed"));
    }        
  cfg = gps.getCfgMsg(UBX_NAV, NAV_TIMEUTC);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_TIMEUTC, 0)) console.println(F("UBX-NAV-TIMEUTC OK")); 
      else console.println(F("UBX-NAV-TIMEUTC failed"));
    }        
  cfg = gps.getCfgMsg(UBX_NAV, NAV_TIMELS);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NAV, NAV_TIMELS, 0)) console.println(F("UBX-NAV-TIMELS OK")); 
      else console.println(F("UBX-NAV-TIMELS failed"));
    }            
  cfg = gps.getCfgMsg(UBX_NAV, NAV_DGPS);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != navRate) {
      if (gps.setCfgMsg(UBX_NAV, NAV_DGPS, navRate)) console.println(F("UBX-NAV-DGPS OK")); 
      else console.println(F("UBX-NAV-DGPS failed"));
    }
  cfg = gps.getCfgMsg(UBX_NAV, NAV_ODO);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != navRate) {
      if (gps.setCfgMsg(UBX_NAV, NAV_ODO, navRate)) console.println(F("UBX-NAV-ODO OK")); 
      else console.println(F("UBX-NAV-ODO failed"));
    }
  cfg = gps.getCfgMsg(UBX_NAV, NAV_SBAS);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != navRate) {
      if (gps.setCfgMsg(UBX_NAV, NAV_SBAS, navRate)) console.println(F("UBX-NAV-SBAS OK")); 
      else console.println(F("UBX-NAV-SBAS failed"));
    }
  cfg = gps.getCfgMsg(UBX_NAV, NAV_SLAS);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != navRate) {
      if (gps.setCfgMsg(UBX_NAV, NAV_SLAS, navRate)) console.println(F("UBX-NAV-SLAS OK")); 
      else console.println(F("UBX-NAV-SLAS failed"));
    }
  cfg = gps.getCfgMsg(UBX_NMEA, NMEA_GNS);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NMEA, NMEA_GNS, 0)) console.println(F("UBX-NMEA-GNS OK")); 
      else console.println(F("UBX-NMEA-GNS failed"));
    }  
  cfg = gps.getCfgMsg(UBX_NMEA, NMEA_GST);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NMEA, NMEA_GST, 0)) console.println(F("UBX-NMEA-GST OK")); 
      else console.println(F("UBX-NMEA-GST failed"));
    }
  cfg = gps.getCfgMsg(UBX_NMEA, NMEA_VLW);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NMEA, NMEA_VLW, 0)) console.println(F("UBX-NMEA-VLW OK")); 
      else console.println(F("UBX-NMEA-VLW failed"));
    }  
  cfg = gps.getCfgMsg(UBX_NMEA, NMEA_ZDA);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_NMEA, NMEA_ZDA, 0)) console.println(F("UBX-NMEA-ZDA OK")); 
      else console.println(F("UBX-NMEA-ZDA failed"));
    }  
  cfg = gps.getCfgMsg(UBX_PUBX, PUBX_POSITION);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_PUBX, PUBX_POSITION, 0)) console.println(F("UBX-PUBX-POSITION OK")); 
      else console.println(F("UBX-PUBX-POSITION failed"));
    }  
  cfg = gps.getCfgMsg(UBX_PUBX, PUBX_TIME);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_PUBX, PUBX_TIME, 0)) console.println(F("UBX-PUBX-TIME OK")); 
      else console.println(F("UBX-PUBX-TIME failed"));
    } 
  //somehow UBX-PUBX-SVSTATUS doesn't work on my arduino mega - use at your own risk!  
  cfg = gps.getCfgMsg(UBX_PUBX, PUBX_SVSTATUS);
  if (cfg != NULL) 
    if (cfg->rate[gnssPort] != 0) {
      if (gps.setCfgMsg(UBX_PUBX, PUBX_SVSTATUS, 0)) console.println(F("UBX-PUBX-SVSTATUS OK")); 
      else console.println(F("UBX-PUBX-SVSTATUS failed"));
    }    
*/

  /*InProtoMask inMask;
  OutProtoMask outMask;
  inMask.inUbx = 1; inMask.inNmea = 1; inMask.inRtcm = 1;
  outMask.outUbx = 1; outMask.outNmea = 1;
  gps.pubxConfig(BAUD_RATE_14400, inMask, outMask);
  gps.end();
  gps.begin(14400, 115200);*/
  
  /*char msgId[4] = "ZDA";
  uint8_t rateDDC = 0, rateCOM1 = 0, rateCOM2 = 0, rateUSB = 0, rateSPI = 0;
  gps.pubxRate(msgId, rateDDC, rateCOM1, rateCOM2, rateUSB, rateSPI);*/  

  //eraseLog();
  //startLogging(MIN_SIZE, 0, true);
  
}

void startLogging(LogSize ls_enum, uint32_t customSize, bool circular) {
  console.print("createLog");
  LogInfo * log = gps.getLogInfo();
  if (log == NULL) { console.println(" logInfo NULL"); return; }
  if (log->flags.inactive) {
    if (gps.createLog(ls_enum, customSize, circular)) console.println(" OK"); else console.println(" logCreate failed");
  }
  if (log->flags.enabled) return;
  CfgLogFilterFlags flags;
  flags.recordingEnabled = 1;
  flags.psmOncePerWakeUpEnabled = 0;
  flags.applyAllFilterSettings = 1;
  console.print(" startLogging");
  if (gps.setCfgLogFilter(0, 15, 0, 0, flags)) {
    console.println(" OK");
    char msg[16] = "Logging started"; 
    if (gps.logMsg(msg, 16)) console.println(String(" logMsg '") + msg + String("' OK")); else console.println(" logMsg failed");
  } else console.println(" failed");
}

void stopLogging() {
  char msg[16] = "Logging stopped";
  if (gps.logMsg(msg, 16)) console.println(String(" logMsg '") + msg + String("' OK")); else console.println(" logMsg failed");
  CfgLogFilterFlags flags;
  flags.recordingEnabled = 0;
  flags.psmOncePerWakeUpEnabled = 0;
  flags.applyAllFilterSettings = 0;
  console.print("stopLogging");
  if (gps.setCfgLogFilter(0, 0, 0, 0, flags)) console.println(" OK"); else console.println(" failed");
}

void eraseLog() {
  console.print("eraseLog");
  LogInfo * log = gps.getLogInfo();
  if (log == NULL) { console.println(" logInfo NULL"); return; }
  if (log->flags.enabled) {
    CfgLogFilterFlags flags;
    flags.recordingEnabled = 0;
    flags.psmOncePerWakeUpEnabled = 0;
    flags.applyAllFilterSettings = 0;
    if (!gps.setCfgLogFilter(0, 0, 0, 0, flags)) { console.println(" setLogFilter failed"); return; }
  }
  if (!log->flags.inactive) {
    if (!gps.eraseLog()) console.println(" failed");  
  }
  console.println(" OK");
}

void retrieveLogs(uint32_t idx, uint32_t count) {
  if (!gps.logRetrieve(idx, count)) { console.println("logRetrieve failed"); return; }
}

void loop() {
  if (getLog) {
    stopLogging();
    LogInfo * log = gps.getLogInfo();
    if (log != NULL){
      uint32_t num = log->entryCount;
      console.println(String("Retrieving ") + num + String(" log records"));
      uint32_t j = 0, k = num / 256;
      for (uint32_t i = 0; i < k; i++) {
        retrieveLogs(j, 256);
        j += 256;
      }
      retrieveLogs(j, num % 256);        
    } else console.println(" LogInfo NULL");
    getLog = false;
  }
  
  if (gpsDataAvailable) gps.get();
  gpsDataAvailable = false;
}

#if defined(Gps) && Gps == Serial
void serialEvent() {
  gpsDataAvailable = true;
}
#endif
#if defined(Gps) && Gps == Serial1
void serialEvent1() {
  gpsDataAvailable = true;
}
#endif
#if defined(Gps) && Gps == Serial2
void serialEvent2() {
  gpsDataAvailable = true;
}
#endif
#if defined(Gps) && Gps == Serial3
void serialEvent3() {
  gpsDataAvailable = true;
}
#endif

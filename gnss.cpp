#include "gnss.h"

template <class T> Gnss<T, null>::Gnss(T &serial) : serial(serial) {
	payloadLength = 0;
	pollPayloadLength = 0;
    offset = 0;
	error = NO_ERROR;
	payload = NULL;
	pollPayload = NULL;
	nmea = false;
	nmeaChecksumNext = false;
	nmeaValid = false;
	nmeaPayload = "", nmeaChecksum = "";
	endOfNavEpoch = false;
	iTOW = 0;
	ttp = false;
}

template <class T, class D> Gnss<T,D>::Gnss(T &serial, D &debug) : serial(serial), debug(debug) {
	payloadLength = 0;
	pollPayloadLength = 0;
    offset = 0;
	error = NO_ERROR;
	payload = NULL;
	pollPayload = NULL;
	nmea = false;
	nmeaChecksumNext = false;
	nmeaValid = false;
	nmeaPayload = "", nmeaChecksum = "";
	endOfNavEpoch = false;
	iTOW = 0;
	ttp = false;
}

template <class T> void Gnss<T,null>::tp() {
  pps = true;
  if (ttp) { set_system_time(utcTime); ttp = false; }//use this with timTp message only
  else system_tick(); //use this with normal time info such as navPvt, etc
}

template <class T, class D> void Gnss<T,D>::tp() {
  pps = true;
  if (ttp) { set_system_time(utcTime); ttp = false; }//use this with timTp message only
  else system_tick(); //use this with normal time info such as navPvt, etc
}

template <class T> void Gnss<T, null>::begin(uint32_t gpsSpeed) {
  serial.begin(gpsSpeed);
#ifdef DEBUG_LEVEL_UBX
  InfoMsgMask mask = 0;
  switch (DEBUG_LEVEL_UBX) {
  case DEBUG_LEVEL_ERROR: mask.error = 1; break;
  case DEBUG_LEVEL_WARNING: mask.warning = 1; mask.error = 1; break;
  case DEBUG_LEVEL_NOTICE: mask.notice = 1; mask.warning = 1; mask.error = 1; break;
  case DEBUG_LEVEL_TEST: mask.test = 1; mask.notice = 1; mask.warning = 1; mask.error = 1; break;
  case DEBUG_LEVEL_DEBUG: mask.debug = 1; mask.test = 1; mask.notice = 1; mask.warning = 1; mask.error = 1; break;
  }
  setCfgInf(mask, UBX);
#endif
#ifdef DEBUG_LEVEL_NMEA
  InfoMsgMask mask;
  switch (DEBUG_LEVEL_NMEA) {
  case DEBUG_LEVEL_ERROR: mask.error = 1; break;
  case DEBUG_LEVEL_WARNING: mask.warning = 1; mask.error = 1; break;
  case DEBUG_LEVEL_NOTICE: mask.notice = 1; mask.warning = 1; mask.error = 1; break;
  case DEBUG_LEVEL_TEST: mask.test = 1; mask.notice = 1; mask.warning = 1; mask.error = 1; break;
  case DEBUG_LEVEL_DEBUG: mask.debug = 1; mask.test = 1; mask.notice = 1; mask.warning = 1; mask.error = 1; break;
  }
  setCfgInf(mask, NMEA);
#endif
}

template <class T, class D> void Gnss<T,D>::begin(uint32_t gpsSpeed, uint32_t debugSpeed) {
  serial.begin(gpsSpeed);
  debug.begin(debugSpeed);
#ifdef DEBUG_LEVEL_UBX
  InfoMsgMask mask = 0;
  switch (DEBUG_LEVEL_UBX) {
  case DEBUG_LEVEL_ERROR: mask.error = 1; break;
  case DEBUG_LEVEL_WARNING: mask.warning = 1; mask.error = 1; break;
  case DEBUG_LEVEL_NOTICE: mask.notice = 1; mask.warning = 1; mask.error = 1; break;
  case DEBUG_LEVEL_TEST: mask.test = 1; mask.notice = 1; mask.warning = 1; mask.error = 1; break;
  case DEBUG_LEVEL_DEBUG: mask.debug = 1; mask.test = 1; mask.notice = 1; mask.warning = 1; mask.error = 1; break;
  }
  setCfgInf(mask, UBX);
#endif
#ifdef DEBUG_LEVEL_NMEA
  InfoMsgMask mask;
  switch (DEBUG_LEVEL_NMEA) {
  case DEBUG_LEVEL_ERROR: mask.error = 1; break;
  case DEBUG_LEVEL_WARNING: mask.warning = 1; mask.error = 1; break;
  case DEBUG_LEVEL_NOTICE: mask.notice = 1; mask.warning = 1; mask.error = 1; break;
  case DEBUG_LEVEL_TEST: mask.test = 1; mask.notice = 1; mask.warning = 1; mask.error = 1; break;
  case DEBUG_LEVEL_DEBUG: mask.debug = 1; mask.test = 1; mask.notice = 1; mask.warning = 1; mask.error = 1; break;
  }
  setCfgInf(mask, NMEA);
#endif
}

template <class T> void Gnss<T, null>::end() {
  serial.end();
}

template <class T, class D> void Gnss<T,D>::end() {
  serial.end();
  debug.end();
}

template <class T> uint16_t Gnss<T,null>::available() {
  return serial.available();
}

template <class T, class D> uint16_t Gnss<T,D>::available() {
  return serial.available();
}

template <class T> uint8_t Gnss<T,null>::read() {
  return serial.read();
}

template <class T, class D> uint8_t Gnss<T,D>::read() {
  return serial.read();
}

template <class T> void Gnss<T,null>::calculateChecksum(uint8_t msgClass, uint8_t msgId, uint16_t len, uint8_t * pload) {
      checksum[0] = 0;
	  checksum[1] = 0;
      checksum[0] += msgClass;
	  checksum[1] += checksum[0];
	  checksum[0] += msgId;
	  checksum[1] += checksum[0];		
	  checksum[0] += (uint8_t)(len & 0xFF); 
	  checksum[1] += checksum[0];
	  checksum[0] += (uint8_t)(len >> 8);
	  checksum[1] += checksum[0];

	  if (pload != NULL) {
	  	  for (uint16_t i = 0; i < len; i++) {
		  	  checksum[0] += pload[i];
			  checksum[1] += checksum[0];
		  }
	  }
}

template <class T, class D> void Gnss<T,D>::calculateChecksum(uint8_t msgClass, uint8_t msgId, uint16_t len, uint8_t * pload) {
      checksum[0] = 0;
	  checksum[1] = 0;
      checksum[0] += msgClass;
	  checksum[1] += checksum[0];
	  checksum[0] += msgId;
	  checksum[1] += checksum[0];		
	  checksum[0] += (uint8_t)(len & 0xFF); 
	  checksum[1] += checksum[0];
	  checksum[0] += (uint8_t)(len >> 8);
	  checksum[1] += checksum[0];

	  if (pload != NULL) {
	  	  for (uint16_t i = 0; i < len; i++) {
		  	  checksum[0] += pload[i];
			  checksum[1] += checksum[0];
		  }
	  }
}

template <class T> void Gnss<T,null>::nmeaVerifyChecksum() {
	uint8_t x = 0, hex, high_byte = 0, low_byte = 0;
	uint16_t len = nmeaPayload.length();
	const char * cString = nmeaPayload.c_str();

	for (uint8_t i = 0; i < len; i++ ) {
		x ^= cString[i];
	}

	if (isDigit(nmeaChecksum[0])) high_byte = (nmeaChecksum[0] - 48);
    else if (isHexadecimalDigit(nmeaChecksum[0])) high_byte = (nmeaChecksum[0] - 55);
    if (isDigit(nmeaChecksum[1])) low_byte = (nmeaChecksum[1] - 48);
    else if (isHexadecimalDigit(nmeaChecksum[1])) low_byte = (nmeaChecksum[1] - 55);
    //hex = high_byte * 16 + low_byte;
	hex = (high_byte << 4) | low_byte;

	if (hex == x) nmeaValid = true; else nmeaValid = false;
}

template <class T, class D> void Gnss<T,D>::nmeaVerifyChecksum() {
	uint8_t x = 0, hex, high_byte = 0, low_byte = 0;
	uint16_t len = nmeaPayload.length();
	const char * cString = nmeaPayload.c_str();

	for (uint8_t i = 0; i < len; i++ ) {
		x ^= cString[i];
	}

	if (isDigit(nmeaChecksum[0])) high_byte = (nmeaChecksum[0] - 48);
    else if (isHexadecimalDigit(nmeaChecksum[0])) high_byte = (nmeaChecksum[0] - 55);
    if (isDigit(nmeaChecksum[1])) low_byte = (nmeaChecksum[1] - 48);
    else if (isHexadecimalDigit(nmeaChecksum[1])) low_byte = (nmeaChecksum[1] - 55);
	hex = (high_byte << 4) | low_byte;

	if (hex == x) nmeaValid = true; else nmeaValid = false;
}

template <class T> bool Gnss<T, null>::ready() {
  while (available()) {
    uint8_t c = read();
    if (!nmea) { //UBX message
      if (offset < 6) {
			switch (offset) {
				case 0:
					if (c == NMEA_START) { nmea = true; nmeaChecksumNext = false; nmeaPayload = ""; nmeaChecksum = ""; continue; }
					if (c != SYNC_CHARS[0]) continue;
					break;
				case 1:
					if (c != SYNC_CHARS[1])
					{
						offset = 0;
						continue;
					}
					break;					
				case 2:
					messageClass = c;
					break;
				case 3:					
					messageId = c;
					break;
				case 4: payloadLength = c; break;
				case 5: 
					payloadLength |= (((uint16_t)c) << 8); 
					payload = (uint8_t *)realloc(payload, payloadLength); 
					if (payload == NULL && payloadLength > 0) { offset = 0; error = OUT_OF_MEMORY; return true;}
					break;
			}			
	        offset++;
	  }
      else { //offset >= 6
			if ((offset < (payloadLength + 6))) {
				payload[offset - 6] = c;
				offset++;
			}
			else if (offset == payloadLength + 6) {
				calculateChecksum(messageClass, messageId, payloadLength, payload);
				if (c == checksum[0]) offset++;
				else {
					offset = 0;
					error = CHECKSUM_ERROR; return true;
				}
			}
			else if (offset == payloadLength + 7) {
			  offset = 0;
			  if (c == checksum[1])	{
				error = NO_ERROR;
				if (messageClass == UBX_NAV && messageId == NAV_EOE) { endOfNavEpoch = true; iTOW = *(uint32_t *)payload; } else endOfNavEpoch = false;
				return true;
			  }
			  else 	error = CHECKSUM_ERROR; return true;
			}
	  }
	} else { //nmea message
		String cls = "", msgId = "";
		switch (c) {
		   case '*': nmeaChecksumNext = true; break;
		   case '\r': break;
		   case '\n': 
			nmea = false; 
			nmeaVerifyChecksum(); 
			if (nmeaValid) { 
				cls = nmeaPayload.substring(2,5);
				if (cls == "RMC") { messageClass = UBX_NMEA; messageId = NMEA_RMC; }
				else if (cls == "VTG") { messageClass = UBX_NMEA; messageId = NMEA_VTG; }
				else if (cls == "GGA") { messageClass = UBX_NMEA; messageId = NMEA_GGA; }
				else if (cls == "GSA") { messageClass = UBX_NMEA; messageId = NMEA_GSA; }
				else if (cls == "GSV") { messageClass = UBX_NMEA; messageId = NMEA_GSV; }
				else if (cls == "GLL") { messageClass = UBX_NMEA; messageId = NMEA_GLL; }
				else if (cls == "GST") { messageClass = UBX_NMEA; messageId = NMEA_GST; }
				else if (cls == "VLW") { messageClass = UBX_NMEA; messageId = NMEA_VLW; }
				else if (cls == "GNS") { messageClass = UBX_NMEA; messageId = NMEA_GNS; }
				else if (cls == "ZDA") { messageClass = UBX_NMEA; messageId = NMEA_ZDA; }
				cls = nmeaPayload.substring(0,4);
				if (cls == "PUBX") { messageClass = UBX_PUBX; 
					msgId = nmeaPayload.substring(5, 7);
					if (msgId == "41") messageId = PUBX_CONFIG;
					else if (msgId == "00") messageId = PUBX_POSITION;
					else if (msgId == "40") messageId = PUBX_RATE;
					else if (msgId == "03") messageId = PUBX_SVSTATUS;
					else if (msgId == "04") messageId = PUBX_TIME;
				}
				return true; 
			} else return false;
			default: 
				if (!nmeaChecksumNext) nmeaPayload += char(c); 
				else nmeaChecksum += char(c);
				break;
		}
	}
  }
  return false;
}

template <class T, class D> bool Gnss<T,D>::ready() {
  while (available()) {
    uint8_t c = read();
    if (!nmea) { //UBX message
      if (offset < 6) {
			switch (offset) {
				case 0:
				    //debug.print(c, HEX); debug.print(" ");
					if (c == NMEA_START) { nmea = true; nmeaChecksumNext = false; nmeaPayload = ""; nmeaChecksum = ""; continue; }
					if (c != SYNC_CHARS[0]) continue;
					break;
				case 1:
				    //debug.print(c, HEX); 
					if (c != SYNC_CHARS[1]) {
						offset = 0;
						continue;
					}
					break;					
				case 2:
					//debug.print(" Receiving UBX... ");
				    //debug.print("msgCls "); debug.print(c, HEX);  
					messageClass = c;
					break;
				case 3:					
				    //debug.print(", msgId "); debug.print(c, HEX);  
					messageId = c;
					break;
				case 4: payloadLength = c; break;
				case 5: 
					payloadLength |= (((uint16_t)c) << 8); 
					//debug.print(", len "); debug.print(payloadLength); debug.print(", payload: ");
					payload = (uint8_t *)realloc(payload, payloadLength); 
					if (payload == NULL && payloadLength > 0) { offset = 0; error = OUT_OF_MEMORY; return true;}
					break;
			}			
	        offset++;
	  }
      else { //offset >= 6
			if ((offset < (payloadLength + 6))) {
				payload[offset - 6] = c;
				//debug.print(c, HEX); debug.print(" ");
				offset++;
			}
			else if (offset == payloadLength + 6) {
			    //debug.println();
				calculateChecksum(messageClass, messageId, payloadLength, payload);
				//debug.print(" checksum0 "); debug.print(checksum[0], HEX);  debug.print(" checksum1 "); debug.println(checksum[1],HEX);
				if (c == checksum[0]) offset++;
				else {
					offset = 0;
					//debug.print("checksum0 "); debug.print(checksum[0], HEX);
					//debug.print(" - wrong checksum0: "); //debug.println(c, HEX);
					//poll(pollMessageClass, pollMessageId, pollPayloadLength, pollPayload);
					error = CHECKSUM_ERROR; return true;
				}
				//debug.print(" rec chksum0 "); debug.print(c, HEX);
			}
			else if (offset == payloadLength + 7) {
			  offset = 0;
			  //debug.print(", rec chksum1 "); debug.println(c, HEX);
			  if (c == checksum[1])	{
				if (messageClass == UBX_NAV && messageId == NAV_EOE) { endOfNavEpoch = true; iTOW = *(uint32_t *)payload; } else endOfNavEpoch = false;
				error = NO_ERROR;
				return true;
			  }
			  else {
				//debug.print("checksum1 "); debug.print(checksum[1], HEX);
				//debug.print(" - wrong checksum1: "); //debug.println(c, HEX);
				//poll(pollMessageClass, pollMessageId, pollPayloadLength, pollPayload);
				error = CHECKSUM_ERROR; return true;
			  }
			}
	  }
	} else { //nmea message
		String cls = "", msgId = "";
		switch (c) {
		   case '*': nmeaChecksumNext = true; break;
		   case '\r': break;
		   case '\n': 
			nmea = false; 
			nmeaVerifyChecksum(); 
			//debug.println(nmeaPayload);
			if (nmeaValid) { 
				cls = nmeaPayload.substring(2,5);
				if (cls == "RMC") { messageClass = UBX_NMEA; messageId = NMEA_RMC; }
				else if (cls == "VTG") { messageClass = UBX_NMEA; messageId = NMEA_VTG; }
				else if (cls == "GGA") { messageClass = UBX_NMEA; messageId = NMEA_GGA; }
				else if (cls == "GSA") { messageClass = UBX_NMEA; messageId = NMEA_GSA; }
				else if (cls == "GSV") { messageClass = UBX_NMEA; messageId = NMEA_GSV; }
				else if (cls == "GLL") { messageClass = UBX_NMEA; messageId = NMEA_GLL; }
				else if (cls == "GST") { messageClass = UBX_NMEA; messageId = NMEA_GST; }
				else if (cls == "VLW") { messageClass = UBX_NMEA; messageId = NMEA_VLW; }
				else if (cls == "GNS") { messageClass = UBX_NMEA; messageId = NMEA_GNS; }
				else if (cls == "ZDA") { messageClass = UBX_NMEA; messageId = NMEA_ZDA; }
				cls = nmeaPayload.substring(0,4);
				if (cls == "PUBX") { messageClass = UBX_PUBX; 
					msgId = nmeaPayload.substring(5, 7);
					if (msgId == "41") messageId = PUBX_CONFIG;
					else if (msgId == "00") messageId = PUBX_POSITION;
					else if (msgId == "40") messageId = PUBX_RATE;
					else if (msgId == "03") messageId = PUBX_SVSTATUS;
					else if (msgId == "04") messageId = PUBX_TIME;
				}
				return true; 
			} else { debug.println(String("nmea checksum error: ") + nmeaPayload); return false; }
		    default: 
				if (!nmeaChecksumNext) nmeaPayload += char(c); 
				else nmeaChecksum += char(c);
				break;
		}
	}
  }
  return false;
}

template <class T> void Gnss<T,null>::get() {
	if(ready() && error == NO_ERROR) {
		switch(messageClass) {
			case UBX_NAV:
				switch(messageId) {
				  case NAV_CLOCK: navClock(); break; 
				  case NAV_DGPS: navDgps(); break; 
				  case NAV_DOP: navDop(); break; 
				  case NAV_GEOFENCE: navGeoFence(); break; 
				  case NAV_ODO: navOdo(); break; 
				  case NAV_ORB: navOrb(); break; 
				  case NAV_POSLLH: navPosLlh(); break; 
				  case NAV_PVT: navPvt(); break; 
				  case NAV_SAT: navSat(); break; 
				  case NAV_SBAS: navSbas(); break; 
				  case NAV_SLAS: navSlas(); break; 
				  case NAV_TIMEBDS: navTimeBds(); break; 
				  case NAV_TIMEGAL: navTimeGal(); break; 
				  case NAV_TIMEGLO: navTimeGlo(); break; 
				  case NAV_TIMEGPS: navTimeGps(); break; 
				  case NAV_TIMELS: navTimeLs(); break; 
				  case NAV_TIMEUTC: navTimeUtc(); break; 
				} break;
			case UBX_CFG:
				switch(messageId) {
					case CFG_GNSS: cfgGnss(); break;
					case CFG_INF: cfgInf(); break;
					case CFG_LOGFILTER: cfgLogFilter(); break;
					case CFG_MSG: cfgMsg(); break;
					case CFG_NAV5: cfgNav(); break;
					case CFG_ODO: cfgOdo(); break;
					case CFG_PM2: cfgPm(); break;
					case CFG_PMS: cfgPms(); break;
					case CFG_PRT: cfgPrt(); break;
					case CFG_PWR: cfgPwr(); break;
					case CFG_RATE: cfgRate(); break;
					case CFG_RXM: cfgRxm(); break;
					case CFG_SBAS: cfgSbas(); break;
					case CFG_SLAS: cfgSlas(); break;
					case CFG_TP5: cfgTp(); break;
					case CFG_GEOFENCE: cfgGeoFence(); break;
					case CFG_NMEA: cfgNmea(); break;
				} break;
			case UBX_MON: 
				switch(messageId) {
					case MON_VER: monVer(MON_VER_EXTENSION_NUMBER); break;
					case MON_GNSS: monGnss(); break;
					case MON_PATCH: monPatch(); break;
				} break;
			case UBX_TIM:
				switch(messageId) {
					case TIM_TM2: timTm(); break;
					case TIM_TP: timTp(); break;
				} break;
			case UBX_LOG:
				switch(messageId) {
					case LOG_INFO: logInfo(); break;
					case LOG_RETRIEVEPOS: logRetrievePos(); break;
					case LOG_RETRIEVEPOSEXTRA: logRetrievePosExtra(); break;
					case LOG_RETRIEVESTRING: logRetrieveString(); break;
					case LOG_FINDTIME: logFindTime(); break;
				} break;
			case UBX_INF:
				char infLevel[8];
				switch (messageId) {
				case INF_DEBUG: sprintf(infLevel, String(F("debug")).c_str()); break;
				case INF_ERROR: sprintf(infLevel, String(F("error")).c_str()); break;
				case INF_NOTICE: sprintf(infLevel, String(F("notice")).c_str()); break;
				case INF_TEST: sprintf(infLevel, String(F("test")).c_str()); break;
				case INF_WARNING: sprintf(infLevel, String(F("warning")).c_str()); break;
				default: sprintf(infLevel, String(F("unknown")).c_str()); break;
				}
				infMsg(infLevel);
				break;
			case UBX_ACK:
				switch(messageId) {
					case ACK_ACK: ackAck(); break;
					case ACK_NAK: ackNak(); break;
				} break;
			case UBX_NMEA:
				switch(messageId) {
					case NMEA_RMC: nmeaRmc(); break;
					case NMEA_VTG: nmeaVtg(); break;
					case NMEA_GGA: nmeaGga(); break;
					case NMEA_GSA: nmeaGsa(); break;
					case NMEA_GSV: nmeaGsv(); break;
					case NMEA_GLL: nmeaGll(); break;
					case NMEA_GST: nmeaGst(); break;
					case NMEA_VLW: nmeaVlw(); break;
					case NMEA_GNS: nmeaGns(); break;
					case NMEA_ZDA: nmeaZda(); break;
				} break;
			case UBX_PUBX:
				switch(messageId) {
					case PUBX_POSITION: pubxPosition(); break;
					case PUBX_SVSTATUS: pubxSvStatus(); break;
					case PUBX_TIME: pubxTime(); break;
				} break;
		}
	}
}

template <class T, class D> void Gnss<T,D>::get() {
	if(ready() && error == NO_ERROR) {
		switch(messageClass) {
			case UBX_NAV:
				switch(messageId) {
				  case NAV_CLOCK: navClock(); break; 
				  case NAV_DGPS: navDgps(); break; 
				  case NAV_DOP: navDop(); break; 
				  case NAV_GEOFENCE: navGeoFence(); break; 
				  case NAV_ODO: navOdo(); break; 
				  case NAV_ORB: navOrb(); break; 
				  case NAV_POSLLH: navPosLlh(); break; 
				  case NAV_PVT: navPvt(); break; 
				  case NAV_SAT: navSat(); break; 
				  case NAV_SBAS: navSbas(); break; 
				  case NAV_SLAS: navSlas(); break; 
				  case NAV_TIMEBDS: navTimeBds(); break; 
				  case NAV_TIMEGAL: navTimeGal(); break; 
				  case NAV_TIMEGLO: navTimeGlo(); break; 
				  case NAV_TIMEGPS: navTimeGps(); break; 
				  case NAV_TIMELS: navTimeLs(); break; 
				  case NAV_TIMEUTC: navTimeUtc(); break; 
				} break;
			case UBX_CFG:
				switch(messageId) {
					case CFG_GNSS: cfgGnss(); break;
					case CFG_INF: cfgInf(); break;
					case CFG_LOGFILTER: cfgLogFilter(); break;
					case CFG_MSG: cfgMsg(); break;
					case CFG_NAV5: cfgNav(); break;
					case CFG_ODO: cfgOdo(); break;
					case CFG_PM2: cfgPm(); break;
					case CFG_PMS: cfgPms(); break;
					case CFG_PRT: cfgPrt(); break;
					case CFG_PWR: cfgPwr(); break;
					case CFG_RATE: cfgRate(); break;
					case CFG_RXM: cfgRxm(); break;
					case CFG_SBAS: cfgSbas(); break;
					case CFG_SLAS: cfgSlas(); break;
					case CFG_TP5: cfgTp(); break;
					case CFG_GEOFENCE: cfgGeoFence(); break;
					case CFG_NMEA: cfgNmea(); break;
				} break;
			case UBX_MON: 
				switch(messageId) {
					case MON_VER: monVer(MON_VER_EXTENSION_NUMBER); break;
					case MON_GNSS: monGnss(); break;
					case MON_PATCH: monPatch(); break;
				} break;
			case UBX_TIM:
				switch(messageId) {
					case TIM_TM2: timTm(); break;
					case TIM_TP: timTp(); break;
				} break;
			case UBX_LOG:
				switch(messageId) {
					case LOG_INFO: logInfo(); break;
					case LOG_RETRIEVEPOS: logRetrievePos(); break;
					case LOG_RETRIEVEPOSEXTRA: logRetrievePosExtra(); break;
					case LOG_RETRIEVESTRING: logRetrieveString(); break;
					case LOG_FINDTIME: logFindTime(); break;
				} break;
			case UBX_INF:
				char infLevel[8];
				switch (messageId) {
					case INF_DEBUG: sprintf(infLevel, String(F("debug")).c_str()); break;
					case INF_ERROR: sprintf(infLevel, String(F("error")).c_str()); break;
					case INF_NOTICE: sprintf(infLevel, String(F("notice")).c_str()); break;
					case INF_TEST: sprintf(infLevel, String(F("test")).c_str()); break;
					case INF_WARNING: sprintf(infLevel, String(F("warning")).c_str()); break;
					default: sprintf(infLevel, String(F("unknown")).c_str()); break;
				}
				infMsg(infLevel);
				break;
			case UBX_ACK:
				switch(messageId) {
					case ACK_ACK: ackAck(); break;
					case ACK_NAK: ackNak(); break;
				} break;
			case UBX_NMEA:
				switch(messageId) {
					case NMEA_RMC: nmeaRmc(); break;
					case NMEA_VTG: nmeaVtg(); break;
					case NMEA_GGA: nmeaGga(); break;
					case NMEA_GSA: nmeaGsa(); break;
					case NMEA_GSV: nmeaGsv(); break;
					case NMEA_GLL: nmeaGll(); break;
					case NMEA_GST: nmeaGst(); break;
					case NMEA_VLW: nmeaVlw(); break;
					case NMEA_GNS: nmeaGns(); break;
					case NMEA_ZDA: nmeaZda(); break;
				} break;
			case UBX_PUBX:
				switch(messageId) {
					case PUBX_POSITION: pubxPosition(); break;
					case PUBX_SVSTATUS: pubxSvStatus(); break;
					case PUBX_TIME: pubxTime(); break;
				} break;
		}
	}
}

template <class T> void Gnss<T,null>::navClock() {
}

template <class T, class D> void Gnss<T,D>::navClock() {
  NavClock * p = (NavClock *)payload;
  char s[128];
  sprintf(s, String(F("u-blox Clock: iTow %lums, bias %ldns, drift %ldns/s, tAcc %luns, fAcc %lups/s")).c_str(), p->iTOW, p->clkB, p->clkD, p->tAcc, p->fAcc);
  debug.println(s);
}

template <class T> void Gnss<T,null>::navDgps() {
}

template <class T, class D> void Gnss<T,D>::navDgps() {
  DGPSCorrData * p = NULL;
  NavDGPS * dgps = (NavDGPS *)payload;
  char s[196];
  for (uint8_t i = 0; i < dgps->numCh; i++) {
    p = (DGPSCorrData *)(payload + sizeof(NavDGPS) * i);
    if (p->flags.used) {
		sprintf(s, String(F("DGPS stationId %u, health %u, status %u: svid %u, channel %u, used %u, age %ums, prc %fm, prrc %fm/s")).c_str(), dgps->baseId, dgps->baseHealth, dgps->status, p->svid, p->flags.channel, p->flags.used, p->ageC, (double)(p->prc), (double)(p->prrc));
		debug.println(s);
	}
  }
}

template <class T> void Gnss<T,null>::navDop() {
}

template <class T, class D> void Gnss<T,D>::navDop() {
	NavDOP * p = (NavDOP *)payload;
	char s[196];
	sprintf(s, String(F("DOP iTOW %lu, gDOP %u, pDOP %u, tDOP %u, vDOP %u, hDOP %u, nDOP %u, eDOP %u 10^-2")).c_str(), p->iTOW, p->gDOP, p->pDOP, p->tDOP, p->vDOP, p->hDOP, p->nDOP, p->eDOP);
	debug.println(s);
}

template <class T> void Gnss<T,null>::navGeoFence() {
}

template <class T, class D> void Gnss<T,D>::navGeoFence() {
	NavGeofence * p = (NavGeofence *)payload;
	char s[128];
	char g[8];

	switch(p->combState) {
		case UKNOWN_GEOFENCE_STATE: sprintf(g, String(F("Unknown")).c_str()); break;
		case INSIDE_GEOFENCE_STATE: sprintf(g, String(F("Inside")).c_str()); break;
		case OUTSIDE_GEOFENCE_STATE: sprintf(g, String(F("Outside")).c_str()); break;
	}
	sprintf(s, String(F("GeoFence iTOW %lu, status %u, numFences %u, combState %s")).c_str(), p->iTOW, p->status, p->numFences, g);
	debug.println(s);
	for (uint8_t i = 0; i < p->numFences; i++) {
		switch(*(payload + sizeof(NavGeofence) + 2 * i)) {
			case UKNOWN_GEOFENCE_STATE: sprintf(g, String(F("Unknown")).c_str()); break;
			case INSIDE_GEOFENCE_STATE: sprintf(g, String(F("Inside")).c_str()); break;
			case OUTSIDE_GEOFENCE_STATE: sprintf(g, String(F("Outside")).c_str()); break;
		}
		sprintf(s, String(F("fenceId %u: state %s ")).c_str(), i, g);
		debug.print(s);
	}
	debug.println();
}

template <class T> void Gnss<T,null>::navOdo() {
}

template <class T, class D> void Gnss<T,D>::navOdo() {
  NavODO * p = (NavODO *)payload;
  char s[64];
  sprintf(s, String(F("navODO iTOW %lu, distance %lu +/- %lum, total %lum")).c_str(), p->iTOW, p->distance, p->distanceStd, p->totalDistance);
  debug.println(s);
}

template <class T> void Gnss<T,null>::navOrb() {
}

template <class T, class D> void Gnss<T,D>::navOrb() {
	NavOrb * o = (NavOrb *)payload;
	OrbData * p = NULL;
	char s[32];
	uint8_t u1, u2;
	char unknown[8] = "Unknown";
	for (uint8_t i = 0; i < o->numSv; i++) {
		p = (OrbData *)(payload + sizeof(NavOrb) + i * sizeof(OrbData));
		sprintf(s, String(F("gnssId %u svId %u")).c_str(), p->gnssId, p->svId);
		debug.print(s + String(F(", status ")));
		switch(p->svFlags.health) {
			case UNKNOWN: sprintf(s, unknown); break;
			case HEALTHY: sprintf(s, String(F("Healthy")).c_str()); break;
			case UNHEALTHY: sprintf(s, String(F("Unhealthy")).c_str()); break;
		}
		debug.print(s + String(F(", pos ")));
		switch(p->svFlags.visibility) {
			case UNKNOWN_VISIBILITY: sprintf(s, unknown); break;
			case BELOW_HIRIZON: sprintf(s, String(F("Above Horizon")).c_str()); break;
			case ABOVE_HORIZON: sprintf(s, String(F("Below Horizon")).c_str()); break;
			case ABOVE_ELEVATION_THRESHOLD: sprintf(s, String(F("Above Elevation Threshold")).c_str()); break;
		}
		debug.print(s + String(F(", ephUsability ")));
		switch(p->ephFlags.Usability) {
			case 31: sprintf(s, unknown); break;
			case 30: sprintf(s, String(F(">450min")).c_str()); break;
			case 0: sprintf(s, String(F("Unused")).c_str()); break;
			default:
				u1 = (p->ephFlags.Usability - 1) * 15;
				u2 = p->ephFlags.Usability * 15;
				sprintf(s, String(F("between %u and %u min")).c_str(), u1, u2); 
				break;
		}
		debug.print(s + String(F(", ephSource ")));
		switch(p->ephFlags.Source) {
			case 0: sprintf(s, String(F("N/A")).c_str()); break;
			case 1: sprintf(s, String(F("GNSS")).c_str()); break;
			case 2: sprintf(s, String(F("Ext Aid")).c_str()); break;
			default:
				sprintf(s, String(F("other")).c_str()); 
				break;
		}
		debug.print(s + String(F(" almUsability")));
		switch(p->almFlags.Usability) {
			case 31: sprintf(s, unknown); break;
			case 30: sprintf(s, String(F(">450days")).c_str()); break;
			case 0: sprintf(s, String(F("Unused")).c_str()); break;
			default:
				u1 = (p->almFlags.Usability - 1) * 15;
				u2 = p->almFlags.Usability * 15;
				sprintf(s, String(F("between %u and %u days")).c_str(), u1, u2); 
				break;
		}
		debug.print(s + String(F(", almSource ")));
		switch(p->almFlags.Source) {
			case 0: sprintf(s, String(F("N/A")).c_str()); break;
			case 1: sprintf(s, String(F("GNSS")).c_str()); break;
			case 2: sprintf(s, String(F("Ext Aid")).c_str()); break;
			default:
				sprintf(s, String(F("other")).c_str()); 
				break;
		}
		debug.print(s + String(F(", otherOrb ")));
		switch(p->otherFlags.Usability) {
			case 31: sprintf(s, unknown); break;
			case 30: sprintf(s, String(F(">450days")).c_str()); break;
			case 0: sprintf(s, String(F("Unused")).c_str()); break;
			default:
				u1 = (p->otherFlags.Usability - 1) * 15;
				u2 = p->otherFlags.Usability * 15;
				sprintf(s, String(F("between %u and %u days")).c_str(), u1, u2); 
				break;
		}
		debug.print(s + String(F(", almSource ")));
		switch(p->otherFlags.Source) {
			case 0: sprintf(s, String(F("N/A")).c_str()); break;
			case 1: sprintf(s, String(F("AssistNowOffline")).c_str()); break;
			case 2: sprintf(s, String(F("AssistNowAutonomous")).c_str()); break;
			default:
				sprintf(s, String(F("other")).c_str()); 
				break;
		}
		debug.print(s);
	}
	debug.println();
}

template <class T> void Gnss<T,null>::navPosLlh() {
}

template <class T, class D> void Gnss<T,D>::navPosLlh() {
	NavPosLlh * p = (NavPosLlh *)payload;
	char s[128];
	Latitude lat;
    Longitude lon;
    longLatToDMS(&lat, p->lat);
    longLonToDMS(&lon, p->lon);
	sprintf(s, String(F("PosLLH iTOW %lu, %s %s +/- %ld, alt %ld, above MSL %ld +/- %ldm")).c_str(), p->iTOW, dmsLatToStr(&lat).c_str(), dmsLonToStr(&lon).c_str(), p->hAcc / 1000, p->alt / 1000, p->altMSL / 1000, p->vAcc / 1000);
	debug.println(s);
}

template <class T> void Gnss<T,null>::navPvt() {
}

template <class T, class D> void Gnss<T,D>::navPvt() {
  NavPvt * navPVT = (NavPvt *)payload;
  if (navPVT->validTime.validDate && navPVT->validTime.validTime && navPVT->validTime.fullyResolved && pps) {
    time_t t1 = time(NULL);
    debug.println(String(F("NatPvt local time: ")) + ctime(&t1));

    pps = false;
    tm time_tm;
    utcTime = mk_gmtime(gps2tm(&(navPVT->dateTime), &time_tm));
    set_system_time(utcTime); //this time is slightly in the past but will be corrected by pps() function 
	                          //if interrupt is attached to an arduino pin where u-blox pps output is connected
							  //alternatively, use timTp() message and set_system_time(utcTime) will be done in pps()
    char ft[9];
    switch (navPVT->fixType) {
      case NONE: sprintf(ft, String(F("None")).c_str()); break;
      case DR: sprintf(ft, String(F("DR")).c_str()); break;
      case TWO_D: sprintf(ft, String(F("2D")).c_str()); break;
      case THREE_D: sprintf(ft, String(F("3D")).c_str()); break;
      case GNSS_DR: sprintf(ft, String(F("GNSS+DR")).c_str()); break;
      case TIME_ONLY: sprintf(ft, String(F("TimeOnly")).c_str()); break;
    }
    char psm[19];
    switch (navPVT->flags.psmState) {
      case NOT_ACTIVE: sprintf(psm, String(F("Not Active")).c_str()); break;
      case ENABLED: sprintf(psm, String(F("Enabled")).c_str()); break;
      case ACQUISITION: sprintf(psm, String(F("Acquisition")).c_str()); break;
      case TRACKING: sprintf(psm, String(F("Tracking")).c_str()); break;
      case POWER_OPTIMIZED_TRACKING: sprintf(psm, String(F("Optimized Tracking")).c_str()); break;
      case INACTIVE: sprintf(psm, String(F("Inactive")).c_str()); break;
    }
    char carrSoln[6];
    switch (navPVT->flags.carrSoln) {
      case NONE: sprintf(carrSoln, String(F("None")).c_str()); break;
      case FLOAT: sprintf(carrSoln, String(F("Float")).c_str()); break;
      case FIXED: sprintf(carrSoln, String(F("Fixed")).c_str()); break;
    }
    Latitude lat;
    Longitude lon;
    longLatToDMS(&lat, navPVT->lat);
    longLonToDMS(&lon, navPVT->lon);
    String flags = "FixOK", yes = "yes", no = "no";
    flags += navPVT->flags.gnssFixOk ? yes : no;
    flags += "|diffFix";
    flags += navPVT->flags.diffSoln ? yes : no;
    flags += "|psm";
    flags += navPVT->flags.psmState ? yes : no;
    flags += "|headingValid";
    flags += navPVT->flags.headingValid ? yes : no;
    flags += "|carrSoln";
    flags += navPVT->flags.carrSoln ? yes : no;
	char str[255];
	sprintf(str, String(F(" %s %s +/- %lum")).c_str(), dmsLatToStr(&lat).c_str(), dmsLonToStr(&lon).c_str(), navPVT->hAcc / 1000);
    debug.println(str);
	sprintf(str, String(F("iTOW %lu, %s +/- %luns, Fix: %s, psm %s, carrSol %s, SVs %u, alt %ldm above elipsoid, %ldm above MSL +/- %lum, pDOP %.2f, SOG %ld +/ %lum/s, heading %ld, motionHeading %ld +/- %ludeg")).c_str(), navPVT->iTOW, asctime(gmtime(&utcTime)), navPVT->tAcc, ft, psm, carrSoln, flags.c_str(), navPVT->numSV, navPVT->alt / 1000, navPVT->altMSL / 1000, navPVT->altMSL / 1000, navPVT->vAcc / 1000, (double)(navPVT->pDOP / 100), navPVT->SOG / 1000, navPVT->sAcc / 1000, navPVT->heading / 100000, navPVT->motionHeading / 100000, navPVT->headAcc / 100000);
    debug.println(str);
  }
}

template <class T> void Gnss<T,null>::navSat() {
}

template <class T, class D> void Gnss<T,D>::navSat() {
  NavSat * navSat = (NavSat *)payload;
  debug.println(String(F("navSat iTOW ")) + navSat->iTOW + String(F(", numSVs ")) + navSat->numSvs);
  for (uint8_t i = 0; i < navSat->numSvs; i++) {
    SatInfo * satInfo = (SatInfo *)((uint8_t *)navSat + sizeof(NavSat) + i * sizeof(SatInfo));
    char sq[37];
    switch(satInfo->flags.signalQuality) {
      case NO_SIGNAL: sprintf(sq, String(F("None")).c_str()); break;
      case SEARCHING_SIGNAL: sprintf(sq, String(F("Searching")).c_str()); break;
      case SIGNAL_ACQUIRED: sprintf(sq, String(F("Acquired")).c_str()); break;
      case SIGNAL_DETECTED_BUT_UNUSABLE: sprintf(sq, String(F("Detected but unusable")).c_str()); break;
      case CODE_LOCKED_TIME_SYNCED: sprintf(sq, String(F("Code locked and time synced")).c_str()); break;
      default: sprintf(sq, String(F("Code and carrier locked and time synced")).c_str()); break;
    }
    char healthy[8];
    switch(satInfo->flags.health)
    {
      case UNKNOWN: sprintf(healthy, String(F("unknown")).c_str()); break;
      case HEALTHY: sprintf(healthy, String(F("Yes")).c_str()); break;
      case UNHEALTHY: sprintf(healthy, String(F("No")).c_str()); break;
    }
    char os[22];
    switch (satInfo->flags.orbitSource)
    {
      case NONE: sprintf(os, String(F("None")).c_str()); break;
      case EPHEMERIS: sprintf(os, String(F("Ephimeris")).c_str()); break;
      case ALMANAC: sprintf(os, String(F("Almanac")).c_str()); break;
      case ASSIST_NOW_OFFLINE: sprintf(os, String(F("Assist Now Offline")).c_str()); break;
      case ASSIST_NOW_AUTONOMOUS: sprintf(os, String(F("Assist Now Autonomous")).c_str()); break;
      default: sprintf(os, String(F("Other")).c_str()); break;
    }
    char str[255];
	sprintf(str, String(F("gnssId %u, svId %u, cno %u, elev %ddeg, azim %ddeg, prRes %dm, signalQuality %s, used %u, healthy %s, diffCorr %u, smoothed %u, orbitSrc %s, ephAvail %u, almAvail %u, anoAvail %u, aopAvail %u, sbas %u, rtcm %u, slas %u, pr %u, cr %u, do %u")).c_str(), satInfo->gnssId, satInfo->svId, satInfo->cno, satInfo->elev, satInfo->azim, satInfo->prRes, sq, satInfo->flags.svUsed, healthy, satInfo->flags.diffCorr, satInfo->flags.smoothed, os, satInfo->flags.ephAvail, satInfo->flags.almAvail, satInfo->flags.anoAvail, satInfo->flags.aopAvail, satInfo->flags.sbasCorrUsed, satInfo->flags.rtcmCorrUsed, satInfo->flags.slasCorrUsed, satInfo->flags.prCorrUsed, satInfo->flags.crCorrUsed, satInfo->flags.doCorrUsed);
    debug.println(str);
  }
}

template <class T> void Gnss<T,null>::navSbas() {
}

template <class T, class D> void Gnss<T,D>::navSbas() {
	SbasSv * sv = NULL;
	NavSbas * p = (NavSbas *)payload;
	char s[255], m[9], g[8], r[40];
	switch (p->mode) {
		case 0: sprintf(m, String(F("disabled")).c_str()); break;
		case 1: sprintf(m, String(F("enabled")).c_str()); break;
		case 3: sprintf(m, String(F("testing")).c_str()); break;
	}
	switch (p->sys) {
		case -1: sprintf(g, String(F("unknown")).c_str()); break;
		case WAAS: sprintf(g, String(F("WAAS")).c_str()); break;
		case EGNOS: sprintf(g, String(F("EGNOS")).c_str()); break;
		case MSAS: sprintf(g, String(F("MSAS")).c_str()); break;
		case GAGAN: sprintf(g, String(F("GAGAN")).c_str()); break;
		case GPS_SYS: sprintf(g, String(F("GPS")).c_str()); break;
	}
	if (p->services.ranging) strcat(r, String(F("ranging")).c_str());
	if (p->services.corrections) strcat(r, String(F("|corrections")).c_str());
	if (p->services.integrity) strcat(r, String(F("|integrity")).c_str());
	if (p->services.testMode) strcat(r, String(F("|testMode")).c_str());
	sprintf(s, String(F("navSbas: iTOW %lu, geo %u, mode %s, sys %s, service %s: ")).c_str(), p->iTOW, p->geo, m, g, r);
	debug.println(s);
	for (uint8_t i = 0; i < p->cnt; i++) {
		sv = (SbasSv *)(payload + sizeof(NavSbas) + i * sizeof(SbasSv));
		switch (sv->sys) {
			case WAAS: sprintf(g, String(F("WAAS")).c_str()); break;
			case EGNOS: sprintf(g, String(F("EGNOS")).c_str()); break;
			case MSAS: sprintf(g, String(F("MSAS")).c_str()); break;
			case GAGAN: sprintf(g, String(F("GAGAN")).c_str()); break;
			case GPS_SYS: sprintf(g, String(F("GPS")).c_str()); break;
		}
		if (sv->services.ranging) strcat(r, String(F("ranging")).c_str());
		if (sv->services.corrections) strcat(r, String(F("|corrections")).c_str());
		if (sv->services.integrity) strcat(r, String(F("|integrity")).c_str());
		if (sv->services.testMode) strcat(r, String(F("|testMode")).c_str());
		sprintf(s, String(F("svId %u, flags %u, status %u, sys %s, service %s, pseudoRange %dcm, ionspherCorr %dcm")).c_str(), sv->svId, sv->flags, sv->status, g, r, sv->prc, sv->ic);
		debug.println(s);
	}
}

template <class T> void Gnss<T,null>::navSlas() {
}

template <class T, class D> void Gnss<T,D>::navSlas() {
	SlasSv * sv = NULL;
	NavSlas * p = (NavSlas *)payload;
	char s[255], f[40];
	if (p->flags.gmsAvailable)  strcat(f, String(F("gmsAvailable")).c_str());
	if (p->flags.qzssSvAvailable) strcat(f, String(F("|qzssSvAvailable")).c_str());
	if (p->flags.testMode) strcat(f, String(F("|testMode")).c_str());
	sprintf(s, String(F("navSlas: iTOW %lu, gmsLon %ld x 10^-3 deg, gmsLat %ld x 10^-3 deg, code %u, svId %u, flags %s")).c_str(), p->iTOW, p->gmsLon, p->gmsLat, p->gmsCode, p->gzssSvId, f);
	debug.println(s);
	for (uint8_t i = 0; i < p->cnt; i++) {
		sv = (SlasSv *)(payload + sizeof(NavSlas) + i * sizeof(SlasSv));
		sprintf(s, String(F("gnssId %u, svId %u, prc %ucm")).c_str(), sv->gnssId, sv->svId, sv->prc);
		debug.println(s);
	}
}

template <class T> void Gnss<T,null>::navTimeBds() {
}

template <class T, class D> void Gnss<T,D>::navTimeBds() {
	NavTimeBdsGal * p = (NavTimeBdsGal *)payload;
	char s[255], f[32];
	if (p->flags.towValid) strcat(f, String(F("towValid")).c_str());
	if (p->flags.weekValid) strcat(f, String(F("|weekValid")).c_str());
	if (p->flags.leapSecValid) strcat(f, String(F("|leapSecValid")).c_str());
	sprintf(s, String(F("timeBDS: iTOW %lums, tow %lu.%lds +/- %luns, weeks %d, leapSec %d, flags %s")).c_str(), p->iTOW, p->tow, p->fTow, p->tAcc, p->weeks, p->leapSec, f);
	debug.print(s);
}

template <class T> void Gnss<T,null>::navTimeGal() {
}

template <class T, class D> void Gnss<T,D>::navTimeGal() {
	NavTimeBdsGal * p = (NavTimeBdsGal *)payload;
	char s[255], f[32];
	if (p->flags.towValid) strcat(f, String(F("towValid")).c_str());
	if (p->flags.weekValid) strcat(f, String(F("|weekValid")).c_str());
	if (p->flags.leapSecValid) strcat(f, String(F("|leapSecValid")).c_str());
	sprintf(s, String(F("timeGal: iTOW %lums, tow %lu.%lds +/- %luns, weeks %d, leapSec %d, flags %s")).c_str(), p->iTOW, p->tow, p->fTow, p->tAcc, p->weeks, p->leapSec, f);
	debug.print(s);
}

template <class T> void Gnss<T,null>::navTimeGlo() {
}

template <class T, class D> void Gnss<T,D>::navTimeGlo() {
	NavTimeGlo * p = (NavTimeGlo *)payload;
	char s[255], f[20];
	uint16_t day = p->day, doy = day % 365;
	uint16_t year = 1992 + 4 * p->year + day / 365;
	if (p->flags.todValid) strcat(f, String(F("todValid")).c_str());
	if (p->flags.dateValid) strcat(f, String(F("|dateValid")).c_str());
	sprintf(s, String(F("timeGlo: iTOW %lums, tod %lu.%lds +/- %luns, dayOfYear %u, year %u, flags %s")).c_str(), p->iTOW, p->tod, p->fTod, p->tAcc, doy, year, f);
	debug.print(s);
}

template <class T> void Gnss<T,null>::navTimeGps() {
}

template <class T, class D> void Gnss<T,D>::navTimeGps() {
	NavTimeGps * p = (NavTimeGps *)payload;
	char s[255], f[32];
	if (p->flags.towValid) strcat(f, String(F("towValid")).c_str());
	if (p->flags.weekValid) strcat(f, String(F("|weekValid")).c_str());
	if (p->flags.leapSecValid) strcat(f, String(F("|leapSecValid")).c_str());
	sprintf(s, String(F("timeGps: iTOW %lu.%lds +/- %luns, weeks %d, leapSec %d, flags %s")).c_str(), p->iTOW / 1000, p->fTow, p->tAcc, p->weeks, p->leapSec, f);
	debug.print(s);
}

template <class T> void Gnss<T,null>::navTimeLs() {
}

template <class T, class D> void Gnss<T,D>::navTimeLs() {
	NavTimeLs * p = (NavTimeLs *)payload;
	char s[255], g[17], cs[10];
	switch (p->source) {
		case DEFAULT_LSS: sprintf(g, String(F("Default")).c_str()); break;
		case GPS_GLONASS_DIFF: sprintf(g, String(F("GPS_GLONASS_DIFF")).c_str()); break;
		case GPS_LSS: sprintf(g, String(F("GPS")).c_str()); break;
		case SBAS_LSS: sprintf(g, String(F("SBAS")).c_str()); break;
		case BEIDOU_LSS: sprintf(g, String(F("BDS")).c_str()); break;
		case GALILEO_LSS: sprintf(g, String(F("Galileo")).c_str()); break;
		case AIDED_DATA: sprintf(g, String(F("AIDED_DATA")).c_str()); break;
		case CONFIGURED_LSS: sprintf(g, String(F("CONFIGURED")).c_str()); break;
		case UNKNOWN_LSS: sprintf(g, String(F("UNKNOWN")).c_str()); break;
	}
	switch (p->changeSource) {
		case NO_SOURCE_LSS: sprintf(cs, String(F("NO_SOURCE")).c_str()); break;
		case GPS_CHANGE_LSS: sprintf(cs, String(F("GPS")).c_str()); break;
		case SBAS_CHANGE_LSS: sprintf(cs, String(F("SBAS")).c_str()); break;
		case BEIDOU_CHANGE_LSS: sprintf(cs, String(F("BDS")).c_str()); break;
		case GALILEO_CHANGE_LSS: sprintf(cs, String(F("Galileo")).c_str()); break;
		case GLONAS_CHANGE_LSS: sprintf(cs, String(F("GLONASS")).c_str()); break;
	}
	sprintf(s, String(F("LeapSecond: iTOW %lu, currLS %u, src %s, changeSrc %s, lsChange %d, timeToLs %ld, week %u, day %u")).c_str(), p->iTOW, p->leapSec, g, cs, p->lsChange, p->timeToLs, p->gpsWeek, p->gpsDay);
	debug.println(s);
}

template <class T> void Gnss<T,null>::navTimeUtc() {
}

template <class T, class D> void Gnss<T,D>::navTimeUtc() {
  NavTimeUtc * t = (NavTimeUtc *)payload;  
  if (t->valid.validTOW && t->valid.validWKN && t->valid.validUTC) {
    struct tm tm_time;
	time_t t1 = mk_gmtime(gps2tm(&(t->dateTime), &tm_time));
	char s[64], src[8];
    switch (t->valid.utcSource)
    {
      case NO_INFO: sprintf(src, String(F("NO_INFO")).c_str()); break;
      case CRL: sprintf(src, String(F("CRL")).c_str()); break;
      case NIST: sprintf(src, String(F("NIST")).c_str()); break;
      case USNO: sprintf(src, String(F("USNO")).c_str()); break;
      case BIPM: sprintf(src, String(F("BIPM")).c_str()); break;
      case EUROLAB: sprintf(src, String(F("EUROLAB")).c_str()); break;
      case SU: sprintf(src, String(F("SU")).c_str()); break;
      case NTSC: sprintf(src, String(F("NTSC")).c_str()); break;
      case UNKNOWN_SOURCE: sprintf(src, String(F("UNKNOWN")).c_str()); break;
    }
	sprintf(s, String(F("UTC: %s, source %s")).c_str(), asctime(gmtime(&t1)), src);
    debug.println(s);
  }
}

template <class T> void Gnss<T,null>::cfgGnss() {
}

template <class T, class D> void Gnss<T,D>::cfgGnss() {
  GnssConf * gnssConf = (GnssConf *)payload;
  GnssCfg * cfg = NULL;
  char s[128];
  sprintf(s, String(F("GnssConf: u-blox numTrkChHw %u, numTrkChUse %u")).c_str(), gnssConf->numTrkChHw, gnssConf->numTrkChUse);
  debug.println(s);
  char gnss_id[8], on_off[9];
  for (uint8_t i = 0; i < gnssConf->numConfigBlocks; i++) {
	cfg = (GnssCfg *)(payload + sizeof(GnssConf) + sizeof(GnssCfg) * i);
    switch (cfg->gnssId) {
      case GPS_ID: sprintf(gnss_id, String(F("GPS")).c_str()); break;
      case SBAS_ID: sprintf(gnss_id, String(F("SBAS")).c_str()); break;
      case Galileo_ID: sprintf(gnss_id, String(F("Galileo")).c_str()); break;
      case BeiDou_ID: sprintf(gnss_id, String(F("BeiDou")).c_str()); break;
      case IMES_ID: sprintf(gnss_id, String(F("IMES")).c_str()); break;
      case QZSS_ID: sprintf(gnss_id, String(F("QZSS")).c_str()); break;
      case GLONASS_ID: sprintf(gnss_id, String(F("GLONASS")).c_str()); break;
    }
    if (cfg->flags.enabled) sprintf(on_off, String(F("enabled")).c_str()); 
	else sprintf(on_off, String(F("disabled")).c_str());
	sprintf(s, String(F("GNSS id %s, minCh %u, maxCh %u: %s")).c_str(), gnss_id, cfg->minTrkCh, cfg->maxTrkCh, on_off);
    debug.println(s);
  }
}

template <class T> void Gnss<T,null>::cfgInf() {
}

template <class T, class D> void Gnss<T,D>::cfgInf() {
  InfoMsgMask * mask = NULL;
  char s[128], pr[5], po[8];
  for (uint8_t i = 0; i < 6; i++) {
	  mask = (InfoMsgMask *)&(payload[i + 4]);
      switch((uint8_t)payload[0]) {
		case UBX: sprintf(pr, String(F("UBX")).c_str()); break;
		case NMEA: sprintf(pr, String(F("NMEA")).c_str()); break;
	  }
	  switch(i) {
		case DDC: sprintf(po, String(F("DDC")).c_str()); break;
		case COM1: sprintf(po, String(F("COM1")).c_str()); break;
		case COM2: sprintf(po, String(F("COM2")).c_str()); break;
		case USB: sprintf(po, String(F("USB")).c_str()); break;
		case SPI: sprintf(po, String(F("SPI")).c_str()); break;
		case RES: sprintf(po, String(F("RES")).c_str()); break;
		default: sprintf(po, String(F("UNKNOWN")).c_str()); break;
	  }
	  sprintf(s, String(F("cfgInfoMask: %s %s INF-MSG mask: err %u, warn %u, info %u, test %u, debug %u")).c_str(), po, pr, mask->error, mask->warning, mask->notice, mask->test, mask->debug);
	  debug.println(s); 
  }
}

template <class T> void Gnss<T,null>::cfgLogFilter() {
}

template <class T, class D> void Gnss<T,D>::cfgLogFilter() {
  CfgLogFilter * filter = (CfgLogFilter *)payload;
  char s[196], flags[70];
  if (filter->flags.recordingEnabled) strcat(flags, String(F("recEnabled")).c_str());   
  if (filter->flags.psmOncePerWakeUpEnabled) strcat(flags, String(F("|psmOncePerWakeUp Enabled")).c_str());   
  if (filter->flags.applyAllFilterSettings) strcat(flags, String(F("|applyAllFilterSettings")).c_str()); 
  sprintf(s, String(F("cfgLogFilter: minInterval %us, timeThreshold %us, speedThreshold %um/s, positionThreshold %um, flags %s")).c_str(), filter->minInterval, filter->timeThreshold, filter->speedThreshold, filter->positionThreshold, flags);
  debug.println(s);  
}

template <class T> void Gnss<T,null>::cfgMsg() {
}

template <class T, class D> void Gnss<T,D>::cfgMsg() {
  char s[255], ids[9], cl[8], po[5];
  if (payloadLength == 8) {
	CfgMsg * msg = (CfgMsg *)payload;
	switch(msg->msgClass) {
	case UBX_NAV: 
		sprintf(cl, String(F("UBX-NAV")).c_str()); 
		switch(msg->msgId) {
			case NAV_CLOCK: sprintf(ids, String(F("CLOCK")).c_str()); break;
			case NAV_DGPS: sprintf(ids, String(F("DGPS")).c_str()); break;
			case NAV_DOP: sprintf(ids, String(F("DOP")).c_str()); break;
			case NAV_EOE: sprintf(ids, String(F("EOE")).c_str()); break;
			case NAV_GEOFENCE: sprintf(ids, String(F("GEOFENCE")).c_str()); break;
			case NAV_ODO: sprintf(ids, String(F("ODO")).c_str()); break;
			case NAV_ORB: sprintf(ids, String(F("ORB")).c_str()); break;
			case NAV_POSLLH: sprintf(ids, String(F("POSLLH")).c_str()); break;
			case NAV_RESETODO: sprintf(ids, String(F("RESETODO")).c_str()); break;
			case NAV_SAT: sprintf(ids, String(F("SAT")).c_str()); break;
			case NAV_SBAS: sprintf(ids, String(F("SBAS")).c_str()); break;
			case NAV_SLAS: sprintf(ids, String(F("SLAS")).c_str()); break;
			case NAV_TIMEBDS: sprintf(ids, String(F("TIMEBDS")).c_str()); break;
			case NAV_TIMEGAL: sprintf(ids, String(F("TIMEGAL")).c_str()); break;
			case NAV_TIMEGLO: sprintf(ids, String(F("TIMEGLO")).c_str()); break;
			case NAV_TIMEGPS: sprintf(ids, String(F("TIMEGPS")).c_str()); break;
			case NAV_TIMELS: sprintf(ids, String(F("TIMELS")).c_str()); break;
			case NAV_TIMEUTC: sprintf(ids, String(F("TIMEUTC")).c_str()); break;
			default: sprintf(ids, "%X", msg->msgId);
		}
		break;
	case UBX_TIM: 
		sprintf(cl, String(F("UBX-TIM")).c_str()); 
		switch(msg->msgId) {
			case TIM_TM2: sprintf(ids, String(F("TM2")).c_str()); break;
			case TIM_TP: sprintf(ids, String(F("TP")).c_str()); break;
			default: sprintf(ids, "%X", msg->msgId);
		}
		break;
	case UBX_NMEA: 
		sprintf(cl, String(F("NMEA")).c_str()); 
		switch(msg->msgId) {
			case NMEA_RMC: sprintf(ids, String(F("RMC")).c_str()); break;
			case NMEA_VTG: sprintf(ids, String(F("VTG")).c_str()); break;
			case NMEA_GGA: sprintf(ids, String(F("GGA")).c_str()); break;
			case NMEA_GSA: sprintf(ids, String(F("GSA")).c_str()); break;
			case NMEA_GSV: sprintf(ids, String(F("GSV")).c_str()); break;
			case NMEA_GLL: sprintf(ids, String(F("GLL")).c_str()); break;
			case NMEA_GST: sprintf(ids, String(F("GST")).c_str()); break;
			case NMEA_VLW: sprintf(ids, String(F("VLW")).c_str()); break;
			case NMEA_GNS: sprintf(ids, String(F("GNS")).c_str()); break;
			case NMEA_ZDA: sprintf(ids, String(F("ZDA")).c_str()); break;
			default: sprintf(ids, "%X", msg->msgId);
		}
		break;
	case UBX_PUBX: 
		sprintf(cl, String(F("PUBX")).c_str()); 
		switch(msg->msgId) {
			case PUBX_CONFIG: sprintf(ids, String(F("CONFIG")).c_str()); break;
			case PUBX_POSITION: sprintf(ids, String(F("POSITION")).c_str()); break;
			case PUBX_SVSTATUS: sprintf(ids, String(F("SVSTATUS")).c_str()); break;
			case PUBX_TIME: sprintf(ids, String(F("TIME")).c_str()); break;
			default: sprintf(ids, "%X", msg->msgId);
		}
		break;
		default: sprintf(cl, "%X", msg->msgClass);
	}
	for (uint8_t portId = 0; portId < 5; portId++) {
		switch(portId) {
		case DDC: sprintf(po, String(F("DDC")).c_str()); break;
		case COM1: sprintf(po, String(F("COM1")).c_str()); break;
		case COM2: sprintf(po, String(F("COM2")).c_str()); break;
		case USB: sprintf(po, String(F("USB")).c_str()); break;
		case SPI: sprintf(po, String(F("SPI")).c_str()); break;
		case RES: sprintf(po, String(F("RES")).c_str()); break;
		}
		sprintf(s, String(F("%s cfgMsg %s-%s: rate per navSol %u")).c_str(), po, cl, ids, msg->rate[portId]);
		debug.println(s);
	}
  } else {
	CfgMsgCOM1 * msg = (CfgMsgCOM1 *)payload;
	switch(msg->msgClass) {
	case UBX_NAV: 
		sprintf(cl, String(F("UBX-NAV")).c_str()); 
		switch(msg->msgId) {
			case NAV_CLOCK: sprintf(ids, String(F("CLOCK")).c_str()); break;
			case NAV_DGPS: sprintf(ids, String(F("DGPS")).c_str()); break;
			case NAV_DOP: sprintf(ids, String(F("DOP")).c_str()); break;
			case NAV_EOE: sprintf(ids, String(F("EOE")).c_str()); break;
			case NAV_GEOFENCE: sprintf(ids, String(F("GEOFENCE")).c_str()); break;
			case NAV_ODO: sprintf(ids, String(F("ODO")).c_str()); break;
			case NAV_ORB: sprintf(ids, String(F("ORB")).c_str()); break;
			case NAV_POSLLH: sprintf(ids, String(F("POSLLH")).c_str()); break;
			case NAV_RESETODO: sprintf(ids, String(F("RESETODO")).c_str()); break;
			case NAV_SAT: sprintf(ids, String(F("SAT")).c_str()); break;
			case NAV_SBAS: sprintf(ids, String(F("SBAS")).c_str()); break;
			case NAV_SLAS: sprintf(ids, String(F("SLAS")).c_str()); break;
			case NAV_TIMEBDS: sprintf(ids, String(F("TIMEBDS")).c_str()); break;
			case NAV_TIMEGAL: sprintf(ids, String(F("TIMEGAL")).c_str()); break;
			case NAV_TIMEGLO: sprintf(ids, String(F("TIMEGLO")).c_str()); break;
			case NAV_TIMEGPS: sprintf(ids, String(F("TIMEGPS")).c_str()); break;
			case NAV_TIMELS: sprintf(ids, String(F("TIMELS")).c_str()); break;
			case NAV_TIMEUTC: sprintf(ids, String(F("TIMEUTC")).c_str()); break;
			default: sprintf(ids, "%X", msg->msgId);
		}
		break;
	case UBX_TIM: 
		sprintf(cl, String(F("UBX-TIM")).c_str()); 
		switch(msg->msgId) {
			case TIM_TM2: sprintf(ids, String(F("TM2")).c_str()); break;
			case TIM_TP: sprintf(ids, String(F("TP")).c_str()); break;
			default: sprintf(ids, "%X", msg->msgId);
		}
		break;
	case UBX_NMEA: 
		sprintf(cl, String(F("NMEA")).c_str()); 
		switch(msg->msgId) {
			case NMEA_RMC: sprintf(ids, String(F("RMC")).c_str()); break;
			case NMEA_VTG: sprintf(ids, String(F("VTG")).c_str()); break;
			case NMEA_GGA: sprintf(ids, String(F("GGA")).c_str()); break;
			case NMEA_GSA: sprintf(ids, String(F("GSA")).c_str()); break;
			case NMEA_GSV: sprintf(ids, String(F("GSV")).c_str()); break;
			case NMEA_GLL: sprintf(ids, String(F("GLL")).c_str()); break;
			case NMEA_GST: sprintf(ids, String(F("GST")).c_str()); break;
			case NMEA_VLW: sprintf(ids, String(F("VLW")).c_str()); break;
			case NMEA_GNS: sprintf(ids, String(F("GNS")).c_str()); break;
			case NMEA_ZDA: sprintf(ids, String(F("ZDA")).c_str()); break;
			default: sprintf(ids, "%X", msg->msgId);
		}
		break;
	case UBX_PUBX: 
		sprintf(cl, String(F("PUBX")).c_str()); 
		switch(msg->msgId) {
			case PUBX_CONFIG: sprintf(ids, String(F("CONFIG")).c_str()); break;
			case PUBX_POSITION: sprintf(ids, String(F("POSITION")).c_str()); break;
			case PUBX_SVSTATUS: sprintf(ids, String(F("SVSTATUS")).c_str()); break;
			case PUBX_TIME: sprintf(ids, String(F("TIME")).c_str()); break;
			default: sprintf(ids, "%X", msg->msgId);
		}
		break;
		default: sprintf(cl, "%X", msg->msgClass);
	}
    sprintf(s, String(F("cfgMsg %s-%s: rate per navSol %u")).c_str(), cl, ids, msg->rate);
	debug.println(s);
  }
}

template <class T> void Gnss<T,null>::cfgNav() {
}

template <class T, class D> void Gnss<T,D>::cfgNav() {
  CfgNav * cfg = (CfgNav *)payload;
  char s[255], dyn[12];
  switch (cfg->dynModel) {
        case PORTABLE: sprintf(dyn, String(F("PORTABLE")).c_str()); break;
        case STATIONARY: sprintf(dyn, String(F("STATIONARY")).c_str()); break;
        case PEDESTRIAN: sprintf(dyn, String(F("PEDESTRIAN")).c_str()); break;
        case AUTOMOTIVE: sprintf(dyn, String(F("AUTOMOTIVE")).c_str()); break;
        case SEA: sprintf(dyn, String(F("SEA")).c_str()); break;
        case AIRBORNE_1G: sprintf(dyn, String(F("AIRBORNE_1G")).c_str()); break;
        case AIRBORNE_2G: sprintf(dyn, String(F("AIRBORNE_2G")).c_str()); break;
        case AIRBORNE_4G: sprintf(dyn, String(F("AIRBORNE_4G")).c_str()); break;
        case WATCH: sprintf(dyn, String(F("WATCH")).c_str()); break;
        case MODEL_ERROR: sprintf(dyn, String(F("MODEL_ERROR")).c_str()); break;
  }
  
  char utc[10];
  switch (cfg->utcStandard) {
    case AUTOMATIC: sprintf(utc, String(F("AUTOMATIC")).c_str()); break;
    case GPS: sprintf(utc, String(F("GPS")).c_str()); break;
    case GLONASS: sprintf(utc, String(F("GLONASS")).c_str()); break;
    case BEIDOU: sprintf(utc, String(F("BEIDOU")).c_str()); break;
    case UTC_ERROR: sprintf(utc, String(F("UTC_ERROR")).c_str()); break;
  }

  char fm[6];
  switch(cfg->fixMode) {
  	  case TWO_D_ONLY: sprintf(fm, String(F("2D")).c_str()); break;
	  case THREE_D_ONLY: sprintf(fm, String(F("3D")).c_str()); break;
	  case AUTO: sprintf(fm, String(F("AUTO")).c_str()); break;
	  case FIX_MODE_ERROR: sprintf(fm, String(F("ERROR")).c_str()); break;
  }

  sprintf(s, String(F("cfgNav: dynModel %s, utcStandard %s, fixMode %s, fixedAlt %ldm, variance %lum2, pDOP %u, tDOP %u, pAcc %um, tAcc %ums?")).c_str(), dyn, utc, fm, cfg->fixedAlt / 100, cfg->fixedAltVar / 10000, cfg->pDOP / 10, cfg->tDOP / 10, cfg->pAcc);
  debug.println(s);
  sprintf(s, String(F("cfgNav: cnoThreshold %udBHz, NumSVs %u, staticHoldThreshold %ucm/s, MaxDist %um, minElev %udeg above hor, dgnssTimeout %us")).c_str(), cfg->cnoThreshold, cfg->cnoThresholdNumSVs, cfg->staticHoldThreshold, cfg->staticHoldMaxDistance, cfg->minElev, cfg->dgnssTimeout);
  debug.println(s);
}

template <class T> void Gnss<T,null>::cfgOdo() {
}

template <class T, class D> void Gnss<T,D>::cfgOdo() {
  ODOCfg * cfg = (ODOCfg *)payload; //getCfgOdo();
  char s[255], flags[128], profile[9], enabled[8], disabled[9];
  sprintf(enabled, String(F("enabled")).c_str());
  sprintf(disabled, String(F("disabled")).c_str());
  sprintf(flags, String(F("ODO ")).c_str());
  if (cfg->flags.ODOenabled) strcat(flags, enabled); else strcat(flags, disabled);
  strcat(flags, String(F("|low speed COG filter ")).c_str());
  if (cfg->flags.COGenabled) strcat(flags, enabled); else strcat(flags, disabled);
  strcat(flags, String(F("|low speed (< 5m/s) filter ")).c_str());
  if (cfg->flags.outputLPvelocity) strcat(flags, enabled); else strcat(flags, disabled);
  strcat(flags, String(F("|2D headingOfMotion ")).c_str());
  if (cfg->flags.outputLPcog) strcat(flags, enabled); else strcat(flags, disabled);

  switch (cfg->odoProfile) {
    case RUNNING: sprintf(profile, String(F("RUNNING")).c_str()); break;
    case CYCLING: sprintf(profile, String(F("CYCLING")).c_str()); break;
    case SWIMMING: sprintf(profile, String(F("SWIMMING")).c_str()); break;
    case DRIVING: sprintf(profile, String(F("DRIVING")).c_str()); break;
    case CUSTOM: sprintf(profile, String(F("CUSTOM")).c_str()); break;
  }
  sprintf(s, String(F("ODOcfg - flags: %s, profile %s, COGlowSpeedFilterThresh %um/s, COGmaxAcceptablePosAccuracy %um, velLPgain %u cogLPgain %u")).c_str(), flags, profile, cfg->cogMaxSpeed, cfg->cogMaxPosAccuracy, cfg->velLPgain, cfg->cogLPgain);
  debug.println(s);

}

template <class T> void Gnss<T,null>::cfgPm() {
}

template <class T, class D> void Gnss<T,D>::cfgPm() {
	CfgPm * p = (CfgPm *)payload;
	char s[196], flags[128];
	if (p->flags.extintSel) strcat(flags, String(F("EXTINT1")).c_str()); else strcat(flags, String(F("EXTINT0")).c_str());
	if (p->flags.extintWake) strcat(flags, String(F("|Wake")).c_str());
	if (p->flags.extintBackup) strcat(flags, String(F("|Backup")).c_str());
	if (p->flags.extintInactivity) strcat(flags, String(F("|Inactivity")).c_str());
	if (p->flags.limitPeakCurr) strcat(flags, String(F("|limitPeakCurr")).c_str());
	if (p->flags.waitTimeFix) strcat(flags, String(F("|waitTimeFix")).c_str());
	if (p->flags.updateRTC) strcat(flags, String(F("|updateRTC")).c_str());
	if (p->flags.updateEph) strcat(flags, String(F("|updateEph")).c_str());
	if (p->flags.doNotEnterOff) strcat(flags, String(F("|doNotEnterOff")).c_str());
	if (p->flags.mode) strcat(flags, String(F("|PSMCT")).c_str()); else strcat(flags, String(F("|PSMOO")).c_str());
	sprintf(s, String(F("CfgPm: maxStartupStateDuration %us, updatePeriod %lums, searchPeriod %lums, gridOffset %lums, onTime %us, minAcqTime %us, extintInactivity %lums, flags %s")).c_str(), p->maxStartupStateDuration, p->updatePeriod, p->searchPeriod, p->gridOffset, p->onTime, p->minAcqTime, p->extintInactivity, flags);
	debug.println(s);
}

template <class T> void Gnss<T,null>::cfgPms() {
}

template <class T, class D> void Gnss<T,D>::cfgPms() {
  PowerMode * mode = (PowerMode *)payload;
  char s[128], pwr[9];
  switch (mode->powerMode)
  {
    case FULL_POWER_MODE: sprintf(pwr, String(F("FULL")).c_str()); break;
    case BALANCED_POWER_MODE: sprintf(pwr, String(F("BALANCED")).c_str()); break;
    case INTERVAL_POWER_MODE: sprintf(pwr, String(F("INTERVAL")).c_str()); break;
    case AGGRESSIVE_1HZ_POWER_MODE: sprintf(pwr, String(F("AGGR_1HZ")).c_str()); break;
    case AGGRESSIVE_2HZ_POWER_MODE: sprintf(pwr, String(F("AGGR_2HZ")).c_str()); break;
    case AGGRESSIVE_4HZ_POWER_MODE: sprintf(pwr, String(F("AGGR_4HZ")).c_str()); break;
    case INVALID_POWER_MODE: sprintf(pwr, String(F("INVALID")).c_str()); break;
  }
  sprintf(s, String(F("powerMode %s, period %us, onTime %us")).c_str(), pwr, mode->period, mode->onTime);
  debug.println(s);
}

template <class T> void Gnss<T,null>::cfgPrt() {
}

template <class T, class D> void Gnss<T,D>::cfgPrt() {
  CfgPrt * cfg = (CfgPrt *)payload;
  char s[128], id[5], mode[64], clen[2], par[5], sbits[4], inProto[14], outProto[9];
  switch (cfg->portId) {
	case DDC: sprintf(id, String(F("DDC")).c_str()); break;
	case COM1: sprintf(id, String(F("COM1")).c_str()); break;
	case COM2: sprintf(id, String(F("COM2")).c_str()); break;
	case USB: sprintf(id, String(F("USB")).c_str()); break;
	case SPI: sprintf(id, String(F("SPI")).c_str()); break;
	case RES: sprintf(id, String(F("RES")).c_str()); break;
  }
  switch (cfg->mode.charLen) {
     case FIVE_BIT: sprintf(clen, String(F("5")).c_str()); break;
     case SIX_BIT: sprintf(clen, String(F("6")).c_str()); break;
     case SEVEN_BIT: sprintf(clen, String(F("7")).c_str()); break;
     case EIGHT_BIT: sprintf(clen, String(F("8")).c_str()); break;
  }
  switch (cfg->mode.parity) {
    case PARITY_EVEN: sprintf(par, String(F("even")).c_str()); break;
    case PARITY_ODD: sprintf(par, String(F("odd")).c_str()); break;
    case NO_PARITY: sprintf(par, String(F("no")).c_str()); break;
    case NO_PARITY2: sprintf(par, String(F("no")).c_str()); break;
  }
  switch (cfg->mode.nStopBits) {
    case ONE_STOP_BIT: sprintf(sbits, String(F("1")).c_str()); break;
    case ONE_AND_HALF_STOP_BIT: sprintf(sbits, String(F("1.5")).c_str()); break;
    case TWO_STOP_BIT: sprintf(sbits, String(F("2")).c_str()); break;
    case HALF_STOP_BIT: sprintf(sbits, String(F("0.5")).c_str()); break;
  }
  if (cfg->inProtoMask.inUbx) sprintf(inProto, String(F("Ubx")).c_str());
  if (cfg->inProtoMask.inNmea) strcat(inProto, String(F("|Nmea")).c_str());
  if (cfg->inProtoMask.inRtcm) strcat(inProto, String(F("|Rtcm")).c_str());
  if (cfg->outProtoMask.outUbx) sprintf(outProto, String(F("Ubx")).c_str());
  if (cfg->outProtoMask.outNmea) strcat(outProto, String(F("|Nmea")).c_str());
  
  sprintf(mode, String(F("%s bit|%s parity|%s stop bit")).c_str(), clen, par, sbits);
  sprintf(s, String(F("cfgPrt: %s, rate %lu baud, mode %s, inProto %s, outProto %s")).c_str(), id, (uint32_t)(cfg->baudRate), mode, inProto, outProto);
  debug.println(s);
}

//UBX-CFG-PWR is deprecated in protocols > 17
template <class T> void Gnss<T,null>::cfgPwr() {
}
template <class T, class D> void Gnss<T,D>::cfgPwr() {
}

template <class T> void Gnss<T,null>::cfgRate() {
}

template <class T, class D> void Gnss<T,D>::cfgRate() {
  NavRate * rate = (NavRate *)payload;
  char s[128], timeRef[8];
  switch (rate->timeRef)
  {
    case UTC_TIME: sprintf(timeRef, String(F("UTC")).c_str()); break;
    case GPS_TIME: sprintf(timeRef, String(F("GPS")).c_str()); break;
    case GLONASS_TIME: sprintf(timeRef, String(F("GLONASS")).c_str()); break;
    case BEIDOU_TIME: sprintf(timeRef, String(F("BEIDOU")).c_str()); break;
    case GALILEO_TIME: sprintf(timeRef, String(F("GALILEO")).c_str()); break;
  }
  sprintf(s, String(F("cfgRate: navRate %ums, measurementsPerNavSol %u, timeRef %s")).c_str(), rate->rate, rate->navSolRate, timeRef);
  debug.println(s);
}

template <class T> void Gnss<T,null>::cfgRxm() {
}

template <class T, class D> void Gnss<T,D>::cfgRxm() {
	CfgRxm * p = (CfgRxm *)(payload);
	char s[24];
	debug.print(F("cfgRxm mode "));
	switch(p->lpMode) {
		case CONTINUOUS_POWER: sprintf(s, String(F("CONTINUOUS_POWER")).c_str()); break;
		case POWER_SAVE_MODE: sprintf(s, String(F("POWER_SAVE_MODE")).c_str()); break;
		case CONTINUOUS_MODE: sprintf(s, String(F("CONTINUOUS_MODE")).c_str()); break;
	}
	debug.println(s);
}

template <class T> void Gnss<T,null>::cfgSbas() {
}

template <class T, class D> void Gnss<T,D>::cfgSbas() {
	CfgSbas * p = (CfgSbas *)payload;
	char s[128];
	char str[64];
	debug.print(F("cfgSbas mode "));
	if (p->mode.enabled) strcat(str, String(F("enabled")).c_str()); else strcat(str, String(F("disabled")).c_str());
	if (p->mode.testMode) strcat(str, String(F("|testMode")).c_str());
	strcat(str, String(F(", usage ")).c_str());
	if (p->usage.range) strcat(str, String(F("range")).c_str());
	if (p->usage.diffCorr) strcat(str, String(F("|diffCorr")).c_str());
	if (p->usage.integrity) strcat(str, String(F("|integrity")).c_str());
	sprintf(s, String(F("%s, maxSbas %u, scanMode1 %lu, scanMode2 %u")).c_str(), str, p->maxSbas, p->scanMode1, p->scanMode2);
	debug.println(s);
}

//UBX-CFG-SLAS Not supported in protocols < 19.2
template <class T> void Gnss<T,null>::cfgSlas() {
}
template <class T, class D> void Gnss<T,D>::cfgSlas() {
}

template <class T> void Gnss<T,null>::cfgTp() {
}

template <class T, class D> void Gnss<T,D>::cfgTp() {
  TimePulse * tp = (TimePulse *)payload;
  char flags[64], fp[3], lc[6], s[255];
  sprintf(fp, String(F("us")).c_str());
  sprintf(lc, String(F("2^-32")).c_str());
  TimePulseFlags f = tp->flags;
  if (f.active) strcat(flags, String(F("active")).c_str());
  if (f.lockGnssFreq) strcat(flags, String(F("|lockGnssFreq")).c_str());
  if (f.lockedOtherSet) strcat(flags, String(F("|lockedOtherSet")).c_str());
  if (f.isFreq) { strcat(flags, String(F("|isFreq")).c_str()); sprintf(fp, String(F("Hz")).c_str()); } else strcat(flags, String(F("|isPeriod")).c_str());
  if (f.isLength) { strcat(flags, String(F("|isLength")).c_str()); sprintf(lc, String(F("us")).c_str()); } else strcat(flags, String(F("|Cycles")).c_str());
  if (f.alignToTOW) strcat(flags, String(F("|alignToTOW")).c_str());
  if (f.polarity) strcat(flags, String(F("|rising_edge")).c_str()); else strcat(flags, String(F("|falling_edge")).c_str());
  switch(f.gridUtcGnss)
  {
    case UTC_TIME: strcat(flags, String(F("|UTC_TIME")).c_str()); break;
    case GPS_TIME: strcat(flags, String(F("|GPS_TIME")).c_str()); break;
    case GLONASS_TIME: strcat(flags, String(F("|GLONASS_TIME")).c_str()); break;
    case BEIDOU_TIME: strcat(flags, String(F("|BEIDOU_TIME")).c_str()); break;
    case GALILEO_TIME: strcat(flags, String(F("|GALILEO_TIME")).c_str()); break;
  }
  sprintf(s, String(F("timePulse id %u, cableDelay %luns, rfDelay %luns, pulsePeriod %lu%s, pulsePeriodLocked %lu%s, pulseLen %lu%s, pulseLenLocked %lu%s, delay %luns, flags %s")).c_str(), tp->pulseId, tp->antCableDelay, tp->rfGroupDelay, tp->pulsePeriod, fp, tp->pulsePeriodLocked, fp, tp->pulseLen, lc, tp->pulseLenLocked, lc, tp->userConfigDelay, flags);
  debug.println(s);
}

template <class T> void Gnss<T,null>::cfgGeoFence() {
}

template <class T, class D> void Gnss<T,D>::cfgGeoFence() {
	GeoFences * p = (GeoFences *)payload;
	char s[64];
	GeoFence * f = NULL;
	Latitude lat;
    Longitude lon;

	sprintf(s, String(F("cfgGeofences confLvl %u, pioEnabled %u, pinPolarity %u, pin %u")).c_str(), p->confidenceLevel, p->pioEnabled, p->pinPolarity, p->pin);
	debug.println(s);
	for (uint8_t i = 0; i < p->numFences; i++) {
		f = (GeoFence *)(payload + sizeof(GeoFences) + i * sizeof(GeoFence));
	    longLatToDMS(&lat, f->lat);
		longLonToDMS(&lon, f->lon);
		sprintf(s, String(F("%u: %s %s radius %lum")).c_str(), i, dmsLatToStr(&lat).c_str(), dmsLonToStr(&lon).c_str(), f->radius / 100);
		debug.println(s);
	}
}

template <class T> void Gnss<T,null>::cfgNmea() {
}

template <class T, class D> void Gnss<T,D>::cfgNmea() {
	CfgNmea * p = (CfgNmea *)payload;
	char s[128], talkerId[13], gsv[8];
	String filter = "", flags = "", gnss = "";
	if (p->filter.failedFix) { sprintf(s, String(F("dispFailedFix")).c_str()); filter += s;	}
	if (p->filter.invalidFix) { sprintf(s, String(F("|dispInvalidFix")).c_str()); filter += s; }
	if (p->filter.invalidTime) { sprintf(s, String(F("|dispInvalidTime")).c_str()); filter += s; }
	if (p->filter.invalidDate) { sprintf(s, String(F("|dispInvalidDate")).c_str()); filter += s; }
	if (p->filter.invalidCog) { sprintf(s, String(F("|dispInvalidCOG")).c_str()); filter += s; }
	if (p->filter.gpsOnly) { sprintf(s, String(F("|dispGPSonly")).c_str()); filter += s; }
	if (p->flags.compat) { sprintf(s, String(F("compatMode")).c_str()); flags += s; }
	if (p->flags.consider) { sprintf(s, String(F("|consideringMode")).c_str()); flags += s; }
	if (p->flags.limit82) { sprintf(s, String(F("|82charsLimit")).c_str()); flags += s; }
	if (p->flags.highPrecision) { sprintf(s, String(F("|highPrecisionMode")).c_str()); flags += s; }
	if (p->gnssFilter.disableGps) { sprintf(s, String(F("disableGps")).c_str()); gnss += s; }
	if (p->gnssFilter.disableSbas) { sprintf(s, String(F("|disableSbas")).c_str()); gnss += s; }
	if (p->gnssFilter.disableQzss) { sprintf(s, String(F("|disableQzss")).c_str()); gnss += s; }
	if (p->gnssFilter.disableGlonass) { sprintf(s, String(F("|disableGlonass")).c_str()); gnss += s; }
	if (p->gnssFilter.disableBeidou) { sprintf(s, String(F("|disableBeidou")).c_str()); gnss += s; }
	switch (p->mainTalkerId) {
		case DEFAULT_TALKER_ID: sprintf(talkerId, String(F("default")).c_str()); break;
		case GP_TALKER_ID: sprintf(talkerId, String(F("GPS|SBAS|QZSS")).c_str()); break;
		case GL_TALKER_ID: sprintf(talkerId, String(F("GLONASS")).c_str()); break;
		case GN_TALKER_ID: sprintf(talkerId, String(F("Combo")).c_str()); break;
		case GA_TALKER_ID: sprintf(talkerId, String(F("Galileo")).c_str()); break;
		case GB_TALKER_ID: sprintf(talkerId, String(F("BeiDou")).c_str()); break;
	}
	if (p->gsvTalkerId) sprintf(gsv, String(F("default")).c_str()); else sprintf(gsv, String(F("main")).c_str());
	sprintf(s, String(F("cfgNmea nmeaVersion %x, maxSVs %u, displayNonNmeaSVs %u, filter %s, flags %s, GNSS %s, talkerId %s, gsvTalkerId %s, dbsTalkerId %s")).c_str(), p->nmeaVersion, p->maxSV, p->displayNonNmeaSVs, filter.c_str(), flags.c_str(), gnss.c_str(), talkerId, gsv, p->dbsTalkerId);
	debug.println(s);
}

template <class T> void Gnss<T,null>::monVer(uint8_t extensionsNumber) {
}

template <class T, class D> void Gnss<T,D>::monVer(uint8_t extensionsNumber) {
  MonVer * ver = (MonVer *)payload;
  debug.println(String(F("Version sw: ")) + ver->swVersion + String(F(", hw: ")) + ver->hwVersion);
  debug.println(F("Extensions: "));
  for (uint8_t i = 0; i < extensionsNumber; i++) debug.println(ver->extensions[i]);
}

template <class T> void Gnss<T,null>::monGnss() {
}

template <class T, class D> void Gnss<T,D>::monGnss() {
  GnssSupport * gnss = (GnssSupport *)payload;
  char supported[32], def[32], enabled[32];
  char s[128];
  if (gnss->supportedGnss.Gps) strcat(supported, String(F("Gps")).c_str());
  if (gnss->supportedGnss.Glonass) strcat(supported, String(F("|Glonass")).c_str());
  if (gnss->supportedGnss.BeiDou) strcat(supported, String(F("|BeiDou")).c_str());
  if (gnss->supportedGnss.Galileo) strcat(supported, String(F("|Galileo")).c_str());
  if (gnss->defaultGnss.Gps) strcat(def, String(F("Gps")).c_str());
  if (gnss->defaultGnss.Glonass) strcat(def, String(F("|Glonass")).c_str());
  if (gnss->defaultGnss.BeiDou) strcat(def, String(F("|BeiDou")).c_str());
  if (gnss->defaultGnss.Galileo) strcat(def, String(F("|Galileo")).c_str());
  if (gnss->enabledGnss.Gps) strcat(enabled, String(F("Gps")).c_str());
  if (gnss->enabledGnss.Glonass) strcat(enabled, String(F("|Glonass")).c_str());
  if (gnss->enabledGnss.BeiDou) strcat(enabled, String(F("|BeiDou")).c_str());
  if (gnss->enabledGnss.Galileo) strcat(enabled, String(F("|Galileo")).c_str());
  sprintf(s, String(F("supportedGNSS %s, defaultGnss: %s, enabledGnss: %s, simul %u")).c_str(), supported, def, enabled, gnss->simultaneous);
  debug.println(s);
}

template <class T> void Gnss<T,null>::monPatch() {
}

template <class T, class D> void Gnss<T,D>::monPatch() {
	Patch * pp = NULL; 
	MonPatches * p = (MonPatches *)(payload);
	char s[128];
	char str[32];
	debug.println(F("Patches"));
	for (uint8_t i = 0; i < p->numPatches; i++) {
		pp = (Patch *)(payload + sizeof(MonPatches) + sizeof(Patch) * i);
		if (pp->patchInfo.activated) strcat(str, String(F(": activated")).c_str()); else strcat(str, String(F(": not activated")).c_str());
		debug.print(i + str);
		switch (pp->patchInfo.location) {
			case EFUSE: strcat(str, String(F("|eFuse")).c_str()); break;
			case ROM: strcat(str, String(F("|ROM")).c_str()); break;
			case BBR: strcat(str, String(F("|BBR")).c_str()); break;
			case FILE_SYSTEM: strcat(str, String(F("|FILE_SYSTEM")).c_str()); break;
		}
		sprintf(s, String(F(", location %s, comparator %lu, address %lu, data %lu")).c_str(), str, pp->comparatorNumber, pp->patchAddress, pp->patchData);
		debug.println(s);
	}
}

template <class T> void Gnss<T,null>::timTm() {
}

template <class T, class D> void Gnss<T,D>::timTm() {
	TimTm * p = (TimTm *)payload;
	char s[255];
	char str[196];
	if (p->flags.mode) strcat(str, String(F("mode running")).c_str()); else strcat(str, String(F("mode signle")).c_str());
	if (p->flags.stopped) strcat(str, String(F("|stopped")).c_str()); else strcat(str, String(F("|armed")).c_str());
	if (p->flags.newFallingEdge) strcat(str, String(F("|newFallingEdge")).c_str());
	switch (p->flags.timeBase) {
		case 0: strcat(str, String(F("|Receiver Time Base")).c_str()); break;
		case 1: strcat(str, String(F("|GNSS Time Base")).c_str()); break;
		case 2: strcat(str, String(F("|UTC Time Base")).c_str()); break;
	}
	if (p->flags.utcAvailable) strcat(str, String(F("|utcAvailable")).c_str()); else strcat(str, String(F("|utcNotAvailable")).c_str());
	if (p->flags.timeValid) strcat(str, String(F("|timeValid")).c_str()); else strcat(str, String(F("|timeNotValid")).c_str());
	if (p->flags.newRisingEdge) strcat(str, String(F("|newRisingEdge")).c_str());
	sprintf(s, String(F("timTm: channel %u, count %u, weekNumRising %u, weekNumFalling %u, towRising %lu.%lums, towFalling %lu.%lu, accEst %luns, flags %s")).c_str(), p->channel, p->count, p->weekRising, p->weekFalling, p->towRising, p->towSubRising, p->towFalling, p->towSubFalling, p->accEst, str);
	debug.print(s);
}

template <class T> void Gnss<T,null>::timTp() {
}

template <class T, class D> void Gnss<T,D>::timTp() {
  TimeTP * t = (TimeTP *)payload;
  utcTime = t->weeks * ONE_DAY * 7 + (t->tow / 1000) - GPS_OFFSET;
  ttp = true;
  char timeBase[64];
  char timeRef[13], utcSource[15], s[128];
  if (t->timeBase.utcBase) {
    switch (t->timeRef.utcSource) {
      case NO_INFO: sprintf(utcSource, String(F("NO_SOURCE_INFO")).c_str()); break;
      case CRL: sprintf(utcSource, String(F("CRL")).c_str()); break;
      case NIST: sprintf(utcSource, String(F("NIST")).c_str()); break;
      case USNO: sprintf(utcSource, String(F("USNO")).c_str()); break;
      case BIPM: sprintf(utcSource, String(F("BIPM")).c_str()); break;
      case EUROLAB: sprintf(utcSource, String(F("EUROLAB")).c_str()); break;
      case SU: sprintf(utcSource, String(F("SU")).c_str()); break;
      case NTSC: sprintf(utcSource, String(F("NTSC")).c_str()); break;
      case UNKNOWN_SOURCE: sprintf(utcSource, String(F("UNKNOWN_SOURCE")).c_str()); break;
    }
    sprintf(timeBase, String(F("UTC: %s")).c_str(), utcSource); 
  } else { 
    switch (t->timeRef.timeRefGnss) {
      case GPS_TP_TIME: sprintf(timeRef, String(F("GPS")).c_str()); break;
      case GLONASS_TP_TIME: sprintf(timeRef, String(F("GLONASS")).c_str()); break;
      case BEIDOU_TP_TIME: sprintf(timeRef, String(F("BEIDOU")).c_str()); break;
      case UNKNOWN_TP_TIME: sprintf(timeRef, String(F("UNKNOWN_GNSS")).c_str()); break;
    }
    sprintf(timeBase, String(F("GNSS: %s")).c_str(), timeRef);
    if (t->timeBase.utc) strcat(timeBase, String(F("|UTC avail")).c_str()); 
  }
  switch (t->timeBase.raim) {
    case RAIM_INFO_NOT_AVAILABLE: strcat(timeBase, String(F("|RAIM_INFO_NOT_AVAIL")).c_str()); break;
    case RAIM_NOT_ACTIVE: strcat(timeBase, String(F("|RAIM_NOT_ACTIVE")).c_str()); break;
    case RAIM_ACTIVE: strcat(timeBase, String(F("|RAIM_ACTIVE")).c_str()); break;
  }
  sprintf(s, String(F("timeTP utcTime %s, tow %lums, subTOW %lums 2^-32, quatErr %ldps, weeks %u, timeBase %s")).c_str(), asctime(gmtime(&utcTime)), t->tow, t->subTOW, t->quantErr, t->weeks, timeBase);
  debug.println(s); 
}

template <class T> void Gnss<T,null>::logInfo() {
}

template <class T, class D> void Gnss<T,D>::logInfo() {
  LogInfo * log = (LogInfo *)payload; //getLogInfo();
  struct tm tm_time;
  char flags[32], f2[10], f3[14], ot[25], nt[25];
  time_t oldest = mk_gmtime(gps2tm(&(log->oldestDateTime), &tm_time));
  gmtime_r(&oldest, &tm_time);
  asctime_r(&tm_time, ot);
  time_t newest = mk_gmtime(gps2tm(&(log->newestDateTime), &tm_time));
  gmtime_r(&newest, &tm_time);
  asctime_r(&tm_time, nt);
  if (log->flags.enabled) sprintf(flags, String(F("enabled")).c_str()); else sprintf(flags, String(F("disabled")).c_str());
  if (log->flags.inactive) sprintf(f2, String(F("|inactive")).c_str()); else sprintf(f2, String(F("|active")).c_str());
  if (log->flags.circular) sprintf(f3, String(F("|circular")).c_str()); else sprintf(f3, String(F("|non-circular")).c_str());
  char s[255];
  sprintf(s, String(F("logInfo: fileStoreCapacity %lu bytes, maxLogSize %lu bytes, logSize %lu bytes, numRecords %lu, oldest %s, newest %s, flags %s")).c_str(), log->fileStoreCapacity, log->currentMaxLogSize, log->currentLogSize, log->entryCount, ot, nt, strcat(strcat(flags, f2), f3));
  debug.println(s);
}

template <class T> void Gnss<T,null>::logRetrievePos() {
}

template <class T, class D> void Gnss<T,D>::logRetrievePos() {
	LogRetrievePos * rec = (LogRetrievePos *)(payload);
	Latitude lat;
    Longitude lon;
    longLatToDMS(&lat, rec->lat);
    longLonToDMS(&lon, rec->lon);
	tm tm_time;
    time_t t = mk_gmtime(gps2tm(&(rec->dateTime), &tm_time));
	char s[196];
	sprintf(s, String(F("idx %lu: %s %s %s +/- %lum, alt %ldm above MSL, speed %lum/s, heading %ludeg, fixType %u, numSV %u")).c_str(), rec->index, asctime(gmtime(&t)), dmsLatToStr(&lat).c_str(), dmsLonToStr(&lon).c_str(), rec->hAcc / 1000, rec->altMSL / 1000, rec->gSpeed / 1000, rec->heading / 100000, rec->fixType, rec->numSV);
    debug.println(s);
}

template <class T> void Gnss<T,null>::logRetrievePosExtra() {
}

template <class T, class D> void Gnss<T,D>::logRetrievePosExtra() {
  tm tm_time;
  char s[64];
  LogRetrievePosExtra * odo = (LogRetrievePosExtra *)(payload); 
  time_t t = mk_gmtime(gps2tm(&(odo->dateTime), &tm_time));
  sprintf(s, String(F("idx %lu: %s distance %lu")).c_str(), odo->index, asctime(gmtime(&t)), odo->distance);
  debug.println(s);
}

template <class T> void Gnss<T,null>::logRetrieveString() {
}

template <class T, class D> void Gnss<T,D>::logRetrieveString() {
  tm tm_time;
  char s[32];
  LogRetrieveString * str = (LogRetrieveString *)(payload);
  time_t t = mk_gmtime(gps2tm(&(str->dateTime), &tm_time)); 
  char * ss = (char *)(payload + sizeof(LogRetrieveString));
  sprintf(s, String(F("idx %lu: %s %s")).c_str(), str->index, asctime(gmtime(&t)), ss);
  debug.println(s);
}

template <class T> void Gnss<T,null>::logFindTime() {
}

template <class T, class D> void Gnss<T,D>::logFindTime() {
	LogFindTimeResponse * res = (LogFindTimeResponse *)payload;
	char s[32];
	sprintf(s, String(F("logFindTime response: index %u")).c_str(), res->index);
	debug.println(s);
}

template <class T> void Gnss<T, null>::infMsg(const char * infLevel) {
}

template <class T, class D> void Gnss<T, D>::infMsg(const char * infLevel) {
	char * s = (char *)malloc(payloadLength);
	sprintf(s, String(F("%s: %s")).c_str(), infLevel, (char *)payload);
	debug.println(s);
	free(s);
}

template <class T> void Gnss<T,null>::ackAck() {
}

template <class T, class D> void Gnss<T,D>::ackAck() {

}

template <class T> void Gnss<T,null>::ackNak() {
}

template <class T, class D> void Gnss<T,D>::ackNak() {

}

//Recommended Minimum data
template <class T> void Gnss<T,null>::nmeaRmc() {
}

template <class T, class D> void Gnss<T,D>::nmeaRmc() {
    GNRMC data;
    getGNRMC(&data);
    if (data.status == 'V') return;
    tm tm_time;
    time_t t = mk_gmtime(gps2tm(&(data.dateTime), &tm_time)); 
    char s[128];
	sprintf(s, String(F("GNRMC: %s %s %s, SOG %.1fkt, COG %.fdeg, mv %.fdeg %c, fixType %c")).c_str(), asctime(gmtime(&t)), dmsLatToStr(&(data.lat)).c_str(), dmsLonToStr(&(data.lon)).c_str(), (double)(data.SOG), (double)(data.COG), (double)(data.magVar), data.magVarEW, data.fixType);
    debug.println(s);
}

//Course over ground and Ground speed
template <class T> void Gnss<T,null>::nmeaVtg() {
}

template <class T, class D> void Gnss<T,D>::nmeaVtg() {
  GNVTG data;
  getGNVTG(&data);
  char s[128];
  sprintf(s, String(F("GNVTG COG: %.fT, %.fM; SOG: %.1fkt, %.1fkm/h; fix type: %c")).c_str(), (double)(data.COGt), (double)(data.COGm), (double)(data.SOGkt), (double)(data.SOGkmh), data.fixType);
  debug.println(s);
}

//Global positioning system fix data
template <class T> void Gnss<T,null>::nmeaGga() {
}

template <class T, class D> void Gnss<T,D>::nmeaGga() {
  GNGGA data;
  getGNGGA(&data);
  if (data.fixType == NO_FIX_TYPE) return;
  char s[256];
  tm tm_time;
  time_t t = mk_gmtime(gps2tm(&(data.dateTime), &tm_time));
  sprintf(s, String(F("GNGGA %s %s %s, fix type: %u, numSVs: %u, hDOP %.2f, alt %.fm, geoidDiff %.fm, ageDC %luc, stId %u")).c_str(), asctime(gmtime(&t)), dmsLatToStr(&(data.lat)).c_str(), dmsLonToStr(&(data.lon)).c_str(), data.fixType, data.numSV, (double)(data.hDOP), (double)(data.alt), (double)(data.geoidEllipsoidDiff), data.ageDiffCorr, data.diffCorrStationId);
  debug.println(s);
}

//GNSS DOP and Active Satellites
template <class T> void Gnss<T,null>::nmeaGsa() {
}

template <class T, class D> void Gnss<T,D>::nmeaGsa() {
  GNGSA data;
  data.vDOP = 0; data.hDOP = 0; data.pDOP = 0, data.opMode = 'A', data.fixType = FIX_NOT_AVAILABLE;
  getGNGSA(&data);
  String sv = "";
  char s[128];
  if (data.fixType == FIX_NOT_AVAILABLE) return;
  for (uint8_t i = 0; i < 12; i++) {
    if (data.svId[i] > 0)
      sv += data.svId[i] + String(" ");
  }
  sprintf(s, String(F("GNGSA SV IDs: (%s) pDOP %.2f, hDOP %.2f, vDOP %.2f, mode: %c, fix type: %u")).c_str(), sv.c_str(), (double)(data.pDOP), (double)(data.hDOP), (double)(data.vDOP), data.opMode, data.fixType);
  debug.println(s);
}

//GNSS Satellites in View
template <class T> void Gnss<T,null>::nmeaGsv() {
}

template <class T, class D> void Gnss<T,D>::nmeaGsv() {
  GNGSV data;  
  getGNGSV(&data);
  String sv = "";
  char s[196];
  for (uint8_t i = 0; i < 4; i++) {
    sprintf(s, String(F("svId %u, elev %u, azim %u, cno %u | ")).c_str(), data.svAttr[i].svId, data.svAttr[i].elev, data.svAttr[i].azim, data.svAttr[i].cno);
	sv += s;
  }
  sprintf(s, String(F("GNGSV: numSV %u; GNSS %s: %s")).c_str(), data.numSV, data.gnss.c_str(), sv.c_str());
  debug.println(s);
}

//Latitude and longitude, with time of position fix and status
template <class T> void Gnss<T,null>::nmeaGll() {
}

template <class T, class D> void Gnss<T,D>::nmeaGll() {
  GNGLL data;
  getGNGLL(&data);
  if (data.fixType == 'N') return;
  if (data.status == 'V') return;
  char s[128];
  tm tm_time;
  time_t t = mk_gmtime(gps2tm(&(data.dateTime), &tm_time));
  sprintf(s, String(F("GNGLL: %s %s %s, fix type: %c")).c_str(), asctime(gmtime(&t)), dmsLatToStr(&(data.lat)).c_str(), dmsLonToStr(&(data.lon)).c_str(), data.fixType);
  debug.println(s);
}

//GNSS Pseudo Range Error Statistics
template <class T> void Gnss<T,null>::nmeaGst() {
}

template <class T, class D> void Gnss<T,D>::nmeaGst() {
  GNGST data;
  getGNGST(&data);
  char s[128];
  tm tm_time;
  time_t t = mk_gmtime(gps2tm(&(data.dateTime), &tm_time));
  sprintf(s, String(F("GNGST: %s, RangeRms %.fm, stdLat %.fm, stdLon %.fm, stdAlt %.fm")).c_str(), asctime(gmtime(&t)), (double)(data.rangeRms), (double)(data.stdLat), (double)(data.stdLon), (double)(data.stdAlt));
  debug.println(s);
}

//Dual ground/water distance
template <class T> void Gnss<T,null>::nmeaVlw() {
}

template <class T, class D> void Gnss<T,D>::nmeaVlw() {
  GNVLW data;
  getGNVLW(&data);
  char s[128];
  sprintf(s, String(F("GNVLW: TWD %.1f%c, WD %.1f%c, TGD %.1f%c, GD %.1f%c")).c_str(), (double)(data.twd), data.twdUnit, (double)(data.wd), data.wdUnit, (double)(data.tgd), data.tgdUnit, (double)(data.gd), data.gdUnit);
  debug.println(s);
}

//GNSS fix data
template <class T> void Gnss<T,null>::nmeaGns() {
}

template <class T, class D> void Gnss<T,D>::nmeaGns() {
    GNGNS data;
    getGNGNS(&data);
	tm tm_time;
    time_t t = mk_gmtime(gps2tm(&(data.dateTime), &tm_time));
    char s[128];
	sprintf(s, String(F("GNGNS: %s %s %s, atl %.fm, fixType[GPS] %c|fixType[GLO] %c, numSV %u, hDOP %.2f, geoidDiff %.fm, AgeDC %luc, stId %u")).c_str(), asctime(gmtime(&t)), dmsLatToStr(&(data.lat)).c_str(), dmsLonToStr(&(data.lon)).c_str(), (double)(data.alt), data.fixType[0], data.fixType[1], data.numSV, (double)(data.hDOP), (double)(data.geoidEllipsoidDiff), data.ageDiffCorr, data.diffCorrStationId);
    debug.println(s);
}

//Time and Date
template <class T> void Gnss<T,null>::nmeaZda() {
}

template <class T, class D> void Gnss<T,D>::nmeaZda() {
    GNZDA data;
    getGNZDA(&data);
	tm tm_time;
    time_t t = mk_gmtime(gps2tm(&(data.dateTime), &tm_time));
    char s[64];
	sprintf(s, String(F("GNZDA: %s %d:%u")).c_str(), asctime(gmtime(&t)), data.utcOffsetHours, data.utcOffsetMinutes);
    debug.println(s);
}

template <class T> void Gnss<T,null>::pubxConfig(BaudRate rate, InProtoMask inMask, OutProtoMask outMask, Port portId, bool autoBauding) {
	uint8_t x = 0;
	char s[128];
	uint16_t im = inMask.inUbx | (inMask.inNmea << 1) | (inMask.inRtcm << 2), om = outMask.outUbx | (outMask.outNmea << 1);
	uint8_t ab = autoBauding ? 1 : 0;
	sprintf(s, String(F("PUBX,41,%u,%.4u,%.4u,%lu,%u")).c_str(), (uint8_t)portId, im, om, (uint32_t)rate, ab);

	for (uint8_t i = 0; i < strlen(s); i++ ) {
		x ^= s[i];
	}
	serial.print("$");
	serial.print(s);
	serial.print("*");
	serial.println(x, HEX);
}

template <class T, class D> void Gnss<T,D>::pubxConfig(BaudRate rate, InProtoMask inMask, OutProtoMask outMask, Port portId, bool autoBauding) {
	uint8_t x = 0;
	char s[128];
	uint16_t im = inMask.inUbx | (inMask.inNmea << 1) | (inMask.inRtcm << 2), om = outMask.outUbx | (outMask.outNmea << 1);
	uint8_t ab = autoBauding ? 1 : 0;
	sprintf(s, String(F("PUBX,41,%u,%.4u,%.4u,%lu,%u")).c_str(), (uint8_t)portId, im, om, (uint32_t)rate, ab);

	for (uint8_t i = 0; i < strlen(s); i++ ) {
		x ^= s[i];
	}

	/*debug.print("$");
	debug.print(s);
	debug.print("*");
	debug.println(x, HEX);*/
	serial.print("$");
	serial.print(s);
	serial.print("*");
	serial.println(x, HEX);
}

template <class T> void Gnss<T,null>::pubxPosition() {
}

template <class T, class D> void Gnss<T,D>::pubxPosition() {
    PubxPosition data;
    getPubxPosition(&data);
	tm tm_time;
    time_t t = mk_gmtime(gps2tm(&(data.dateTime), &tm_time));
    char s[256];
	sprintf(s, String(F("pubxPosition: %s %s %s +/- %.fm, alt %.f +/- %.fm above ellipsoid, SOG %.1fkm/h, COG %u, vVel %.2fm/s, hDOP %.2f, vDOP %.2f, tDOP %.2f, ageDC %luc, numSV %u, fixType %s")).c_str(), asctime(gmtime(&t)), dmsLatToStr(&(data.lat)).c_str(), dmsLonToStr(&(data.lon)).c_str(), (double)(data.hAcc), (double)(data.alt), (double)(data.vAcc), (double)(data.sog), data.cog, (double)(data.vVel), (double)(data.hDOP), (double)(data.vDOP), (double)(data.tDOP), data.ageDiffCorr, data.numSV, data.fixType);
    debug.println(s);
}

template <class T> void Gnss<T,null>::pubxRate(char * msgId, uint8_t rateCom1, uint8_t rateCom2, uint8_t rateDDC, uint8_t rateUsb, uint8_t rateSpi) {
	uint8_t x = 0;
	char s[128];
	sprintf(s, String(F("PUBX,40,%s,%u,%u,%u,%u,%u,%u")).c_str(), msgId, rateDDC, rateCom1, rateCom2, rateUsb, rateSpi, 0);

	for (uint8_t i = 0; i < strlen(s); i++ ) {
		x ^= s[i];
	}

	serial.print("$");
	serial.print(s);
	serial.print("*");
	serial.println(x, HEX);
}

template <class T, class D> void Gnss<T,D>::pubxRate(char * msgId, uint8_t rateCom1, uint8_t rateCom2, uint8_t rateDDC, uint8_t rateUsb, uint8_t rateSpi) {
	uint8_t x = 0;
	char s[128];
	sprintf(s, String(F("PUBX,40,%s,%u,%u,%u,%u,%u,%u")).c_str(), msgId, rateDDC, rateCom1, rateCom2, rateUsb, rateSpi, 0);

	for (uint8_t i = 0; i < strlen(s); i++ ) {
		x ^= s[i];
	}

	/*debug.print("$");
	debug.print(s);
	debug.print("*");
	debug.println(x, HEX);*/
	serial.print("$");
	serial.print(s);
	serial.print("*");
	serial.println(x, HEX);
}

template <class T> void Gnss<T,null>::pubxSvStatus() {
}

template <class T, class D> void Gnss<T,D>::pubxSvStatus() {
  PubxSvStatus data;
  String arr[3];
  split(arr, 3, nmeaPayload);
  uint8_t numSV = arr[2].toInt();
  char s[128];
  memset(s, 0, 128);
  String status[255];
  split(status, 255, nmeaPayload);
  for (uint8_t i = 0; i < numSV; i++) {
	data.svId = 0; data.svStatus = '-'; data.azim = 0; data.elev = 0; data.cno = 0; data.lockTime = 0;
	for (uint8_t j = i * 6 + 3; j < i * 6 + 9; j++) {
		switch (j) {
			case 3: data.svId = status[j].toInt(); break;
			case 4: data.svStatus = status[j].charAt(0); break;
			case 5: data.azim = status[j].toInt(); break;
			case 6: data.elev = status[j].toInt(); break;
			case 7: data.cno = status[j].toInt(); break;
			case 8: data.lockTime = status[j].toInt(); break;
		}		
	}
	sprintf(s, String(F("PubxSvStatus: SvId %u, used: %c, azim: %u, elev %u, cno %u, lockTime %us")).c_str(), data.svId, data.svStatus, data.azim, data.elev, data.cno, data.lockTime);
	debug.println(s);
  }
}

template <class T> void Gnss<T,null>::pubxTime() {
}

template <class T, class D> void Gnss<T,D>::pubxTime() {
  PubxTime data;
  getPubxTime(&data);
  char s[64];
  tm tm_time;
  time_t t = mk_gmtime(gps2tm(&(data.dateTime), &tm_time));
  sprintf(s, String(F("PubxTime %s, utcTow: %lus, utcWeek: %lu, leapSec %u, leapSecSrc %c, clkBias %luns, clkDrift %luns/s, tpGran %luns")).c_str(), asctime(gmtime(&t)), data.utcTow, data.utcWeek, data.leapSec, data.leapSecSrc, data.clkBias, data.clkDrift, data.tpGran);
  debug.println(s);
}

template <class T> void Gnss<T,null>::poll(uint8_t msgClass, uint8_t msgId, uint16_t payload_length, uint8_t * pload) {
	  pollMessageClass = msgClass;
	  pollMessageId = msgId;
	  pollPayload = pload;
	  pollPayloadLength = payload_length;
	  calculateChecksum(pollMessageClass, pollMessageId, pollPayloadLength, pollPayload);
  	  serial.write(SYNC_CHARS[0]);
  	  serial.write(SYNC_CHARS[1]);
  	  serial.write(pollMessageClass);
  	  serial.write(pollMessageId);
	  serial.write(uint8_t(pollPayloadLength & 0xFF));
	  serial.write(uint8_t(pollPayloadLength >> 8));
	  if (pollPayload != NULL)
	  	  for (uint16_t i = 0; i < pollPayloadLength; i++) serial.write(pollPayload[i]);
	  serial.write(checksum[0]);
	  serial.write(checksum[1]);	  
}

template <class T, class D> void Gnss<T,D>::poll(uint8_t msgClass, uint8_t msgId, uint16_t payload_length, uint8_t * pload) {
	  pollMessageClass = msgClass;
	  pollMessageId = msgId;
	  pollPayload = pload;
	  pollPayloadLength = payload_length;
	  calculateChecksum(pollMessageClass, pollMessageId, pollPayloadLength, pollPayload);
  	  serial.write(SYNC_CHARS[0]);
  	  serial.write(SYNC_CHARS[1]);
  	  serial.write(pollMessageClass);
  	  //debug.print("Sending... msgCls "); debug.print(pollMessageClass, HEX);
  	  serial.write(pollMessageId);
  	  //debug.print(", msgID "); debug.print(pollMessageId, HEX);
	  serial.write(uint8_t(pollPayloadLength & 0xFF));
	  //debug.print(" len0 "); debug.print(uint8_t(payload_length & 0xFF), HEX);
	  serial.write(uint8_t(pollPayloadLength >> 8));
	  //debug.print(" len1 "); debug.print(uint8_t(payload_length >> 8), HEX);
	  //debug.print(", len "); debug.print(pollPayloadLength);

	  if (pollPayload != NULL)
	  {
		  //debug.print(", payload: ");
	  	  for (uint16_t i = 0; i < pollPayloadLength; i++)
		  {
			  serial.write(pollPayload[i]);
			  //debug.print(" "); debug.print(pollPayload[i], HEX);
		  }
	  }
	  //debug.println();
	  serial.write(checksum[0]);
	  //debug.print(" cksum0 "); debug.print(checksum[0], HEX);
	  serial.write(checksum[1]);	  
	  //debug.print(" cksum1 "); debug.println(checksum[1], HEX);
}

template <class T> bool Gnss<T,null>::pollNoError(uint32_t timeout) {
	uint32_t starttime = millis();
	while ((millis() - starttime) < timeout) {
		if (ready() && messageClass == pollMessageClass && messageId == pollMessageId) {	  
			switch (error) {
	  			case NO_ERROR: return true;
				case CHECKSUM_ERROR: poll(pollMessageClass, pollMessageId, pollPayloadLength, pollPayload); break;
				case OUT_OF_MEMORY: return false;
			}
		}
	}
	return false;
}

template <class T, class D> bool Gnss<T,D>::pollNoError(uint32_t timeout) {
	uint32_t starttime = millis();
	while ((millis() - starttime) < timeout) {
		if (ready() && messageClass == pollMessageClass && messageId == pollMessageId) {	  
			switch (error) {
	  			case NO_ERROR: return true;
				case CHECKSUM_ERROR: poll(pollMessageClass, pollMessageId, pollPayloadLength, pollPayload); debug.println("chksum err"); break;
				case OUT_OF_MEMORY: debug.println("out of memory"); return false;
			}
		}
	}
	debug.println("poll timeout");
	return false;
}

template <class T> bool Gnss<T,null>::ackNoError(uint32_t timeout) {
  uint32_t starttime = millis();
  starttime = millis();
  while ((millis() - starttime) < timeout) {
	if (ready() && messageClass == UBX_ACK) {
	  switch (error) {
	  	case NO_ERROR: if (messageId == ACK_ACK) {
			UbxAck * ack  = (UbxAck *)(payload);
			if (ack->classId == pollMessageClass && ack->messageId == pollMessageId) return true; else break;
		} else return false; //ACK_NAK
		case CHECKSUM_ERROR: poll(pollMessageClass, pollMessageId, pollPayloadLength, pollPayload); break;
		case OUT_OF_MEMORY: return false;
	  }
	}
  }
  return false;
}

template <class T, class D> bool Gnss<T,D>::ackNoError(uint32_t timeout) {
  uint32_t starttime = millis();
  starttime = millis();
  while ((millis() - starttime) < timeout) {
	if (ready() && messageClass == UBX_ACK) {
	  switch (error) {
	  	case NO_ERROR: if (messageId == ACK_ACK) {
			UbxAck * ack  = (UbxAck *)(payload);
			if (ack->classId == pollMessageClass && ack->messageId == pollMessageId) return true;
			else break;
		} else return false; //ACK_NAK
		case CHECKSUM_ERROR: poll(pollMessageClass, pollMessageId, pollPayloadLength, pollPayload); debug.println("chksum err"); break;
		case OUT_OF_MEMORY: debug.println("out of memory"); return false;
	  }
	}
  }
  debug.println("ack timeout");
  return false;
}


//UBX messages
template <class T> MonVer * Gnss<T,null>::getVersion(uint32_t timeout) {
  	poll(UBX_MON, MON_VER);
	if (pollNoError(timeout)) return (MonVer *)(payload);
	return NULL;
}

template <class T, class D> MonVer * Gnss<T,D>::getVersion(uint32_t timeout) {
  	poll(UBX_MON, MON_VER);
	if (pollNoError(timeout)) return (MonVer *)(payload);
	return NULL;
}

template <class T> MonPatches * Gnss<T,null>::getPatches(uint32_t timeout) {
  	poll(UBX_MON, MON_PATCH);
	if (pollNoError(timeout)) return (MonPatches *)(payload);
	return NULL;	
}

template <class T, class D> MonPatches * Gnss<T,D>::getPatches(uint32_t timeout) {
  	poll(UBX_MON, MON_PATCH);
	if (pollNoError(timeout)) return (MonPatches *)(payload);
	return NULL;	
}

template <class T> CfgNav * Gnss<T,null>::getCfgNav(uint32_t timeout) {
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) return (CfgNav *)(payload);
  return NULL;
}

template <class T, class D> CfgNav * Gnss<T,D>::getCfgNav(uint32_t timeout) {
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) return (CfgNav *)(payload);
  return NULL;
}

template <class T> DynModel Gnss<T,null>::getDynamicModel(uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	  cfgNav = (CfgNav *)(payload);
	  return cfgNav->dynModel;
  }
  return MODEL_ERROR;
}

template <class T, class D> DynModel Gnss<T,D>::getDynamicModel(uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	  cfgNav = (CfgNav *)(payload);
	  return cfgNav->dynModel;
  }
  return MODEL_ERROR;
}

template <class T> bool Gnss<T,null>::setDynamicModel(DynModel model, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  if (cfgNav->dynModel != model) {
	  cfgNav->dynModel = model;
	  cfgNav->mask.dyn = 1;
	  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setDynamicModel(DynModel model, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  if (cfgNav->dynModel != model) {
	  cfgNav->dynModel = model;
	  cfgNav->mask.dyn = 1;
	  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T> UtcStandard Gnss<T,null>::getUtcStandard(uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	  cfgNav = (CfgNav *)(payload);
	  return cfgNav->utcStandard;
  }
  return UTC_ERROR;
}

template <class T, class D> UtcStandard Gnss<T,D>::getUtcStandard(uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	  cfgNav = (CfgNav *)(payload);
	  return cfgNav->utcStandard;
  }
  return UTC_ERROR;
}

template <class T> bool Gnss<T,null>::setUtcStandard(UtcStandard standard, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  if (cfgNav->utcStandard != standard) {
	  cfgNav->utcStandard = standard;
	  cfgNav->mask.utcMask = 1;
	  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setUtcStandard(UtcStandard standard, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  if (cfgNav->utcStandard != standard) {
	  cfgNav->utcStandard = standard;
	  cfgNav->mask.utcMask = 1;
	  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T> FixMode Gnss<T,null>::getFixMode(uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	return cfgNav->fixMode;
  }
  return FIX_MODE_ERROR;
}

template <class T, class D> FixMode Gnss<T,D>::getFixMode(uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	return cfgNav->fixMode;
  }
  return FIX_MODE_ERROR;
}

template <class T> bool Gnss<T,null>::setFixMode(FixMode fixMode, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  if (cfgNav->fixMode != fixMode) {
	  cfgNav->fixMode = fixMode;
	  cfgNav->mask.posFixMode = 1;
	  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setFixMode(FixMode fixMode, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  if (cfgNav->fixMode != fixMode) {
	  cfgNav->fixMode = fixMode;
	  cfgNav->mask.posFixMode = 1;
	  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T> FixedAlt * Gnss<T,null>::getFixedAlt(FixedAlt * fixedAlt, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	fixedAlt->fixedAlt = cfgNav->fixedAlt;
	fixedAlt->fixedAltVar = cfgNav->fixedAltVar;
	return fixedAlt;
  }
  return NULL;
}

template <class T, class D> FixedAlt * Gnss<T,D>::getFixedAlt(FixedAlt * fixedAlt, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	fixedAlt->fixedAlt = cfgNav->fixedAlt;
	fixedAlt->fixedAltVar = cfgNav->fixedAltVar;
	return fixedAlt;
  }
  return NULL;
}

template <class T> bool Gnss<T,null>::setFixedAlt(int32_t fixedAlt, uint32_t fixedAltVar, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  if (cfgNav->fixedAlt != fixedAlt || cfgNav->fixedAltVar != fixedAltVar) {
	  cfgNav->fixedAlt = fixedAlt;
	  cfgNav->fixedAltVar = fixedAltVar;
	  cfgNav->mask.posMask = 1;
	  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setFixedAlt(int32_t fixedAlt, uint32_t fixedAltVar, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  if (cfgNav->fixedAlt != fixedAlt || cfgNav->fixedAltVar != fixedAltVar) {
	  cfgNav->fixedAlt = fixedAlt;
	  cfgNav->fixedAltVar = fixedAltVar;
	  cfgNav->mask.posMask = 1;
	  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T> DOP * Gnss<T,null>::getDop(DOP * dop, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	dop->pDOP = cfgNav->pDOP;
	dop->tDOP = cfgNav->tDOP;
	return dop;
  }
  return NULL;
}

template <class T, class D> DOP * Gnss<T,D>::getDop(DOP * dop, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	dop->pDOP = cfgNav->pDOP;
	dop->tDOP = cfgNav->tDOP;
	return dop;
  }
  return NULL;
}

template <class T> bool Gnss<T,null>::setDop(uint16_t posDOP, uint16_t timeDOP, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  cfgNav->pDOP = posDOP;
  cfgNav->tDOP = timeDOP;
  cfgNav->mask.posMask = 1;
  cfgNav->mask.timeMask = 1;
  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setDop(uint16_t posDOP, uint16_t timeDOP, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  cfgNav->pDOP = posDOP;
  cfgNav->tDOP = timeDOP;
  cfgNav->mask.posMask = 1;
  cfgNav->mask.timeMask = 1;
  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T> Accuracy * Gnss<T,null>::getAccuracy(Accuracy * acc, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	acc->pAcc = cfgNav->pAcc;
	acc->tAcc = cfgNav->tAcc;
	return acc;
  }
  return NULL;
}

template <class T, class D> Accuracy * Gnss<T,D>::getAccuracy(Accuracy * acc, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	acc->pAcc = cfgNav->pAcc;
	acc->tAcc = cfgNav->tAcc;
	return acc;
  }
  return NULL;
}

template <class T> bool Gnss<T,null>::setAccuracy(uint16_t posAccuracy, uint16_t timeAccuracy, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  cfgNav->pAcc = posAccuracy;
  cfgNav->tAcc = timeAccuracy;
  cfgNav->mask.posMask = 1;
  cfgNav->mask.timeMask = 1;
  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setAccuracy(uint16_t posAccuracy, uint16_t timeAccuracy, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  cfgNav->pAcc = posAccuracy;
  cfgNav->tAcc = timeAccuracy;
  cfgNav->mask.posMask = 1;
  cfgNav->mask.timeMask = 1;
  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T> CnoThreshold * Gnss<T,null>::getCnoThreshold(CnoThreshold * cno, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	cno->cnoThreshold = cfgNav->cnoThreshold;
	cno->cnoThresholdNumSVs = cfgNav->cnoThresholdNumSVs;
	return cno;
  }
  return NULL;
}

template <class T, class D> CnoThreshold * Gnss<T,D>::getCnoThreshold(CnoThreshold * cno, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	cno->cnoThreshold = cfgNav->cnoThreshold;
	cno->cnoThresholdNumSVs = cfgNav->cnoThresholdNumSVs;
	return cno;
  }
  return NULL;
}

template <class T> bool Gnss<T,null>::setCnoThreshold(uint8_t cnoThreshold, uint8_t cnoThresholdNumSVs, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  if (cfgNav->cnoThreshold != cnoThreshold || cfgNav->cnoThresholdNumSVs != cnoThresholdNumSVs) {
	  cfgNav->cnoThreshold = cnoThreshold;
	  cfgNav->cnoThresholdNumSVs = cnoThresholdNumSVs;
	  cfgNav->mask.cnoThreshold = 1;
	  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setCnoThreshold(uint8_t cnoThreshold, uint8_t cnoThresholdNumSVs, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  if (cfgNav->cnoThreshold != cnoThreshold || cfgNav->cnoThresholdNumSVs != cnoThresholdNumSVs) {
	  cfgNav->cnoThreshold = cnoThreshold;
	  cfgNav->cnoThresholdNumSVs = cnoThresholdNumSVs;
	  cfgNav->mask.cnoThreshold = 1;
	  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T> StaticHoldThresholds * Gnss<T,null>::getStaticHoldThresholds(StaticHoldThresholds * thresholds, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	thresholds->staticHoldThreshold = cfgNav->staticHoldThreshold;
	thresholds->staticHoldMaxDistance = cfgNav->staticHoldMaxDistance;
	return thresholds;
  }
  return NULL;
}

template <class T, class D> StaticHoldThresholds * Gnss<T,D>::getStaticHoldThresholds(StaticHoldThresholds * thresholds, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	thresholds->staticHoldThreshold = cfgNav->staticHoldThreshold;
	thresholds->staticHoldMaxDistance = cfgNav->staticHoldMaxDistance;
	return thresholds;
  }
  return NULL;
}

template <class T> bool Gnss<T,null>::setStaticHoldThresholds(uint8_t staticHoldThreshold, uint8_t staticHoldMaxDistance, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  if (cfgNav->staticHoldThreshold != staticHoldThreshold || cfgNav->staticHoldMaxDistance != staticHoldMaxDistance) {
	  cfgNav->staticHoldThreshold = staticHoldThreshold;
	  cfgNav->staticHoldMaxDistance = staticHoldMaxDistance;
	  cfgNav->mask.staticHoldMask = 1;
	  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setStaticHoldThresholds(uint8_t staticHoldThreshold, uint8_t staticHoldMaxDistance, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  if (cfgNav->staticHoldThreshold != staticHoldThreshold || cfgNav->staticHoldMaxDistance != staticHoldMaxDistance) {
	  cfgNav->staticHoldThreshold = staticHoldThreshold;
	  cfgNav->staticHoldMaxDistance = staticHoldMaxDistance;
	  cfgNav->mask.staticHoldMask = 1;
	  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T> int8_t Gnss<T,null>::getMinElev(int8_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	return cfgNav->minElev;
  }
  return 0;
}

template <class T, class D> int8_t Gnss<T,D>::getMinElev(int8_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	return cfgNav->minElev;
  }
  return 0;
}

template <class T> bool Gnss<T,null>::setMinElev(int8_t minElev, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  if (cfgNav->minElev != minElev) {
	  cfgNav->minElev = minElev;
	  cfgNav->mask.minElev = 1;
	  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setMinElev(int8_t minElev, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  if (cfgNav->minElev != minElev) {
	  cfgNav->minElev = minElev;
	  cfgNav->mask.minElev = 1;
	  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T> int8_t Gnss<T,null>::getDgnssTimeout(int8_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	return cfgNav->dgnssTimeout;
  }
  return 0;
}

template <class T, class D> int8_t Gnss<T,D>::getDgnssTimeout(int8_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	return cfgNav->dgnssTimeout;
  }
  return 0;
}

template <class T> bool Gnss<T,null>::setDgnssTimeout(int8_t dgnssTimeout, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  if (cfgNav->dgnssTimeout != dgnssTimeout) {
	  cfgNav->dgnssTimeout = dgnssTimeout;
	  cfgNav->mask.dgpsMask = 1;
	  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setDgnssTimeout(int8_t dgnssTimeout, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
 
  if (cfgNav->dgnssTimeout != dgnssTimeout) {
	  cfgNav->dgnssTimeout = dgnssTimeout;
	  cfgNav->mask.dgpsMask = 1;
	  poll(UBX_CFG, CFG_NAV5, payloadLength, (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T> InfoMsgMask * Gnss<T,null>::getCfgInf(Protocol protocolId, Port portId, uint32_t timeout) {
  poll(UBX_CFG, CFG_INF, sizeof(uint8_t), (uint8_t *)&protocolId);
  if (pollNoError(timeout))	return (InfoMsgMask *)&(payload[portId + 4]);
  return NULL;
}

template <class T, class D> InfoMsgMask * Gnss<T,D>::getCfgInf(Protocol protocolId, Port portId, uint32_t timeout) {
  poll(UBX_CFG, CFG_INF, sizeof(uint8_t), (uint8_t *)&protocolId);
  if (pollNoError(timeout))	return (InfoMsgMask *)&(payload[portId + 4]);
  return NULL;
}

template <class T> bool Gnss<T,null>::setCfgInf(InfoMsgMask mask, Protocol protocolId, Port portId, uint32_t timeout) {
  CfgInfo inf;
  inf.protocolId = protocolId;
  inf.reserved = 0;
  inf.reserved2 = 0;
  for (uint8_t port = DDC; port <= RES; port++) {
	inf.mask[port].error = 0;
	inf.mask[port].warning = 0;
	inf.mask[port].notice = 0;
	inf.mask[port].test = 0;
	inf.mask[port].debug = 0;
  }
  inf.mask[portId] = mask;
  poll(UBX_CFG, CFG_INF, sizeof(CfgInfo), (uint8_t *)&inf);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setCfgInf(InfoMsgMask mask, Protocol protocolId, Port portId, uint32_t timeout) {
  CfgInfo inf;
  inf.protocolId = protocolId;
  inf.reserved = 0;
  inf.reserved2 = 0;
  for (uint8_t port = DDC; port <= RES; port++) {
	inf.mask[port].error = 0;
	inf.mask[port].warning = 0;
	inf.mask[port].notice = 0;
	inf.mask[port].test = 0;
	inf.mask[port].debug = 0;
  }
  inf.mask[portId] = mask;
  poll(UBX_CFG, CFG_INF, sizeof(CfgInfo), (uint8_t *)&inf);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T> String Gnss<T,null>::getErrMsg(DebugLevel debugLevel, uint32_t timeout) {
  poll(UBX_INF, debugLevel);
  if (pollNoError(timeout))	return (String)payload;
  return NULL;
}

template <class T, class D> String Gnss<T,D>::getErrMsg(DebugLevel debugLevel, uint32_t timeout) {
  poll(UBX_INF, debugLevel);
  if (pollNoError(timeout))	return (String)payload;
  return NULL;
}

template <class T> CfgPrt * Gnss<T,null>::getCfgPrt(Port portId, uint32_t timeout) {
  poll(UBX_CFG, CFG_PRT, sizeof(uint8_t), (uint8_t *)&portId);
  if (pollNoError(timeout))	return (CfgPrt *)payload;
  return NULL;
}

template <class T, class D> CfgPrt * Gnss<T,D>::getCfgPrt(Port portId, uint32_t timeout) {
  poll(UBX_CFG, CFG_PRT, sizeof(uint8_t), (uint8_t *)&portId);
  if (pollNoError(timeout))	return (CfgPrt *)payload;
  return NULL;
}

template <class T> PowerMode * Gnss<T,null>::getCfgPms(uint32_t timeout) {
  poll(UBX_CFG, CFG_PMS);
  if (pollNoError(timeout))	return (PowerMode *)payload;
  return NULL;
}

template <class T, class D> PowerMode * Gnss<T,D>::getCfgPms(uint32_t timeout) {
  poll(UBX_CFG, CFG_PMS);
  if (pollNoError(timeout))	return (PowerMode *)payload;
  return NULL;
}

template <class T> bool Gnss<T,null>::setCfgPms(PowerModes mode, uint8_t period, uint8_t onTime, uint32_t timeout) {
  PowerMode * pMode = NULL;
  poll(UBX_CFG, CFG_PMS);
  if (pollNoError(timeout)) pMode = (PowerMode *)(payload);
 
  if (pMode->powerMode != mode || pMode->period != period || pMode->onTime != onTime) {
	  if (period > 0 && period <= 5) return false;
	  if ((period > 5 || onTime > 0) && mode != INTERVAL_POWER_MODE) return false;
	  pMode->powerMode = mode;
	  pMode->period = period;
	  pMode->onTime = onTime;
	  poll(UBX_CFG, CFG_PMS, payloadLength, (uint8_t *)pMode);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setCfgPms(PowerModes mode, uint8_t period, uint8_t onTime, uint32_t timeout) {
  PowerMode * pMode = NULL;
  poll(UBX_CFG, CFG_PMS);
  if (pollNoError(timeout)) pMode = (PowerMode *)(payload);
 
  if (pMode->powerMode != mode || pMode->period != period || pMode->onTime != onTime) {
	  if (period > 0 && period <= 5) return false;
	  if ((period > 5 || onTime > 0) && mode != INTERVAL_POWER_MODE) return false;
	  pMode->powerMode = mode;
	  pMode->period = period;
	  pMode->onTime = onTime;
	  poll(UBX_CFG, CFG_PMS, payloadLength, (uint8_t *)pMode);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T> CfgPm * Gnss<T,null>::getCfgPm(uint32_t timeout) {
  poll(UBX_CFG, CFG_PM2);
  if (pollNoError(timeout))	return (CfgPm *)payload;
  return NULL;
}

template <class T, class D> CfgPm * Gnss<T,D>::getCfgPm(uint32_t timeout) {
  poll(UBX_CFG, CFG_PM2);
  if (pollNoError(timeout))	return (CfgPm *)payload;
  return NULL;
}

template <class T> bool Gnss<T,null>::setCfgPm(uint8_t maxStartupStateDuration, uint32_t udpatePeriod, uint32_t searchPeriod, uint32_t gridOffset, uint16_t onTime, uint16_t minAcqTime, uint32_t extintInactivity, CfgPmFlags flags, uint32_t timeout) {
  CfgPm * pMode = NULL;
  poll(UBX_CFG, CFG_PM2);
  if (pollNoError(timeout)) pMode = (CfgPm *)(payload);
  pMode->maxStartupStateDuration = maxStartupStateDuration;
  pMode->updatePeriod = udpatePeriod;
  pMode->searchPeriod = searchPeriod;
  pMode->gridOffset = gridOffset;
  pMode->onTime = onTime;
  pMode->minAcqTime = minAcqTime;
  pMode->extintInactivity = extintInactivity;
  pMode->flags = flags;
  poll(UBX_CFG, CFG_PM2, payloadLength, (uint8_t *)pMode);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setCfgPm(uint8_t maxStartupStateDuration, uint32_t udpatePeriod, uint32_t searchPeriod, uint32_t gridOffset, uint16_t onTime, uint16_t minAcqTime, uint32_t extintInactivity, CfgPmFlags flags, uint32_t timeout) {
  CfgPm * pMode = NULL;
  poll(UBX_CFG, CFG_PM2);
  if (pollNoError(timeout)) pMode = (CfgPm *)(payload);
  pMode->maxStartupStateDuration = maxStartupStateDuration;
  pMode->updatePeriod = udpatePeriod;
  pMode->searchPeriod = searchPeriod;
  pMode->gridOffset = gridOffset;
  pMode->onTime = onTime;
  pMode->minAcqTime = minAcqTime;
  pMode->extintInactivity = extintInactivity;
  pMode->flags = flags;
  poll(UBX_CFG, CFG_PM2, payloadLength, (uint8_t *)pMode);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T> CfgRxm * Gnss<T,null>::getCfgRxm(uint32_t timeout) {
  poll(UBX_CFG, CFG_RXM);
  if (pollNoError(timeout))	return (CfgRxm *)payload;
  return NULL;
}

template <class T, class D> CfgRxm * Gnss<T,D>::getCfgRxm(uint32_t timeout) {
  poll(UBX_CFG, CFG_RXM);
  if (pollNoError(timeout))	return (CfgRxm *)payload;
  return NULL;
}

template <class T> bool Gnss<T,null>::setCfgRxm(CfgRxmLpMode mode, int32_t timeout) {
  CfgRxm * pMode = NULL;
  poll(UBX_CFG, CFG_RXM);
  if (pollNoError(timeout)) pMode = (CfgRxm *)(payload);
  if (pMode->lpMode != mode) {
	pMode->lpMode = mode;
	poll(UBX_CFG, CFG_RXM, payloadLength, (uint8_t *)pMode);
	if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setCfgRxm(CfgRxmLpMode mode, int32_t timeout) {
  CfgRxm * pMode = NULL;
  poll(UBX_CFG, CFG_RXM);
  if (pollNoError(timeout)) pMode = (CfgRxm *)(payload);
  if (pMode->lpMode != mode) {
	pMode->lpMode = mode;
	poll(UBX_CFG, CFG_RXM, payloadLength, (uint8_t *)pMode);
	if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T> bool Gnss<T,null>::setCfgPrt(Port portId, BaudRate rate, PrtMode mode, InProtoMask inMask, OutProtoMask outMask, bool extendedTxTimeout, uint32_t timeout) {
  CfgPrt * prtCfg = NULL;
  poll(UBX_CFG, CFG_PRT, sizeof(uint8_t), (uint8_t *)&portId);
  if (pollNoError(timeout)) prtCfg = (CfgPrt *)(payload);
  prtCfg->baudRate = rate;
  prtCfg->mode.charLen = mode.charLen;
  prtCfg->mode.parity = mode.parity;
  prtCfg->mode.nStopBits = mode.nStopBits;
  prtCfg->inProtoMask.inUbx = inMask.inUbx;
  prtCfg->inProtoMask.inNmea = inMask.inNmea;
  prtCfg->inProtoMask.inRtcm = inMask.inRtcm;
  prtCfg->outProtoMask.outUbx = outMask.outUbx;
  prtCfg->outProtoMask.outNmea = outMask.outNmea;
  prtCfg->flags.extendedTimeout = extendedTxTimeout ? 1 : 0;
  poll(UBX_CFG, CFG_PRT, payloadLength, (uint8_t *)prtCfg);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setCfgPrt(Port portId, BaudRate rate, PrtMode mode, InProtoMask inMask, OutProtoMask outMask, bool extendedTxTimeout, uint32_t timeout) {
  CfgPrt * prtCfg = NULL;
  poll(UBX_CFG, CFG_PRT, sizeof(uint8_t), (uint8_t *)&portId);
  if (pollNoError(timeout)) prtCfg = (CfgPrt *)(payload);
  prtCfg->baudRate = rate;
  prtCfg->mode.charLen = mode.charLen;
  prtCfg->mode.parity = mode.parity;
  prtCfg->mode.nStopBits = mode.nStopBits;
  prtCfg->inProtoMask.inUbx = inMask.inUbx;
  prtCfg->inProtoMask.inNmea = inMask.inNmea;
  prtCfg->inProtoMask.inRtcm = inMask.inRtcm;
  prtCfg->outProtoMask.outUbx = outMask.outUbx;
  prtCfg->outProtoMask.outNmea = outMask.outNmea;
  prtCfg->flags.extendedTimeout = extendedTxTimeout ? 1 : 0;
  poll(UBX_CFG, CFG_PRT, payloadLength, (uint8_t *)prtCfg);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T> void Gnss<T,null>::config(ConfMask mask, ConfigType type) {
  ConfigAllDevs conf;
  switch (type)
  {
  	  case DEFAULT_CONFIG: conf.clearMask = mask; memset(&(conf.saveMask), 0, sizeof(ConfMask)); memset(&(conf.loadMask), 0, sizeof(ConfMask)); break;
  	  case SAVE_CONFIG: memset(&(conf.clearMask), 0, sizeof(ConfMask)); conf.saveMask = mask; memset(&(conf.loadMask), 0, sizeof(ConfMask)); break;
  	  case LOAD_CONFIG: memset(&(conf.clearMask), 0, sizeof(ConfMask)); memset(&(conf.saveMask), 0, sizeof(ConfMask)); conf.loadMask = mask; break;
  }
  poll(UBX_CFG, CFG_CFG, sizeof(ConfigAllDevs), (uint8_t *)&conf);
}

template <class T, class D> void Gnss<T,D>::config(ConfMask mask, ConfigType type) {
  ConfigAllDevs conf;
  switch (type)
  {
  	  case DEFAULT_CONFIG: conf.clearMask = mask; memset(&(conf.saveMask), 0, sizeof(ConfMask)); memset(&(conf.loadMask), 0, sizeof(ConfMask)); break;
  	  case SAVE_CONFIG: memset(&(conf.clearMask), 0, sizeof(ConfMask)); conf.saveMask = mask; memset(&(conf.loadMask), 0, sizeof(ConfMask)); break;
  	  case LOAD_CONFIG: memset(&(conf.clearMask), 0, sizeof(ConfMask)); memset(&(conf.saveMask), 0, sizeof(ConfMask)); conf.loadMask = mask; break;
  }
  poll(UBX_CFG, CFG_CFG, sizeof(ConfigAllDevs), (uint8_t *)&conf);
}

template <class T> void Gnss<T,null>::config(ConfMask mask, ConfigType type, Device dev) {
  ConfigDev conf;
  switch (type)
  {
  	  case DEFAULT_CONFIG: conf.clearMask = mask; memset(&(conf.saveMask), 0, sizeof(ConfMask)); memset(&(conf.loadMask), 0, sizeof(ConfMask)); break;
  	  case SAVE_CONFIG: memset(&(conf.clearMask), 0, sizeof(ConfMask)); conf.saveMask = mask; memset(&(conf.loadMask), 0, sizeof(ConfMask)); break;
  	  case LOAD_CONFIG: memset(&(conf.clearMask), 0, sizeof(ConfMask)); memset(&(conf.saveMask), 0, sizeof(ConfMask)); conf.loadMask = mask; break;
  }
  conf.device = dev;
  poll(UBX_CFG, CFG_CFG, sizeof(ConfigDev), (uint8_t *)&conf);
}

template <class T, class D> void Gnss<T,D>::config(ConfMask mask, ConfigType type, Device dev) {
  ConfigDev conf;
  switch (type)
  {
  	  case DEFAULT_CONFIG: conf.clearMask = mask; memset(&(conf.saveMask), 0, sizeof(ConfMask)); memset(&(conf.loadMask), 0, sizeof(ConfMask)); break;
  	  case SAVE_CONFIG: memset(&(conf.clearMask), 0, sizeof(ConfMask)); conf.saveMask = mask; memset(&(conf.loadMask), 0, sizeof(ConfMask)); break;
  	  case LOAD_CONFIG: memset(&(conf.clearMask), 0, sizeof(ConfMask)); memset(&(conf.saveMask), 0, sizeof(ConfMask)); conf.loadMask = mask; break;
  }
  conf.device = dev;
  poll(UBX_CFG, CFG_CFG, sizeof(ConfigDev), (uint8_t *)&conf);
}

template <class T> void Gnss<T,null>::factoryReset(bool ioPort, bool msgConf, bool infMsg, bool navConf, bool rxmConf, bool senConf, bool rinvConf, bool antConf, bool logConf, bool ftsConf) {
  ConfMask conf;
  conf.ioPort = ioPort ? 1 : 0;
  conf.msgConf = msgConf ? 1 : 0;
  conf.infMsg = infMsg ? 1 : 0;
  conf.navConf = navConf ? 1 : 0;
  conf.rxmConf = rxmConf ? 1 : 0;
  conf.reserved = 0;
  conf.senConf = senConf ? 1 : 0;
  conf.rinvConf = rinvConf ? 1 : 0;
  conf.antConf = antConf ? 1 : 0;
  conf.logConf = logConf ? 1 : 0;
  conf.ftsConf = ftsConf ? 1 : 0;
  conf.reserved2 = 0;
  config(conf, DEFAULT_CONFIG);
}

template <class T, class D> void Gnss<T,D>::factoryReset(bool ioPort, bool msgConf, bool infMsg, bool navConf, bool rxmConf, bool senConf, bool rinvConf, bool antConf, bool logConf, bool ftsConf) {
  ConfMask conf;
  conf.ioPort = ioPort ? 1 : 0;
  conf.msgConf = msgConf ? 1 : 0;
  conf.infMsg = infMsg ? 1 : 0;
  conf.navConf = navConf ? 1 : 0;
  conf.rxmConf = rxmConf ? 1 : 0;
  conf.reserved = 0;
  conf.senConf = senConf ? 1 : 0;
  conf.rinvConf = rinvConf ? 1 : 0;
  conf.antConf = antConf ? 1 : 0;
  conf.logConf = logConf ? 1 : 0;
  conf.ftsConf = ftsConf ? 1 : 0;
  conf.reserved2 = 0;
  config(conf, DEFAULT_CONFIG);
}

template <class T> void Gnss<T,null>::saveConfig(bool ioPort, bool msgConf, bool infMsg, bool navConf, bool rxmConf, bool senConf, bool rinvConf, bool antConf, bool logConf, bool ftsConf) {
  ConfMask conf;
  conf.ioPort = ioPort ? 1 : 0;
  conf.msgConf = msgConf ? 1 : 0;
  conf.infMsg = infMsg ? 1 : 0;
  conf.navConf = navConf ? 1 : 0;
  conf.rxmConf = rxmConf ? 1 : 0;
  conf.reserved = 0;
  conf.senConf = senConf ? 1 : 0;
  conf.rinvConf = rinvConf ? 1 : 0;
  conf.antConf = antConf ? 1 : 0;
  conf.logConf = logConf ? 1 : 0;
  conf.ftsConf = ftsConf ? 1 : 0;
  conf.reserved2 = 0;
  config(conf, SAVE_CONFIG);
}

template <class T, class D> void Gnss<T,D>::saveConfig(bool ioPort, bool msgConf, bool infMsg, bool navConf, bool rxmConf, bool senConf, bool rinvConf, bool antConf, bool logConf, bool ftsConf) {
  ConfMask conf;
  conf.ioPort = ioPort ? 1 : 0;
  conf.msgConf = msgConf ? 1 : 0;
  conf.infMsg = infMsg ? 1 : 0;
  conf.navConf = navConf ? 1 : 0;
  conf.rxmConf = rxmConf ? 1 : 0;
  conf.reserved = 0;
  conf.senConf = senConf ? 1 : 0;
  conf.rinvConf = rinvConf ? 1 : 0;
  conf.antConf = antConf ? 1 : 0;
  conf.logConf = logConf ? 1 : 0;
  conf.ftsConf = ftsConf ? 1 : 0;
  conf.reserved2 = 0;
  config(conf, SAVE_CONFIG);
}

template <class T> void Gnss<T,null>::loadConfig(bool ioPort, bool msgConf, bool infMsg, bool navConf, bool rxmConf, bool senConf, bool rinvConf, bool antConf, bool logConf, bool ftsConf) {
  ConfMask conf;
  conf.ioPort = ioPort ? 1 : 0;
  conf.msgConf = msgConf ? 1 : 0;
  conf.infMsg = infMsg ? 1 : 0;
  conf.navConf = navConf ? 1 : 0;
  conf.rxmConf = rxmConf ? 1 : 0;
  conf.reserved = 0;
  conf.senConf = senConf ? 1 : 0;
  conf.rinvConf = rinvConf ? 1 : 0;
  conf.antConf = antConf ? 1 : 0;
  conf.logConf = logConf ? 1 : 0;
  conf.ftsConf = ftsConf ? 1 : 0;
  conf.reserved2 = 0;
  config(conf, LOAD_CONFIG);
}

template <class T, class D> void Gnss<T,D>::loadConfig(bool ioPort, bool msgConf, bool infMsg, bool navConf, bool rxmConf, bool senConf, bool rinvConf, bool antConf, bool logConf, bool ftsConf) {
  ConfMask conf;
  conf.ioPort = ioPort ? 1 : 0;
  conf.msgConf = msgConf ? 1 : 0;
  conf.infMsg = infMsg ? 1 : 0;
  conf.navConf = navConf ? 1 : 0;
  conf.rxmConf = rxmConf ? 1 : 0;
  conf.reserved = 0;
  conf.senConf = senConf ? 1 : 0;
  conf.rinvConf = rinvConf ? 1 : 0;
  conf.antConf = antConf ? 1 : 0;
  conf.logConf = logConf ? 1 : 0;
  conf.ftsConf = ftsConf ? 1 : 0;
  conf.reserved2 = 0;
  config(conf, LOAD_CONFIG);
}

template <class T> CfgMsg * Gnss<T,null>::getCfgMsg(uint8_t msgClass, uint8_t msgId, uint32_t timeout) {
  uint16_t pload = ((uint16_t)(msgId) << 8) | msgClass;
  poll(UBX_CFG, CFG_MSG, sizeof(uint16_t), (uint8_t *)&pload);
  if (pollNoError(timeout))	return (CfgMsg *)payload;
  return NULL;
}

template <class T, class D> CfgMsg * Gnss<T,D>::getCfgMsg(uint8_t msgClass, uint8_t msgId, uint32_t timeout) {
  uint16_t pload = ((uint16_t)(msgId) << 8) | msgClass;
  poll(UBX_CFG, CFG_MSG, sizeof(uint16_t), (uint8_t *)&pload);
  if (pollNoError(timeout))	return (CfgMsg *)payload;
  return NULL;
}

template <class T> bool Gnss<T,null>::setCfgMsg(uint8_t msgClass, uint8_t msgId, uint8_t rate, uint32_t timeout) {
  CfgMsg * cfgMsg = NULL;
  uint16_t pload = ((uint16_t)(msgId) << 8) | msgClass;
  poll(UBX_CFG, CFG_MSG, sizeof(uint16_t), (uint8_t *)&pload);
  if (pollNoError(timeout))	cfgMsg = (CfgMsg *)payload;
  
  if (cfgMsg->rate[COM1] != rate) {
	  CfgMsgCOM1 cfgMsg2;
	  cfgMsg2.msgClass = msgClass;
	  cfgMsg2.msgId = msgId;
	  cfgMsg2.rate = rate;
	  poll(UBX_CFG, CFG_MSG, sizeof(CfgMsgCOM1), (uint8_t *)&cfgMsg2);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setCfgMsg(uint8_t msgClass, uint8_t msgId, uint8_t rate, uint32_t timeout) {
  CfgMsg * cfgMsg = NULL;
  uint16_t pload = ((uint16_t)(msgId) << 8) | msgClass;
  poll(UBX_CFG, CFG_MSG, sizeof(uint16_t), (uint8_t *)&pload);
  if (pollNoError(timeout))	cfgMsg = (CfgMsg *)payload;
  
  if (cfgMsg->rate[COM1] != rate) {
	  CfgMsgCOM1 cfgMsg2;
	  cfgMsg2.msgClass = msgClass;
	  cfgMsg2.msgId = msgId;
	  cfgMsg2.rate = rate;
	  poll(UBX_CFG, CFG_MSG, sizeof(CfgMsgCOM1), (uint8_t *)&cfgMsg2);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

template <class T> ODOCfg * Gnss<T,null>::getCfgOdo(uint32_t timeout) {
  poll(UBX_CFG, CFG_ODO);
  if (pollNoError(timeout))	return (ODOCfg *)payload;
  return NULL;
}

template <class T, class D> ODOCfg * Gnss<T,D>::getCfgOdo(uint32_t timeout) {
  poll(UBX_CFG, CFG_ODO);
  if (pollNoError(timeout))	return (ODOCfg *)payload;
  return NULL;
}

template <class T> bool Gnss<T,null>::setCfgOdo(OdoFlags flags, OdoProfile profile, uint8_t maxSpeed, uint8_t maxPosAccuracy, uint8_t velGain, uint8_t COGgain, uint32_t timeout) {
  ODOCfg * odoCfg = NULL;
  poll(UBX_CFG, CFG_ODO);
  if (pollNoError(timeout)) odoCfg = (ODOCfg *)(payload);
  odoCfg->flags.ODOenabled = flags.ODOenabled;
  odoCfg->flags.COGenabled = flags.COGenabled;
  odoCfg->flags.outputLPvelocity = flags.outputLPvelocity;
  odoCfg->flags.outputLPcog = flags.outputLPcog; 
  odoCfg->odoProfile = profile; 
  odoCfg->cogMaxSpeed = maxSpeed; 
  odoCfg->cogMaxPosAccuracy = maxPosAccuracy;
  odoCfg->velLPgain = velGain; odoCfg->cogLPgain = COGgain;
  poll(UBX_CFG, CFG_ODO, payloadLength, (uint8_t *)odoCfg);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setCfgOdo(OdoFlags flags, OdoProfile profile, uint8_t maxSpeed, uint8_t maxPosAccuracy, uint8_t velGain, uint8_t COGgain, uint32_t timeout) {
  ODOCfg * odoCfg = NULL;
  poll(UBX_CFG, CFG_ODO);
  if (pollNoError(timeout)) odoCfg = (ODOCfg *)(payload);
  odoCfg->flags.ODOenabled = flags.ODOenabled;
  odoCfg->flags.COGenabled = flags.COGenabled;
  odoCfg->flags.outputLPvelocity = flags.outputLPvelocity;
  odoCfg->flags.outputLPcog = flags.outputLPcog; 
  odoCfg->odoProfile = profile; 
  odoCfg->cogMaxSpeed = maxSpeed; 
  odoCfg->cogMaxPosAccuracy = maxPosAccuracy;
  odoCfg->velLPgain = velGain; odoCfg->cogLPgain = COGgain;
  poll(UBX_CFG, CFG_ODO, payloadLength, (uint8_t *)odoCfg);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T> CfgSbas *  Gnss<T,null>::getCfgSbas(uint32_t timeout) {
  poll(UBX_CFG, CFG_SBAS);
  if (pollNoError(timeout))	return (CfgSbas *)payload;
  return NULL;
}

template <class T, class D> CfgSbas *  Gnss<T,D>::getCfgSbas(uint32_t timeout) {
  poll(UBX_CFG, CFG_SBAS);
  if (pollNoError(timeout))	return (CfgSbas *)payload;
  return NULL;
}

template <class T> bool Gnss<T,null>::setCfgSbas(CfgSbasUsage usage, uint32_t scanMode1, uint8_t scanMode2, uint32_t timeout) {
  CfgSbas * cfg = NULL;
  poll(UBX_CFG, CFG_SBAS);
  if (pollNoError(timeout)) cfg = (CfgSbas *)(payload);
  cfg->usage = usage;
  cfg->scanMode1 = scanMode1;
  cfg->scanMode2 = scanMode2;
  poll(UBX_CFG, CFG_SBAS, payloadLength, (uint8_t *)cfg);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setCfgSbas(CfgSbasUsage usage, uint32_t scanMode1, uint8_t scanMode2, uint32_t timeout) {
  CfgSbas * cfg = NULL;
  poll(UBX_CFG, CFG_SBAS);
  if (pollNoError(timeout)) cfg = (CfgSbas *)(payload);
  cfg->usage = usage;
  cfg->scanMode1 = scanMode1;
  cfg->scanMode2 = scanMode2;
  poll(UBX_CFG, CFG_SBAS, payloadLength, (uint8_t *)cfg);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T> CfgNmea * Gnss<T,null>::getCfgNmea(uint32_t timeout) {
  poll(UBX_CFG, CFG_NMEA);
  if (pollNoError(timeout))	return (CfgNmea *)payload;
  return NULL;
}

template <class T, class D> CfgNmea * Gnss<T,D>::getCfgNmea(uint32_t timeout) {
  poll(UBX_CFG, CFG_NMEA);
  if (pollNoError(timeout))	return (CfgNmea *)payload;
  return NULL;
}

template <class T> bool Gnss<T,null>::setCfgNmea(CfgNmeaFilter filter, uint8_t nmeaVersion, uint8_t maxSVs, CfgNmeaFlags flags, CfgNmeaGnss gnssFilter, bool displayNonNmeaSVs,	CfgNmeaTalkerId mainTalkerId, bool gsvTalkerIdIsMain, char * dbsTalkerId, uint32_t timeout) {
  CfgNmea * cfg = NULL;
  poll(UBX_CFG, CFG_NMEA);
  if (pollNoError(timeout)) cfg = (CfgNmea *)(payload);
  cfg->filter = filter;
  cfg->nmeaVersion = nmeaVersion;
  cfg->flags = flags;
  cfg->gnssFilter = gnssFilter;
  cfg->displayNonNmeaSVs = displayNonNmeaSVs ? 1 : 0;
  cfg->mainTalkerId = mainTalkerId;
  cfg->gsvTalkerId = gsvTalkerIdIsMain ? 1 : 0;
  cfg->dbsTalkerId[0] = dbsTalkerId[0];
  cfg->dbsTalkerId[1] = dbsTalkerId[1];
  poll(UBX_CFG, CFG_SBAS, payloadLength, (uint8_t *)cfg);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setCfgNmea(CfgNmeaFilter filter, uint8_t nmeaVersion, uint8_t maxSVs, CfgNmeaFlags flags, CfgNmeaGnss gnssFilter, bool displayNonNmeaSVs, CfgNmeaTalkerId mainTalkerId, bool gsvTalkerIdIsMain, char * dbsTalkerId, uint32_t timeout) {
  CfgNmea * cfg = NULL;
  poll(UBX_CFG, CFG_NMEA);
  if (pollNoError(timeout)) cfg = (CfgNmea *)(payload);
  cfg->filter = filter;
  cfg->nmeaVersion = nmeaVersion;
  cfg->flags = flags;
  cfg->gnssFilter = gnssFilter;
  cfg->displayNonNmeaSVs = displayNonNmeaSVs ? 1 : 0;
  cfg->mainTalkerId = mainTalkerId;
  cfg->gsvTalkerId = gsvTalkerIdIsMain ? 1 : 0;
  cfg->dbsTalkerId[0] = dbsTalkerId[0];
  cfg->dbsTalkerId[1] = dbsTalkerId[1];
  poll(UBX_CFG, CFG_SBAS, payloadLength, (uint8_t *)cfg);
  if (ackNoError(timeout)) return true;
  return false;
}


//LOG messages

template <class T> CfgLogFilter * Gnss<T,null>::getCfgLogFilter(uint32_t timeout) {
  poll(UBX_CFG, CFG_LOGFILTER);
  if (pollNoError(timeout))	return (CfgLogFilter *)payload;
  return NULL;
}

template <class T, class D> CfgLogFilter * Gnss<T,D>::getCfgLogFilter(uint32_t timeout) {
  poll(UBX_CFG, CFG_LOGFILTER);
  if (pollNoError(timeout))	return (CfgLogFilter *)payload;
  return NULL;
}

template <class T> bool Gnss<T,null>::setCfgLogFilter(uint8_t minInterval, uint8_t timeThreshold, uint8_t speedThreshold, uint8_t positionThreshold, CfgLogFilterFlags flags, uint32_t timeout) {
  CfgLogFilter * cfgLogFilter = NULL;
  poll(UBX_CFG, CFG_LOGFILTER);
  if (pollNoError(timeout))	cfgLogFilter = (CfgLogFilter *)payload;
  cfgLogFilter->minInterval = minInterval;
  cfgLogFilter->timeThreshold = timeThreshold;
  cfgLogFilter->speedThreshold = speedThreshold;
  cfgLogFilter->positionThreshold = positionThreshold;
  cfgLogFilter->flags = flags;
  poll(UBX_CFG, CFG_LOGFILTER, payloadLength, (uint8_t *)cfgLogFilter);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setCfgLogFilter(uint8_t minInterval, uint8_t timeThreshold, uint8_t speedThreshold, uint8_t positionThreshold, CfgLogFilterFlags flags, uint32_t timeout) {
  CfgLogFilter * cfgLogFilter = NULL;
  poll(UBX_CFG, CFG_LOGFILTER);
  if (pollNoError(timeout))	cfgLogFilter = (CfgLogFilter *)payload;
  cfgLogFilter->minInterval = minInterval;
  cfgLogFilter->timeThreshold = timeThreshold;
  cfgLogFilter->speedThreshold = speedThreshold;
  cfgLogFilter->positionThreshold = positionThreshold;
  cfgLogFilter->flags = flags;
  poll(UBX_CFG, CFG_LOGFILTER, payloadLength, (uint8_t *)cfgLogFilter);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T> LogInfo * Gnss<T,null>::getLogInfo(uint32_t timeout) {
  poll(UBX_LOG, LOG_INFO);
  if (pollNoError(timeout))	return (LogInfo *)payload;
  return NULL;
}

template <class T, class D> LogInfo * Gnss<T,D>::getLogInfo(uint32_t timeout) {
  poll(UBX_LOG, LOG_INFO);
  if (pollNoError(timeout))	return (LogInfo *)payload;
  return NULL;
}

template <class T> bool Gnss<T,null>::createLog(LogSize logSize, uint32_t userDefinedSize, bool circular, uint32_t timeout) {
  LogCreate logCreate;
  logCreate.version = 0;
  logCreate.circular = circular ? 1 : 0;
  logCreate.reserved = 0;
  logCreate.logSize = logSize;
  logCreate.userDefinedSize = userDefinedSize;
  poll(UBX_LOG, LOG_CREATE, sizeof(LogCreate), (uint8_t *)&logCreate);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::createLog(LogSize logSize, uint32_t userDefinedSize, bool circular, uint32_t timeout) {
  LogCreate logCreate;
  logCreate.version = 0;
  logCreate.circular = circular ? 1 : 0;
  logCreate.reserved = 0;
  logCreate.logSize = logSize;
  logCreate.userDefinedSize = userDefinedSize;
  poll(UBX_LOG, LOG_CREATE, sizeof(LogCreate), (uint8_t *)&logCreate);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T> bool Gnss<T,null>::eraseLog(uint32_t timeout) {
  poll(UBX_LOG, LOG_ERASE);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::eraseLog(uint32_t timeout) {
  poll(UBX_LOG, LOG_ERASE);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T> uint32_t Gnss<T,null>::logFind(DateTime dateTime, uint32_t timeout) {
	LogFindTimeRequest req;
	LogFindTimeResponse * res = NULL;
	req.version = 0;
	req.type = 0;
	req.reserved = 0;
	req.dateTime = dateTime;
	req.reserved2 = 0;
	poll(UBX_LOG, LOG_FINDTIME, sizeof(LogFindTimeRequest), (uint8_t *)&req);
	if (pollNoError(timeout)) {
		res = (LogFindTimeResponse *)payload;
		return res->index;
    } else return 0xFFFFFFFF;
}

template <class T, class D> uint32_t Gnss<T,D>::logFind(DateTime dateTime, uint32_t timeout) {
	LogFindTimeRequest req;
	LogFindTimeResponse * res = NULL;
	req.version = 0;
	req.type = 0;
	req.reserved = 0;
	req.dateTime = dateTime;
	req.reserved2 = 0;
	poll(UBX_LOG, LOG_FINDTIME, sizeof(LogFindTimeRequest), (uint8_t *)&req);
	if (pollNoError(timeout)) {
		res = (LogFindTimeResponse *)payload;
		return res->index;
    } else return 0xFFFFFFFF;
}

template <class T> bool Gnss<T,null>::logMsg(char * msg, uint8_t len, uint32_t timeout) {
  poll(UBX_LOG, LOG_STRING, len, (uint8_t *)msg);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::logMsg(char * msg, uint8_t len, uint32_t timeout) {
  poll(UBX_LOG, LOG_STRING, len, (uint8_t *)msg);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T> bool  Gnss<T,null>::logRetrieve(uint32_t index, uint32_t count) {
  struct {
	uint32_t index;
	uint32_t count; //256 max!
	uint8_t version;
	uint16_t reserved;
	uint8_t reserved2;
  } msg;
  msg.index = index;
  msg.count = count;
  msg.version = 0;
  msg.reserved = 0;
  msg.reserved2 = 0;
  poll(UBX_LOG, LOG_RETRIEVE, sizeof(msg), (uint8_t *)&msg);
  return true;
}

template <class T, class D> bool  Gnss<T,D>::logRetrieve(uint32_t index, uint32_t count) {
  struct {
	uint32_t index;
	uint32_t count; //256 max!
	uint8_t version;
	uint16_t reserved;
	uint8_t reserved2;
  } msg;
  msg.index = index;
  msg.count = count;
  msg.version = 0;
  msg.reserved = 0;
  msg.reserved2 = 0;
  poll(UBX_LOG, LOG_RETRIEVE, sizeof(msg), (uint8_t *)&msg);
  //if (ackNoError(timeout)) return true; //ACK-NAK will be sent after the log messages, 
  //especially NAK is sent when there are more than 256 records to retrieve after first 256 records are sent to indicate that
  //logRetrieve command should be repeated with the new index!
  return true;
}

template <class T> GnssSupport * Gnss<T,null>::getSupportedGnss(uint32_t timeout) {
  poll(UBX_MON, MON_GNSS);
  if (pollNoError(timeout))	return (GnssSupport *)payload;
  return NULL;
}

template <class T, class D> GnssSupport * Gnss<T,D>::getSupportedGnss(uint32_t timeout) {
  poll(UBX_MON, MON_GNSS);
  if (pollNoError(timeout))	return (GnssSupport *)payload;
  return NULL;
}

template <class T> GnssConf * Gnss<T,null>::getGnss(uint32_t timeout) {
  poll(UBX_CFG, CFG_GNSS);
  if (pollNoError(timeout))	return (GnssConf *)payload;
  return NULL;
}

template <class T, class D> GnssConf * Gnss<T,D>::getGnss(uint32_t timeout) {
  poll(UBX_CFG, CFG_GNSS);
  if (pollNoError(timeout))	return (GnssConf *)payload;
  return NULL;
}

template <class T> bool Gnss<T,null>::setGnss(MajorGnss gnss, bool enableSBAS, bool enableIMES, uint32_t timeout) {
  GnssConf * gnssConf = NULL;
  GnssCfg * c = NULL;
  poll(UBX_CFG, CFG_GNSS);
  if (pollNoError(timeout)) {
	gnssConf = (GnssConf *)(payload);
	for (uint8_t i = 0; i < gnssConf->numConfigBlocks; i++) {
		c = (GnssCfg *)(payload + sizeof(GnssConf) + sizeof(GnssCfg) * i);
		switch (i) {
			case 0: c->flags.enabled = gnss.Gps; break;
			case 1: c->flags.enabled = gnss.Gps; break; //QZSS should be enabled or disabled togather with GPS
			case 2: c->flags.enabled = gnss.Galileo; break;
			case 3: c->flags.enabled = gnss.BeiDou; break;
			case 4: c->flags.enabled = gnss.Glonass; break;
			case 5: c->flags.enabled = enableSBAS; break;
			case 6: c->flags.enabled = enableIMES; break;
		}
	}
  }
  poll(UBX_CFG, CFG_GNSS, payloadLength, (uint8_t *)gnssConf);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setGnss(MajorGnss gnss, bool enableSBAS, bool enableIMES, uint32_t timeout) {
  GnssConf * gnssConf = NULL;
  GnssCfg * c = NULL;
  poll(UBX_CFG, CFG_GNSS);
  if (pollNoError(timeout)) {
	gnssConf = (GnssConf *)(payload);
	for (uint8_t i = 0; i < gnssConf->numConfigBlocks; i++) {
		c = (GnssCfg *)(payload + sizeof(GnssConf) + sizeof(GnssCfg) * i);
		switch (i) {
			case 0: c->flags.enabled = gnss.Gps; break;
			case 1: c->flags.enabled = gnss.Gps; break; //QZSS should be enabled or disabled togather with GPS
			case 2: c->flags.enabled = gnss.Galileo; break;
			case 3: c->flags.enabled = gnss.BeiDou; break;
			case 4: c->flags.enabled = gnss.Glonass; break;
			case 5: c->flags.enabled = enableSBAS; break;
			case 6: c->flags.enabled = enableIMES; break;
		}
	}
  }
  poll(UBX_CFG, CFG_GNSS, payloadLength, (uint8_t *)gnssConf);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T> NavRate * Gnss<T,null>::getNavRate(uint32_t timeout) {
  poll(UBX_CFG, CFG_RATE);
  if (pollNoError(timeout))	return (NavRate *)payload;
  return NULL;	
}

template <class T, class D> NavRate * Gnss<T,D>::getNavRate(uint32_t timeout) {
  poll(UBX_CFG, CFG_RATE);
  if (pollNoError(timeout))	return (NavRate *)payload;
  return NULL;	
}

template <class T> bool Gnss<T,null>::setNavRate(uint16_t measurementRate, uint16_t navSolRate, TimeRef timeRef, uint32_t timeout) {
  NavRate * rate = NULL;
  poll(UBX_CFG, CFG_RATE);
  if (pollNoError(timeout)) rate = (NavRate *)(payload);
  rate->rate = measurementRate; //ms
  rate->navSolRate = navSolRate; //cycles - number of measurements for each NavSol, max 127.
  rate->timeRef = timeRef; 
  poll(UBX_CFG, CFG_RATE, payloadLength, (uint8_t *)rate);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setNavRate(uint16_t measurementRate, uint16_t navSolRate, TimeRef timeRef, uint32_t timeout) {
  NavRate * rate = NULL;
  poll(UBX_CFG, CFG_RATE);
  if (pollNoError(timeout)) rate = (NavRate *)(payload);
  rate->rate = measurementRate; //ms
  rate->navSolRate = navSolRate; //cycles - number of measurements for each NavSol, max 127.
  rate->timeRef = timeRef; 
  poll(UBX_CFG, CFG_RATE, payloadLength, (uint8_t *)rate);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T> TimePulse * Gnss<T,null>::getTimePulse(uint32_t timeout) {
  poll(UBX_CFG, CFG_TP5);
  if (pollNoError(timeout))	return (TimePulse *)payload;
  return NULL;
}

template <class T, class D> TimePulse * Gnss<T,D>::getTimePulse(uint32_t timeout) {
  poll(UBX_CFG, CFG_TP5);
  if (pollNoError(timeout))	return (TimePulse *)payload;
  return NULL;
}

template <class T> bool Gnss<T,null>::setTimePulse(uint32_t pulse_period, uint32_t pulse_len, uint32_t pulse_period_locked, uint32_t pulse_len_locked, int32_t delay, TimePulseFlags flags, uint32_t timeout) {
  TimePulse * tp = NULL;
  poll(UBX_CFG, CFG_TP5);
  if (pollNoError(timeout)) tp = (TimePulse *)(payload);
  tp->pulsePeriod = pulse_period;
  tp->pulseLen = pulse_len; 
  tp->pulsePeriodLocked = pulse_period_locked;
  tp->pulseLenLocked = pulse_len_locked; 
  tp->userConfigDelay = delay; 
  tp->flags.active = flags.active;
  tp->flags.lockGnssFreq = flags.lockGnssFreq;
  tp->flags.lockedOtherSet = flags.lockedOtherSet;
  tp->flags.isFreq = flags.isFreq;
  tp->flags.isLength = flags.isLength;
  tp->flags.alignToTOW = flags.alignToTOW;
  tp->flags.gridUtcGnss = flags.gridUtcGnss;
  tp->flags.syncMode = flags.syncMode;
  poll(UBX_CFG, CFG_TP5, payloadLength, (uint8_t *)tp);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setTimePulse(uint32_t pulse_period, uint32_t pulse_len, uint32_t pulse_period_locked, uint32_t pulse_len_locked, int32_t delay, TimePulseFlags flags, uint32_t timeout) {
  TimePulse * tp = NULL;
  poll(UBX_CFG, CFG_TP5);
  if (pollNoError(timeout)) tp = (TimePulse *)(payload);
  tp->pulsePeriod = pulse_period;
  tp->pulseLen = pulse_len; 
  tp->pulsePeriodLocked = pulse_period_locked;
  tp->pulseLenLocked = pulse_len_locked; 
  tp->userConfigDelay = delay; 
  tp->flags.active = flags.active;
  tp->flags.lockGnssFreq = flags.lockGnssFreq;
  tp->flags.lockedOtherSet = flags.lockedOtherSet;
  tp->flags.isFreq = flags.isFreq;
  tp->flags.isLength = flags.isLength;
  tp->flags.alignToTOW = flags.alignToTOW;
  tp->flags.gridUtcGnss = flags.gridUtcGnss;
  tp->flags.syncMode = flags.syncMode;
  poll(UBX_CFG, CFG_TP5, payloadLength, (uint8_t *)tp);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T> NavPvt * Gnss<T,null>::getNavPvt(uint32_t timeout) {
  poll(UBX_NAV, NAV_PVT);
  if (pollNoError(timeout))	return (NavPvt *)payload;
  return NULL;
}

template <class T, class D> NavPvt * Gnss<T,D>::getNavPvt(uint32_t timeout) {
  poll(UBX_NAV, NAV_PVT);
  if (pollNoError(timeout))	return (NavPvt *)payload;
  return NULL;
}

template <class T> NavSat * Gnss<T,null>::getNavSat(uint32_t timeout) {
  poll(UBX_NAV, NAV_SAT);
  if (pollNoError(timeout))	return (NavSat *)payload;
  return NULL;
}

template <class T, class D> NavSat * Gnss<T,D>::getNavSat(uint32_t timeout) {
  poll(UBX_NAV, NAV_SAT);
  if (pollNoError(timeout))	return (NavSat *)payload;
  return NULL;
}

template <class T> NavTimeUtc * Gnss<T,null>::getNavTimeUtc(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMEUTC);
  if (pollNoError(timeout))	return (NavTimeUtc *)payload;
  return NULL;
}

template <class T, class D> NavTimeUtc * Gnss<T,D>::getNavTimeUtc(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMEUTC);
  if (pollNoError(timeout))	return (NavTimeUtc *)payload;
  return NULL;
}

template <class T> NavClock * Gnss<T,null>::getNavClock(uint32_t timeout) {
  poll(UBX_NAV, NAV_CLOCK);
  if (pollNoError(timeout))	return (NavClock *)payload;
  return NULL;
}

template <class T, class D> NavClock * Gnss<T,D>::getNavClock(uint32_t timeout) {
  poll(UBX_NAV, NAV_CLOCK);
  if (pollNoError(timeout))	return (NavClock *)payload;
  return NULL;
}

template <class T> NavDGPS * Gnss<T,null>::getNavDgps(uint32_t timeout) {
  poll(UBX_NAV, NAV_DGPS);
  if (pollNoError(timeout))	return (NavDGPS *)payload;
  return NULL;
}

template <class T, class D> NavDGPS * Gnss<T,D>::getNavDgps(uint32_t timeout) {
  poll(UBX_NAV, NAV_DGPS);
  if (pollNoError(timeout))	return (NavDGPS *)payload;
  return NULL;
}

template <class T> NavDOP * Gnss<T,null>::getNavDop(uint32_t timeout) {
  poll(UBX_NAV, NAV_DOP);
  if (pollNoError(timeout))	return (NavDOP *)payload;
  return NULL;
}

template <class T, class D> NavDOP * Gnss<T,D>::getNavDop(uint32_t timeout) {
  poll(UBX_NAV, NAV_DOP);
  if (pollNoError(timeout))	return (NavDOP *)payload;
  return NULL;
}

template <class T> NavGeofence * Gnss<T,null>::getNavGeofence(uint32_t timeout) {
  poll(UBX_NAV, NAV_GEOFENCE);
  if (pollNoError(timeout))	return (NavGeofence *)payload;
  return NULL;  
}

template <class T, class D> NavGeofence * Gnss<T,D>::getNavGeofence(uint32_t timeout) {
  poll(UBX_NAV, NAV_GEOFENCE);
  if (pollNoError(timeout))	return (NavGeofence *)payload;
  return NULL;  
}

template <class T> GeoFences * Gnss<T,null>::getCfgGeofences(uint32_t timeout) {
  poll(UBX_CFG, CFG_GEOFENCE);
  if (pollNoError(timeout))	return (GeoFences *)payload;
  return NULL;
}

template <class T, class D> GeoFences * Gnss<T,D>::getCfgGeofences(uint32_t timeout) {
  poll(UBX_CFG, CFG_GEOFENCE);
  if (pollNoError(timeout))	return (GeoFences *)payload;
  return NULL;
}

template <class T> bool Gnss<T,null>::setCfgGeofence(GeoFence * geofence, uint8_t confidenceLevel, uint32_t timeout) {
  GeoFences * fences = NULL;
  poll(UBX_CFG, CFG_GEOFENCE);
  if (pollNoError(timeout))	fences = (GeoFences *)payload; else return false;
  CfgGeofences cfgFences;
  cfgFences.geoFences.version = fences->version;
  cfgFences.geoFences.numFences = 1;
  cfgFences.geoFences.confidenceLevel = confidenceLevel;
  cfgFences.geoFences.reserved = fences->reserved;
  cfgFences.geoFences.pioEnabled = fences->pioEnabled;
  cfgFences.geoFences.pinPolarity = fences->pinPolarity;
  cfgFences.geoFences.pin = fences->pin;
  cfgFences.geoFences.reserved2 = fences->reserved2;
  cfgFences.geoFence.lat = geofence->lat;
  cfgFences.geoFence.lon = geofence->lon;
  cfgFences.geoFence.radius = geofence->radius;

  poll(UBX_CFG, CFG_GEOFENCE, sizeof(CfgGeofences), (uint8_t *)&cfgFences);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T, class D> bool Gnss<T,D>::setCfgGeofence(GeoFence * geofence, uint8_t confidenceLevel, uint32_t timeout) {
  GeoFences * fences = NULL;
  poll(UBX_CFG, CFG_GEOFENCE);
  if (pollNoError(timeout))	fences = (GeoFences *)payload; else return false;
  CfgGeofences cfgFences;
  cfgFences.geoFences.version = fences->version;
  cfgFences.geoFences.numFences = 1;
  cfgFences.geoFences.confidenceLevel = confidenceLevel;
  cfgFences.geoFences.reserved = fences->reserved;
  cfgFences.geoFences.pioEnabled = fences->pioEnabled;
  cfgFences.geoFences.pinPolarity = fences->pinPolarity;
  cfgFences.geoFences.pin = fences->pin;
  cfgFences.geoFences.reserved2 = fences->reserved2;
  cfgFences.geoFence.lat = geofence->lat;
  cfgFences.geoFence.lon = geofence->lon;
  cfgFences.geoFence.radius = geofence->radius;

  poll(UBX_CFG, CFG_GEOFENCE, sizeof(CfgGeofences), (uint8_t *)&cfgFences);
  if (ackNoError(timeout)) return true;
  return false;
}

template <class T> NavODO * Gnss<T,null>::getNavOdo(uint32_t timeout) {
  poll(UBX_NAV, NAV_ODO);
  if (pollNoError(timeout))	return (NavODO *)payload;
  return NULL;  
}

template <class T, class D> NavODO * Gnss<T,D>::getNavOdo(uint32_t timeout) {
  poll(UBX_NAV, NAV_ODO);
  if (pollNoError(timeout))	return (NavODO *)payload;
  return NULL;  
}

template <class T> bool Gnss<T,null>::resetOdo(uint32_t timeout) {
  poll(UBX_NAV, NAV_RESETODO);
  if (ackNoError(timeout))	return true;
  return false;  
}

template <class T, class D> bool Gnss<T,D>::resetOdo(uint32_t timeout) {
  poll(UBX_NAV, NAV_RESETODO);
  if (ackNoError(timeout))	return true;
  return false;  
}

template <class T> NavOrb * Gnss<T,null>::getNavOrb(uint32_t timeout) {
  poll(UBX_NAV, NAV_ORB);
  if (pollNoError(timeout))	return (NavOrb *)payload;
  return NULL;  
}

template <class T, class D> NavOrb * Gnss<T,D>::getNavOrb(uint32_t timeout) {
  poll(UBX_NAV, NAV_ORB);
  if (pollNoError(timeout))	return (NavOrb *)payload;
  return NULL;  
}

template <class T> NavPosLlh * Gnss<T,null>::getNavPosLlh(uint32_t timeout) {
  poll(UBX_NAV, NAV_POSLLH);
  if (pollNoError(timeout))	return (NavPosLlh *)payload;
  return NULL; 
}

template <class T, class D> NavPosLlh * Gnss<T,D>::getNavPosLlh(uint32_t timeout) {
  poll(UBX_NAV, NAV_POSLLH);
  if (pollNoError(timeout))	return (NavPosLlh *)payload;
  return NULL; 
}

template <class T> NavSbas * Gnss<T,null>::getNavSbas(uint32_t timeout) {
  poll(UBX_NAV, NAV_SBAS);
  if (pollNoError(timeout))	return (NavSbas *)payload;
  return NULL; 
}

template <class T, class D> NavSbas * Gnss<T,D>::getNavSbas(uint32_t timeout) {
  poll(UBX_NAV, NAV_SBAS);
  if (pollNoError(timeout))	return (NavSbas *)payload;
  return NULL; 
}

template <class T> NavSlas * Gnss<T,null>::getNavSlas(uint32_t timeout) {
  poll(UBX_NAV, NAV_SLAS);
  if (pollNoError(timeout))	return (NavSlas *)payload;
  return NULL; 
}

template <class T, class D> NavSlas * Gnss<T,D>::getNavSlas(uint32_t timeout) {
  poll(UBX_NAV, NAV_SLAS);
  if (pollNoError(timeout))	return (NavSlas *)payload;
  return NULL; 
}

template <class T> NavTimeBdsGal * Gnss<T,null>::getNavTimeBds(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMEBDS);
  if (pollNoError(timeout))	return (NavTimeBdsGal *)payload;
  return NULL;
}

template <class T, class D> NavTimeBdsGal * Gnss<T,D>::getNavTimeBds(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMEBDS);
  if (pollNoError(timeout))	return (NavTimeBdsGal *)payload;
  return NULL;
}

template <class T> NavTimeBdsGal * Gnss<T,null>::getNavTimeGal(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMEGAL);
  if (pollNoError(timeout))	return (NavTimeBdsGal *)payload;
  return NULL;
}

template <class T, class D> NavTimeBdsGal * Gnss<T,D>::getNavTimeGal(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMEGAL);
  if (pollNoError(timeout))	return (NavTimeBdsGal *)payload;
  return NULL;
}

template <class T> NavTimeGps * Gnss<T,null>::getNavTimeGps(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMEGPS);
  if (pollNoError(timeout))	return (NavTimeGps *)payload;
  return NULL;
}

template <class T, class D> NavTimeGps * Gnss<T,D>::getNavTimeGps(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMEGPS);
  if (pollNoError(timeout))	return (NavTimeGps *)payload;
  return NULL;
}

template <class T> NavTimeGlo * Gnss<T,null>::getNavTimeGlo(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMEGLO);
  if (pollNoError(timeout))	return (NavTimeGlo *)payload;
  return NULL;
}

template <class T, class D> NavTimeGlo * Gnss<T,D>::getNavTimeGlo(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMEGLO);
  if (pollNoError(timeout))	return (NavTimeGlo *)payload;
  return NULL;
}

template <class T> NavTimeLs * Gnss<T,null>::getNavTimeLs(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMELS);
  if (pollNoError(timeout))	return (NavTimeLs *)payload;
  return NULL;
}

template <class T, class D> NavTimeLs * Gnss<T,D>::getNavTimeLs(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMELS);
  if (pollNoError(timeout))	return (NavTimeLs *)payload;
  return NULL;
}

template <class T> TimTm * Gnss<T,null>::getTimTm(uint32_t timeout) {
  poll(UBX_TIM, TIM_TM2);
  if (pollNoError(timeout))	return (TimTm *)payload;
  return NULL;
}

template <class T, class D> TimTm * Gnss<T,D>::getTimTm(uint32_t timeout) {
  poll(UBX_TIM, TIM_TM2);
  if (pollNoError(timeout))	return (TimTm *)payload;
  return NULL;
}

template <class T> TimeTP * Gnss<T,null>::getTimeTp(uint32_t timeout) {
  poll(UBX_TIM, TIM_TP);
  if (pollNoError(timeout))	return (TimeTP *)payload;
  return NULL;
}

template <class T, class D> TimeTP * Gnss<T,D>::getTimeTp(uint32_t timeout) {
  poll(UBX_TIM, TIM_TP);
  if (pollNoError(timeout))	return (TimeTP *)payload;
  return NULL;
}

//NMEA messages
template <class T> void Gnss<T,null>::getGNRMC(GNRMC * data) {
//	if (!nmeaPayload.startsWith("GNRMC")) return false;
	char NS = '\0', EW = '\0';
	String time, date, lat, lon, rmc[13];
	memset(&(data->dateTime), 0, sizeof(DateTime));
	memset(&(data->lat), 0, sizeof(Latitude));
	memset(&(data->lon), 0, sizeof(Longitude));
	data->SOG = 0; data->COG = 0; data->magVar = 0; data->magVarEW = '\0'; data->status = 'V'; data->fixType = 'N';
	
	split(rmc, 13, nmeaPayload);
	
	for (uint8_t i = 1; i < 13; i++) {
		switch (i) {
			case 1: time = rmc[i]; break;
			case 2: 
				data->status = rmc[i][0];
				if (data->status == 'V') return false;
				break;
			case 3: lat = rmc[i]; break;
			case 4: NS = rmc[i][0]; break;
			case 5: lon = rmc[i]; break;
			case 6: EW = rmc[i][0]; break;
			case 7: data->SOG = rmc[i].toFloat(); break;
			case 8: data->COG = rmc[i].toFloat(); break;
			case 9: date = rmc[i]; break;
			case 10: data->magVar = rmc[i].toFloat(); break;
			case 11: data->magVarEW = rmc[i][0]; break;
			case 12: 
				data->fixType = rmc[i][0]; 
				if (data->fixType == 'N') return false;
				break;
			//case 13: data->navStatus = rmc[13][0]; break;
		}		
	}
	nmeaDate = date;
	nmeaToUtc(&(data->dateTime), date, time);
	nmeaLonToDMS(&(data->lon), lon, EW);
	nmeaLatToDMS(&(data->lat), lat, NS);
//	return true;
}

template <class T, class D> void Gnss<T,D>::getGNRMC(GNRMC * data) {
//	if (!nmeaPayload.startsWith("GNRMC")) return false;
	char NS = '\0', EW = '\0';
	String time, date, lat, lon, rmc[13];
	memset(&(data->dateTime), 0, sizeof(DateTime));
	memset(&(data->lat), 0, sizeof(Latitude));
	memset(&(data->lon), 0, sizeof(Longitude));
	data->SOG = 0; data->COG = 0; data->magVar = 0; data->magVarEW = '\0'; data->status = 'V'; data->fixType = 'N';
	
	split(rmc, 13, nmeaPayload);
	
	for (uint8_t i = 1; i < 13; i++) {
		switch (i) {
			case 1: time = rmc[i]; break;
			case 2: 
				data->status = rmc[i][0];
				//if (data->status == 'V') return false;
				break;
			case 3: lat = rmc[i]; break;
			case 4: NS = rmc[i][0]; break;
			case 5: lon = rmc[i]; break;
			case 6: EW = rmc[i][0]; break;
			case 7: data->SOG = rmc[i].toFloat(); break;
			case 8: data->COG = rmc[i].toFloat(); break;
			case 9: date = rmc[i]; break;
			case 10: data->magVar = rmc[i].toFloat(); break;
			case 11: data->magVarEW = rmc[i][0]; break;
			case 12: 
				data->fixType = rmc[i][0]; 
				//if (data->fixType == 'N') return false;
				break;
			//case 13: data->navStatus = rmc[13][0]; break;
		}		
	}
	nmeaDate = date;
	nmeaToUtc(&(data->dateTime), date, time);
	tm tm_time;
	utcTime = mk_gmtime(gps2tm(&(data->dateTime), &tm_time));
	nmeaLonToDMS(&(data->lon), lon, EW);
	nmeaLatToDMS(&(data->lat), lat, NS);
//	return true;
}

template <class T> void Gnss<T,null>::getGNGGA(GNGGA * data) {
//	if (!nmeaPayload.startsWith("GNGGA")) return false;
	memset(&(data->dateTime), 0, sizeof(DateTime));
	memset(&(data->lat), 0, sizeof(Latitude));
	memset(&(data->lon), 0, sizeof(Longitude));
	data->fixType = NO_FIX_TYPE; data->numSV = 0; data->hDOP = 0; data->alt = 0; data->geoidEllipsoidDiff = 0; data->ageDiffCorr = 0; data->diffCorrStationId = 0;
	String time = "", date = nmeaDate, lat, lon, gga[15];
	char NS = '\0', EW = '\0';
	
	split(gga, 15, nmeaPayload);
	
	for (uint8_t i = 1; i < 15; i++) {
		switch (i) {
			case 1: time = gga[i]; break;
			case 2: lat = gga[i]; break;
			case 3: NS = gga[i][0]; break;
			case 4: lon = gga[i]; break;
			case 5: EW = gga[i][0]; break;
			case 6: 
				data->fixType = gga[i].toInt(); 
				//if (data->fixType == NO_FIX_TYPE) return false;
				break;
			case 7: data->numSV = gga[i].toInt(); break;
			case 8: data->hDOP = gga[i].toFloat(); break;
			case 9: data->alt = gga[i].toFloat(); break;
			case 10: break;
			case 11: data->geoidEllipsoidDiff = gga[i].toFloat(); break;
			case 12: break;
			case 13: data->ageDiffCorr = gga[i].toInt(); break;
			case 14: data->diffCorrStationId = gga[i].toInt(); break;
		}		
	}
	nmeaToUtc(&(data->dateTime), date, time);
	nmeaLonToDMS(&(data->lon), lon, EW);
	nmeaLatToDMS(&(data->lat), lat, NS);
//	return true;
}

template <class T, class D> void Gnss<T,D>::getGNGGA(GNGGA * data) {
//	if (!nmeaPayload.startsWith("GNGGA")) return false;
	memset(&(data->dateTime), 0, sizeof(DateTime));
	memset(&(data->lat), 0, sizeof(Latitude));
	memset(&(data->lon), 0, sizeof(Longitude));
	data->fixType = NO_FIX_TYPE; data->numSV = 0; data->hDOP = 0; data->alt = 0; data->geoidEllipsoidDiff = 0; data->ageDiffCorr = 0; data->diffCorrStationId = 0;
	String time = "", date = nmeaDate, lat, lon, gga[15];
	char NS = '\0', EW = '\0';
	
	split(gga, 15, nmeaPayload);
	
	for (uint8_t i = 1; i < 15; i++) {
		switch (i) {
			case 1: time = gga[i]; break;
			case 2: lat = gga[i]; break;
			case 3: NS = gga[i][0]; break;
			case 4: lon = gga[i]; break;
			case 5: EW = gga[i][0]; break;
			case 6: 
				data->fixType = gga[i].toInt(); 
				//if (data->fixType == NO_FIX_TYPE) return false;
				break;
			case 7: data->numSV = gga[i].toInt(); break;
			case 8: data->hDOP = gga[i].toFloat(); break;
			case 9: data->alt = gga[i].toFloat(); break;
			case 10: break;
			case 11: data->geoidEllipsoidDiff = gga[i].toFloat(); break;
			case 12: break;
			case 13: data->ageDiffCorr = gga[i].toInt(); break;
			case 14: data->diffCorrStationId = gga[i].toInt(); break;
		}		
	}
	nmeaToUtc(&(data->dateTime), date, time);
	nmeaLonToDMS(&(data->lon), lon, EW);
	nmeaLatToDMS(&(data->lat), lat, NS);
//	return true;
}

template <class T> void Gnss<T,null>::getGNGLL(GNGLL * data) {
//	if (!nmeaPayload.startsWith("GNGLL")) return false;
	memset(&(data->dateTime), 0, sizeof(DateTime));
	memset(&(data->lat), 0, sizeof(Latitude));
	memset(&(data->lon), 0, sizeof(Longitude));
	data->fixType = 'N'; data->status = 'V';
	String time = "", date = nmeaDate, lat, lon, gll[8];
	char NS = '\0', EW = '\0';

	split(gll, 8, nmeaPayload);

	for (uint8_t i = 1; i < 8; i++) {
		switch (i) {
			case 1: lat = gll[i]; break;
			case 2: NS = gll[i][0]; break;
			case 3: lon = gll[i]; break;
			case 4: EW = gll[i][0]; break;
			case 5: time = gll[i]; break;
			case 6: 
				data->status = gll[i][0]; 
				if (data->status == 'V') return false;
				break;
			case 7: 
				data->fixType = gll[i][0]; 
				if (data->fixType == 'N') return false;
				break;
		}		
	}
	nmeaToUtc(&(data->dateTime), date, time);
	nmeaLonToDMS(&(data->lon), lon, EW);
	nmeaLatToDMS(&(data->lat), lat, NS);
//	return true;
}

template <class T, class D> void Gnss<T,D>::getGNGLL(GNGLL * data) {
//	if (!nmeaPayload.startsWith("GNGLL")) return false;
	memset(&(data->dateTime), 0, sizeof(DateTime));
	memset(&(data->lat), 0, sizeof(Latitude));
	memset(&(data->lon), 0, sizeof(Longitude));
	data->fixType = 'N'; data->status = 'V';
	String time = "", date = nmeaDate, lat, lon, gll[8];
	char NS = '\0', EW = '\0';

	split(gll, 8, nmeaPayload);

	for (uint8_t i = 1; i < 8; i++) {
		switch (i) {
			case 1: lat = gll[i]; break;
			case 2: NS = gll[i][0]; break;
			case 3: lon = gll[i]; break;
			case 4: EW = gll[i][0]; break;
			case 5: time = gll[i]; break;
			case 6: 
				data->status = gll[i][0]; 
				//if (data->status == 'V') return false;
				break;
			case 7: 
				data->fixType = gll[i][0]; 
				//if (data->fixType == 'N') return false;
				break;
		}		
	}
	nmeaToUtc(&(data->dateTime), date, time);
	nmeaLonToDMS(&(data->lon), lon, EW);
	nmeaLatToDMS(&(data->lat), lat, NS);
//	return true;
}

template <class T> void Gnss<T,null>::getGNVTG(GNVTG * data) {
//	if (!nmeaPayload.startsWith("GNVTG")) return false;	
	data->COGt = 0; data->COGm = 0; data->SOGkt = 0; data->SOGkmh = 0; data->fixType = 'N';
	String vtg[10];

	split(vtg, 10, nmeaPayload);

	for (uint8_t i = 1; i < 10; i++) {
		switch (i) {
			case 1: data->COGt = vtg[i].toFloat(); break;
			case 2: break;
			case 3: data->COGm = vtg[i].toFloat(); break;
			case 4: break;
			case 5: data->SOGkt = vtg[i].toFloat(); break;
			case 6: break;
			case 7: data->SOGkmh = vtg[i].toFloat(); break;
			case 8: break;
			case 9: 
				data->fixType = vtg[i][0]; 
				if (data->fixType == 'N') return false;
				break;
		}		
	}
//	return true;
}

template <class T, class D> void Gnss<T,D>::getGNVTG(GNVTG * data) {
//	if (!nmeaPayload.startsWith("GNVTG")) return false;	
	data->COGt = 0; data->COGm = 0; data->SOGkt = 0; data->SOGkmh = 0; data->fixType = 'N';
	String vtg[10];

	split(vtg, 10, nmeaPayload);

	for (uint8_t i = 1; i < 10; i++) {
		switch (i) {
			case 1: data->COGt = vtg[i].toFloat(); break;
			case 2: break;
			case 3: data->COGm = vtg[i].toFloat(); break;
			case 4: break;
			case 5: data->SOGkt = vtg[i].toFloat(); break;
			case 6: break;
			case 7: data->SOGkmh = vtg[i].toFloat(); break;
			case 8: break;
			case 9: 
				data->fixType = vtg[i][0]; 
				//if (data->fixType == 'N') return false;
				break;
		}		
	}
//	return true;
}

template <class T> void Gnss<T,null>::getGNGSA(GNGSA * data) {
//	if (!nmeaPayload.startsWith("GNGSA")) return false;	
	memset(&(data->svId), 0, sizeof(uint8_t[12]));
	data->fixType = FIX_NOT_AVAILABLE; data->opMode = 'A'; data->pDOP = 0; data->hDOP = 0; data->vDOP = 0;
	String s, gsa[18];

	split(gsa, 18, nmeaPayload);

	for (uint8_t i = 1; i < 18; i++) {
		switch (i) {
			case 1: data->opMode = gsa[i][0]; break;
			case 2: 
				data->fixType = gsa[i].toInt(); 
				//if (data->fixType == FIX_NOT_AVAILABLE) return;
				break;
			case 15: data->pDOP = gsa[i].toFloat(); break;
			case 16: data->hDOP = gsa[i].toFloat(); break;
			case 17: data->vDOP = gsa[i].toFloat(); break;
//			case 18: data->gnssId = nmeaPayload.substring(from, to).toInt(); break;
			default:
					s = gsa[i];
					if (s == "") data->svId[i - 3] = 0;
					//else if (s.startsWith("0")) data->svId[i - 3] = s.substring(1).toInt();
					else data->svId[i - 3] = s.toInt(); 
		}		
	}
//	return true;
}

template <class T, class D> void Gnss<T,D>::getGNGSA(GNGSA * data) {
//	if (!nmeaPayload.startsWith("GNGSA")) return false;	
	memset(&(data->svId), 0, sizeof(uint8_t[12]));
	data->fixType = FIX_NOT_AVAILABLE; data->opMode = 'A'; data->pDOP = 0; data->hDOP = 0; data->vDOP = 0;
	String s, gsa[18];

	split(gsa, 18, nmeaPayload);

	for (uint8_t i = 1; i < 18; i++) {
		switch (i) {
			case 1: data->opMode = gsa[i][0]; break;
			case 2: 
				data->fixType = gsa[i].toInt(); 
				//if (data->fixType == FIX_NOT_AVAILABLE) return;
				break;
			case 15: data->pDOP = gsa[i].toFloat(); break;
			case 16: data->hDOP = gsa[i].toFloat(); break;
			case 17: data->vDOP = gsa[i].toFloat(); break;
//			case 18: data->gnssId = nmeaPayload.substring(from, to).toInt(); break;
			default:
					s = gsa[i];
					if (s == "") data->svId[i - 3] = 0;
					//else if (s.startsWith("0")) data->svId[i - 3] = s.substring(1).toInt();
					else data->svId[i - 3] = s.toInt(); 
		}		
	}
//	return true;
}

template <class T> void Gnss<T,null>::getGNGSV(GNGSV * data) {
//	if (!nmeaPayload.startsWith("GPGSV") && !nmeaPayload.startsWith("GLGSV")) return false;
	memset(&(data->svAttr), 0, sizeof(uint8_t[4]));
	data->gnss = ""; data->totalMsg = 0; data->msgNum = 0; data->numSV = 0;
	String gsv[20];

	split(gsv, 20, nmeaPayload);

	for (uint8_t i = 0; i < 20; i++) {
		switch (i) {
			case 0:
				switch(gsv[i].charAt(1)) {
					case 'P': data->gnss = "GPS"; break;
					case 'L': data->gnss = "GLONASS"; break;
					case 'A': data->gnss = "Galileo"; break;
					case 'B': data->gnss = "BeiDuo"; break;					
					case 'N': data->gnss = "Combo"; break;					
				}
				break;
			case 1: data->totalMsg = gsv[i].toInt(); break;
			case 2: data->msgNum = gsv[i].toInt(); break;
			case 3: data->numSV = gsv[i].toInt(); break;
			case 4: data->svAttr[0].svId = gsv[i].toInt(); break;
			case 5: data->svAttr[0].elev = gsv[i].toInt(); break;
			case 6: data->svAttr[0].azim = gsv[i].toInt(); break;
			case 7: data->svAttr[0].cno = gsv[i].toInt(); break;
			case 8: data->svAttr[1].svId = gsv[i].toInt(); break;
			case 9: data->svAttr[1].elev = gsv[i].toInt(); break;
			case 10: data->svAttr[1].azim = gsv[i].toInt(); break;
			case 11: data->svAttr[1].cno = gsv[i].toInt(); break;
			case 12: data->svAttr[2].svId = gsv[i].toInt(); break;
			case 13: data->svAttr[2].elev = gsv[i].toInt(); break;
			case 14: data->svAttr[2].azim = gsv[i].toInt(); break;
			case 15: data->svAttr[2].cno = gsv[i].toInt(); break;
			case 16: data->svAttr[3].svId = gsv[i].toInt(); break;
			case 17: data->svAttr[3].elev = gsv[i].toInt(); break;
			case 18: data->svAttr[3].azim = gsv[i].toInt(); break;
			case 19: data->svAttr[3].cno = gsv[i].toInt(); break;
//			case 20: data->gnssId = nmeaPayload.substring(from, to).toInt(); break;
		}		
	}
//	return true;
}

template <class T, class D> void Gnss<T,D>::getGNGSV(GNGSV * data) {
	memset(&(data->svAttr), 0, sizeof(uint8_t[4]));
	data->gnss = ""; data->totalMsg = 0; data->msgNum = 0; data->numSV = 0;
	String gsv[20];

	split(gsv, 20, nmeaPayload);

	for (uint8_t i = 0; i < 20; i++) {
		switch (i) {
			case 0:
				switch(gsv[i].charAt(1)) {
					case 'P': data->gnss = "GPS"; break;
					case 'L': data->gnss = "GLONASS"; break;
					case 'A': data->gnss = "Galileo"; break;
					case 'B': data->gnss = "BeiDuo"; break;					
					case 'N': data->gnss = "Combo"; break;					
				}
				break;
			case 1: data->totalMsg = gsv[i].toInt(); break;
			case 2: data->msgNum = gsv[i].toInt(); break;
			case 3: data->numSV = gsv[i].toInt(); break;
			case 4: data->svAttr[0].svId = gsv[i].toInt(); break;
			case 5: data->svAttr[0].elev = gsv[i].toInt(); break;
			case 6: data->svAttr[0].azim = gsv[i].toInt(); break;
			case 7: data->svAttr[0].cno = gsv[i].toInt(); break;
			case 8: data->svAttr[1].svId = gsv[i].toInt(); break;
			case 9: data->svAttr[1].elev = gsv[i].toInt(); break;
			case 10: data->svAttr[1].azim = gsv[i].toInt(); break;
			case 11: data->svAttr[1].cno = gsv[i].toInt(); break;
			case 12: data->svAttr[2].svId = gsv[i].toInt(); break;
			case 13: data->svAttr[2].elev = gsv[i].toInt(); break;
			case 14: data->svAttr[2].azim = gsv[i].toInt(); break;
			case 15: data->svAttr[2].cno = gsv[i].toInt(); break;
			case 16: data->svAttr[3].svId = gsv[i].toInt(); break;
			case 17: data->svAttr[3].elev = gsv[i].toInt(); break;
			case 18: data->svAttr[3].azim = gsv[i].toInt(); break;
			case 19: data->svAttr[3].cno = gsv[i].toInt(); break;
//			case 20: data->gnssId = nmeaPayload.substring(from, to).toInt(); break;
		}		
	}
}

template <class T> void Gnss<T,null>::getGNGST(GNGST * data) {
	memset(&(data->dateTime), 0, sizeof(DateTime));
	data->rangeRms = 0; data->stdMajor = 0; data->stdMinor = 0; data->orient = 0; data->stdLat = 0; data->stdLon = 0; data->stdAlt = 0;
	String time = "", date = nmeaDate;
	String gst[9];

	split(gst, 9, nmeaPayload);

	for (uint8_t i = 1; i < 9; i++) {
		switch (i) {
			case 1: time = gst[i]; break;
			case 2: data->rangeRms = gst[i].toFloat(); break;
			case 3: data->stdMajor = gst[i].toFloat(); break;
			case 4: data->stdMinor = gst[i].toFloat(); break;
			case 5: data->orient = gst[i].toFloat(); break;
			case 6: data->stdLat = gst[i].toFloat(); break;
			case 7: data->stdLon = gst[i].toFloat(); break;
			case 8: data->stdAlt = gst[i].toFloat(); break;
		}		
	}
	nmeaToUtc(&(data->dateTime), date, time);
}

template <class T, class D> void Gnss<T,D>::getGNGST(GNGST * data) {
	memset(&(data->dateTime), 0, sizeof(DateTime));
	data->rangeRms = 0; data->stdMajor = 0; data->stdMinor = 0; data->orient = 0; data->stdLat = 0; data->stdLon = 0; data->stdAlt = 0;
	String time = "", date = nmeaDate;
	String gst[9];

	split(gst, 9, nmeaPayload);

	for (uint8_t i = 1; i < 9; i++) {
		switch (i) {
			case 1: time = gst[i]; break;
			case 2: data->rangeRms = gst[i].toFloat(); break;
			case 3: data->stdMajor = gst[i].toFloat(); break;
			case 4: data->stdMinor = gst[i].toFloat(); break;
			case 5: data->orient = gst[i].toFloat(); break;
			case 6: data->stdLat = gst[i].toFloat(); break;
			case 7: data->stdLon = gst[i].toFloat(); break;
			case 8: data->stdAlt = gst[i].toFloat(); break;
		}		
	}
	nmeaToUtc(&(data->dateTime), date, time);
}

//Dual ground/water distance
template <class T> void Gnss<T,null>::getGNVLW(GNVLW * data) {
	data->twd = 0; data->wd = 0; data->tgd = 0; data->gd = 0; data->twdUnit = 'N'; data->wdUnit = 'N'; data->tgdUnit = 'N'; data->gdUnit = 'N';
	String vlw[8];

	split(vlw, 8, nmeaPayload);

	for (uint8_t i = 1; i < 8; i++) {
		switch (i) {
			case 1: data->twd = vlw[i].toFloat(); break;
			case 2: data->twdUnit = vlw[i].charAt(0); break;
			case 3: data->wd = vlw[i].toFloat(); break;
			case 4: data->wdUnit = vlw[i].charAt(0); break;
			case 5: data->tgd = vlw[i].toFloat(); break;
			case 6: data->tgdUnit = vlw[i].charAt(0); break;
			case 7: data->gd = vlw[i].toFloat(); break;
			case 8: data->gdUnit = vlw[i].charAt(0); break;
		}		
	}
}

template <class T, class D> void Gnss<T,D>::getGNVLW(GNVLW * data) {
	data->twd = 0; data->wd = 0; data->tgd = 0; data->gd = 0; data->twdUnit = 'N'; data->wdUnit = 'N'; data->tgdUnit = 'N'; data->gdUnit = 'N';
	String vlw[8];

	split(vlw, 8, nmeaPayload);

	for (uint8_t i = 1; i < 8; i++) {
		switch (i) {
			case 1: data->twd = vlw[i].toFloat(); break;
			case 2: data->twdUnit = vlw[i].charAt(0); break;
			case 3: data->wd = vlw[i].toFloat(); break;
			case 4: data->wdUnit = vlw[i].charAt(0); break;
			case 5: data->tgd = vlw[i].toFloat(); break;
			case 6: data->tgdUnit = vlw[i].charAt(0); break;
			case 7: data->gd = vlw[i].toFloat(); break;
			case 8: data->gdUnit = vlw[i].charAt(0); break;
		}		
	}
}

//GNSS fix data
template <class T> void Gnss<T,null>::getGNGNS(GNGNS * data) {
	memset(&(data->dateTime), 0, sizeof(DateTime));
	memset(&(data->lat), 0, sizeof(Latitude));
	memset(&(data->lon), 0, sizeof(Longitude));
	data->fixType[0] = 'N'; data->fixType[1] = 'N';
	data->numSV = 0; data->hDOP = 0; data->alt = 0; data->geoidEllipsoidDiff = 0; data->ageDiffCorr = 0; data->diffCorrStationId = 0;
	String time = "", date = nmeaDate, lat = "", lon = "";
	char NS = '\0', EW = '\0';
	String gns[13];

	split(gns, 13, nmeaPayload);

	for (uint8_t i = 1; i < 13; i++) {
		switch (i) {
			case 1: time = gns[i]; break;
			case 2: data->lat = gns[i].toFloat(); break;
			case 3: NS = gns[i].charAt(0); break;
			case 4: data->lon = gns[i].toFloat(); break;
			case 5: EW = gns[i].charAt(0); break;
			case 6: data->fixType[0] = gns[i].charAt(0); data->fixType[1] = gns[i].charAt(1); break;
			case 7: data->numSV = gns[i].toInt(); break;
			case 8: data->hDOP = gns[i].toFloat(); break;
			case 9: data->alt = gns[i].toFloat(); break;
			case 10: data->geoidEllipsoidDiff = gns[i].toFloat(); break;
			case 11: data->ageDiffCorr = gns[i].toFloat(); break;
			case 12: data->diffCorrStationId = gns[i].toInt(); break;
			//case 13: data->navStatus = gns[i].charAt(0); break;
		}		
	}
	nmeaToUtc(&(data->dateTime), date, time);
	nmeaLonToDMS(&(data->lon), lon, EW);
	nmeaLatToDMS(&(data->lat), lat, NS);
}

template <class T, class D> void Gnss<T,D>::getGNGNS(GNGNS * data) {
	memset(&(data->dateTime), 0, sizeof(DateTime));
	memset(&(data->lat), 0, sizeof(Latitude));
	memset(&(data->lon), 0, sizeof(Longitude));
	data->fixType[0] = 'N'; data->fixType[1] = 'N';
	data->numSV = 0; data->hDOP = 0; data->alt = 0; data->geoidEllipsoidDiff = 0; data->ageDiffCorr = 0; data->diffCorrStationId = 0;
	String time = "", date = nmeaDate, lat = "", lon = "";
	char NS = '\0', EW = '\0';
	String gns[13];

	split(gns, 13, nmeaPayload);

	for (uint8_t i = 1; i < 13; i++) {
		switch (i) {
			case 1: time = gns[i]; break;
			case 2: lat = gns[i]; break;
			case 3: NS = gns[i].charAt(0); break;
			case 4: lon = gns[i]; break;
			case 5: EW = gns[i].charAt(0); break;
			case 6: data->fixType[0] = gns[i].charAt(0); data->fixType[1] = gns[i].charAt(1); break;
			case 7: data->numSV = gns[i].toInt(); break;
			case 8: data->hDOP = gns[i].toFloat(); break;
			case 9: data->alt = gns[i].toFloat(); break;
			case 10: data->geoidEllipsoidDiff = gns[i].toFloat(); break;
			case 11: data->ageDiffCorr = gns[i].toFloat(); break;
			case 12: data->diffCorrStationId = gns[i].toInt(); break;
			//case 13: data->navStatus = gns[i].charAt(0); break;
		}		
	}
	nmeaToUtc(&(data->dateTime), date, time);
	nmeaLonToDMS(&(data->lon), lon, EW);
	nmeaLatToDMS(&(data->lat), lat, NS);
}

//Time and Date
template <class T> void Gnss<T,null>::getGNZDA(GNZDA * data) {
	memset(&(data->dateTime), 0, sizeof(DateTime));
	data->utcOffsetHours = 0; data->utcOffsetMinutes = 0;
	String time = "", date = "";
	String zda[7];

	split(zda, 7, nmeaPayload);

	for (uint8_t i = 1; i < 7; i++) {
		switch (i) {
			case 1: time = zda[i]; break;
			case 2: date = zda[i]; break;
			case 3: date += zda[i]; break;
			case 4: date += zda[i].substring(2,4); break;
			case 5: data->utcOffsetHours = zda[i].toInt(); break;
			case 6: data->utcOffsetMinutes = zda[i].toInt(); break;
		}		
	}
	nmeaToUtc(&(data->dateTime), date, time);
}

template <class T, class D> void Gnss<T,D>::getGNZDA(GNZDA * data) {
	memset(&(data->dateTime), 0, sizeof(DateTime));
	data->utcOffsetHours = 0; data->utcOffsetMinutes = 0;
	String time = "", date = "";
	String zda[7];

	split(zda, 7, nmeaPayload);

	for (uint8_t i = 1; i < 7; i++) {
		switch (i) {
			case 1: time = zda[i]; break;
			case 2: date = zda[i]; break;
			case 3: date += zda[i]; break;
			case 4: date += zda[i].substring(2,4); break;
			case 5: data->utcOffsetHours = zda[i].toInt(); break;
			case 6: data->utcOffsetMinutes = zda[i].toInt(); break;
		}		
	}
	nmeaToUtc(&(data->dateTime), date, time);
}

template <class T> void Gnss<T,null>::getPubxPosition(PubxPosition * data) {
	memset(&(data->dateTime), 0, sizeof(DateTime));
	memset(&(data->lat), 0, sizeof(Latitude));
	memset(&(data->lon), 0, sizeof(Longitude));
	data->alt = 0; data->hAcc = 0; data->vAcc = 0; data->sog = 0; data->cog = 0; data->vVel = 0; data->ageDiffCorr = 0;
	data->hDOP = 0; data->vDOP = 0; data->tDOP = 0; data->numSV = 0; data->drUsed = false; 
	String time = "", date = nmeaDate, lat = "", lon = "";
	char NS = '\0', EW = '\0';
	String pos[21], p = "";
	sprintf(data->fixType, "NF");

	split(pos, 21, nmeaPayload);

	for (uint8_t i = 2; i < 21; i++) {
		switch (i) {
			case 2: time = pos[i]; break;
			case 3: lat = pos[i]; break;
			case 4: NS = pos[i].charAt(0); break;
			case 5: lon = pos[i]; break;
			case 6: EW = pos[i].charAt(0); break;
			case 7: data->alt = pos[i].toFloat(); break;
			case 8: data->fixType[0] = pos[i].charAt(0); data->fixType[1] = pos[i].charAt(1); break;
			case 9: data->hAcc = pos[i].toFloat(); break;
			case 10: data->vAcc = pos[i].toFloat(); break;
			case 11: data->sog = pos[i].toFloat(); break;
			case 12: data->cog = pos[i].toInt(); break;
			case 13: data->vVel = pos[i].toFloat(); break;
			case 14: data->ageDiffCorr = pos[i].toInt(); break;
			case 15: data->hDOP = pos[i].toFloat(); break;
			case 16: data->vDOP = pos[i].toFloat(); break;
			case 17: data->tDOP = pos[i].toFloat(); break;
			case 18: data->numSV = pos[i].toInt(); break;
			case 19: break;				
			case 20: if (pos[i].toInt() > 0) data->drUsed = true; break;
		}		
	}
	nmeaToUtc(&(data->dateTime), date, time);
	nmeaLonToDMS(&(data->lon), lon, EW);
	nmeaLatToDMS(&(data->lat), lat, NS);
}

template <class T, class D> void Gnss<T,D>::getPubxPosition(PubxPosition * data) {
	memset(&(data->dateTime), 0, sizeof(DateTime));
	memset(&(data->lat), 0, sizeof(Latitude));
	memset(&(data->lon), 0, sizeof(Longitude));
	data->alt = 0;  data->hAcc = 0; data->vAcc = 0; data->sog = 0; data->cog = 0; data->vVel = 0; data->ageDiffCorr = 0;
	data->hDOP = 0; data->vDOP = 0; data->tDOP = 0; data->numSV = 0; data->drUsed = false; 
	String time = "", date = nmeaDate, lat = "", lon = "";
	char NS = '\0', EW = '\0';
	String pos[21], p = "";
	sprintf(data->fixType, "NF");

	split(pos, 21, nmeaPayload);

	for (uint8_t i = 2; i < 21; i++) {
		switch (i) {
			case 2: time = pos[i]; break;
			case 3: lat = pos[i]; break;
			case 4: NS = pos[i].charAt(0); break;
			case 5: lon = pos[i]; break;
			case 6: EW = pos[i].charAt(0); break;
			case 7: data->alt = pos[i].toFloat(); break;
			case 8: data->fixType[0] = pos[i].charAt(0); data->fixType[1] = pos[i].charAt(1); break;
			case 9: data->hAcc = pos[i].toFloat(); break;
			case 10: data->vAcc = pos[i].toFloat(); break;
			case 11: data->sog = pos[i].toFloat(); break;
			case 12: data->cog = pos[i].toInt(); break;
			case 13: data->vVel = pos[i].toFloat(); break;
			case 14: data->ageDiffCorr = pos[i].toInt(); break;
			case 15: data->hDOP = pos[i].toFloat(); break;
			case 16: data->vDOP = pos[i].toFloat(); break;
			case 17: data->tDOP = pos[i].toFloat(); break;
			case 18: data->numSV = pos[i].toInt(); break;
			case 19: break;				
			case 20: if (pos[i].toInt() > 0) data->drUsed = true; break;
		}		
	}
	nmeaToUtc(&(data->dateTime), date, time);
	nmeaLonToDMS(&(data->lon), lon, EW);
	nmeaLatToDMS(&(data->lat), lat, NS);
}

template <class T> void Gnss<T,null>::getPubxTime(PubxTime * data) {
	memset(&(data->dateTime), 0, sizeof(DateTime));
	data->utcTow = 0; data->utcWeek = 0; data->leapSec = 0; data->clkBias = 0; data->clkDrift = 0; data->tpGran = 0; data->leapSecSrc = 'S';
	String time = "", date = "", leapSec = "";
	String tim[10];

	split(tim, 10, nmeaPayload);

	for (uint8_t i = 2; i < 10; i++) {
		switch (i) {
			case 2: time = tim[i]; break;
			case 3: date = tim[i]; break;
			case 4: data->utcTow = tim[i].toInt(); break;
			case 5: data->utcWeek = tim[i].toInt(); break;
			case 6: leapSec = tim[i];
				if (leapSec.endsWith("D")) { 
					data->leapSecSrc = 'D';
					leapSec.remove(leapSec.length() - 1);
					data->leapSec = leapSec.toInt();
				} else data->leapSec = leapSec.toInt(); break;
			case 7: data->clkBias = tim[i].toInt(); break;
			case 8: data->clkDrift = tim[i].toInt(); break;
			case 9: data->tpGran = tim[i].toInt(); break;
		}		
	}
	nmeaToUtc(&(data->dateTime), date, time);
}

template <class T, class D> void Gnss<T,D>::getPubxTime(PubxTime * data) {
	memset(&(data->dateTime), 0, sizeof(DateTime));
	data->utcTow = 0; data->utcWeek = 0; data->leapSec = 0; data->clkBias = 0; data->clkDrift = 0; data->tpGran = 0; data->leapSecSrc = 'S';
	String time = "", date = "", leapSec = "";
	String tim[10];

	split(tim, 10, nmeaPayload);

	for (uint8_t i = 2; i < 10; i++) {
		switch (i) {
			case 2: time = tim[i]; break;
			case 3: date = tim[i]; break;
			case 4: data->utcTow = tim[i].toInt(); break;
			case 5: data->utcWeek = tim[i].toInt(); break;
			case 6: leapSec = tim[i];
				if (leapSec.endsWith("D")) { 
					data->leapSecSrc = 'D';
					leapSec.remove(leapSec.length() - 1);
					data->leapSec = leapSec.toInt();
				} else data->leapSec = leapSec.toInt(); break;
			case 7: data->clkBias = tim[i].toInt(); break;
			case 8: data->clkDrift = tim[i].toInt(); break;
			case 9: data->tpGran = tim[i].toInt(); break;
		}		
	}
	nmeaToUtc(&(data->dateTime), date, time);
}

//helper functions
void split(String msg[], uint8_t array_size, String pload) {
	uint16_t len = pload.length(), j = 0;
	for (uint16_t i = 0; i < len; i++) {
		msg[j] = "";
		while (pload[i] != ',' && i < len) {
			msg[j] += pload[i++];
		}
		j++;
		if (j >= array_size) break;
	}
}

void nmeaLatToDMS(Latitude * latitude, String lat, char NS) {
	float m = lat.substring(2).toFloat();
	latitude->deg = lat.substring(0, 2).toInt();
	latitude->min = (uint8_t)m;
	latitude->sec = (m - (float)(latitude->min)) * 60;
	latitude->NS = NS;
}

void nmeaLonToDMS(Longitude * longitude, String lon, char EW) {
	float m = lon.substring(3).toFloat();
	longitude->deg = lon.substring(0, 3).toInt();
	longitude->min = (uint8_t)m;
	longitude->sec = (m - (float)(longitude->min)) * 60;
	longitude->EW = EW;
}

void longLatToDMS(Latitude * latitude, int32_t lat) {
      uint32_t l;
	  if (lat < 0) {
		latitude->NS = 'S';
		l = -lat;
	  }
      else 
	  {
		latitude->NS = 'N';      
		l = lat;
	  }
   	  latitude->deg = l / 10000000L;
      l %= 10000000L;
      latitude->min = l * 6 / 1000000L;
      latitude->sec = l * 36.0 / 100000.0 - latitude->min * 60.0;
}

int32_t dmsToLongLat(Latitude * latitude) {
      int32_t lat = latitude->deg * 10000000L + latitude->min * 1000000L / 6 + latitude->sec * 100000.0 / 36.0;
	  if (latitude->NS == 'S') lat = -lat;
	  return lat;
}

void longLonToDMS(Longitude * longitude, int32_t lon) {
      uint32_t l;
      if (lon < 0) {
		longitude->EW = 'W';
		l = -lon;
	  }
      else 
	  {
		longitude->EW = 'E';      
		l = lon;
	  }
	  longitude->deg = l / 10000000L;
      l %= 10000000L;
      longitude->min = l * 6 / 1000000L;
      longitude->sec = l * 36.0 / 100000.0 - longitude->min * 60.0;
}

int32_t dmsToLongLon(Longitude * longitude) {
      int32_t lon = longitude->deg * 10000000L + longitude->min * 1000000L / 6 + longitude->sec * 100000.0 / 36.0;
	  if (longitude->EW == 'W') lon = -lon;
	  return lon;
}

String dmsLatToStr(Latitude * lat) {
   char l[16];
   byte deg[3] = { 0xc2, 0xb0 };
   char apo = 39, quotes = 34;
   sprintf(l, "%.u%.2s%.u%.1s%.2f%.1s%c", lat->deg, (char *)(&deg), lat->min, &apo, (double)lat->sec, &quotes, lat->NS);
   return String(l);
}

String dmsLonToStr(Longitude * lon) {
   char l[16];
   byte deg[3] = { 0xc2, 0xb0 };
   char apo = 39, quotes = 34;
   sprintf(l, "%.u%.2s%.u%.1s%.2f%.1s%c", lon->deg, (char *)(&deg), lon->min, &apo, (double)lon->sec, &quotes, lon->EW);
   return String(l);
}

void nmeaToUtc(DateTime * dt, String date, String time) {
	dt->day = date.substring(0, 2).toInt();
	dt->month = date.substring(2, 4).toInt();
	dt->year = date.substring(4, 6).toInt();
	if (dt->year > 0) dt->year += 2000;
	dt->hour = time.substring(0, 2).toInt();
	dt->minute = time.substring(2, 4).toInt();
	dt->second = time.substring(4, 6).toInt();
}

tm * gps2tm(DateTime * dt, tm * time_tm) {
	time_tm->tm_year = dt->year - 1900;
	time_tm->tm_mon = dt->month - 1;
	time_tm->tm_mday = dt->day;
	time_tm->tm_hour = dt->hour;
	time_tm->tm_min = dt->minute;
	time_tm->tm_sec = dt->second;
	return time_tm;
}

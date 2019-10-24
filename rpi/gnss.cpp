#include "gnss.h"
using namespace std;

Gnss::Gnss() {
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
	loggingEnabled = false;
	logFileExists = false;
	if (setenv("TZ", "", 1) != 0) printf("setenv() error: %s", strerror(errno));
}

void Gnss::tp() {
  struct timeval tv;
  tv.tv_usec = 0;
  if (ttp) { //use this with timTp message only
	  tv.tv_sec = utcTime;
	  settimeofday(&tv, NULL); 
	  ttp = false; 
  }
  else {
	  tv.tv_sec = utcTime + 1;
	  settimeofday(&tv, NULL); //use this with normal time info such as navPvt, etc
	  pps = true;
  }
}

int Gnss::begin(const char * dev, speed_t baudRate) {
	fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		printf("error opening device %s: %s\n", dev, strerror(errno)); return -1;
	}
	struct termios settings;
	memset(&settings, 0, sizeof(termios));
	if (tcgetattr(fd, &settings) != 0) {
		printf("tcgetattr() error: %s\n", strerror(errno));
		return -1;
	}
	cfmakeraw(&settings); //switch to raw packet processing

	if (cfsetspeed(&settings, baudRate) != 0) {
		printf("cfsetspeed() error: %s\n", strerror(errno));
		return -1;
	}
	this->baudRate = baudRate;
	settings.c_cflag &= ~CSTOPB; //1 stop bit
	settings.c_iflag &= ~(IXON | IXOFF | IXANY); //no software flow control
	settings.c_cflag &= ~CRTSCTS; //no hardware flow control
	settings.c_cc[VMIN] = 0; //minimum number of char to read
	settings.c_cc[VTIME] = 0; //timeout in decisec (3 sec)
	if ((tcsetattr(fd, TCSANOW, &settings)) != 0) {
		printf("error in setting dev %s attributes: %s\n", dev, strerror(errno));
		return -1;
	}
	sleep(1);
	if (tcflush(fd, TCIOFLUSH) != 0) {
		printf("error flushing input and output queues of device %s: %s\n", dev, strerror(errno));
		return -1;
	}

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
  return 0;
}

void Gnss::end() {
	if (fd > 0) {
		tcflush(fd, TCIOFLUSH);
		close(fd);
		fd = 0;
	}
}

void Gnss::calculateChecksum(uint8_t msgClass, uint8_t msgId, uint16_t len, uint8_t * pload) {
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

void Gnss::nmeaVerifyChecksum()  {
	uint8_t x = 0, hex, high_byte = 0, low_byte = 0;
	uint16_t len = nmeaPayload.length();
	const char * cstring = nmeaPayload.c_str();

	for (uint8_t i = 0; i < len; i++) {
		x ^= cstring[i];
	}

	if (isdigit(nmeaChecksum[0])) high_byte = (nmeaChecksum[0] - 48);
    else if (isxdigit(nmeaChecksum[0])) high_byte = (nmeaChecksum[0] - 55);
    if (isdigit(nmeaChecksum[1])) low_byte = (nmeaChecksum[1] - 48);
    else if (isxdigit(nmeaChecksum[1])) low_byte = (nmeaChecksum[1] - 55);
	hex = (high_byte << 4) | low_byte;

	if (hex == x) nmeaValid = true; else nmeaValid = false;
}

bool Gnss::ready() {
  uint8_t c;
  clock_t t = clock();
  while (read(fd, &c, 1) > 0) {
      if (offset < 6) {
			switch (offset) {
				case 0:
					//if (nmea && (c != '\r')) printf("%c", c); else printf("%X ", c); 
					if (c == NMEA_START) { 
						nmea = true; //possible NMEA message as '$' can appear in UBX messages as well
						nmeaChecksumNext = false; nmeaPayload = ""; nmeaChecksum = ""; 
					}
					else if (c == SYNC_CHARS[0]) { //most likely UBX message as NMEA seems to be using ASCII chars only
						offset = 1; nmea = false; endOfNavEpoch = false; continue;
					}
					break;
				case 1:
					//printf("%X ", c);
					if (c != SYNC_CHARS[1]) {
						offset = 0;
					} else offset = 2;
					break;					
				case 2:
					//printf(" Receiving UBX...");
				    //printf(" msgCls %X", c);  
					messageClass = c;
					offset = 3;
					break;
				case 3:					
				    //printf(", msgId %X", c);  
					messageId = c;
					offset = 4;
					break;
				case 4: payloadLength = c; offset = 5; break;
				case 5: 
					payloadLength |= (((uint16_t)c) << 8); 
					//printf(", len %u, payload: ", payloadLength);
					payload = (uint8_t *)realloc(payload, payloadLength); 
					if (payload == NULL && payloadLength > 0) { offset = 0; error = OUT_OF_MEMORY; return true;}
					offset = 6;
					break;
			}			
	        //offset++;
	  }
      else { //offset >= 6
			if ((offset < (payloadLength + 6))) {
				payload[offset - 6] = c;
				//printf("%X ", c);
				offset++;
			}
			else if (offset == payloadLength + 6) {
				calculateChecksum(messageClass, messageId, payloadLength, payload);
				//printf(" calculated checksum0 %X, checksum1 %X", checksum[0], checksum[1]);
				if (c == checksum[0]) offset++;
				else {
					offset = 0;
					//printf("checksum0 %X - wrong checksum0: %X", checksum[0], c);
					error = CHECKSUM_ERROR; 
					return true;
				}
				//printf(" receieved chksum0 %X", c);
			}
			else if (offset == payloadLength + 7) {
			  offset = 0;
			  //printf(", chksum1 %X\n", c);
			  if (c == checksum[1])	{
				if (messageClass == UBX_NAV && messageId == NAV_EOE) { 
					//printf("end of nav epoch\n");
					endOfNavEpoch = true; 
					iTOW = *(uint32_t *)payload; 
				}
				error = NO_ERROR;
				return true;
			  }
			  else {
				//printf("checksum1 %X - wrong checksum1: %X\n", checksum[1], c);
				error = CHECKSUM_ERROR; 
				return true;
			  }
			}
	  }
	  if (nmea) {
		string cls = "", msgId = "";
		switch (c) {
		   case '$': continue;
		   case '*': nmeaChecksumNext = true; break;
		   case '\r': break;
		   case '\n': 
				nmea = false;
				nmeaVerifyChecksum();
				//printf("%s*%s\n", nmeaPayload.c_str(), nmeaChecksum.c_str());
				if (nmeaValid) {
					if (nmeaPayload.length() >= 5) {
						cls = nmeaPayload.substr(2, 3);
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
						else if (cls == "GBS") { messageClass = UBX_NMEA; messageId = NMEA_GBS; }
						else if (cls == "DTM") { messageClass = UBX_NMEA; messageId = NMEA_DTM; }
						else if (cls == "GRS") { messageClass = UBX_NMEA; messageId = NMEA_GRS; }
						else if (cls == "TXT") { messageClass = UBX_NMEA; messageId = NMEA_TXT; }
					}
					if (nmeaPayload.length() >= 4) {
						cls = nmeaPayload.substr(0, 4);
						if (cls == "PUBX") {
							messageClass = UBX_PUBX;
							if (nmeaPayload.length() >= 7) {
								msgId = nmeaPayload.substr(5, 2);
								if (msgId == "41") messageId = PUBX_CONFIG;
								else if (msgId == "00") messageId = PUBX_POSITION;
								else if (msgId == "40") messageId = PUBX_RATE;
								else if (msgId == "03") messageId = PUBX_SVSTATUS;
								else if (msgId == "04") messageId = PUBX_TIME;
							}
						}
					}
					return true;
				}
				else { printf("nmea checksum error: %s\n", nmeaPayload); return false; }
		    default: 
				if (!nmeaChecksumNext) nmeaPayload += char(c); 
				else {
					nmeaChecksum += char(c);
				}
				break;
		}
	  }
  }
  t = ((clock() - t) * 1000000) / CLOCKS_PER_SEC; //execution delay from the start of the function in microseconds
												  //substract it from theoretical delay between characters arriving to
                                                  //serial port at a given baudrate (bit per sec): 
                                                  // delay in seconds = 1/(baudrate/10bits); where 10bits = 1 start + 8 data + 1 stop
  //to avoid 100% CPU exhaustion, need to rest and share
  switch (baudRate) {
	case B4800: if (t < 2083) usleep(2083 - t); break;
	case B9600: if (t < 1042) usleep(1042 - t); break;
	case B19200: if (t < 520) usleep(520 - t); break;
	case B38400: if (t < 260) usleep(260 - t); break;
	case B57600: if (t < 174) usleep(174 - t); break;
	case B115200: if (t < 87) usleep(87 - t); break;
	case B230400: if (t < 43) usleep(43 - t); break;
  }
  return false;
}

void Gnss::poll(uint8_t msgClass, uint8_t msgId, uint16_t payload_length, uint8_t* pload) {
	pollMessageClass = msgClass;
	pollMessageId = msgId;
	pollPayload = pload;
	pollPayloadLength = payload_length;
	calculateChecksum(pollMessageClass, pollMessageId, pollPayloadLength, pollPayload);
	uint16_t packetSize = payload_length + 8;
	uint8_t* buffer = (uint8_t*)malloc(packetSize);
	if (buffer == NULL) {
		printf("poll(): malloc() error - buffer is null, packet size %u\n", packetSize); return;
	}
	buffer[0] = SYNC_CHARS[0];
	buffer[1] = SYNC_CHARS[1];
	buffer[2] = msgClass;
	buffer[3] = msgId;
	buffer[4] = payload_length & 0xFF;
	buffer[5] = payload_length >> 8;

	//printf("Sending... msgCls %X", pollMessageClass);
	//printf(", msgID %X", pollMessageId);
	//printf(", len0 %X", ttt);
	//printf(", len1 %X", ttt);
	//printf(", len %u", pollPayloadLength);
	if (pload != NULL)
	{
		//printf(", payload: ");
		for (uint16_t i = 0; i < payload_length; i++)
		{
			buffer[i + 6] = pload[i];
			//printf(" %X", pload[i]);
		}
	}
	//printf(", cksum0 %X", checksum[0]);
	//printf(", cksum1 %X\n", checksum[1]);
	buffer[payload_length + 6] = checksum[0];
	buffer[payload_length + 7] = checksum[1];
	if (write(fd, buffer, packetSize) != packetSize) printf("poll(): write() error %d\n", errno);
	tcdrain(fd);
	free(buffer);
}

bool Gnss::pollNoError(uint32_t timeout) {
	uint32_t starttime = time(NULL);
	while (difftime(time(NULL), starttime) < timeout) {
		if (ready() && messageClass == pollMessageClass && messageId == pollMessageId) {
			switch (error) {
				case NO_ERROR: return true;
				case CHECKSUM_ERROR: poll(pollMessageClass, pollMessageId, pollPayloadLength, pollPayload); printf("chksum err\n"); break;
				case OUT_OF_MEMORY: printf("out of memory\n"); return false;
			}
		}
	}
	printf("poll timeout\n");
	return false;
}

bool Gnss::ackNoError(uint32_t timeout) {
	uint32_t starttime = time(NULL);
	while (difftime(time(NULL), starttime) < timeout) {
		if (ready() && messageClass == UBX_ACK) {
			UbxAck* ack = (UbxAck*)(payload);
			switch (error) {
				case NO_ERROR:
					if (ack->classId == pollMessageClass && ack->messageId == pollMessageId) {
						if (messageId == ACK_ACK) return true;
						else return false;//ACK_NAK
					}
					else break;
				case CHECKSUM_ERROR: poll(pollMessageClass, pollMessageId, pollPayloadLength, pollPayload); printf("chksum err\n"); break;
				case OUT_OF_MEMORY: printf("out of memory\n"); return false;
			}
		}
	}
	printf("ack timeout\n");
	return false;
}

void Gnss::get() {
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
					case INF_DEBUG: sprintf(infLevel, "debug"); break;
					case INF_ERROR: sprintf(infLevel, "error"); break;
					case INF_NOTICE: sprintf(infLevel, "notice"); break;
					case INF_TEST: sprintf(infLevel, "test"); break;
					case INF_WARNING: sprintf(infLevel, "warning"); break;
					default: sprintf(infLevel, "unknown"); break;
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
					case NMEA_GBS: nmeaGbs(); break;
					case NMEA_DTM: nmeaDtm(); break;
					case NMEA_GRS: nmeaGrs(); break;
					case NMEA_TXT: nmeaTxt(); break;
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

void Gnss::navClock() {
  NavClock * p = (NavClock *)payload;
  printf("u-blox Clock: iTow %lums, bias %ldns, drift %ldns/s, tAcc %luns, fAcc %lups/s\n", p->iTOW, p->clkB, p->clkD, p->tAcc, p->fAcc);
}

void Gnss::navDgps() {
  DGPSCorrData * p = NULL;
  NavDGPS * dgps = (NavDGPS *)payload;
  
  for (uint8_t i = 0; i < dgps->numCh; i++) {
    p = (DGPSCorrData *)(payload + sizeof(NavDGPS) * i);
    if (p->flags.used) {
		printf("DGPS stationId %u, health %u, status %u: svid %u, channel %u, used %u, age %ums, prc %fm, prrc %fm/s\n", dgps->baseId, dgps->baseHealth, dgps->status, p->svid, p->flags.channel, p->flags.used, p->ageC, (double)(p->prc), (double)(p->prrc));		
	}
  }
}

void Gnss::navDop() {
	NavDOP * p = (NavDOP *)payload;
	printf("DOP iTOW %lu, gDOP %u, pDOP %u, tDOP %u, vDOP %u, hDOP %u, nDOP %u, eDOP %u 10^-2\n", p->iTOW, p->gDOP, p->pDOP, p->tDOP, p->vDOP, p->hDOP, p->nDOP, p->eDOP);	
}

void Gnss::navGeoFence() {
	NavGeofence * p = (NavGeofence *)payload;
	
	char g[8];

	switch(p->combState) {
		case UKNOWN_GEOFENCE_STATE: sprintf(g, "Unknown"); break;
		case INSIDE_GEOFENCE_STATE: sprintf(g, "Inside"); break;
		case OUTSIDE_GEOFENCE_STATE: sprintf(g, "Outside"); break;
	}
	printf("GeoFence iTOW %lu, status %u, numFences %u, combState %s\n", p->iTOW, p->status, p->numFences, g);
	
	for (uint8_t i = 0; i < p->numFences; i++) {
		switch(*(payload + sizeof(NavGeofence) + 2 * i)) {
			case UKNOWN_GEOFENCE_STATE: sprintf(g, "Unknown"); break;
			case INSIDE_GEOFENCE_STATE: sprintf(g, "Inside"); break;
			case OUTSIDE_GEOFENCE_STATE: sprintf(g, "Outside"); break;
		}
		printf("fenceId %u: state %s ", i, g);		
	}
	printf("\n");
}

void Gnss::navOdo() {
  NavODO * p = (NavODO *)payload;
  printf("navODO iTOW %lu, distance %lu +/- %lum, total %lum\n", p->iTOW, p->distance, p->distanceStd, p->totalDistance);
}

void Gnss::navOrb() {
	NavOrb * o = (NavOrb *)payload;
	OrbData * p = NULL;
	
	uint8_t u1, u2;
	for (uint8_t i = 0; i < o->numSv; i++) {
		p = (OrbData *)(payload + sizeof(NavOrb) + i * sizeof(OrbData));
		printf("gnssId %u svId %u, status ", p->gnssId, p->svId);
		switch(p->svFlags.health) {
			case UNKNOWN: printf("Unknown"); break;
			case HEALTHY: printf("Healthy"); break;
			case UNHEALTHY: printf("Unhealthy"); break;
		}
		printf("; pos ");
		switch(p->svFlags.visibility) {
			case UNKNOWN_VISIBILITY: printf("Unknown"); break;
			case BELOW_HIRIZON: printf("Above Horizon"); break;
			case ABOVE_HORIZON: printf("Below Horizon"); break;
			case ABOVE_ELEVATION_THRESHOLD: printf("Above Elevation Threshold"); break;
		}
		printf("; ephUsability ");
		switch(p->ephFlags.Usability) {
			case 31: printf("Unknown"); break;
			case 30: printf(">450min"); break;
			case 0: printf("Unused"); break;
			default:
				u1 = (p->ephFlags.Usability - 1) * 15;
				u2 = p->ephFlags.Usability * 15;
				printf("between %u and %u min", u1, u2); 
				break;
		}
		printf("; ephSource ");
		switch(p->ephFlags.Source) {
			case 0: printf("N/A"); break;
			case 1: printf("GNSS"); break;
			case 2: printf("Ext Aid"); break;
			default:
				printf("other");
				break;
		}
		printf("; almUsability ");
		switch(p->almFlags.Usability) {
			case 31: printf("Unknown"); break;
			case 30: printf(">450days"); break;
			case 0: printf("Unused"); break;
			default:
				u1 = (p->almFlags.Usability - 1) * 15;
				u2 = p->almFlags.Usability * 15;
				printf("between %u and %u days", u1, u2); 
				break;
		}
		printf("; almSource ");
		switch(p->almFlags.Source) {
			case 0: printf("N/A"); break;
			case 1: printf("GNSS"); break;
			case 2: printf("Ext Aid"); break;
			default:
				printf("other"); 
				break;
		}
		printf("; otherOrb ");
		switch(p->otherFlags.Usability) {
			case 31: printf("Unknown"); break;
			case 30: printf(">450days"); break;
			case 0: printf("Unused"); break;
			default:
				u1 = (p->otherFlags.Usability - 1) * 15;
				u2 = p->otherFlags.Usability * 15;
				printf("between %u and %u days", u1, u2); 
				break;
		}
		printf("; almSource ");
		switch(p->otherFlags.Source) {
			case 0: printf("N/A"); break;
			case 1: printf("AssistNowOffline"); break;
			case 2: printf("AssistNowAutonomous"); break;
			default:
				printf("other"); 
				break;
		}
		printf("\n");
	}
}

void Gnss::navPosLlh() {
	NavPosLlh * p = (NavPosLlh *)payload;
	
	Latitude lat;
    Longitude lon;
    longLatToDMS(&lat, p->lat);
    longLonToDMS(&lon, p->lon);
	printf("PosLLH iTOW %lu, %s %s +/- %ld, alt %ld, above MSL %ld +/- %ldm\n", p->iTOW, dmsLatToStr(&lat).c_str(), dmsLonToStr(&lon).c_str(), p->hAcc / 1000, p->alt / 1000, p->altMSL / 1000, p->vAcc / 1000);
}

void Gnss::navPvt() {
  NavPvt * navPVT = (NavPvt *)payload;
  if (navPVT->validTime.validDate && navPVT->validTime.validTime && navPVT->validTime.fullyResolved && pps) {
	char tt[32];
	time_t t = time(NULL);
	tm * time_tm = localtime(&t);
	strftime(tt, 32, "%c", time_tm);
	printf("NatPvt local time: %s\n", tt);

    pps = false;
	memset(time_tm, 0, sizeof(tm));
    utcTime = mktime(gps2tm(&(navPVT->dateTime), time_tm));
	/*struct timeval tv;
	tv.tv_sec = utcTime;
	tc.tv_usec = 0;
    settimeofday(&tv, NULL);*/ //this time is slightly in the past but will be corrected by pps() function 
	                          //if interrupt is attached to an arduino pin where u-blox pps output is connected
							  //alternatively, use timTp() message and set_system_time(utcTime) will be done in pps()
    char ft[9];
    switch (navPVT->fixType) {
      case NONE: sprintf(ft, "None"); break;
      case DR: sprintf(ft, "DR"); break;
      case TWO_D: sprintf(ft, "2D"); break;
      case THREE_D: sprintf(ft, "3D"); break;
      case GNSS_DR: sprintf(ft, "GNSS+DR"); break;
      case TIME_ONLY: sprintf(ft, "TimeOnly"); break;
    }
    char psm[19];
    switch (navPVT->flags.psmState) {
      case NOT_ACTIVE: sprintf(psm, "Not Active"); break;
      case ENABLED: sprintf(psm, "Enabled"); break;
      case ACQUISITION: sprintf(psm, "Acquisition"); break;
      case TRACKING: sprintf(psm, "Tracking"); break;
      case POWER_OPTIMIZED_TRACKING: sprintf(psm, "Optimized Tracking"); break;
      case INACTIVE: sprintf(psm, "Inactive"); break;
    }
    char carrSoln[6];
    switch (navPVT->flags.carrSoln) {
      case NONE: sprintf(carrSoln, "None"); break;
      case FLOAT: sprintf(carrSoln, "Float"); break;
      case FIXED: sprintf(carrSoln, "Fixed"); break;
    }
    Latitude lat;
    Longitude lon;
    longLatToDMS(&lat, navPVT->lat);
    longLonToDMS(&lon, navPVT->lon);
    string flags = "FixOK", yes = "yes", no = "no";
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
	printf(" %s %s +/- %lum\n", dmsLatToStr(&lat).c_str(), dmsLonToStr(&lon).c_str(), navPVT->hAcc / 1000);
	printf("iTOW %lu, %s +/- %luns, Fix: %s, psm %s, carrSol %s, SVs %u, alt %ldm above elipsoid, %ldm above MSL +/- %lum, pDOP %.2f, SOG %ld +/- %lum/s, heading %ld, motionHeading %ld +/- %ludeg\n", navPVT->iTOW, asctime(gmtime(&utcTime)), navPVT->tAcc, ft, psm, carrSoln, flags, navPVT->numSV, navPVT->alt / 1000, navPVT->altMSL / 1000, navPVT->altMSL / 1000, navPVT->vAcc / 1000, (double)(navPVT->pDOP / 100), navPVT->SOG / 1000, navPVT->sAcc / 1000, navPVT->heading / 100000, navPVT->motionHeading / 100000, navPVT->headAcc / 100000);
  }
}

void Gnss::navSat() {
  NavSat * navSat = (NavSat *)payload;
  printf("navSat iTOW %lu, numSVs %u\n", navSat->iTOW, navSat->numSvs);
  for (uint8_t i = 0; i < navSat->numSvs; i++) {
    SatInfo * satInfo = (SatInfo *)((uint8_t *)navSat + sizeof(NavSat) + i * sizeof(SatInfo));
    char sq[37];
    switch(satInfo->flags.signalQuality) {
      case NO_SIGNAL: sprintf(sq, "None"); break;
      case SEARCHING_SIGNAL: sprintf(sq, "Searching"); break;
      case SIGNAL_ACQUIRED: sprintf(sq, "Acquired"); break;
      case SIGNAL_DETECTED_BUT_UNUSABLE: sprintf(sq, "Detected but unusable"); break;
      case CODE_LOCKED_TIME_SYNCED: sprintf(sq, "Code locked and time synced"); break;
      default: sprintf(sq, "Code and carrier locked and time synced"); break;
    }
    char healthy[8];
    switch(satInfo->flags.health)
    {
      case UNKNOWN: sprintf(healthy, "unknown"); break;
      case HEALTHY: sprintf(healthy, "Yes"); break;
      case UNHEALTHY: sprintf(healthy, "No"); break;
    }
    char os[22];
    switch (satInfo->flags.orbitSource)
    {
      case NONE: sprintf(os, "None"); break;
      case EPHEMERIS: sprintf(os, "Ephimeris"); break;
      case ALMANAC: sprintf(os, "Almanac"); break;
      case ASSIST_NOW_OFFLINE: sprintf(os, "Assist Now Offline"); break;
      case ASSIST_NOW_AUTONOMOUS: sprintf(os, "Assist Now Autonomous"); break;
      default: sprintf(os, "Other"); break;
    }
    char str[255];
	printf("gnssId %u, svId %u, cno %u, elev %ddeg, azim %ddeg, prRes %dm, signalQuality %s, used %u, healthy %s, diffCorr %u, smoothed %u, orbitSrc %s, ephAvail %u, almAvail %u, anoAvail %u, aopAvail %u, sbas %u, rtcm %u, slas %u, pr %u, cr %u, do %u\n", satInfo->gnssId, satInfo->svId, satInfo->cno, satInfo->elev, satInfo->azim, satInfo->prRes, sq, satInfo->flags.svUsed, healthy, satInfo->flags.diffCorr, satInfo->flags.smoothed, os, satInfo->flags.ephAvail, satInfo->flags.almAvail, satInfo->flags.anoAvail, satInfo->flags.aopAvail, satInfo->flags.sbasCorrUsed, satInfo->flags.rtcmCorrUsed, satInfo->flags.slasCorrUsed, satInfo->flags.prCorrUsed, satInfo->flags.crCorrUsed, satInfo->flags.doCorrUsed);
  }
}

void Gnss::navSbas() {
	SbasSv * sv = NULL;
	NavSbas * p = (NavSbas *)payload;
	char m[9], g[8], r[40];
	memset(m, 0, 9);
	memset(g, 0, 8);
	memset(r, 0, 40);
	switch (p->mode) {
		case 0: sprintf(m, "disabled"); break;
		case 1: sprintf(m, "enabled"); break;
		case 3: sprintf(m, "testing"); break;
	}
	switch (p->sys) {
		case -1: sprintf(g, "unknown"); break;
		case WAAS: sprintf(g, "WAAS"); break;
		case EGNOS: sprintf(g, "EGNOS"); break;
		case MSAS: sprintf(g, "MSAS"); break;
		case GAGAN: sprintf(g, "GAGAN"); break;
		case GPS_SYS: sprintf(g, "GPS"); break;
	}
	if (p->services.ranging) strcat(r, "ranging");
	if (p->services.corrections) strcat(r, "|corrections");
	if (p->services.integrity) strcat(r, "|integrity");
	if (p->services.testMode) strcat(r, "|testMode");
	if (r[0] == 0) sprintf(r, "N/A");
	printf("navSbas: iTOW %lu, geo %u, mode %s, sys %s, service %s: \n", p->iTOW, p->geo, m, g, r);
	
	for (uint8_t i = 0; i < p->cnt; i++) {
		sv = (SbasSv *)(payload + sizeof(NavSbas) + i * sizeof(SbasSv));
		switch (sv->sys) {
			case WAAS: sprintf(g, "WAAS"); break;
			case EGNOS: sprintf(g, "EGNOS"); break;
			case MSAS: sprintf(g, "MSAS"); break;
			case GAGAN: sprintf(g, "GAGAN"); break;
			case GPS_SYS: sprintf(g, "GPS"); break;
		}
		if (sv->services.ranging) strcat(r, "ranging");
		if (sv->services.corrections) strcat(r, "|corrections");
		if (sv->services.integrity) strcat(r, "|integrity");
		if (sv->services.testMode) strcat(r, "|testMode");
		if (r[0] == 0) sprintf(r, "N/A");
		printf("svId %u, flags %u, status %u, sys %s, service %s, pseudoRange %dcm, ionspherCorr %dcm\n", sv->svId, sv->flags, sv->status, g, r, sv->prc, sv->ic);		
	}
}

void Gnss::navSlas() {
	SlasSv * sv = NULL;
	NavSlas * p = (NavSlas *)payload;
	char f[40];
	memset(f, 0, 40);
	if (p->flags.gmsAvailable)  strcat(f, "gmsAvailable");
	if (p->flags.qzssSvAvailable) strcat(f, "|qzssSvAvailable");
	if (p->flags.testMode) strcat(f, "|testMode");
	printf("navSlas: iTOW %lu, gmsLon %ld x 10^-3 deg, gmsLat %ld x 10^-3 deg, code %u, svId %u, flags %s\n", p->iTOW, p->gmsLon, p->gmsLat, p->gmsCode, p->gzssSvId, f);
	for (uint8_t i = 0; i < p->cnt; i++) {
		sv = (SlasSv *)(payload + sizeof(NavSlas) + i * sizeof(SlasSv));
		printf("gnssId %u, svId %u, prc %ucm\n", sv->gnssId, sv->svId, sv->prc);
	}
}

void Gnss::navTimeBds() {
	NavTimeBdsGal * p = (NavTimeBdsGal *)payload;
	char f[32];
	memset(f, 0, 32);
	if (p->flags.towValid) strcat(f, "towValid");
	if (p->flags.weekValid) strcat(f, "|weekValid");
	if (p->flags.leapSecValid) strcat(f, "|leapSecValid");
	printf("timeBDS: iTOW %lums, tow %lu.%lds +/- %luns, weeks %d, leapSec %d, flags %s", p->iTOW, p->tow, p->fTow, p->tAcc, p->weeks, p->leapSec, f);
	
}

void Gnss::navTimeGal() {
	NavTimeBdsGal * p = (NavTimeBdsGal *)payload;
	char f[32];
	memset(f, 0, 32);
	if (p->flags.towValid) strcat(f, "towValid");
	if (p->flags.weekValid) strcat(f, "|weekValid");
	if (p->flags.leapSecValid) strcat(f, "|leapSecValid");
	printf("timeGal: iTOW %lums, tow %lu.%lds +/- %luns, weeks %d, leapSec %d, flags %s\n", p->iTOW, p->tow, p->fTow, p->tAcc, p->weeks, p->leapSec, f);
}

void Gnss::navTimeGlo() {
	NavTimeGlo * p = (NavTimeGlo *)payload;
	char f[20];
	memset(f, 0, 20);
	uint16_t day = p->day, doy = day % 365;
	uint16_t year = 1992 + 4 * p->year + day / 365;
	if (p->flags.todValid) strcat(f, "todValid");
	if (p->flags.dateValid) strcat(f, "|dateValid");
	printf("timeGlo: iTOW %lums, tod %lu.%lds +/- %luns, dayOfYear %u, year %u, flags %s\n", p->iTOW, p->tod, p->fTod, p->tAcc, doy, year, f);
}

void Gnss::navTimeGps() {
	NavTimeGps * p = (NavTimeGps *)payload;
	char f[32];
	memset(f, 0, 32);
	if (p->flags.towValid) strcat(f, "towValid");
	if (p->flags.weekValid) strcat(f, "|weekValid");
	if (p->flags.leapSecValid) strcat(f, "|leapSecValid");
	printf("timeGps: iTOW %lu.%lds +/- %luns, weeks %d, leapSec %d, flags %s\n", p->iTOW / 1000, p->fTow, p->tAcc, p->weeks, p->leapSec, f);
}

void Gnss::navTimeLs() {
	NavTimeLs * p = (NavTimeLs *)payload;
	char g[17], cs[10];
	memset(g, 0, 17);
	memset(cs, 0, 10);
	switch (p->source) {
		case DEFAULT_LSS: sprintf(g, "Default"); break;
		case GPS_GLONASS_DIFF: sprintf(g, "GPS_GLONASS_DIFF"); break;
		case GPS_LSS: sprintf(g, "GPS"); break;
		case SBAS_LSS: sprintf(g, "SBAS"); break;
		case BEIDOU_LSS: sprintf(g, "BDS"); break;
		case GALILEO_LSS: sprintf(g, "Galileo"); break;
		case AIDED_DATA: sprintf(g, "AIDED_DATA"); break;
		case CONFIGURED_LSS: sprintf(g, "CONFIGURED"); break;
		case UNKNOWN_LSS: sprintf(g, "UNKNOWN"); break;
	}
	switch (p->changeSource) {
		case NO_SOURCE_LSS: sprintf(cs, "NO_SOURCE"); break;
		case GPS_CHANGE_LSS: sprintf(cs, "GPS"); break;
		case SBAS_CHANGE_LSS: sprintf(cs, "SBAS"); break;
		case BEIDOU_CHANGE_LSS: sprintf(cs, "BDS"); break;
		case GALILEO_CHANGE_LSS: sprintf(cs, "Galileo"); break;
		case GLONAS_CHANGE_LSS: sprintf(cs, "GLONASS"); break;
	}
	printf("LeapSecond: iTOW %lu, currLS %u, src %s, changeSrc %s, lsChange %d, timeToLs %ld, week %u, day %u\n", p->iTOW, p->leapSec, g, cs, p->lsChange, p->timeToLs, p->gpsWeek, p->gpsDay);
}

void Gnss::navTimeUtc() {
  NavTimeUtc * t = (NavTimeUtc *)payload;  
  if (t->valid.validTOW && t->valid.validWKN && t->valid.validUTC) {
    struct tm tm_time;
	char tt[32];

	time_t t1 = mktime(gps2tm(&(t->dateTime), &tm_time));
	tm* time_tm = localtime(&t1);
	strftime(tt, 32, "%c", time_tm);

	char src[8];
	memset(src, 0, 8);
    switch (t->valid.utcSource)
    {
      case NO_INFO: sprintf(src, "NO_INFO"); break;
      case CRL: sprintf(src, "CRL"); break;
      case NIST: sprintf(src, "NIST"); break;
      case USNO: sprintf(src, "USNO"); break;
      case BIPM: sprintf(src, "BIPM"); break;
      case EUROLAB: sprintf(src, "EUROLAB"); break;
      case SU: sprintf(src, "SU"); break;
      case NTSC: sprintf(src, "NTSC"); break;
      case UNKNOWN_SOURCE: sprintf(src, "UNKNOWN"); break;
    }
	printf("UTC: %s, source %s\n", tt, src);
    
  }
}

void Gnss::cfgGnss() {
  GnssConf * gnssConf = (GnssConf *)payload;
  GnssCfg * cfg = NULL;
  
  printf("GnssConf: u-blox numTrkChHw %u, numTrkChUse %u\n", gnssConf->numTrkChHw, gnssConf->numTrkChUse);
  char gnss_id[8], on_off[9];
  memset(gnss_id, 0, 8);
  memset(on_off, 0, 9);
  for (uint8_t i = 0; i < gnssConf->numConfigBlocks; i++) {
	cfg = (GnssCfg *)(payload + sizeof(GnssConf) + sizeof(GnssCfg) * i);
    switch (cfg->gnssId) {
      case GPS_ID: sprintf(gnss_id, "GPS"); break;
      case SBAS_ID: sprintf(gnss_id, "SBAS"); break;
      case Galileo_ID: sprintf(gnss_id, "Galileo"); break;
      case BeiDou_ID: sprintf(gnss_id, "BeiDou"); break;
      case IMES_ID: sprintf(gnss_id, "IMES"); break;
      case QZSS_ID: sprintf(gnss_id, "QZSS"); break;
      case GLONASS_ID: sprintf(gnss_id, "GLONASS"); break;
    }
    if (cfg->flags.enabled) sprintf(on_off, "enabled"); 
	else sprintf(on_off, "disabled");
	printf("GNSS id %s, minCh %u, maxCh %u: %s\n", gnss_id, cfg->minTrkCh, cfg->maxTrkCh, on_off);
  }
}

void Gnss::cfgInf() {
  InfoMsgMask * mask = NULL;
  char pr[5], po[8];
  memset(pr, 0, 5);
  memset(po, 0, 8);
  for (uint8_t i = 0; i < 6; i++) {
	  mask = (InfoMsgMask *)&(payload[i + 4]);
      switch((uint8_t)payload[0]) {
		case UBX: sprintf(pr, "UBX"); break;
		case NMEA: sprintf(pr, "NMEA"); break;
	  }
	  switch(i) {
		case DDC: sprintf(po, "DDC"); break;
		case COM1: sprintf(po, "COM1"); break;
		case COM2: sprintf(po, "COM2"); break;
		case USB: sprintf(po, "USB"); break;
		case SPI: sprintf(po, "SPI"); break;
		case RES: sprintf(po, "RES"); break;
		default: sprintf(po, "UNKNOWN"); break;
	  }
	  printf("cfgInfoMask: %s %s INF-MSG mask: err %u, warn %u, info %u, test %u, debug %u\n", po, pr, mask->error, mask->warning, mask->notice, mask->test, mask->debug);
  }
}

void Gnss::cfgLogFilter() {
  CfgLogFilter * filter = (CfgLogFilter *)payload;
  char flags[70];
  memset(flags, 0, 70);
  if (filter->flags.recordingEnabled) strcat(flags, "recEnabled");   
  if (filter->flags.psmOncePerWakeUpEnabled) strcat(flags, "|psmOncePerWakeUp Enabled");   
  if (filter->flags.applyAllFilterSettings) strcat(flags, "|applyAllFilterSettings"); 
  printf("cfgLogFilter: minInterval %us, timeThreshold %us, speedThreshold %um/s, positionThreshold %um, flags %s\n", filter->minInterval, filter->timeThreshold, filter->speedThreshold, filter->positionThreshold, flags);
    
}

void Gnss::cfgMsg() {
  char ids[9], cl[8], po[5];
  memset(ids, 0, 9);
  memset(cl, 0, 8);
  memset(po, 0, 5);
  if (payloadLength == 8) {
	CfgMsg * msg = (CfgMsg *)payload;
	switch(msg->msgClass) {
	case UBX_NAV: 
		sprintf(cl, "UBX-NAV"); 
		switch(msg->msgId) {
			case NAV_CLOCK: sprintf(ids, "CLOCK"); break;
			case NAV_DGPS: sprintf(ids, "DGPS"); break;
			case NAV_DOP: sprintf(ids, "DOP"); break;
			case NAV_EOE: sprintf(ids, "EOE"); break;
			case NAV_GEOFENCE: sprintf(ids, "GEOFENCE"); break;
			case NAV_ODO: sprintf(ids, "ODO"); break;
			case NAV_ORB: sprintf(ids, "ORB"); break;
			case NAV_POSLLH: sprintf(ids, "POSLLH"); break;
			case NAV_RESETODO: sprintf(ids, "RESETODO"); break;
			case NAV_SAT: sprintf(ids, "SAT"); break;
			case NAV_SBAS: sprintf(ids, "SBAS"); break;
			case NAV_SLAS: sprintf(ids, "SLAS"); break;
			case NAV_TIMEBDS: sprintf(ids, "TIMEBDS"); break;
			case NAV_TIMEGAL: sprintf(ids, "TIMEGAL"); break;
			case NAV_TIMEGLO: sprintf(ids, "TIMEGLO"); break;
			case NAV_TIMEGPS: sprintf(ids, "TIMEGPS"); break;
			case NAV_TIMELS: sprintf(ids, "TIMELS"); break;
			case NAV_TIMEUTC: sprintf(ids, "TIMEUTC"); break;
			default: sprintf(ids, "%X", msg->msgId);
		}
		break;
	case UBX_TIM: 
		sprintf(cl, "UBX-TIM"); 
		switch(msg->msgId) {
			case TIM_TM2: sprintf(ids, "TM2"); break;
			case TIM_TP: sprintf(ids, "TP"); break;
			default: sprintf(ids, "%X", msg->msgId);
		}
		break;
	case UBX_NMEA: 
		sprintf(cl, "NMEA"); 
		switch(msg->msgId) {
			case NMEA_RMC: sprintf(ids, "RMC"); break;
			case NMEA_VTG: sprintf(ids, "VTG"); break;
			case NMEA_GGA: sprintf(ids, "GGA"); break;
			case NMEA_GSA: sprintf(ids, "GSA"); break;
			case NMEA_GSV: sprintf(ids, "GSV"); break;
			case NMEA_GLL: sprintf(ids, "GLL"); break;
			case NMEA_GST: sprintf(ids, "GST"); break;
			case NMEA_VLW: sprintf(ids, "VLW"); break;
			case NMEA_GNS: sprintf(ids, "GNS"); break;
			case NMEA_ZDA: sprintf(ids, "ZDA"); break;
			default: sprintf(ids, "%X", msg->msgId);
		}
		break;
	case UBX_PUBX: 
		sprintf(cl, "PUBX"); 
		switch(msg->msgId) {
			case PUBX_CONFIG: sprintf(ids, "CONFIG"); break;
			case PUBX_POSITION: sprintf(ids, "POSITION"); break;
			case PUBX_SVSTATUS: sprintf(ids, "SVSTATUS"); break;
			case PUBX_TIME: sprintf(ids, "TIME"); break;
			default: sprintf(ids, "%X", msg->msgId);
		}
		break;
		default: sprintf(cl, "%X", msg->msgClass);
	}
	for (uint8_t portId = 0; portId < 5; portId++) {
		switch(portId) {
		case DDC: sprintf(po, "DDC"); break;
		case COM1: sprintf(po, "COM1"); break;
		case COM2: sprintf(po, "COM2"); break;
		case USB: sprintf(po, "USB"); break;
		case SPI: sprintf(po, "SPI"); break;
		case RES: sprintf(po, "RES"); break;
		}
		printf("%s cfgMsg %s-%s: rate per navSol %u\n", po, cl, ids, msg->rate[portId]);
		
	}
  } else {
	CfgMsgCOM1 * msg = (CfgMsgCOM1 *)payload;
	switch(msg->msgClass) {
	case UBX_NAV: 
		sprintf(cl, "UBX-NAV"); 
		switch(msg->msgId) {
			case NAV_CLOCK: sprintf(ids, "CLOCK"); break;
			case NAV_DGPS: sprintf(ids, "DGPS"); break;
			case NAV_DOP: sprintf(ids, "DOP"); break;
			case NAV_EOE: sprintf(ids, "EOE"); break;
			case NAV_GEOFENCE: sprintf(ids, "GEOFENCE"); break;
			case NAV_ODO: sprintf(ids, "ODO"); break;
			case NAV_ORB: sprintf(ids, "ORB"); break;
			case NAV_POSLLH: sprintf(ids, "POSLLH"); break;
			case NAV_RESETODO: sprintf(ids, "RESETODO"); break;
			case NAV_SAT: sprintf(ids, "SAT"); break;
			case NAV_SBAS: sprintf(ids, "SBAS"); break;
			case NAV_SLAS: sprintf(ids, "SLAS"); break;
			case NAV_TIMEBDS: sprintf(ids, "TIMEBDS"); break;
			case NAV_TIMEGAL: sprintf(ids, "TIMEGAL"); break;
			case NAV_TIMEGLO: sprintf(ids, "TIMEGLO"); break;
			case NAV_TIMEGPS: sprintf(ids, "TIMEGPS"); break;
			case NAV_TIMELS: sprintf(ids, "TIMELS"); break;
			case NAV_TIMEUTC: sprintf(ids, "TIMEUTC"); break;
			default: sprintf(ids, "%X", msg->msgId);
		}
		break;
	case UBX_TIM: 
		sprintf(cl, "UBX-TIM"); 
		switch(msg->msgId) {
			case TIM_TM2: sprintf(ids, "TM2"); break;
			case TIM_TP: sprintf(ids, "TP"); break;
			default: sprintf(ids, "%X", msg->msgId);
		}
		break;
	case UBX_NMEA: 
		sprintf(cl, "NMEA"); 
		switch(msg->msgId) {
			case NMEA_RMC: sprintf(ids, "RMC"); break;
			case NMEA_VTG: sprintf(ids, "VTG"); break;
			case NMEA_GGA: sprintf(ids, "GGA"); break;
			case NMEA_GSA: sprintf(ids, "GSA"); break;
			case NMEA_GSV: sprintf(ids, "GSV"); break;
			case NMEA_GLL: sprintf(ids, "GLL"); break;
			case NMEA_GST: sprintf(ids, "GST"); break;
			case NMEA_VLW: sprintf(ids, "VLW"); break;
			case NMEA_GNS: sprintf(ids, "GNS"); break;
			case NMEA_ZDA: sprintf(ids, "ZDA"); break;
			default: sprintf(ids, "%X", msg->msgId);
		}
		break;
	case UBX_PUBX: 
		sprintf(cl, "PUBX"); 
		switch(msg->msgId) {
			case PUBX_CONFIG: sprintf(ids, "CONFIG"); break;
			case PUBX_POSITION: sprintf(ids, "POSITION"); break;
			case PUBX_SVSTATUS: sprintf(ids, "SVSTATUS"); break;
			case PUBX_TIME: sprintf(ids, "TIME"); break;
			default: sprintf(ids, "%X", msg->msgId);
		}
		break;
		default: sprintf(cl, "%X", msg->msgClass);
	}
    printf("cfgMsg %s-%s: rate per navSol %u\n", cl, ids, msg->rate);
	
  }
}

void Gnss::cfgNav() {
  CfgNav * cfg = (CfgNav *)payload;
  char dyn[12];
  memset(dyn, 0, 12);
  switch (cfg->dynModel) {
        case PORTABLE: sprintf(dyn, "PORTABLE"); break;
        case STATIONARY: sprintf(dyn, "STATIONARY"); break;
        case PEDESTRIAN: sprintf(dyn, "PEDESTRIAN"); break;
        case AUTOMOTIVE: sprintf(dyn, "AUTOMOTIVE"); break;
        case SEA: sprintf(dyn, "SEA"); break;
        case AIRBORNE_1G: sprintf(dyn, "AIRBORNE_1G"); break;
        case AIRBORNE_2G: sprintf(dyn, "AIRBORNE_2G"); break;
        case AIRBORNE_4G: sprintf(dyn, "AIRBORNE_4G"); break;
        case WATCH: sprintf(dyn, "WATCH"); break;
        case MODEL_ERROR: sprintf(dyn, "MODEL_ERROR"); break;
  }
  
  char utc[10];
  memset(utc, 0, 10);
  switch (cfg->utcStandard) {
    case AUTOMATIC: sprintf(utc, "AUTOMATIC"); break;
    case GPS: sprintf(utc, "GPS"); break;
    case GLONASS: sprintf(utc, "GLONASS"); break;
    case BEIDOU: sprintf(utc, "BEIDOU"); break;
    case UTC_ERROR: sprintf(utc, "UTC_ERROR"); break;
  }

  char fm[6];
  memset(fm, 0, 6);
  switch(cfg->fixMode) {
  	  case TWO_D_ONLY: sprintf(fm, "2D"); break;
	  case THREE_D_ONLY: sprintf(fm, "3D"); break;
	  case AUTO: sprintf(fm, "AUTO"); break;
	  case FIX_MODE_ERROR: sprintf(fm, "ERROR"); break;
  }
  printf("cfgNav: dynModel %s, utcStandard %s, fixMode %s, fixedAlt %ldm, variance %lum2, pDOP %u, tDOP %u, pAcc %um, tAcc %ums?\n", dyn, utc, fm, cfg->fixedAlt / 100, cfg->fixedAltVar / 10000, cfg->pDOP / 10, cfg->tDOP / 10, cfg->pAcc);
  printf("cfgNav: cnoThreshold %udBHz, NumSVs %u, staticHoldThreshold %ucm/s, MaxDist %um, minElev %udeg above hor, dgnssTimeout %us\n", cfg->cnoThreshold, cfg->cnoThresholdNumSVs, cfg->staticHoldThreshold, cfg->staticHoldMaxDistance, cfg->minElev, cfg->dgnssTimeout); 
}

void Gnss::cfgOdo() {
  ODOCfg * cfg = (ODOCfg *)payload; //getCfgOdo();
  char flags[255], profile[9], enabled[8], disabled[9];
  memset(flags, 0, 255);
  memset(profile, 0, 9);
  memset(enabled, 0, 8);
  memset(disabled, 0, 9);

  sprintf(enabled, "enabled");
  sprintf(disabled, "disabled");
  sprintf(flags, "ODO ");
  if (cfg->flags.ODOenabled) strcat(flags, enabled); else strcat(flags, disabled);
  strcat(flags, "|low speed COG filter ");
  if (cfg->flags.COGenabled) strcat(flags, enabled); else strcat(flags, disabled);
  strcat(flags, "|low speed (< 5m/s) filter ");
  if (cfg->flags.outputLPvelocity) strcat(flags, enabled); else strcat(flags, disabled);
  strcat(flags, "|2D headingOfMotion ");
  if (cfg->flags.outputLPcog) strcat(flags, enabled); else strcat(flags, disabled);

  switch (cfg->odoProfile) {
    case RUNNING: sprintf(profile, "RUNNING"); break;
    case CYCLING: sprintf(profile, "CYCLING"); break;
    case SWIMMING: sprintf(profile, "SWIMMING"); break;
    case DRIVING: sprintf(profile, "DRIVING"); break;
    case CUSTOM: sprintf(profile, "CUSTOM"); break;
  }
  printf("ODOcfg - flags: %s, profile %s, COGlowSpeedFilterThresh %um/s, COGmaxAcceptablePosAccuracy %um, velLPgain %u cogLPgain %u\n", flags, profile, cfg->cogMaxSpeed, cfg->cogMaxPosAccuracy, cfg->velLPgain, cfg->cogLPgain);
}

void Gnss::cfgPm() {
	CfgPm * p = (CfgPm *)payload;
	char flags[128];
	memset(flags, 0, 128);
	if (p->flags.extintSel) strcat(flags, "EXTINT1"); else strcat(flags, "EXTINT0");
	if (p->flags.extintWake) strcat(flags, "|Wake");
	if (p->flags.extintBackup) strcat(flags, "|Backup");
	if (p->flags.extintInactivity) strcat(flags, "|Inactivity");
	if (p->flags.limitPeakCurr) strcat(flags, "|limitPeakCurr");
	if (p->flags.waitTimeFix) strcat(flags, "|waitTimeFix");
	if (p->flags.updateRTC) strcat(flags, "|updateRTC");
	if (p->flags.updateEph) strcat(flags, "|updateEph");
	if (p->flags.doNotEnterOff) strcat(flags, "|doNotEnterOff");
	if (p->flags.mode) strcat(flags, "|PSMCT"); else strcat(flags, "|PSMOO");
	printf("CfgPm: maxStartupStateDuration %us, updatePeriod %lums, searchPeriod %lums, gridOffset %lums, onTime %us, minAcqTime %us, extintInactivity %lums, flags %s\n", p->maxStartupStateDuration, p->updatePeriod, p->searchPeriod, p->gridOffset, p->onTime, p->minAcqTime, p->extintInactivity, flags);
}

void Gnss::cfgPms() {
  PowerMode * mode = (PowerMode *)payload;
  char pwr[9];
  memset(pwr, 0, 9);

  switch (mode->powerMode)
  {
    case FULL_POWER_MODE: sprintf(pwr, "FULL"); break;
    case BALANCED_POWER_MODE: sprintf(pwr, "BALANCED"); break;
    case INTERVAL_POWER_MODE: sprintf(pwr, "INTERVAL"); break;
    case AGGRESSIVE_1HZ_POWER_MODE: sprintf(pwr, "AGGR_1HZ"); break;
    case AGGRESSIVE_2HZ_POWER_MODE: sprintf(pwr, "AGGR_2HZ"); break;
    case AGGRESSIVE_4HZ_POWER_MODE: sprintf(pwr, "AGGR_4HZ"); break;
    case INVALID_POWER_MODE: sprintf(pwr, "INVALID"); break;
  }
  printf("powerMode %s, period %us, onTime %us\n", pwr, mode->period, mode->onTime); 
}

void Gnss::cfgPrt() {
  CfgPrt * cfg = (CfgPrt *)payload;
  char id[5], mode[64], clen[2], par[5], sbits[4], inProto[14], outProto[9];
  memset(id, 0, 5);
  memset(mode, 0, 64);
  memset(clen, 0, 2);
  memset(par, 0, 5);
  memset(sbits, 0, 4);
  memset(inProto, 0, 14);
  memset(outProto, 0, 9);

  switch (cfg->portId) {
	case DDC: sprintf(id, "DDC"); break;
	case COM1: sprintf(id, "COM1"); break;
	case COM2: sprintf(id, "COM2"); break;
	case USB: sprintf(id, "USB"); break;
	case SPI: sprintf(id, "SPI"); break;
	case RES: sprintf(id, "RES"); break;
  }
  switch (cfg->mode.charLen) {
     case FIVE_BIT: sprintf(clen, "5"); break;
     case SIX_BIT: sprintf(clen, "6"); break;
     case SEVEN_BIT: sprintf(clen, "7"); break;
     case EIGHT_BIT: sprintf(clen, "8"); break;
  }
  switch (cfg->mode.parity) {
    case PARITY_EVEN: sprintf(par, "even"); break;
    case PARITY_ODD: sprintf(par, "odd"); break;
    case NO_PARITY: sprintf(par, "no"); break;
    case NO_PARITY2: sprintf(par, "no"); break;
  }
  switch (cfg->mode.nStopBits) {
    case ONE_STOP_BIT: sprintf(sbits, "1"); break;
    case ONE_AND_HALF_STOP_BIT: sprintf(sbits, "1.5"); break;
    case TWO_STOP_BIT: sprintf(sbits, "2"); break;
    case HALF_STOP_BIT: sprintf(sbits, "0.5"); break;
  }
  if (cfg->inProtoMask.inUbx) sprintf(inProto, "Ubx");
  if (cfg->inProtoMask.inNmea) strcat(inProto, "|Nmea");
  if (cfg->inProtoMask.inRtcm) strcat(inProto, "|Rtcm");
  if (cfg->outProtoMask.outUbx) sprintf(outProto, "Ubx");
  if (cfg->outProtoMask.outNmea) strcat(outProto, "|Nmea");
  
  sprintf(mode, "%s bit|%s parity|%s stop bit", clen, par, sbits);
  printf("cfgPrt: %s, rate %lu baud, mode %s, inProto %s, outProto %s\n", id, (uint32_t)(cfg->baudRate), mode, inProto, outProto); 
}

//UBX-CFG-PWR is deprecated in protocols > 17
void Gnss::cfgPwr() {
}

void Gnss::cfgRate() {
  NavRate * rate = (NavRate *)payload;
  char timeRef[8];
  memset(timeRef, 0, 8);
  switch (rate->timeRef)
  {
    case UTC_TIME: sprintf(timeRef, "UTC"); break;
    case GPS_TIME: sprintf(timeRef, "GPS"); break;
    case GLONASS_TIME: sprintf(timeRef, "GLONASS"); break;
    case BEIDOU_TIME: sprintf(timeRef, "BEIDOU"); break;
    case GALILEO_TIME: sprintf(timeRef, "GALILEO"); break;
  }
  printf("cfgRate: navRate %ums, measurementsPerNavSol %u, timeRef %s\n", rate->rate, rate->navSolRate, timeRef);
}

void Gnss::cfgRxm() {
	CfgRxm * p = (CfgRxm *)(payload);
	printf("cfgRxm mode: ");
	switch(p->lpMode) {
		case CONTINUOUS_POWER: printf("CONTINUOUS_POWER\n"); break;
		case POWER_SAVE_MODE: printf("POWER_SAVE_MODE\n"); break;
		case CONTINUOUS_MODE: printf("CONTINUOUS_MODE\n"); break;
	}	
}

void Gnss::cfgSbas() {
	CfgSbas * p = (CfgSbas *)payload;
	char str[64];
	memset(str, 0, 64);
	printf("cfgSbas mode ");
	if (p->mode.enabled) strcat(str, "enabled"); else strcat(str, "disabled");
	if (p->mode.testMode) strcat(str, "|testMode");
	strcat(str, ", usage ");
	if (p->usage.range) strcat(str, "range");
	if (p->usage.diffCorr) strcat(str, "|diffCorr");
	if (p->usage.integrity) strcat(str, "|integrity");
	printf("%s, maxSbas %u, scanMode1 %lu, scanMode2 %u\n", str, p->maxSbas, p->scanMode1, p->scanMode2);
}

//UBX-CFG-SLAS Not supported in protocols < 19.2
void Gnss::cfgSlas() {
}

void Gnss::cfgTp() {
  TimePulse * tp = (TimePulse *)payload;
  char flags[255], fp[3], lc[6];
  memset(flags, 0, 255);
  memset(fp, 0, 3);
  memset(lc, 0, 6);
  sprintf(fp, "us");
  sprintf(lc, "2^-32");
  TimePulseFlags f = tp->flags;
  if (f.active) strcat(flags, "active");
  if (f.lockGnssFreq) strcat(flags, "|lockGnssFreq");
  if (f.lockedOtherSet) strcat(flags, "|lockedOtherSet");
  if (f.isFreq) { strcat(flags, "|isFreq"); sprintf(fp, "Hz"); } else strcat(flags, "|isPeriod");
  if (f.isLength) { strcat(flags, "|isLength"); sprintf(lc, "us"); } else strcat(flags, "|Cycles");
  if (f.alignToTOW) strcat(flags, "|alignToTOW");
  if (f.polarity) strcat(flags, "|rising_edge"); else strcat(flags, "|falling_edge");
  switch(f.gridUtcGnss)
  {
    case UTC_TIME: strcat(flags, "|UTC_TIME"); break;
    case GPS_TIME: strcat(flags, "|GPS_TIME"); break;
    case GLONASS_TIME: strcat(flags, "|GLONASS_TIME"); break;
    case BEIDOU_TIME: strcat(flags, "|BEIDOU_TIME"); break;
    case GALILEO_TIME: strcat(flags, "|GALILEO_TIME"); break;
  }
  printf("timePulse id %u, cableDelay %luns, rfDelay %luns, pulsePeriod %lu%s, pulsePeriodLocked %lu%s, pulseLen %lu%s, pulseLenLocked %lu%s, delay %luns, flags %s\n", tp->pulseId, tp->antCableDelay, tp->rfGroupDelay, tp->pulsePeriod, fp, tp->pulsePeriodLocked, fp, tp->pulseLen, lc, tp->pulseLenLocked, lc, tp->userConfigDelay, flags);
}

void Gnss::cfgGeoFence() {
	GeoFences * p = (GeoFences *)payload;
	
	GeoFence * f = NULL;
	Latitude lat;
    Longitude lon;

	printf("cfgGeofences confLvl %u, pioEnabled %u, pinPolarity %u, pin %u\n", p->confidenceLevel, p->pioEnabled, p->pinPolarity, p->pin);
	
	for (uint8_t i = 0; i < p->numFences; i++) {
		f = (GeoFence *)(payload + sizeof(GeoFences) + i * sizeof(GeoFence));
	    longLatToDMS(&lat, f->lat);
		longLonToDMS(&lon, f->lon);
		printf("%u: %s %s radius %lum\n", i, dmsLatToStr(&lat).c_str(), dmsLonToStr(&lon).c_str(), f->radius / 100);
	}
}

void Gnss::cfgNmea() {
	CfgNmea * p = (CfgNmea *)payload;
	char talkerId[14], gsv[8], filter[128], flags[96], gnss[96];
	memset(talkerId, 0, 14);
	memset(gsv, 0, 8);
	memset(filter, 0, 128);
	memset(flags, 0, 96);
	memset(gnss, 0, 96);

	if (p->filter.failedFix) strcat(filter, "dispFailedFix");
	if (p->filter.invalidFix) strcat(filter, "|dispInvalidFix");
	if (p->filter.invalidTime) strcat(filter, "|dispInvalidTime");
	if (p->filter.invalidDate) strcat(filter, "|dispInvalidDate");
	if (p->filter.invalidCog) strcat(filter, "|dispInvalidCOG");
	if (p->filter.gpsOnly) strcat(filter, "|dispGPSonly");
	if (filter[0] == 0) sprintf(filter, "none");
	if (p->flags.compat) strcat(flags, "compatMode");
	if (p->flags.consider) strcat(flags, "|consideringMode");
	if (p->flags.limit82) strcat(flags, "|82charsLimit");
	if (p->flags.highPrecision) strcat(flags, "|highPrecisionMode");
	if (p->gnssFilter.disableGps) strcat(gnss, "disableGps");
	if (p->gnssFilter.disableSbas) strcat(gnss, "|disableSbas");
	if (p->gnssFilter.disableQzss) strcat(gnss, "|disableQzss");
	if (p->gnssFilter.disableGlonass) strcat(gnss, "|disableGlonass");
	if (p->gnssFilter.disableBeidou) strcat(gnss, "|disableBeidou");
	if (gnss[0] == 0) sprintf(gnss, "none");
	switch (p->mainTalkerId) {
		case DEFAULT_TALKER_ID: sprintf(talkerId, "default"); break;
		case GP_TALKER_ID: sprintf(talkerId, "GPS|SBAS|QZSS"); break;
		case GL_TALKER_ID: sprintf(talkerId, "GLONASS"); break;
		case GN_TALKER_ID: sprintf(talkerId, "Combo"); break;
		case GA_TALKER_ID: sprintf(talkerId, "Galileo"); break;
		case GB_TALKER_ID: sprintf(talkerId, "BeiDou"); break;
	}
	if (p->gsvTalkerId) sprintf(gsv, "default"); else sprintf(gsv, "main");
	printf("cfgNmea nmeaVersion %x, maxSVs %u, displayNonNmeaSVs %u, filter %s, flags %s, Disabled GNSS %s, talkerId %s, gsvTalkerId %s, dbsTalkerId %s\n", p->nmeaVersion, p->maxSV, p->displayNonNmeaSVs, filter, flags, gnss, talkerId, gsv, p->dbsTalkerId);
}

void Gnss::monVer(uint8_t extensionsNumber) {
  MonVer * ver = (MonVer *)payload;
  printf("Version sw: %s, hw: %s\n", ver->swVersion, ver->hwVersion);
  printf("Extensions: \n");
  for (uint8_t i = 0; i < extensionsNumber; i++) printf("%s\n", ver->extensions[i]);
}

void Gnss::monGnss() {
  GnssSupport * gnss = (GnssSupport *)payload;
  char supported[32], def[32], enabled[32];
  memset(supported, 0, 32);
  memset(def, 0, 32);
  memset(enabled, 0, 32);
  if (gnss->supportedGnss.Gps) strcat(supported, "Gps");
  if (gnss->supportedGnss.Glonass) strcat(supported, "|Glonass");
  if (gnss->supportedGnss.BeiDou) strcat(supported, "|BeiDou");
  if (gnss->supportedGnss.Galileo) strcat(supported, "|Galileo");
  if (gnss->defaultGnss.Gps) strcat(def, "Gps");
  if (gnss->defaultGnss.Glonass) strcat(def, "|Glonass");
  if (gnss->defaultGnss.BeiDou) strcat(def, "|BeiDou");
  if (gnss->defaultGnss.Galileo) strcat(def, "|Galileo");
  if (gnss->enabledGnss.Gps) strcat(enabled, "Gps");
  if (gnss->enabledGnss.Glonass) strcat(enabled, "|Glonass");
  if (gnss->enabledGnss.BeiDou) strcat(enabled, "|BeiDou");
  if (gnss->enabledGnss.Galileo) strcat(enabled, "|Galileo");
  printf("supportedGNSS %s, defaultGnss: %s, enabledGnss: %s, simul %hhu\n", supported, def, enabled, gnss->simultaneous);
}

void Gnss::monPatch() {
	Patch * pp = NULL; 
	MonPatches * p = (MonPatches *)(payload);
	
	char str[32];
	memset(str, 0, 32);
	printf("Patches");
	for (uint8_t i = 0; i < p->numPatches; i++) {
		pp = (Patch *)(payload + sizeof(MonPatches) + sizeof(Patch) * i);
		if (pp->patchInfo.activated) strcat(str, ": activated"); else strcat(str, ": not activated");
		printf("%hhu, %s", i, str);
		memset(str, 0, 32);
		switch (pp->patchInfo.location) {
			case EFUSE: strcat(str, "|eFuse"); break;
			case ROM: strcat(str, "|ROM"); break;
			case BBR: strcat(str, "|BBR"); break;
			case FILE_SYSTEM: strcat(str, "|FILE_SYSTEM"); break;
		}
		printf(", location %s, comparator %lu, address %lu, data %lu\n", str, pp->comparatorNumber, pp->patchAddress, pp->patchData);
	}
}

void Gnss::timTm() {
	TimTm * p = (TimTm *)payload;
	char str[196];
	memset(str, 0, 196);
	if (p->flags.mode) strcat(str, "mode running"); else strcat(str, "mode signle");
	if (p->flags.stopped) strcat(str, "|stopped"); else strcat(str, "|armed");
	if (p->flags.newFallingEdge) strcat(str, "|newFallingEdge");
	switch (p->flags.timeBase) {
		case 0: strcat(str, "|Receiver Time Base"); break;
		case 1: strcat(str, "|GNSS Time Base"); break;
		case 2: strcat(str, "|UTC Time Base"); break;
	}
	if (p->flags.utcAvailable) strcat(str, "|utcAvailable"); else strcat(str, "|utcNotAvailable");
	if (p->flags.timeValid) strcat(str, "|timeValid"); else strcat(str, "|timeNotValid");
	if (p->flags.newRisingEdge) strcat(str, "|newRisingEdge");
	printf("timTm: channel %u, count %u, weekNumRising %u, weekNumFalling %u, towRising %lu.%lums, towFalling %lu.%lu, accEst %luns, flags %s\n", p->channel, p->count, p->weekRising, p->weekFalling, p->towRising, p->towSubRising, p->towFalling, p->towSubFalling, p->accEst, str);
}

void Gnss::timTp() {
  TimeTP * t = (TimeTP *)payload;
  utcTime = (uint32_t)(t->weeks) * ONE_DAY * 7 + (t->tow / 1000) + GPS_UNIX_DIFF;

  time_t ttt = time(NULL);
  tm * gmt = gmtime(&ttt), * lt = localtime(&ttt);
  char gmt_time[32], loc_time[32];
  strftime(gmt_time, 32, "%c", gmt);
  strftime(loc_time, 32, "%c", lt);
  printf("utcTime %lu, time(NULL) %lu, diff %lu\n", utcTime, ttt, (ttt - utcTime)/3600);
  printf("UTC time: %s, local time: %s\n", gmt_time, loc_time);

  ttp = true;
  char timeBase[64], timeRef[13], utcSource[15], tt[32];
  memset(timeBase, 0, 64);
  memset(timeRef, 0, 13);
  memset(utcSource, 0, 15);

  if (t->timeBase.utcBase) {
    switch (t->timeRef.utcSource) {
      case NO_INFO: sprintf(utcSource, "NO_SOURCE_INFO"); break;
      case CRL: sprintf(utcSource, "CRL"); break;
      case NIST: sprintf(utcSource, "NIST"); break;
      case USNO: sprintf(utcSource, "USNO"); break;
      case BIPM: sprintf(utcSource, "BIPM"); break;
      case EUROLAB: sprintf(utcSource, "EUROLAB"); break;
      case SU: sprintf(utcSource, "SU"); break;
      case NTSC: sprintf(utcSource, "NTSC"); break;
      case UNKNOWN_SOURCE: sprintf(utcSource, "UNKNOWN_SOURCE"); break;
    }
    sprintf(timeBase, "UTC: %s", utcSource); 
  } else { 
    switch (t->timeRef.timeRefGnss) {
      case GPS_TP_TIME: sprintf(timeRef, "GPS"); break;
      case GLONASS_TP_TIME: sprintf(timeRef, "GLONASS"); break;
      case BEIDOU_TP_TIME: sprintf(timeRef, "BEIDOU"); break;
      case UNKNOWN_TP_TIME: sprintf(timeRef, "UNKNOWN_GNSS"); break;
    }
    sprintf(timeBase, "GNSS: %s", timeRef);
    if (t->timeBase.utc) strcat(timeBase, "|UTC avail"); 
  }
  switch (t->timeBase.raim) {
    case RAIM_INFO_NOT_AVAILABLE: strcat(timeBase, "|RAIM_INFO_NOT_AVAIL"); break;
    case RAIM_NOT_ACTIVE: strcat(timeBase, "|RAIM_NOT_ACTIVE"); break;
    case RAIM_ACTIVE: strcat(timeBase, "|RAIM_ACTIVE"); break;
  }
  strftime(tt, 32, "%c", localtime(&utcTime));
  printf("timeTP utcTime %s, tow %lums, subTOW %lums 2^-32, quatErr %ldps, weeks %u, timeBase %s\n", tt, t->tow, t->subTOW, t->quantErr, t->weeks, timeBase);
}

void Gnss::logInfo() {
  LogInfo * log = (LogInfo *)payload; //getLogInfo();
  struct tm tm_time;
  char flags[64], ot[32], nt[32];
  memset(flags, 0, 64);
  memset(ot, 0, 32);
  memset(nt, 0, 32);
  time_t oldest = mktime(gps2tm(&(log->oldestDateTime), &tm_time));
  strftime(ot, 32, "%c", localtime(&oldest));
  time_t newest = mktime(gps2tm(&(log->newestDateTime), &tm_time));
  strftime(nt, 32, "%c", localtime(&newest));
  if (log->flags.enabled) sprintf(flags, "enabled"); else sprintf(flags, "disabled");
  if (log->flags.inactive) strcat(flags, "|inactive"); else strcat(flags, "|active");
  if (log->flags.circular) strcat(flags, "|circular"); else strcat(flags, "|non-circular");
  printf("logInfo: fileStoreCapacity %lu bytes, maxLogSize %lu bytes, logSize %lu bytes, numRecords %lu, oldest %s, newest %s, flags %s\n", log->fileStoreCapacity, log->currentMaxLogSize, log->currentLogSize, log->entryCount, ot, nt, flags);
}

void Gnss::logRetrievePos() {
	LogRetrievePos * rec = (LogRetrievePos *)(payload);
	Latitude lat;
    Longitude lon;
    longLatToDMS(&lat, rec->lat);
    longLonToDMS(&lon, rec->lon);
	tm tm_time;
	char tt[32];
	memset(tt, 0, 32);
	time_t t = mktime(gps2tm(&(rec->dateTime), &tm_time));
	strftime(tt, 32, "%c", localtime(&t));
	printf("idx %lu: %s %s %s +/- %lum, alt %ldm above MSL, speed %lum/s, heading %ludeg, fixType %u, numSV %u\n", rec->index, tt, dmsLatToStr(&lat).c_str(), dmsLonToStr(&lon).c_str(), rec->hAcc / 1000, rec->altMSL / 1000, rec->gSpeed / 1000, rec->heading / 100000, rec->fixType, rec->numSV);
}

void Gnss::logRetrievePosExtra() {
  tm tm_time;
  char tt[32];
  memset(tt, 0, 32);
  LogRetrievePosExtra * odo = (LogRetrievePosExtra *)(payload); 
  time_t t = mktime(gps2tm(&(odo->dateTime), &tm_time));
  strftime(tt, 32, "%c", localtime(&t));
  printf("idx %lu: %s distance %lu\n", odo->index, tt, odo->distance);
}

void Gnss::logRetrieveString() {
  tm tm_time;
  char tt[32];
  memset(tt, 0, 32);
  LogRetrievestring * str = (LogRetrievestring *)(payload);
  time_t t = mktime(gps2tm(&(str->dateTime), &tm_time)); 
  strftime(tt, 32, "%c", localtime(&t));
  char * ss = (char *)(payload + sizeof(LogRetrievestring));
  printf("idx %lu: %s %s\n", str->index, tt, ss);
}

void Gnss::logFindTime() {
	LogFindTimeResponse * res = (LogFindTimeResponse *)payload;
	printf("logFindTime response: index %u\n", res->index);
}

void Gnss::infMsg(const char * infLevel) {
	char * s = (char *)malloc(payloadLength);
	printf("%s: %s\n", infLevel, (char *)payload);
	free(s);
}

void Gnss::ackAck() {

}

void Gnss::ackNak() {

}

//Recommended Minimum data
void Gnss::nmeaRmc() {
    GNRMC data;
    getGNRMC(&data);
    if (data.status == 'V') return;
    tm tm_time;
	char tt[32];
	memset(tt, 0, 32);
	time_t t = mktime(gps2tm(&(data.dateTime), &tm_time));
	strftime(tt, 32, "%c", localtime(&t));
	printf("GNRMC: %s %s %s, SOG %.1fkt, COG %.fdeg, mv %.fdeg %c, fixType %c\n", tt, dmsLatToStr(&(data.lat)).c_str(), dmsLonToStr(&(data.lon)).c_str(), (double)(data.SOG), (double)(data.COG), (double)(data.magVar), data.magVarEW, data.fixType);
}

//Course over ground and Ground speed
void Gnss::nmeaVtg() {
  GNVTG data;
  getGNVTG(&data);
  printf("GNVTG COG: %.fT, %.fM; SOG: %.1fkt, %.1fkm/h; fix type: %c\n", (double)(data.COGt), (double)(data.COGm), (double)(data.SOGkt), (double)(data.SOGkmh), data.fixType);
}

//Global positioning system fix data
void Gnss::nmeaGga() {
  GNGGA data;
  getGNGGA(&data);
  if (data.fixType == NO_FIX_TYPE) return;
  char tt[32];
  tm tm_time;
  time_t t = mktime(gps2tm(&(data.dateTime), &tm_time));
  strftime(tt, 32, "%c", localtime(&t));
  printf("GNGGA %s %s %s, fix type: %u, numSVs: %u, hDOP %.2f, alt %.fm, geoidDiff %.fm, ageDC %lus, stId %u\n", tt, dmsLatToStr(&(data.lat)).c_str(), dmsLonToStr(&(data.lon)).c_str(), data.fixType, data.numSV, (double)(data.hDOP), (double)(data.alt), (double)(data.geoidEllipsoidDiff), data.ageDiffCorr, data.diffCorrStationId);
}

//GNSS DOP and Active Satellites
void Gnss::nmeaGsa() {
  GNGSA data;
  data.vDOP = 0; data.hDOP = 0; data.pDOP = 0, data.opMode = 'A', data.fixType = FIX_NOT_AVAILABLE;
  getGNGSA(&data);
  string sv = "";
  
  if (data.fixType == FIX_NOT_AVAILABLE) return;
  for (uint8_t i = 0; i < 12; i++) {
    if (data.svId[i] > 0)
      sv += to_string(data.svId[i]) + " ";
  }
  printf("GNGSA SV IDs: (%s) pDOP %.2f, hDOP %.2f, vDOP %.2f, mode: %c, fix type: %u\n", sv.c_str(), (double)(data.pDOP), (double)(data.hDOP), (double)(data.vDOP), data.opMode, data.fixType);
}

//GNSS Satellites in View
void Gnss::nmeaGsv() {
  GNGSV data;  
  getGNGSV(&data);
  string sv = "";
  char s[255];
  memset(s, 0, 255);
  
  for (uint8_t i = 0; i < 4; i++) {
    sprintf(s, "svId %u, elev %u, azim %u, cno %u | ", data.svAttr[i].svId, data.svAttr[i].elev, data.svAttr[i].azim, data.svAttr[i].cno);
	sv += string(s);
  }
  printf("GNGSV: numSV %u; GNSS %s: %s\n", data.numSV, data.gnss.c_str(), sv.c_str()); 
}

//Latitude and longitude, with time of position fix and status
void Gnss::nmeaGll() {
  GNGLL data;
  getGNGLL(&data);
  if (data.fixType == 'N') return;
  if (data.status == 'V') return;
  char tt[32];
  memset(tt, 0, 32);
  tm tm_time;
  time_t t = mktime(gps2tm(&(data.dateTime), &tm_time));
  strftime(tt, 32, "%c", localtime(&t));
  printf("GNGLL: %s %s %s, fix type: %c\n", tt, dmsLatToStr(&(data.lat)).c_str(), dmsLonToStr(&(data.lon)).c_str(), data.fixType);
}

//GNSS Pseudo Range Error Statistics
void Gnss::nmeaGst() {
  GNGST data;
  getGNGST(&data);
  char tt[32];
  memset(tt, 0, 32);
  tm tm_time;
  time_t t = mktime(gps2tm(&(data.dateTime), &tm_time));
  strftime(tt, 32, "%c", localtime(&t));
  printf("GNGST: %s, RangeRms %.fm, stdLat %.fm, stdLon %.fm, stdAlt %.fm\n", tt, (double)(data.rangeRms), (double)(data.stdLat), (double)(data.stdLon), (double)(data.stdAlt));
}

//Dual ground/water distance
void Gnss::nmeaVlw() {
  GNVLW data;
  getGNVLW(&data);
  printf("GNVLW: TWD %.1f%c, WD %.1f%c, TGD %.1f%c, GD %.1f%c\n", (double)(data.twd), data.twdUnit, (double)(data.wd), data.wdUnit, (double)(data.tgd), data.tgdUnit, (double)(data.gd), data.gdUnit); 
}

//GNSS fix data
void Gnss::nmeaGns() {
    GNGNS data;
    getGNGNS(&data);
	tm tm_time;
	char tt[32];
	memset(tt, 0, 32);
	time_t t = mktime(gps2tm(&(data.dateTime), &tm_time));
	strftime(tt, 32, "%c", localtime(&t));
	printf("GNGNS: %s %s %s, atl %.fm, fixType[GPS] %c|fixType[GLO] %c, numSV %u, hDOP %.2f, geoidDiff %.fm, AgeDC %luc, stId %u\n", tt, dmsLatToStr(&(data.lat)).c_str(), dmsLonToStr(&(data.lon)).c_str(), (double)(data.alt), data.fixType[0], data.fixType[1], data.numSV, (double)(data.hDOP), (double)(data.geoidEllipsoidDiff), data.ageDiffCorr, data.diffCorrStationId);
}

//Time and Date
void Gnss::nmeaZda() {
    GNZDA data;
    getGNZDA(&data);
	char tt[32];
	memset(tt, 0, 32);
	tm tm_time;
	time_t t = mktime(gps2tm(&(data.dateTime), &tm_time));
	strftime(tt, 32, "%c", localtime(&t));
	printf("GNZDA: %s %d:%u\n", tt, data.utcOffsetHours, data.utcOffsetMinutes);
    
}

void Gnss::nmeaTxt() {
	GNTXT data;
	char t[8] = "";
	getGNTXT(&data);
	switch (data.msgType) {
		case 0: sprintf(t, "Error"); break;
		case 1: sprintf(t, "Warning"); break;
		case 2: sprintf(t, "Notice"); break;
		case 7: sprintf(t, "User"); break;
	}
	printf("GNTXT: message %u out of %u \"%s! %s\"\n", data.msgNum, data.numMsg, t, data.text.c_str());
}

void Gnss::nmeaGbs() {
	GBS data;
	getGbs(&data);
	tm tm_time;
	char tt[32];
	memset(tt, 0, 32);
	time_t t = mktime(gps2tm(&(data.dateTime), &tm_time));
	strftime(tt, 32, "%c", localtime(&t));
	printf("GBS: %s Lat err %.1fm, Lon err %.1fm, Alt err %.1fm, failed svId %u, bias %.1fm, stddev %.1fm, sysId %u, sigId %u\n", tt, (double)(data.errLat), (double)(data.errLon), (double)(data.errAlt), data.svId, (double)(data.bias), (double)(data.stddev), data.systemId, data.signalId);
}

void Gnss::nmeaDtm() {
	DTM data;
	getDtm(&data);
	printf("DTM: datum %s differs from refDatum %s by %.1f\'%c %.1f\'%c %.1fm\n", data.datum.c_str(), data.refDatum.c_str(), (double)(data.lat), data.NS, (double)(data.lon), data.EW, (double)(data.alt));
}

void Gnss::nmeaGrs() {
	GRS data;
	getGrs(&data);
	tm tm_time;
	char tt[32], s[16], ss[255];
	memset(tt, 0, 32);
	memset(s, 0, 16);
	memset(ss, 0, 255);
	time_t t = mktime(gps2tm(&(data.dateTime), &tm_time));
	strftime(tt, 32, "%c", localtime(&t));
	for (uint8_t i = 0; i < 12; i++) {
		sprintf(s, "%.1fm ", (double)(data.residual[i]));
		strcat(ss, s);
	}
	printf("GRS: %s %s sysId %u, sigId %u\n", tt, ss, data.systemId, data.signalId);
}

void Gnss::nmeaGpq(const char* talkerId, const char* msgId) {
	uint8_t x = 0;
	char s[32], ss[32];
	memset(s, 0, 32);
	memset(ss, 0, 32);

	sprintf(s, "%sGPQ,%s", talkerId, msgId);

	for (uint8_t i = 0; i < strlen(s); i++) {
		x ^= s[i];
	}
	sprintf(ss, "$%s*%X\r\n", s, x);
	//printf(ss);
	if (write(fd, ss, strlen(ss)) != strlen(ss)) printf("poll(): write() error: %s\n", strerror(errno));
	tcdrain(fd);
}

void Gnss::nmeaGnq(const char* talkerId, const char* msgId) {
	uint8_t x = 0;
	char s[32], ss[32];
	memset(s, 0, 32);
	memset(ss, 0, 32);

	sprintf(s, "%sGNQ,%s", talkerId, msgId);

	for (uint8_t i = 0; i < strlen(s); i++) {
		x ^= s[i];
	}
	sprintf(ss, "$%s*%X\r\n", s, x);
	//printf(ss);
	if (write(fd, ss, strlen(ss)) != strlen(ss)) printf("poll(): write() error: %s\n", strerror(errno));
	tcdrain(fd);
}

void Gnss::nmeaGlq(const char* talkerId, const char* msgId) {
	uint8_t x = 0;
	char s[32], ss[32];
	memset(s, 0, 32);
	memset(ss, 0, 32);

	sprintf(s, "%sGLQ,%s", talkerId, msgId);

	for (uint8_t i = 0; i < strlen(s); i++) {
		x ^= s[i];
	}
	sprintf(ss, "$%s*%X\r\n", s, x);
	//printf(ss);
	if (write(fd, ss, strlen(ss)) != strlen(ss)) printf("poll(): write() error: %s\n", strerror(errno));
	tcdrain(fd);
}

void Gnss::nmeaGbq(const char* talkerId, const char* msgId) {
	uint8_t x = 0;
	char s[32], ss[32];
	memset(s, 0, 32);
	memset(ss, 0, 32);

	sprintf(s, "%sGBQ,%s", talkerId, msgId);

	for (uint8_t i = 0; i < strlen(s); i++) {
		x ^= s[i];
	}
	sprintf(ss, "$%s*%X\r\n", s, x);
	//printf(ss);
	if (write(fd, ss, strlen(ss)) != strlen(ss)) printf("poll(): write() error: %s\n", strerror(errno));
	tcdrain(fd);
}

void Gnss::pubxPosition() {
	PubxPosition data;
	getPubxPosition(&data);
	char tt[32];
	memset(tt, 0, 32);
	tm tm_time;
	time_t t = mktime(gps2tm(&(data.dateTime), &tm_time));
	strftime(tt, 32, "%c", localtime(&t));
	printf("pubxPosition: %s %s %s +/- %.fm, alt %.f +/- %.fm above ellipsoid, SOG %.1fkm/h, COG %u, vVel %.2fm/s, hDOP %.2f, vDOP %.2f, tDOP %.2f, ageDC %luc, numSV %u, fixType %s\n", tt, dmsLatToStr(&(data.lat)).c_str(), dmsLonToStr(&(data.lon)).c_str(), (double)(data.hAcc), (double)(data.alt), (double)(data.vAcc), (double)(data.sog), data.cog, (double)(data.vVel), (double)(data.hDOP), (double)(data.vDOP), (double)(data.tDOP), data.ageDiffCorr, data.numSV, data.fixType);
}

void Gnss::pubxConfig(BaudRate rate, InProtoMask inMask, OutProtoMask outMask, Port portId, bool autoBauding) {
	uint8_t x = 0;
	char s[255], ss[255];
	memset(s, 0, 255);
	memset(ss, 0, 255);

	uint16_t im = inMask.inUbx | (inMask.inNmea << 1) | (inMask.inRtcm << 2), om = outMask.outUbx | (outMask.outNmea << 1);
	uint8_t ab = autoBauding ? 1 : 0;
	sprintf(s, "PUBX,41,%u,%.4u,%.4u,%lu,%u", (uint8_t)portId, im, om, (uint32_t)rate, ab);

	for (uint8_t i = 0; i < strlen(s); i++ ) {
		x ^= s[i];
	}
	sprintf(ss, "$%s*%X\r\n", s, x);
	//printf(ss);
	if (write(fd, ss, strlen(ss)) != strlen(ss)) printf("poll(): write() error: %s\n", strerror(errno));
	tcdrain(fd);
}

void Gnss::pubxRate(const char * msgId, uint8_t rateCom1, uint8_t rateCom2, uint8_t rateDDC, uint8_t rateUsb, uint8_t rateSpi) {
	uint8_t x = 0;
	char s[255], ss[255];
	memset(s, 0, 255);
	
	sprintf(s, "PUBX,40,%s,%u,%u,%u,%u,%u,%u", msgId, rateDDC, rateCom1, rateCom2, rateUsb, rateSpi, 0);

	for (uint8_t i = 0; i < strlen(s); i++ ) {
		x ^= s[i];
	}
	sprintf(ss, "$%s*%X\r\n", s, x);
	//printf(ss);
	if (write(fd, ss, strlen(ss)) != strlen(ss)) printf("poll(): write() error: %s\n", strerror(errno));
	tcdrain(fd);
}

void Gnss::pubxSvStatus() {
  PubxSvStatus data;
  string arr[3];
  split(arr, 3, nmeaPayload);
  uint8_t numSV = stoi(arr[2], nullptr, 10);
  
  string status[255];
  split(status, 255, nmeaPayload);
  for (uint8_t i = 0; i < numSV; i++) {
	data.svId = 0; data.svStatus = '-'; data.azim = 0; data.elev = 0; data.cno = 0; data.lockTime = 0;
	for (uint8_t j = i * 6 + 3; j < i * 6 + 9; j++) {
		if (status[j] != "")
		switch (j) {
			case 3: data.svId = stoi(status[j], nullptr, 10); break;
			case 4: data.svStatus = status[j][0]; break;
			case 5: data.azim = stoi(status[j], nullptr, 10); break;
			case 6: data.elev = stoi(status[j], nullptr, 10); break;
			case 7: data.cno = stoi(status[j], nullptr, 10); break;
			case 8: data.lockTime = stoi(status[j], nullptr, 10); break;
		}		
	}
	printf("PubxSvStatus: SvId %u, used: %c, azim: %u, elev %u, cno %u, lockTime %us\n", data.svId, data.svStatus, data.azim, data.elev, data.cno, data.lockTime);	
  }
}

void Gnss::pubxTime() {
  PubxTime data;
  getPubxTime(&data);
  char tt[32];
  memset(tt, 0, 32);
  tm tm_time;
  time_t t = mktime(gps2tm(&(data.dateTime), &tm_time));
  strftime(tt, 32, "%c", localtime(&t));
  printf("PubxTime %s, utcTow: %lus, utcWeek: %lu, leapSec %u, leapSecSrc %c, clkBias %luns, clkDrift %luns/s, tpGran %luns\n", tt, data.utcTow, data.utcWeek, data.leapSec, data.leapSecSrc, data.clkBias, data.clkDrift, data.tpGran);
}

//UBX messages
MonVer * Gnss::getVersion(uint32_t timeout) {
  	poll(UBX_MON, MON_VER);
	if (pollNoError(timeout)) return (MonVer *)(payload);
	return NULL;
}

MonPatches * Gnss::getPatches(uint32_t timeout) {
  	poll(UBX_MON, MON_PATCH);
	if (pollNoError(timeout)) return (MonPatches *)(payload);
	return NULL;	
}

CfgNav * Gnss::getCfgNav(uint32_t timeout) {
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) return (CfgNav *)(payload);
  return NULL;
}

DynModel Gnss::getDynamicModel(uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	  cfgNav = (CfgNav *)(payload);
	  return cfgNav->dynModel;
  }
  return MODEL_ERROR;
}

bool Gnss::setDynamicModel(DynModel model, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
  else return false;

  if (cfgNav->dynModel != model) {
	  cfgNav->dynModel = model;
	  cfgNav->mask.dyn = 1;
	  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

UtcStandard Gnss::getUtcStandard(uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	  cfgNav = (CfgNav *)(payload);
	  return cfgNav->utcStandard;
  }
  return UTC_ERROR;
}

bool Gnss::setUtcStandard(UtcStandard standard, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
  else return false;

  if (cfgNav->utcStandard != standard) {
	  cfgNav->utcStandard = standard;
	  cfgNav->mask.utcMask = 1;
	  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

FixMode Gnss::getFixMode(uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	return cfgNav->fixMode;
  }
  return FIX_MODE_ERROR;
}

bool Gnss::setFixMode(FixMode fixMode, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
  else return false;

  if (cfgNav->fixMode != fixMode) {
	  cfgNav->fixMode = fixMode;
	  cfgNav->mask.posFixMode = 1;
	  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

FixedAlt * Gnss::getFixedAlt(FixedAlt * fixedAlt, uint32_t timeout) {
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

bool Gnss::setFixedAlt(int32_t fixedAlt, uint32_t fixedAltVar, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
  else return false;

  if (cfgNav->fixedAlt != fixedAlt || cfgNav->fixedAltVar != fixedAltVar) {
	  cfgNav->fixedAlt = fixedAlt;
	  cfgNav->fixedAltVar = fixedAltVar;
	  cfgNav->mask.posMask = 1;
	  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

DOP * Gnss::getDop(DOP * dop, uint32_t timeout) {
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

bool Gnss::setDop(uint16_t posDOP, uint16_t timeDOP, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
  else return false;

  cfgNav->pDOP = posDOP;
  cfgNav->tDOP = timeDOP;
  cfgNav->mask.posMask = 1;
  cfgNav->mask.timeMask = 1;
  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)cfgNav);
  if (ackNoError(timeout)) return true;
  return false;
}

Accuracy * Gnss::getAccuracy(Accuracy * acc, uint32_t timeout) {
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

bool Gnss::setAccuracy(uint16_t posAccuracy, uint16_t timeAccuracy, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
  else return false;

  cfgNav->pAcc = posAccuracy;
  cfgNav->tAcc = timeAccuracy;
  cfgNav->mask.posMask = 1;
  cfgNav->mask.timeMask = 1;
  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)cfgNav);
  if (ackNoError(timeout)) return true;
  return false;
}

CnoThreshold * Gnss::getCnoThreshold(CnoThreshold * cno, uint32_t timeout) {
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

bool Gnss::setCnoThreshold(uint8_t cnoThreshold, uint8_t cnoThresholdNumSVs, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
  else return false;

  if (cfgNav->cnoThreshold != cnoThreshold || cfgNav->cnoThresholdNumSVs != cnoThresholdNumSVs) {
	  cfgNav->cnoThreshold = cnoThreshold;
	  cfgNav->cnoThresholdNumSVs = cnoThresholdNumSVs;
	  cfgNav->mask.cnoThreshold = 1;
	  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

StaticHoldThresholds * Gnss::getStaticHoldThresholds(StaticHoldThresholds * thresholds, uint32_t timeout) {
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

bool Gnss::setStaticHoldThresholds(uint8_t staticHoldThreshold, uint8_t staticHoldMaxDistance, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
  else return false;

  if (cfgNav->staticHoldThreshold != staticHoldThreshold || cfgNav->staticHoldMaxDistance != staticHoldMaxDistance) {
	  cfgNav->staticHoldThreshold = staticHoldThreshold;
	  cfgNav->staticHoldMaxDistance = staticHoldMaxDistance;
	  cfgNav->mask.staticHoldMask = 1;
	  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

int8_t Gnss::getMinElev(uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	return cfgNav->minElev;
  }
  return 0;
}

bool Gnss::setMinElev(int8_t minElev, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
  else return false;

  if (cfgNav->minElev != minElev) {
	  cfgNav->minElev = minElev;
	  cfgNav->mask.minElev = 1;
	  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

int8_t Gnss::getDgnssTimeout(uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) {
	cfgNav = (CfgNav *)(payload);
	return cfgNav->dgnssTimeout;
  }
  return 0;
}

bool Gnss::setDgnssTimeout(int8_t dgnssTimeout, uint32_t timeout) {
  CfgNav * cfgNav = NULL;
  poll(UBX_CFG, CFG_NAV5);
  if (pollNoError(timeout)) cfgNav = (CfgNav *)(payload);
  else return false;

  if (cfgNav->dgnssTimeout != dgnssTimeout) {
	  cfgNav->dgnssTimeout = dgnssTimeout;
	  cfgNav->mask.dgpsMask = 1;
	  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)cfgNav);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

InfoMsgMask * Gnss::getCfgInf(Protocol protocolId, Port portId, uint32_t timeout) {
  poll(UBX_CFG, CFG_INF, sizeof(uint8_t), (uint8_t *)&protocolId);
  if (pollNoError(timeout))	return (InfoMsgMask *)&(payload[portId + 4]);
  return NULL;
}

bool Gnss::setCfgInf(InfoMsgMask mask, Protocol protocolId, Port portId, uint32_t timeout) {
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

char * Gnss::getErrMsg(DebugLevel debugLevel, uint32_t timeout) {
  poll(UBX_INF, debugLevel);
  if (pollNoError(timeout))	return (char *)payload;
  return NULL;
}

CfgPrt * Gnss::getCfgPrt(Port portId, uint32_t timeout) {
  poll(UBX_CFG, CFG_PRT, sizeof(uint8_t), (uint8_t *)&portId);
  if (pollNoError(timeout)) {
	  CfgPrt* p = (CfgPrt*)payload;
	  switch (p->baudRate) {
		case BAUD_RATE_4800: baudRate = B4800; break;
		case BAUD_RATE_9600: baudRate = B9600; break;
		case BAUD_RATE_19200: baudRate = B19200; break;
		case BAUD_RATE_38400: baudRate = B38400; break;
		case BAUD_RATE_57600: baudRate = B57600; break;
		case BAUD_RATE_115200: baudRate = B115200; break;
		case BAUD_RATE_230400: baudRate = B230400; break;
		//case BAUD_RATE_460800: baudRate = B460800; break;
	  }
	  return (CfgPrt*)payload;
  }
  return NULL;
}

PowerMode * Gnss::getCfgPms(uint32_t timeout) {
  poll(UBX_CFG, CFG_PMS);
  if (pollNoError(timeout))	return (PowerMode *)payload;
  return NULL;
}

bool Gnss::setCfgPms(PowerModes mode, uint8_t period, uint8_t onTime, uint32_t timeout) {
  PowerMode * pMode = NULL;
  poll(UBX_CFG, CFG_PMS);
  if (pollNoError(timeout)) pMode = (PowerMode *)(payload);
  else return false;

  if (pMode->powerMode != mode || pMode->period != period || pMode->onTime != onTime) {
	  if (period > 0 && period <= 5) return false;
	  if ((period > 5 || onTime > 0) && mode != INTERVAL_POWER_MODE) return false;
	  pMode->powerMode = mode;
	  pMode->period = period;
	  pMode->onTime = onTime;
	  poll(UBX_CFG, CFG_PMS, sizeof(PowerMode), (uint8_t *)pMode);
	  if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

CfgPm * Gnss::getCfgPm(uint32_t timeout) {
  poll(UBX_CFG, CFG_PM2);
  if (pollNoError(timeout))	return (CfgPm *)payload;
  return NULL;
}

bool Gnss::setCfgPm(uint8_t maxStartupStateDuration, uint32_t udpatePeriod, uint32_t searchPeriod, uint32_t gridOffset, uint16_t onTime, uint16_t minAcqTime, uint32_t extintInactivity, CfgPmFlags flags, uint32_t timeout) {
  CfgPm * pMode = NULL;
  poll(UBX_CFG, CFG_PM2);
  if (pollNoError(timeout)) pMode = (CfgPm *)(payload);
  else return false;
  pMode->maxStartupStateDuration = maxStartupStateDuration;
  pMode->updatePeriod = udpatePeriod;
  pMode->searchPeriod = searchPeriod;
  pMode->gridOffset = gridOffset;
  pMode->onTime = onTime;
  pMode->minAcqTime = minAcqTime;
  pMode->extintInactivity = extintInactivity;
  pMode->flags = flags;
  poll(UBX_CFG, CFG_PM2, sizeof(pMode), (uint8_t *)pMode);
  if (ackNoError(timeout)) return true;
  return false;
}

CfgRxm * Gnss::getCfgRxm(uint32_t timeout) {
  poll(UBX_CFG, CFG_RXM);
  if (pollNoError(timeout))	return (CfgRxm *)payload;
  return NULL;
}

bool Gnss::setCfgRxm(CfgRxmLpMode mode, uint32_t timeout) {
  CfgRxm * pMode = NULL;
  poll(UBX_CFG, CFG_RXM);
  if (pollNoError(timeout)) pMode = (CfgRxm *)(payload);
  else return false;
  if (pMode->lpMode != mode) {
	pMode->lpMode = mode;
	poll(UBX_CFG, CFG_RXM, sizeof(CfgRxm), (uint8_t *)pMode);
	if (ackNoError(timeout)) return true;
  } else return true;
  return false;
}

bool Gnss::setCfgPrt(Port portId, BaudRate rate, PrtMode mode, InProtoMask inMask, OutProtoMask outMask, bool extendedTxTimeout, uint32_t timeout) {
  CfgPrt * prtCfg = NULL;
  poll(UBX_CFG, CFG_PRT, sizeof(uint8_t), (uint8_t *)&portId);
  if (pollNoError(timeout)) prtCfg = (CfgPrt *)(payload);
  else return false;
  //printf("baud rate: %lu\n", rate);
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
  poll(UBX_CFG, CFG_PRT, sizeof(CfgPrt), (uint8_t *)prtCfg);
  if (ackNoError(timeout)) return true;
  return false;
}

bool Gnss::config(ConfMask mask, ConfigType type, uint32_t timeout) {
  ConfigAllDevs conf;
  switch (type)
  {
  	  case DEFAULT_CONFIG: conf.clearMask = mask; memset(&(conf.saveMask), 0, sizeof(ConfMask)); memset(&(conf.loadMask), 0, sizeof(ConfMask)); break;
  	  case SAVE_CONFIG: memset(&(conf.clearMask), 0, sizeof(ConfMask)); conf.saveMask = mask; memset(&(conf.loadMask), 0, sizeof(ConfMask)); break;
  	  case LOAD_CONFIG: memset(&(conf.clearMask), 0, sizeof(ConfMask)); memset(&(conf.saveMask), 0, sizeof(ConfMask)); conf.loadMask = mask; break;
  }
  poll(UBX_CFG, CFG_CFG, sizeof(ConfigAllDevs), (uint8_t *)&conf);
  if (ackNoError(timeout)) return true;
  return false;
}

bool Gnss::config(ConfMask mask, ConfigType type, Device dev, uint32_t timeout) {
  ConfigDev conf;
  switch (type)
  {
  	  case DEFAULT_CONFIG: conf.clearMask = mask; memset(&(conf.saveMask), 0, sizeof(ConfMask)); memset(&(conf.loadMask), 0, sizeof(ConfMask)); break;
  	  case SAVE_CONFIG: memset(&(conf.clearMask), 0, sizeof(ConfMask)); conf.saveMask = mask; memset(&(conf.loadMask), 0, sizeof(ConfMask)); break;
  	  case LOAD_CONFIG: memset(&(conf.clearMask), 0, sizeof(ConfMask)); memset(&(conf.saveMask), 0, sizeof(ConfMask)); conf.loadMask = mask; break;
  }
  conf.device = dev;
  poll(UBX_CFG, CFG_CFG, sizeof(ConfigDev), (uint8_t *)&conf);
  if (ackNoError(timeout)) return true;
  return false;
}

bool Gnss::defaultConfig(bool ioPort, bool msgConf, bool infMsg, bool navConf, bool rxmConf, bool senConf, bool rinvConf, bool antConf, bool logConf, bool ftsConf, uint32_t timeout) {
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
  if (config(conf, DEFAULT_CONFIG, timeout)) {
	  if (!config(conf, LOAD_CONFIG, timeout)) return false;
  } else return false;
  return true;
}

bool Gnss::saveConfig(bool ioPort, bool msgConf, bool infMsg, bool navConf, bool rxmConf, bool senConf, bool rinvConf, bool antConf, bool logConf, bool ftsConf, uint32_t timeout) {
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
  if (!config(conf, SAVE_CONFIG, timeout)) return false;
  return true;
}

bool Gnss::loadConfig(bool ioPort, bool msgConf, bool infMsg, bool navConf, bool rxmConf, bool senConf, bool rinvConf, bool antConf, bool logConf, bool ftsConf, uint32_t timeout) {
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
  conf.reserved2 = 0L;
  if (!config(conf, LOAD_CONFIG, timeout)) return false;
  return true;
}

void Gnss::cfgRst(StartType type, ResetMode mode) {
	CfgRst rset;
	rset.navBbrMask = type.mask;
	rset.resetMode = mode;
	poll(UBX_CFG, CFG_RST, sizeof(CfgRst), (uint8_t*)&rset);
}

void Gnss::stopGnss(StartTypes startType) {
	StartType type;
	type.start = startType;
	cfgRst(type, GNSS_STOP);
}

void Gnss::startGnss(StartTypes startType) {
	StartType type;
	type.start = startType;
	cfgRst(type, GNSS_START);
}

void Gnss::resetGnss(StartTypes startType) {
	StartType type;
	type.start = startType;
	cfgRst(type, SOFTWARE_RESET_GNSS_ONLY);
}

void Gnss::reset(bool soft, bool afterShutdown, StartTypes startType) {
	StartType type;
	type.start = startType;
	if (soft) cfgRst(type, SOFTWARE_RESET);
	else {
		if (afterShutdown) cfgRst(type, HARDWARE_RESET_AFTER_SHUTDOWN);
		else cfgRst(type, HARDWARE_RESET);
	}
}

CfgMsg * Gnss::getCfgMsg(uint8_t msgClass, uint8_t msgId, uint32_t timeout) {
  uint16_t pload = ((uint16_t)(msgId) << 8) | msgClass;
  poll(UBX_CFG, CFG_MSG, sizeof(uint16_t), (uint8_t *)&pload);
  if (pollNoError(timeout))	return (CfgMsg *)payload;
  return NULL;
}

bool Gnss::setCfgMsg(uint8_t msgClass, uint8_t msgId, uint8_t rate, uint32_t timeout) {
  CfgMsg * cfgMsg = NULL;
  uint16_t pload = ((uint16_t)(msgId) << 8) | msgClass;
  poll(UBX_CFG, CFG_MSG, sizeof(uint16_t), (uint8_t *)&pload);
  if (pollNoError(timeout))	cfgMsg = (CfgMsg *)payload;
  else return false;

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

ODOCfg * Gnss::getCfgOdo(uint32_t timeout) {
  poll(UBX_CFG, CFG_ODO);
  if (pollNoError(timeout))	return (ODOCfg *)payload;
  return NULL;
}

bool Gnss::setCfgOdo(OdoFlags flags, OdoProfile profile, uint8_t maxSpeed, uint8_t maxPosAccuracy, uint8_t velGain, uint8_t COGgain, uint32_t timeout) {
  ODOCfg * odoCfg = NULL;
  poll(UBX_CFG, CFG_ODO);
  if (pollNoError(timeout)) odoCfg = (ODOCfg *)(payload);
  else return false;
  odoCfg->flags.ODOenabled = flags.ODOenabled;
  odoCfg->flags.COGenabled = flags.COGenabled;
  odoCfg->flags.outputLPvelocity = flags.outputLPvelocity;
  odoCfg->flags.outputLPcog = flags.outputLPcog; 
  odoCfg->odoProfile = profile; 
  odoCfg->cogMaxSpeed = maxSpeed; 
  odoCfg->cogMaxPosAccuracy = maxPosAccuracy;
  odoCfg->velLPgain = velGain; odoCfg->cogLPgain = COGgain;
  poll(UBX_CFG, CFG_ODO, sizeof(ODOCfg), (uint8_t *)odoCfg);
  if (ackNoError(timeout)) return true;
  return false;
}

CfgSbas *  Gnss::getCfgSbas(uint32_t timeout) {
  poll(UBX_CFG, CFG_SBAS);
  if (pollNoError(timeout))	return (CfgSbas *)payload;
  return NULL;
}

bool Gnss::setCfgSbas(CfgSbasUsage usage, uint32_t scanMode1, uint8_t scanMode2, uint32_t timeout) {
  CfgSbas * cfg = NULL;
  poll(UBX_CFG, CFG_SBAS);
  if (pollNoError(timeout)) cfg = (CfgSbas *)(payload);
  else return false;
  cfg->usage = usage;
  cfg->scanMode1 = scanMode1;
  cfg->scanMode2 = scanMode2;
  poll(UBX_CFG, CFG_SBAS, sizeof(CfgSbas), (uint8_t *)cfg);
  if (ackNoError(timeout)) return true;
  return false;
}

CfgNmea * Gnss::getCfgNmea(uint32_t timeout) {
  poll(UBX_CFG, CFG_NMEA);
  if (pollNoError(timeout))	return (CfgNmea *)payload;
  return NULL;
}

bool Gnss::setCfgNmea(CfgNmeaFilter filter, uint8_t nmeaVersion, uint8_t maxSVs, CfgNmeaFlags flags, CfgNmeaGnss gnssFilter, bool displayNonNmeaSVs, CfgNmeaTalkerId mainTalkerId, bool gsvTalkerIdIsMain, char * dbsTalkerId, uint32_t timeout) {
  CfgNmea * cfg = NULL;
  poll(UBX_CFG, CFG_NMEA);
  if (pollNoError(timeout)) cfg = (CfgNmea *)(payload);
  else return false;
  cfg->filter = filter;
  cfg->nmeaVersion = nmeaVersion;
  cfg->flags = flags;
  cfg->gnssFilter = gnssFilter;
  cfg->displayNonNmeaSVs = displayNonNmeaSVs ? 1 : 0;
  cfg->mainTalkerId = mainTalkerId;
  cfg->gsvTalkerId = gsvTalkerIdIsMain ? 1 : 0;
  cfg->dbsTalkerId[0] = dbsTalkerId[0];
  cfg->dbsTalkerId[1] = dbsTalkerId[1];
  poll(UBX_CFG, CFG_SBAS, sizeof(CfgNmea), (uint8_t *)cfg);
  if (ackNoError(timeout)) return true;
  return false;
}

//LOG messages
CfgLogFilter * Gnss::getCfgLogFilter(uint32_t timeout) {
  poll(UBX_CFG, CFG_LOGFILTER);
  if (pollNoError(timeout)) {
	CfgLogFilter * lf = (CfgLogFilter*)payload;
	loggingEnabled = lf->flags.recordingEnabled;
	return (CfgLogFilter*)payload;
  }
  return NULL;
}

bool Gnss::setCfgLogFilter(uint8_t minInterval, uint8_t timeThreshold, uint8_t speedThreshold, uint8_t positionThreshold, CfgLogFilterFlags flags, uint32_t timeout) {
  CfgLogFilter * cfgLogFilter = NULL;
  poll(UBX_CFG, CFG_LOGFILTER);
  if (pollNoError(timeout))	cfgLogFilter = (CfgLogFilter*)payload;
  else return false;
  cfgLogFilter->minInterval = minInterval;
  cfgLogFilter->timeThreshold = timeThreshold;
  cfgLogFilter->speedThreshold = speedThreshold;
  cfgLogFilter->positionThreshold = positionThreshold;
  cfgLogFilter->flags = flags;
  poll(UBX_CFG, CFG_LOGFILTER, sizeof(cfgLogFilter), (uint8_t*)cfgLogFilter);
  if (ackNoError(timeout)) return true;
  return false;
}

bool Gnss::enableLogging(uint8_t interval, uint32_t timeout) {
	CfgLogFilterFlags flags;
	flags.recordingEnabled = 1;
	flags.psmOncePerWakeUpEnabled = 0;
	flags.applyAllFilterSettings = 1;
	uint8_t minInterval = 0, timeThreshold = interval, speedThreshold = 0, positionThreshold = 0;
	if (setCfgLogFilter(minInterval, timeThreshold, speedThreshold, positionThreshold, flags, timeout)) return true;
	else return false;
}

bool Gnss::disableLogging(uint32_t timeout) {
	CfgLogFilterFlags flags;
	flags.recordingEnabled = 0;
	flags.psmOncePerWakeUpEnabled = 0;
	flags.applyAllFilterSettings = 0;
	uint8_t minInterval = 0, timeThreshold = 0, speedThreshold = 0, positionThreshold = 0;
	if (setCfgLogFilter(minInterval, timeThreshold, speedThreshold, positionThreshold, flags, timeout)) return true;
	else return false;
}

LogInfo * Gnss::getLogInfo(uint32_t timeout) {
  poll(UBX_LOG, LOG_INFO);
  if (pollNoError(timeout)) {
	LogInfo * logInfo = (LogInfo*)payload;
	loggingEnabled = logInfo->flags.enabled == 1 ? true : false;
	logFileExists = logInfo->flags.inactive == 1 ? false : true;
	return logInfo;
  }
  return NULL;
}

bool Gnss::createLog(LogSize logSize, uint32_t userDefinedSize, bool circular, uint32_t timeout) {
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

bool Gnss::eraseLog(uint32_t timeout) {
  poll(UBX_LOG, LOG_ERASE);
  if (ackNoError(timeout)) return true;
  return false;
}

uint32_t Gnss::logFind(DateTime dateTime, uint32_t timeout) {
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

bool Gnss::logMsg(const char * msg, uint8_t len, uint32_t timeout) {
  poll(UBX_LOG, LOG_STRING, len, (uint8_t *)msg);
  if (ackNoError(timeout)) return true;
  return false;
}

bool  Gnss::logRetrieve(uint32_t index, uint32_t count) {
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

GnssSupport * Gnss::getSupportedGnss(uint32_t timeout) {
  poll(UBX_MON, MON_GNSS);
  if (pollNoError(timeout))	return (GnssSupport *)payload;
  return NULL;
}

GnssConf * Gnss::getGnss(uint32_t timeout) {
  poll(UBX_CFG, CFG_GNSS);
  if (pollNoError(timeout))	return (GnssConf *)payload;
  return NULL;
}

bool Gnss::setGnss(MajorGnss gnss, bool enableSBAS, bool enableIMES, uint32_t timeout) {
  GnssConf * gnssConf = NULL;
  GnssCfg * c = NULL;
  poll(UBX_CFG, CFG_GNSS);
  if (pollNoError(timeout)) {
	  gnssConf = (GnssConf*)(payload);
	  for (uint8_t i = 0; i < gnssConf->numConfigBlocks; i++) {
		  c = (GnssCfg*)(payload + sizeof(GnssConf) + sizeof(GnssCfg) * i);
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
  else return false;
  poll(UBX_CFG, CFG_GNSS, sizeof(GnssConf), (uint8_t *)gnssConf);
  if (ackNoError(timeout)) return true;
  return false;
}

NavRate * Gnss::getNavRate(uint32_t timeout) {
  poll(UBX_CFG, CFG_RATE);
  if (pollNoError(timeout))	return (NavRate *)payload;
  return NULL;	
}

bool Gnss::setNavRate(uint16_t measurementRate, uint16_t navSolRate, TimeRef timeRef, uint32_t timeout) {
  NavRate * rate = NULL;
  poll(UBX_CFG, CFG_RATE);
  if (pollNoError(timeout)) rate = (NavRate*)(payload);
  else return false;
  rate->rate = measurementRate; //ms
  rate->navSolRate = navSolRate; //cycles - number of measurements for each NavSol, max 127.
  rate->timeRef = timeRef; 
  poll(UBX_CFG, CFG_RATE, sizeof(NavRate), (uint8_t *)rate);
  if (ackNoError(timeout)) return true;
  return false;
}

TimePulse * Gnss::getTimePulse(uint32_t timeout) {
  poll(UBX_CFG, CFG_TP5);
  if (pollNoError(timeout))	return (TimePulse *)payload;
  return NULL;
}

bool Gnss::setTimePulse(uint32_t pulse_period, uint32_t pulse_len, uint32_t pulse_period_locked, uint32_t pulse_len_locked, int32_t delay, TimePulseFlags flags, uint32_t timeout) {
  TimePulse * tp = NULL;
  poll(UBX_CFG, CFG_TP5);
  if (pollNoError(timeout)) tp = (TimePulse*)(payload);
  else return false;
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
  poll(UBX_CFG, CFG_TP5, sizeof(TimePulse), (uint8_t *)tp);
  if (ackNoError(timeout)) return true;
  return false;
}

NavPvt * Gnss::getNavPvt(uint32_t timeout) {
  poll(UBX_NAV, NAV_PVT);
  if (pollNoError(timeout))	return (NavPvt *)payload;
  return NULL;
}

NavSat * Gnss::getNavSat(uint32_t timeout) {
  poll(UBX_NAV, NAV_SAT);
  if (pollNoError(timeout))	return (NavSat *)payload;
  return NULL;
}

NavTimeUtc * Gnss::getNavTimeUtc(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMEUTC);
  if (pollNoError(timeout))	return (NavTimeUtc *)payload;
  return NULL;
}

NavClock * Gnss::getNavClock(uint32_t timeout) {
  poll(UBX_NAV, NAV_CLOCK);
  if (pollNoError(timeout))	return (NavClock *)payload;
  return NULL;
}

NavDGPS * Gnss::getNavDgps(uint32_t timeout) {
  poll(UBX_NAV, NAV_DGPS);
  if (pollNoError(timeout))	return (NavDGPS *)payload;
  return NULL;
}

NavDOP * Gnss::getNavDop(uint32_t timeout) {
  poll(UBX_NAV, NAV_DOP);
  if (pollNoError(timeout))	return (NavDOP *)payload;
  return NULL;
}

NavGeofence * Gnss::getNavGeofence(uint32_t timeout) {
  poll(UBX_NAV, NAV_GEOFENCE);
  if (pollNoError(timeout))	return (NavGeofence *)payload;
  return NULL;  
}

GeoFences * Gnss::getCfgGeofences(uint32_t timeout) {
  poll(UBX_CFG, CFG_GEOFENCE);
  if (pollNoError(timeout))	return (GeoFences *)payload;
  return NULL;
}

bool Gnss::setCfgGeofence(GeoFence * geofence, uint8_t confidenceLevel, uint32_t timeout) {
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

NavODO * Gnss::getNavOdo(uint32_t timeout) {
  poll(UBX_NAV, NAV_ODO);
  if (pollNoError(timeout))	return (NavODO *)payload;
  return NULL;  
}

bool Gnss::resetOdo(uint32_t timeout) {
  poll(UBX_NAV, NAV_RESETODO);
  if (ackNoError(timeout))	return true;
  return false;  
}

NavOrb * Gnss::getNavOrb(uint32_t timeout) {
  poll(UBX_NAV, NAV_ORB);
  if (pollNoError(timeout))	return (NavOrb *)payload;
  return NULL;  
}

NavPosLlh * Gnss::getNavPosLlh(uint32_t timeout) {
  poll(UBX_NAV, NAV_POSLLH);
  if (pollNoError(timeout))	return (NavPosLlh *)payload;
  return NULL; 
}

NavSbas * Gnss::getNavSbas(uint32_t timeout) {
  poll(UBX_NAV, NAV_SBAS);
  if (pollNoError(timeout))	return (NavSbas *)payload;
  return NULL; 
}

NavSlas * Gnss::getNavSlas(uint32_t timeout) {
  poll(UBX_NAV, NAV_SLAS);
  if (pollNoError(timeout))	return (NavSlas *)payload;
  return NULL; 
}

NavTimeBdsGal * Gnss::getNavTimeBds(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMEBDS);
  if (pollNoError(timeout))	return (NavTimeBdsGal *)payload;
  return NULL;
}

NavTimeBdsGal * Gnss::getNavTimeGal(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMEGAL);
  if (pollNoError(timeout))	return (NavTimeBdsGal *)payload;
  return NULL;
}

NavTimeGps * Gnss::getNavTimeGps(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMEGPS);
  if (pollNoError(timeout))	return (NavTimeGps *)payload;
  return NULL;
}

NavTimeGlo * Gnss::getNavTimeGlo(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMEGLO);
  if (pollNoError(timeout))	return (NavTimeGlo *)payload;
  return NULL;
}

NavTimeLs * Gnss::getNavTimeLs(uint32_t timeout) {
  poll(UBX_NAV, NAV_TIMELS);
  if (pollNoError(timeout))	return (NavTimeLs *)payload;
  return NULL;
}

TimTm * Gnss::getTimTm(uint32_t timeout) {
  poll(UBX_TIM, TIM_TM2);
  if (pollNoError(timeout))	return (TimTm *)payload;
  return NULL;
}

TimeTP * Gnss::getTimeTp(uint32_t timeout) {
  poll(UBX_TIM, TIM_TP);
  if (pollNoError(timeout))	return (TimeTP *)payload;
  return NULL;
}

//NMEA messages
void Gnss::getGNRMC(GNRMC * data) {
	char NS = '\0', EW = '\0';
	string time, date, lat, lon, rmc[13];
	memset(&(data->dateTime), 0, sizeof(DateTime));
	memset(&(data->lat), 0, sizeof(Latitude));
	memset(&(data->lon), 0, sizeof(Longitude));
	data->SOG = 0; data->COG = 0; data->magVar = 0; data->magVarEW = '\0'; data->status = 'V'; data->fixType = 'N';
	
	split(rmc, 13, nmeaPayload);
	
	for (uint8_t i = 1; i < 13; i++) {
		if (rmc[i] != "")
		switch (i) {
			case 1: time = rmc[i]; break;
			case 2: 
				data->status = rmc[i][0];
				//if (data->status == 'V') return false;
				break;
			case 3: lat = rmc[i]; break;
			case 4: NS = rmc[i].at(0); break;
			case 5: lon = rmc[i]; break;
			case 6: EW = rmc[i][0]; break;
			case 7: data->SOG = stof(rmc[i], nullptr); break;
			case 8: data->COG = stof(rmc[i], nullptr); break;
			case 9: date = rmc[i]; break;
			case 10: data->magVar = stof(rmc[i], nullptr); break;
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
	utcTime = mktime(gps2tm(&(data->dateTime), &tm_time));
	nmeaLonToDMS(&(data->lon), lon, EW);
	nmeaLatToDMS(&(data->lat), lat, NS);
}

void Gnss::getGNGGA(GNGGA * data) {
	memset(&(data->dateTime), 0, sizeof(DateTime));
	memset(&(data->lat), 0, sizeof(Latitude));
	memset(&(data->lon), 0, sizeof(Longitude));
	data->fixType = NO_FIX_TYPE; data->numSV = 0; data->hDOP = 0; data->alt = 0; data->geoidEllipsoidDiff = 0; data->ageDiffCorr = 0; data->diffCorrStationId = 0;
	string time = "", lat, lon, gga[15];
	char NS = '\0', EW = '\0';
	
	split(gga, 15, nmeaPayload);
	
	for (uint8_t i = 1; i < 15; i++) {
		if (gga[i] != "")
		switch (i) {
			case 1: time = gga[i]; break;
			case 2: lat = gga[i]; break;
			case 3: NS = gga[i][0]; break;
			case 4: lon = gga[i]; break;
			case 5: EW = gga[i][0]; break;
			case 6: 
				data->fixType = stoi(gga[i], nullptr, 10); 
				//if (data->fixType == NO_FIX_TYPE) return false;
				break;
			case 7: data->numSV = stoi(gga[i], nullptr, 10); break;
			case 8: data->hDOP = stof(gga[i], nullptr); break;
			case 9: data->alt = stof(gga[i], nullptr); break;
			case 10: break;
			case 11: data->geoidEllipsoidDiff = stof(gga[i], nullptr); break;
			case 12: break;
			case 13: data->ageDiffCorr = stoi(gga[i], nullptr, 10); break;
			case 14: data->diffCorrStationId = stoi(gga[i], nullptr, 10); break;
		}		
	}
	nmeaToUtc(&(data->dateTime), nmeaDate, time);
	nmeaLonToDMS(&(data->lon), lon, EW);
	nmeaLatToDMS(&(data->lat), lat, NS);
}

void Gnss::getGNGLL(GNGLL * data) {
	memset(&(data->dateTime), 0, sizeof(DateTime));
	memset(&(data->lat), 0, sizeof(Latitude));
	memset(&(data->lon), 0, sizeof(Longitude));
	data->fixType = 'N'; data->status = 'V';
	string time = "", lat, lon, gll[8];
	char NS = '\0', EW = '\0';

	split(gll, 8, nmeaPayload);

	for (uint8_t i = 1; i < 8; i++) {
		if (gll[i] != "")
		switch (i) {
			case 1: lat = gll[i]; break;
			case 2: NS = gll[i].at(0); break;
			case 3: lon = gll[i]; break;
			case 4: EW = gll[i].at(0); break;
			case 5: time = gll[i]; break;
			case 6: 
				data->status = gll[i].at(0); 
				//if (data->status == 'V') return false;
				break;
			case 7: 
				data->fixType = gll[i].at(0); 
				//if (data->fixType == 'N') return false;
				break;
		}		
	}
	nmeaToUtc(&(data->dateTime), nmeaDate, time);
	nmeaLonToDMS(&(data->lon), lon, EW);
	nmeaLatToDMS(&(data->lat), lat, NS);
}

void Gnss::getGNVTG(GNVTG * data) {
	data->COGt = 0; data->COGm = 0; data->SOGkt = 0; data->SOGkmh = 0; data->fixType = 'N';
	string vtg[10];

	split(vtg, 10, nmeaPayload);

	for (uint8_t i = 1; i < 10; i++) {
		if (vtg[i] != "")
		switch (i) {
			case 1: data->COGt = stof(vtg[i], nullptr); break;
			case 2: break;
			case 3: data->COGm = stof(vtg[i], nullptr); break;
			case 4: break;
			case 5: data->SOGkt = stof(vtg[i], nullptr); break;
			case 6: break;
			case 7: data->SOGkmh = stof(vtg[i], nullptr); break;
			case 8: break;
			case 9: data->fixType =  vtg[i].at(0);
				//if (data->fixType == 'N') return false;
				break;
		}		
	}
}

void Gnss::getGNGSA(GNGSA * data) {
	memset(&(data->svId), 0, sizeof(uint8_t[12]));
	data->fixType = FIX_NOT_AVAILABLE; data->opMode = 'A'; data->pDOP = 0; data->hDOP = 0; data->vDOP = 0;
	string s, gsa[18];

	split(gsa, 18, nmeaPayload);

	for (uint8_t i = 1; i < 18; i++) {
		if (gsa[i] != "")
		switch (i) {
			case 1: data->opMode = gsa[i].at(0); break;
			case 2: 
				data->fixType = stoi(gsa[i], nullptr, 10); 
				//if (data->fixType == FIX_NOT_AVAILABLE) return;
				break;
			case 15: data->pDOP = stof(gsa[i], nullptr); break;
			case 16: data->hDOP = stof(gsa[i], nullptr); break;
			case 17: data->vDOP = stof(gsa[i], nullptr); break;
//			case 18: data->gnssId = stoi(nmeaPayload.substr(from, len), nullptr, 10); break;
			default:
					s = gsa[i];
					if (s == "") data->svId[i - 3] = 0;
					//else if (s.startsWith("0")) data->svId[i - 3] = stoi(s.substr(1), nullptr, 10);
					else data->svId[i - 3] = stoi(s, nullptr, 10); 
		}		
	}
}

void Gnss::getGNGSV(GNGSV * data) {
	memset(&(data->svAttr), 0, sizeof(uint8_t[4]));
	data->gnss = ""; data->totalMsg = 0; data->msgNum = 0; data->numSV = 0;
	string gsv[20];

	split(gsv, 20, nmeaPayload);

	for (uint8_t i = 0; i < 20; i++) {
		if (gsv[i] != "")
		switch (i) {
			case 0:
				switch(gsv[i].c_str()[1]) {
					case 'P': data->gnss = "GPS"; break;
					case 'L': data->gnss = "GLONASS"; break;
					case 'A': data->gnss = "Galileo"; break;
					case 'B': data->gnss = "BeiDuo"; break;					
					case 'N': data->gnss = "Combo"; break;					
				}
				break;
			case 1: data->totalMsg = stoi(gsv[i], nullptr, 10); break;
			case 2: data->msgNum = stoi(gsv[i], nullptr, 10); break;
			case 3: data->numSV = stoi(gsv[i], nullptr, 10); break;
			case 4: data->svAttr[0].svId = stoi(gsv[i], nullptr, 10); break;
			case 5: data->svAttr[0].elev = stoi(gsv[i], nullptr, 10); break;
			case 6: data->svAttr[0].azim = stoi(gsv[i], nullptr, 10); break;
			case 7: data->svAttr[0].cno = stoi(gsv[i], nullptr, 10); break;
			case 8: data->svAttr[1].svId = stoi(gsv[i], nullptr, 10); break;
			case 9: data->svAttr[1].elev = stoi(gsv[i], nullptr, 10); break;
			case 10: data->svAttr[1].azim = stoi(gsv[i], nullptr, 10); break;
			case 11: data->svAttr[1].cno = stoi(gsv[i], nullptr, 10); break;
			case 12: data->svAttr[2].svId = stoi(gsv[i], nullptr, 10); break;
			case 13: data->svAttr[2].elev = stoi(gsv[i], nullptr, 10); break;
			case 14: data->svAttr[2].azim = stoi(gsv[i], nullptr, 10); break;
			case 15: data->svAttr[2].cno = stoi(gsv[i], nullptr, 10); break;
			case 16: data->svAttr[3].svId = stoi(gsv[i], nullptr, 10); break;
			case 17: data->svAttr[3].elev = stoi(gsv[i], nullptr, 10); break;
			case 18: data->svAttr[3].azim = stoi(gsv[i], nullptr, 10); break;
			case 19: data->svAttr[3].cno = stoi(gsv[i], nullptr, 10); break;
//			case 20: data->gnssId = stoi(nmeaPayload.substr(from, len), nullptr, 10); break;
		}		
	}
}

void Gnss::getGNGST(GNGST * data) {
	memset(&(data->dateTime), 0, sizeof(DateTime));
	data->rangeRms = 0; data->stdMajor = 0; data->stdMinor = 0; data->orient = 0; data->stdLat = 0; data->stdLon = 0; data->stdAlt = 0;
	string time = "";
	string gst[9];

	split(gst, 9, nmeaPayload);

	for (uint8_t i = 1; i < 9; i++) {
		if (gst[i] != "")
		switch (i) {
			case 1: time = gst[i]; break;
			case 2: data->rangeRms = stof(gst[i], nullptr); break;
			case 3: data->stdMajor = stof(gst[i], nullptr); break;
			case 4: data->stdMinor = stof(gst[i], nullptr); break;
			case 5: data->orient = stof(gst[i], nullptr); break;
			case 6: data->stdLat = stof(gst[i], nullptr); break;
			case 7: data->stdLon = stof(gst[i], nullptr); break;
			case 8: data->stdAlt = stof(gst[i], nullptr); break;
		}		
	}
	nmeaToUtc(&(data->dateTime), nmeaDate, time);
}

//Dual ground/water distance
void Gnss::getGNVLW(GNVLW * data) {
	data->twd = 0; data->wd = 0; data->tgd = 0; data->gd = 0; data->twdUnit = 'N'; data->wdUnit = 'N'; data->tgdUnit = 'N'; data->gdUnit = 'N';
	string vlw[8];

	split(vlw, 8, nmeaPayload);

	for (uint8_t i = 1; i < 8; i++) {
		if (vlw[i] != "")
		switch (i) {
			case 1: data->twd = stof(vlw[i], nullptr); break;
			case 2: data->twdUnit = vlw[i].at(0); break;
			case 3: data->wd = stof(vlw[i], nullptr); break;
			case 4: data->wdUnit = vlw[i].at(0); break;
			case 5: data->tgd = stof(vlw[i], nullptr); break;
			case 6: data->tgdUnit = vlw[i].at(0); break;
			case 7: data->gd = stof(vlw[i], nullptr); break;
			case 8: data->gdUnit = vlw[i].at(0); break;
		}		
	}
}

//GNSS fix data
void Gnss::getGNGNS(GNGNS * data) {
	memset(&(data->dateTime), 0, sizeof(DateTime));
	memset(&(data->lat), 0, sizeof(Latitude));
	memset(&(data->lon), 0, sizeof(Longitude));
	data->fixType[0] = 'N'; data->fixType[1] = 'N';
	data->numSV = 0; data->hDOP = 0; data->alt = 0; data->geoidEllipsoidDiff = 0; data->ageDiffCorr = 0; data->diffCorrStationId = 0;
	string time = "", lat = "", lon = "";
	char NS = '\0', EW = '\0';
	string gns[13];

	split(gns, 13, nmeaPayload);

	for (uint8_t i = 1; i < 13; i++) {
		if (gns[i] != "")
		switch (i) {
			case 1: time = gns[i]; break;
			case 2: lat = gns[i]; break;
			case 3: NS = gns[i].at(0); break;
			case 4: lon = gns[i]; break;
			case 5: EW = gns[i].at(0); break;
			case 6: data->fixType[0] = gns[i].at(0); data->fixType[1] = gns[i].at(1); break;
			case 7: data->numSV = stoi(gns[i], nullptr, 10); break;
			case 8: data->hDOP = stof(gns[i], nullptr); break;
			case 9: data->alt = stof(gns[i], nullptr); break;
			case 10: data->geoidEllipsoidDiff = stof(gns[i], nullptr); break;
			case 11: data->ageDiffCorr = stof(gns[i], nullptr); break;
			case 12: data->diffCorrStationId = stoi(gns[i], nullptr, 10); break;
			//case 13: data->navStatus = gns[i].at(0); break;
		}		
	}
	nmeaToUtc(&(data->dateTime), nmeaDate, time);
	nmeaLonToDMS(&(data->lon), lon, EW);
	nmeaLatToDMS(&(data->lat), lat, NS);
}

//Time and Date
void Gnss::getGNZDA(GNZDA * data) {
	memset(&(data->dateTime), 0, sizeof(DateTime));
	data->utcOffsetHours = 0; data->utcOffsetMinutes = 0;
	string time = "", date = "";
	string zda[7];

	split(zda, 7, nmeaPayload);

	for (uint8_t i = 1; i < 7; i++) {	
		if (zda[i] != "")
		switch (i) {
			case 1: time = zda[i]; break;
			case 2: date = zda[i]; break;
			case 3: date += zda[i]; break;
			case 4: if (zda[i].length() >= 4) date += zda[i].substr(2,2); break;
			case 5: data->utcOffsetHours = stoi(zda[i], nullptr, 10); break;
			case 6: data->utcOffsetMinutes = stoi(zda[i], nullptr, 10); break;
		}		
	}
	nmeaToUtc(&(data->dateTime), date, time);
}

//Info messages such as Error, Warning, Notice
void Gnss::getGNTXT(GNTXT* data) {
	string txt[5];
	data->text = ""; data->msgNum = 0; data->numMsg = 0; data->msgType = 0;

	split(txt, 5, nmeaPayload);

	for (uint8_t i = 1; i < 5; i++) {
		if (txt[i] != "")
			switch (i) {
				case 1: data->numMsg = stoi(txt[i], nullptr, 10); break;
				case 2: data->msgNum = stoi(txt[i], nullptr, 10); break;
				case 3: data->msgType = stoi(txt[i], nullptr, 10); break;
				case 4: data->text = txt[i]; break;
			}
	}
}

//Gnss fault detection
void Gnss::getGbs(GBS* data) {
	string gbs[11];
	memset(&(data->dateTime), 0, sizeof(DateTime));
	data->errLat = 0; data->errLon = 0; data->bias = 0; data->errAlt = 0;
	data->signalId = 0; data->stddev = 0; data->svId = 0; data->systemId = 0; data->prob = 0;
	string time = "";

	split(gbs, 11, nmeaPayload);

	for (uint8_t i = 1; i < 11; i++) {
		if (gbs[i] != "")
			switch (i) {
				case 1: time = gbs[i]; break;
				case 2: data->errLat = stof(gbs[i], nullptr); break;
				case 3: data->errLon = stof(gbs[i], nullptr); break;
				case 4: data->errAlt = stof(gbs[i], nullptr); break;
				case 5: data->svId = stoi(gbs[i], nullptr, 10); break;
				case 6: data->prob = stof(gbs[i], nullptr); break;
				case 7: data->bias = stof(gbs[i], nullptr); break;
				case 8: data->stddev = stof(gbs[i], nullptr); break;
				case 9: data->systemId = stoi(gbs[i], nullptr, 10); break;
				case 10: data->signalId = stoi(gbs[i], nullptr, 10); break;
			}
	}
	nmeaToUtc(&(data->dateTime), nmeaDate, time);
}

void Gnss::getGrs(GRS* data) {
	string grs[17];
	memset(&(data->dateTime), 0, sizeof(DateTime));
	data->signalId = 0; data->systemId = 0;
	for (uint8_t i = 0; i < 12; i++) data->residual[i] = 0;
	string time = "";

	split(grs, 17, nmeaPayload);

	for (uint8_t i = 1; i < 17; i++) {
		if (grs[i] != "")
			switch (i) {
				case 1: time = grs[i]; break;
				case 2: data->mode = stoi(grs[i], nullptr, 10); break;
				case 3: data->residual[0] = stof(grs[i], nullptr); break;
				case 4: data->residual[1] = stof(grs[i], nullptr); break;
				case 5: data->residual[2] = stof(grs[i], nullptr); break;
				case 6: data->residual[3] = stof(grs[i], nullptr); break;
				case 7: data->residual[4] = stof(grs[i], nullptr); break;
				case 8: data->residual[5] = stof(grs[i], nullptr); break;
				case 9: data->residual[6] = stof(grs[i], nullptr); break;
				case 10: data->residual[7] = stof(grs[i], nullptr); break;
				case 11: data->residual[8] = stof(grs[i], nullptr); break;
				case 12: data->residual[9] = stof(grs[i], nullptr); break;
				case 13: data->residual[10] = stof(grs[i], nullptr); break;
				case 14: data->residual[11] = stof(grs[i], nullptr); break;
				case 15: data->systemId = stoi(grs[i], nullptr, 10); break;
				case 16: data->signalId = stoi(grs[i], nullptr, 10); break;
			}
	}
	nmeaToUtc(&(data->dateTime), nmeaDate, time);
}

void Gnss::getDtm(DTM* data) {
	string dtm[9]; 
	data->lat = 0; data->lon = 0; data->alt = 0; data->subDatum = ""; data->datum = ""; data->refDatum = "";

	split(dtm, 9, nmeaPayload);

	for (uint8_t i = 1; i < 9; i++) {
		if (dtm[i] != "")
			switch (i) {
				case 1: data->datum = dtm[i]; break;
				case 2: break;
				case 3: data->lat = stof(dtm[i], nullptr); break;
				case 4: data->NS = dtm[i].at(0); break;
				case 5: data->lon = stof(dtm[i], nullptr); break;
				case 6: data->EW = dtm[i].at(0); break;
				case 7: data->alt = stof(dtm[i], nullptr); break;
				case 8: data->refDatum = dtm[i]; break;
			}
	}
}

//UBX proprietary messages
void Gnss::getPubxPosition(PubxPosition * data) {
	memset(&(data->dateTime), 0, sizeof(DateTime));
	memset(&(data->lat), 0, sizeof(Latitude));
	memset(&(data->lon), 0, sizeof(Longitude));
	data->alt = 0;  data->hAcc = 0; data->vAcc = 0; data->sog = 0; data->cog = 0; data->vVel = 0; data->ageDiffCorr = 0;
	data->hDOP = 0; data->vDOP = 0; data->tDOP = 0; data->numSV = 0; data->drUsed = false; 
	string time = "", date = nmeaDate, lat = "", lon = "";
	char NS = '\0', EW = '\0';
	string pos[21], p = "";
	sprintf(data->fixType, "NF");

	split(pos, 21, nmeaPayload);

	for (uint8_t i = 2; i < 21; i++) {
		if (pos[i] != "")
		switch (i) {
			case 2: time = pos[i]; break;
			case 3: lat = pos[i]; break;
			case 4: NS = pos[i].at(0); break;
			case 5: lon = pos[i]; break;
			case 6: EW = pos[i].at(0); break;
			case 7: data->alt = stof(pos[i], nullptr); break;
			case 8: data->fixType[0] = pos[i].at(0); data->fixType[1] = pos[i].at(1); break;
			case 9: data->hAcc = stof(pos[i], nullptr); break;
			case 10: data->vAcc = stof(pos[i], nullptr); break;
			case 11: data->sog = stof(pos[i], nullptr); break;
			case 12: data->cog = stoi(pos[i], nullptr, 10); break;
			case 13: data->vVel = stof(pos[i], nullptr); break;
			case 14: data->ageDiffCorr = stoi(pos[i], nullptr, 10); break;
			case 15: data->hDOP = stof(pos[i], nullptr); break;
			case 16: data->vDOP = stof(pos[i], nullptr); break;
			case 17: data->tDOP = stof(pos[i], nullptr); break;
			case 18: data->numSV = stoi(pos[i], nullptr, 10); break;
			case 19: break;				
			case 20: if (stoi(pos[i], nullptr, 10) > 0) data->drUsed = true; break;
		}		
	}
	nmeaToUtc(&(data->dateTime), date, time);
	nmeaLonToDMS(&(data->lon), lon, EW);
	nmeaLatToDMS(&(data->lat), lat, NS);
}

void Gnss::getPubxTime(PubxTime * data) {
	memset(&(data->dateTime), 0, sizeof(DateTime));
	data->utcTow = 0; data->utcWeek = 0; data->leapSec = 0; data->clkBias = 0; data->clkDrift = 0; data->tpGran = 0; data->leapSecSrc = 'S';
	string time = "", date = "", leapSec = "";
	string tim[10];

	split(tim, 10, nmeaPayload);

	for (uint8_t i = 2; i < 10; i++) {
		if (tim[i] != "")
		switch (i) {
			case 2: time = tim[i]; break;
			case 3: date = tim[i]; break;
			case 4: data->utcTow = stoi(tim[i], nullptr, 10); break;
			case 5: data->utcWeek = stoi(tim[i], nullptr, 10); break;
			case 6: leapSec = tim[i];
				if (leapSec.back() == 'D') { 
					data->leapSecSrc = 'D';
					leapSec.erase(0, leapSec.length() - 1);
					data->leapSec = stoi(leapSec, nullptr, 10);
				} else data->leapSec = stoi(leapSec, nullptr, 10); break;
			case 7: data->clkBias = stoi(tim[i], nullptr, 10); break;
			case 8: data->clkDrift = stoi(tim[i], nullptr, 10); break;
			case 9: data->tpGran = stoi(tim[i], nullptr, 10); break;
		}		
	}
	nmeaToUtc(&(data->dateTime), date, time);
}

//helper functions
void split(string msg[], uint8_t array_size, string pload) {
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

void nmeaLatToDMS(Latitude * latitude, string lat, char NS) {
	if (lat.length() < 2) return;
	if (lat.substr(2) == "" || lat.substr(0, 2) == "") return;
	float m = stof(lat.substr(2), nullptr);
	latitude->deg = stoi(lat.substr(0, 2), nullptr, 10);
	latitude->min = (uint8_t)m;
	latitude->sec = (m - (float)(latitude->min)) * 60;
	latitude->NS = NS;
}

void nmeaLonToDMS(Longitude * longitude, string lon, char EW) {
	if (lon.length() < 3) return;
	if (lon.substr(3) == "" || lon.substr(0, 3) == "") return;
	float m = stof(lon.substr(3), nullptr);
	longitude->deg = stoi(lon.substr(0, 3), nullptr, 10);
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

string dmsLatToStr(Latitude * lat) {
   char l[16];
   memset(l, 0, 16);
   uint8_t deg[3] = { 0xc2, 0xb0 };
   char apo = 39, quotes = 34;
   sprintf(l, "%.u%.2s%.u%.1s%.2f%.1s%c", lat->deg, (char *)(&deg), lat->min, &apo, (double)lat->sec, &quotes, lat->NS);
   return string(l);
}

string dmsLonToStr(Longitude * lon) {
   char l[16];
   memset(l, 0, 16);
   uint8_t deg[3] = { 0xc2, 0xb0 };
   char apo = 39, quotes = 34;
   sprintf(l, "%.u%.2s%.u%.1s%.2f%.1s%c", lon->deg, (char *)(&deg), lon->min, &apo, (double)lon->sec, &quotes, lon->EW);
   return string(l);
}

void nmeaToUtc(DateTime * dt, string date, string time) {
	if (date.length() < 6) return;
	if (date.substr(0, 2) != "") dt->day = stoi(date.substr(0, 2), nullptr, 10);
	if (date.substr(2, 2) != "") dt->month = stoi(date.substr(2, 2), nullptr, 10);
	if (date.substr(4, 2) != "") dt->year = stoi(date.substr(4, 2), nullptr, 10);
	if (dt->year > 0) dt->year += 2000;
	if (time.substr(0, 2) != "") dt->hour = stoi(time.substr(0, 2), nullptr, 10);
	if (time.substr(2, 2) != "") dt->minute = stoi(time.substr(2, 2), nullptr, 10);
	if (time.substr(4, 2) != "") dt->second = stoi(time.substr(4, 2), nullptr, 10);
}

tm * gps2tm(DateTime * dt, tm * time_tm) {
	memset(time_tm, 0, sizeof(tm));
	time_tm->tm_year = dt->year - 1900;
	time_tm->tm_mon = dt->month - 1;
	time_tm->tm_mday = dt->day;
	time_tm->tm_hour = dt->hour;
	time_tm->tm_min = dt->minute;
	time_tm->tm_sec = dt->second;
	return time_tm;
}

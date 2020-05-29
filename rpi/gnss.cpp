#include <sys/stat.h>
#include "gnss.h"
#include <errno.h>
using namespace std;
 
Gnss::Gnss() {
	isReady = false;
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
	if (setenv("TZ", "", 1) != 0) printf("setenv() error: %s", strerror(errno));
	utcTime = time(NULL);
	cfgGnssOk = false; cfgInfOk = false; cfgLogfilterOk = false; cfgMsgOk = false; cfgNavOk = false;
	cfgOdoOk = false; cfgPmOk = false; cfgPmsOk = false; cfgPrtOk = false; cfgRateOk = false; cfgRstOk = false;
	cfgRxmOk = false;  cfgSbasOk = false; cfgSlasOk = false; cfgTpOk = false; cfgNmeaOk = false; cfgCfgOk = false;
	cfgGeofenceOk = false; resetOdoOk = false; logCreateOk = false; logEraseOk = false;
	disp_nav = true; disp_cfg = true; disp_tim = true; disp_nmea = true; disp_pubx = true, disp_err = true;
}

/*void Gnss::tp() {
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
}*/

int Gnss::begin(const char * dev, speed_t rate) {
	int flags;
	printf("opening dev %s\n", dev); 
	fd = open(dev, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		printf("error opening device %s: %s\n", dev, strerror(errno));  return -1;
	}
	//printf("getting cntl flags\n");
	if ((flags = fcntl(fd, F_GETFL)) == -1) {
		printf("fnctl GETFL error: %s", strerror(errno));  return -1;
	}
	//configure signal-driven non-blocking IO
	//printf("setting cntl flags\n");
	if (fcntl(fd, F_SETFL, flags | O_ASYNC | O_NONBLOCK) == -1) {
		printf("fnctl F_SETFL(flags | O_ASYNC | O_NONBLOCK) error: %s", strerror(errno));  return -1;
	}
	struct termios settings;
	memset(&settings, 0, sizeof(termios));
	//printf("getting termios settings\n");
	if (tcgetattr(fd, &settings) != 0) {
		printf("tcgetattr() error: %s\n", strerror(errno)); 
		return -1;
	}
	//printf("switching to raw processing\n");
	cfmakeraw(&settings); //switch to raw packet processing
	printf("setting termios speed\n");
	if (cfsetspeed(&settings, rate) != 0) {
		printf("cfsetspeed() error: %s\n", strerror(errno)); 
		return -1;
	}
	switch (rate) {
		case B4800: this->gnssPort.baudRate = BAUD_RATE_4800; break;
		case B9600: this->gnssPort.baudRate = BAUD_RATE_9600; break;
		case B19200: this->gnssPort.baudRate = BAUD_RATE_19200; break;
		case B38400: this->gnssPort.baudRate = BAUD_RATE_38400; break;
		case B57600: this->gnssPort.baudRate = BAUD_RATE_57600; break;
		case B115200: this->gnssPort.baudRate = BAUD_RATE_115200; break;
		case B230400: this->gnssPort.baudRate = BAUD_RATE_230400; break;
	}

	settings.c_iflag &= ~(IXON | IXOFF | IXANY); //no software flow control
//	settings.c_iflag |= IGNBRK; //Ignore BREAK condition on input./
//	settings.c_iflag |= IGNCR; //Ignore carriage return on input
//	settings.c_iflag &= ~(IXON | IXOFF | INPCK | ISTRIP | INLCR | IGNPAR); //no software flow control, 
																  //no parity checking, 
																  //do not strip off 8th bit
																  //do not translate NL to CR on input
																  //do not gnore framing errors and parity errors
//	settings.c_cflag |= CS8; //8-bit - set by cfmakeraw() as well as no parity (~PARENB)
	settings.c_cflag &= ~(CSTOPB | CRTSCTS);//1 stop bit, no hardware flow control
//	settings.c_cflag &= ~(CSTOPB | CRTSCTS | PARENB | HUPCL | CSIZE); //1 stop bit
													 //no hardware flow control
	                                                 //no parity
													 //do not hang up after last process closes the device
//	settings.c_cflag |= CREAD; //enable receiver
//	settings.c_cflag |= CLOCAL; //Ignore modem control lines

//	settings.c_oflag &= ~(OPOST | ONLCR | OCRNL | ONOCR | ONLRET | OFILL); //no implementation-defined output processing
											//do not map NL to CR-NL on output
											//do not map CR to NL on output
											//output CR at column 0
											//output CR
											//do not send fill characters for a delay, use a timed delay
											
//	settings.c_lflag &= ~(ISIG | ICANON | ECHO | TOSTOP | IEXTEN); //do not generate the corresponding signal, When any of the characters INTR, QUIT, SUSP, or DSUSP are received
	                                      //disable canonical mode
	                                      //do not echo input symbol
	                                      //do not send  the SIGTTOU signal to the process group of a background process which tries to write to its controlling termina
										  //no implementation-defined  input  processing
	settings.c_cc[VMIN] = 0; //minimum number of char to read
	settings.c_cc[VTIME] = 0; //timeout in decisec
	//printf("setting other termios flags\n");
	if ((tcsetattr(fd, TCSAFLUSH, &settings)) != 0) {
		printf("error in setting dev %s attributes: %s\n", dev, strerror(errno));
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
  printf("marking gps as ready\n");
  size_t len = strlen(dev) + 1;
  device = (char *)malloc(len);
  strncpy(device, dev, len);
  isReady = true;
  return 0;
}

void Gnss::end() {
	if (isReady) {
		isReady = false;
		tcdrain(fd);
		tcflush(fd, TCIFLUSH);
		close(fd);
		free(device);
	}
}

void Gnss::calculateChecksum(uint8_t msgClass, uint8_t msgId, uint16_t len, uint8_t * pload, Checksum * checksum) {
      checksum->checksum0 = 0;
	  checksum->checksum1 = 0;
      checksum->checksum0 += msgClass;
	  checksum->checksum1 += checksum->checksum0;
	  checksum->checksum0 += msgId;
	  checksum->checksum1 += checksum->checksum0;		
	  checksum->checksum0 += (uint8_t)(len & 0xFF); 
	  checksum->checksum1 += checksum->checksum0;
	  checksum->checksum0 += (uint8_t)(len >> 8);
	  checksum->checksum1 += checksum->checksum0;

	  if (pload != NULL) {
	  	  for (uint16_t i = 0; i < len; i++) {
		  	  checksum->checksum0 += pload[i];
			  checksum->checksum1 += checksum->checksum0;
		  }
	  }
}

void Gnss::nmeaVerifyChecksum()  {
	uint8_t x = 0, hex, high_byte = 0, low_byte = 0;
	uint16_t len = nmeaBuffer.length();
	const char * cstring = nmeaBuffer.c_str();

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
  Checksum checksum;

  while (read(fd, &c, 1) > 0) {
	  //printf("%c\n", c);
      if (offset < 6) {
			switch (offset) {
				case 0:
					//if (nmea && (c != '\r')) printf("%c", c); else printf("%X ", c); 
					if (c == NMEA_START) { 
						nmea = true; //possible NMEA message as '$' can appear in UBX messages as well
						nmeaChecksumNext = false; nmeaBuffer = ""; nmeaChecksum = ""; error = NO_ERROR;
					}
					else if (c == SYNC_CHARS[0]) { //most likely UBX message as NMEA seems to be using ASCII chars only
						offset = 1; nmea = false; endOfNavEpoch = false; error = NO_ERROR; continue;
					}
					break;
				case 1:
					//printf("%X ", c);
					if (c != SYNC_CHARS[1]) {
						offset = 0;
					} else offset = 2;
					break;					
				case 2:
					if (DEBUG_UBX) {
						printf(" Receiving UBX...");
						printf(" msgCls %X", c);
					}
					messageClass = c;
					offset = 3;
					break;
				case 3:					
					if (DEBUG_UBX) printf(", msgId %X", c);
					messageId = c;
					offset = 4;
					break;
				case 4: payloadLength = c; offset = 5; break;
				case 5: 
					payloadLength |= (((uint16_t)c) << 8); 
					if (DEBUG_UBX) printf(", len %u, payload: ", payloadLength);
					buffer = (uint8_t *)malloc(payloadLength);
					if (buffer == NULL && payloadLength > 0) { offset = 0; error = OUT_OF_MEMORY; return true;}
					offset = 6;
					break;
			}			
	        //offset++;
	  }
      else { //offset >= 6
			if ((offset < (payloadLength + 6))) {
				buffer[offset - 6] = c;
				if (DEBUG_UBX) printf("%X ", c);
				offset++;
			}
			else if (offset == payloadLength + 6) {
				calculateChecksum(messageClass, messageId, payloadLength, buffer, &checksum);
				if (DEBUG_UBX) printf(" calculated checksum0 %X, checksum1 %X;", checksum.checksum0, checksum.checksum1);
				if (c == checksum.checksum0) offset++;
				else {
					offset = 0;
					printf("checksum0 %X - wrong checksum0: %X", checksum.checksum0, c);
					error = CHECKSUM_ERROR; 
					return true;
				}
				if (DEBUG_UBX) printf(" received chksum0 %X", c);
			}
			else if (offset == payloadLength + 7) {
			  offset = 0;
			  if (DEBUG_UBX) printf(", chksum1 %X\n", c);
			  if (c == checksum.checksum1)	{
				if (messageClass == UBX_NAV && messageId == NAV_EOE) { 
					//printf("end of nav epoch\n");
					endOfNavEpoch = true; 
					iTOW = *(uint32_t *)buffer;
				}
				error = NO_ERROR;
				payload = (uint8_t *)malloc(payloadLength);
				if (payload == NULL && payloadLength > 0) { offset = 0; error = OUT_OF_MEMORY; return true; }
				memcpy(payload, buffer, payloadLength);
				free(buffer);
				return true;
			  }
			  else {
				printf("checksum1 %X - wrong checksum1: %X\n", checksum.checksum1, c);
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
				if (DEBUG_NMEA) printf("Received NMEA: %s*%s\n", nmeaBuffer.c_str(), nmeaChecksum.c_str());
				if (nmeaValid) {
					if (nmeaBuffer.length() >= 5) {
						cls = nmeaBuffer.substr(2, 3);
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
					if (nmeaBuffer.length() >= 4) {
						cls = nmeaBuffer.substr(0, 4);
						if (cls == "PUBX") {
							messageClass = UBX_PUBX;
							if (nmeaBuffer.length() >= 7) {
								msgId = nmeaBuffer.substr(5, 2);
								if (msgId == "41") messageId = PUBX_CONFIG;
								else if (msgId == "00") messageId = PUBX_POSITION;
								else if (msgId == "40") messageId = PUBX_RATE;
								else if (msgId == "03") messageId = PUBX_SVSTATUS;
								else if (msgId == "04") messageId = PUBX_TIME;
							}
						}
					}
					nmeaPayload = nmeaBuffer;
					return true;
				}
				else { printf("nmea checksum error: %s\n", nmeaBuffer.c_str());  return false; }
		    default: 
				if (!nmeaChecksumNext) nmeaBuffer += char(c);
				else nmeaChecksum += char(c);
				break;
		}
	  }
  }
  return false;
}

void Gnss::poll(uint8_t msgClass, uint8_t msgId, uint16_t payload_length, uint8_t* pload) {
	if (!isReady) {
		printf("GNSS receiver is not ready - please open a port first\n"); return;
	}

	int bytesWritten = 0, bufferOffset = 0;
	pollMessageClass = msgClass;
	pollMessageId = msgId;
	pollPayload = pload;
	pollPayloadLength = payload_length;
	Checksum checksum;
	calculateChecksum(pollMessageClass, pollMessageId, pollPayloadLength, pollPayload, &checksum);
	uint16_t packetSize = payload_length + 8;
	uint8_t* buf = (uint8_t*)malloc(packetSize);
	if (buf == NULL) {
		printf("poll(): malloc() error - buffer is null, packet size %u\n", packetSize); return;
	}
	buf[0] = SYNC_CHARS[0];
	buf[1] = SYNC_CHARS[1];
	buf[2] = msgClass;
	buf[3] = msgId;
	buf[4] = payload_length & 0xFF;
	buf[5] = payload_length >> 8;

	if (DEBUG_UBX) {
		printf("Sending UBX... msgCls %X", pollMessageClass);
		printf(", msgID %X", pollMessageId);
		printf(", len %u", pollPayloadLength);
	}
	if (pload != NULL)
	{
		if (DEBUG_UBX) printf(", payload: ");
		memcpy(buf + 6, pload, payload_length); 
		for (uint16_t i = 0; i < payload_length; i++)
			if (DEBUG_UBX) printf(" %X", buf[i + 6]);
	}
	if (DEBUG_UBX) {
		printf(", cksum0 %X", checksum.checksum0);
		printf(", cksum1 %X\n", checksum.checksum1);
	}
	buf[payload_length + 6] = checksum.checksum0;
	buf[payload_length + 7] = checksum.checksum1;

	while (bytesWritten < packetSize) {
		if ((bytesWritten = write(fd, &(buf[bufferOffset]), packetSize)) == -1) {
			printf("poll(): write() error %s\n", strerror(errno));
			return;
		}
		tcdrain(fd);
		bufferOffset += bytesWritten;
		packetSize -= bytesWritten;
	}
	free(buf);
	delay();
}

void Gnss::get() {
	if (ready()) {
		switch (error) {
			case NO_ERROR: 
			switch (messageClass) {
			case UBX_NAV:
				if (!disp_nav) break;
				switch (messageId) {
				case NAV_EOE: break;
				case NAV_CLOCK: navClock(); break;
				case NAV_DGPS: navDgps(); break;
				case NAV_DOP: navDop(); break;
				case NAV_GEOFENCE: navGeoFence(); break;
				case NAV_ODO: navOdo(); break;
				case NAV_ORB: navOrb(); break;
				case NAV_POSECEF: navPosEcef(); break;
				case NAV_POSLLH: navPosLlh(); break;
				case NAV_PVT: navPvt(); break;
				case NAV_SAT: navSat(); break;
				case NAV_SBAS: navSbas(); break;
				case NAV_SLAS: navSlas(); break;
				case NAV_STATUS: navStatus(); break;
				case NAV_TIMEBDS: navTimeBds(); break;
				case NAV_TIMEGAL: navTimeGal(); break;
				case NAV_TIMEGLO: navTimeGlo(); break;
				case NAV_TIMEGPS: navTimeGps(); break;
				case NAV_TIMELS: navTimeLs(); break;
				case NAV_TIMEUTC: navTimeUtc(); break;
				case NAV_VELECEF: navVelEcef(); break;
				case NAV_VELNED: navVelNed(); break;
				default: printf("UBX-NAV-%X has no function handler\n", messageId); break;
				} break;
			case UBX_CFG:
				switch (messageId) {
				case CFG_GNSS: cfgGnss(); break;
				case CFG_INF: cfgInf(); break;
				case CFG_LOGFILTER: cfgLogFilter(); break;
				case CFG_MSG: cfgMsg(); break;
				case CFG_NAV5: cfgNav(); break;
				case CFG_ODO: cfgOdo(); break;
				case CFG_PM2: cfgPm(); break;
				case CFG_PMS: cfgPms(); break;
				case CFG_PRT: cfgPrt(); break;
				case CFG_PWR: cfgPwr(); break; //deprecated in protocol > 17
				case CFG_RATE: cfgRate(); break;
				case CFG_RXM: cfgRxm(); break;
				case CFG_SBAS: cfgSbas(); break;
				case CFG_SLAS: cfgSlas(); break;
				case CFG_TP5: cfgTp(); break;
				case CFG_GEOFENCE: cfgGeoFence(); break;
				case CFG_NMEA: cfgNmea(); break;
				default: printf("UBX-CFG-%X has no function handler\n", messageId); break;
				} break;
			case UBX_MON:
				switch (messageId) {
				case MON_VER: monVer(MON_VER_EXTENSION_NUMBER); break;
				case MON_GNSS: monGnss(); break;
				case MON_PATCH: monPatch(); break;
				default: printf("UBX-MON-%X has no function handler\n", messageId); break;
				} break;
			case UBX_TIM:
				if (!disp_tim) break;
				switch (messageId) {
				case TIM_TM2: timTm(); break;
				case TIM_TP: timTp(); break;
				default: printf("UBX-TIM-%X has no function handler\n", messageId); break;
				} break;
			case UBX_LOG:
				switch (messageId) {
				case LOG_INFO: logInfo(); break;
				case LOG_RETRIEVEPOS: logRetrievePos(); break;
				case LOG_RETRIEVEPOSEXTRA: logRetrievePosExtra(); break;
				case LOG_RETRIEVESTRING: logRetrieveString(); break;
				case LOG_FINDTIME: logFindTime(); break;
				default: printf("UBX-LOG-%X has no function handler\n", messageId); break;
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
				switch (messageId) {
				case ACK_ACK: ackAck(); break;
				case ACK_NAK: ackNak(); break;
				} break;
			case UBX_NMEA:
				if (!disp_nmea) break;
				switch (messageId) {
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
				default: printf("UBX-NMEA-%X has no function handler\n", messageId); break;
				} break;
			case UBX_PUBX:
				if (!disp_pubx) break;
				switch (messageId) {
				case PUBX_POSITION: pubxPosition(); break;
				case PUBX_SVSTATUS: pubxSvStatus(); break;
				case PUBX_TIME: pubxTime(); break;
				default: printf("UBX-PUBX-%X has no function handler\n", messageId); break;
				} break;
			default: printf("MsgClass %X, msgId %X has no function handler\n", messageClass, messageId);
			} break;
			case CHECKSUM_ERROR: 
				poll(pollMessageClass, pollMessageId, pollPayloadLength, pollPayload); 
				if (disp_err)
					printf("chksum err: msgClass %x, msgId %x\n", pollMessageClass, pollMessageId); 
				break;
			case OUT_OF_MEMORY: if (disp_err) printf("out of memory\n"); break;
			default: if (disp_err) printf("MsgClass %X, msgId %X: unknown error\n", messageClass, messageId);
		}
	}
}

void Gnss::navClock() {
  gnssNavClock = *((NavClock *)payload);
  printf("u-blox Clock: iTow %ums, bias %dns, drift %dns/s, tAcc %uns, fAcc %ups/s\n", gnssNavClock.iTOW, gnssNavClock.clkB, gnssNavClock.clkD, gnssNavClock.tAcc, gnssNavClock.fAcc);
  free(payload);
}

void Gnss::navDgps() {
  DGPSCorrData * p = NULL;
  NavDGPS * dgps = (NavDGPS *)payload;
  char s[16]; memset(s, 0, sizeof(s));
  switch (dgps->status) {
  case 0: sprintf(s, "none"); break;
  case 1: sprintf(s, "PR+PRR"); break;
  }
  
  printf("DGPS: iTOW %u, age %dms, baseId %d, baseHealth %d, numCh %u, status %s\n", dgps->iTOW, dgps->age, dgps->baseId, dgps->baseHealth, dgps->numCh, s);
  for (uint8_t i = 0; i < dgps->numCh; i++) {
    p = (DGPSCorrData *)(payload + sizeof(NavDGPS) * i);
    if (p->flags.used) {
		printf("DGPS stationId %u, health %u, status %u: svid %u, channel %u, used %u, age %ums, prc %fm, prrc %fm/s\n", dgps->baseId, dgps->baseHealth, dgps->status, p->svid, p->flags.channel, p->flags.used, p->ageC, (double)(p->prc), (double)(p->prrc));		
	}
  }
  free(payload);
}

void Gnss::navDop() {
	gnssNavDop = *((NavDOP *)payload);
	printf("DOP iTOW %u, gDOP %u, pDOP %u, tDOP %u, vDOP %u, hDOP %u, nDOP %u, eDOP %u 10^-2\n", gnssNavDop.iTOW, gnssNavDop.gDOP, gnssNavDop.pDOP, gnssNavDop.tDOP, gnssNavDop.vDOP, gnssNavDop.hDOP, gnssNavDop.nDOP, gnssNavDop.eDOP);
  free(payload);
}

void Gnss::navGeoFence() {
	gnssNavGeoFence = *((NavGeofence *)payload);
	
	char g[8];

	switch(gnssNavGeoFence.combState) {
		case UKNOWN_GEOFENCE_STATE: sprintf(g, "Unknown"); break;
		case INSIDE_GEOFENCE_STATE: sprintf(g, "Inside"); break;
		case OUTSIDE_GEOFENCE_STATE: sprintf(g, "Outside"); break;
	}
	printf("GeoFence iTOW %u, status %u, numFences %u, combState %s\n", gnssNavGeoFence.iTOW, gnssNavGeoFence.status, gnssNavGeoFence.numFences, g);
	
	for (uint8_t i = 0; i < gnssNavGeoFence.numFences; i++) {
		switch(*(payload + sizeof(NavGeofence) + 2 * i)) {
			case UKNOWN_GEOFENCE_STATE: sprintf(g, "Unknown"); break;
			case INSIDE_GEOFENCE_STATE: sprintf(g, "Inside"); break;
			case OUTSIDE_GEOFENCE_STATE: sprintf(g, "Outside"); break;
		}
		printf("fenceId %u: state %s ", i, g);		
	}
	printf("\n");
  free(payload);
}

void Gnss::navOdo() {
	gnssOdo = *((NavODO *)payload);
  printf("navODO iTOW %u, distance %u +/- %um, total %um\n", gnssOdo.iTOW, gnssOdo.distance, gnssOdo.distanceStd, gnssOdo.totalDistance);
  free(payload);
}

void Gnss::navOrb() {
	NavOrb * o = (NavOrb *)payload;
	OrbData * p = NULL;
	char g[8];
	
	
	uint8_t u1, u2;
	for (uint8_t i = 0; i < o->numSv; i++) {
		p = (OrbData *)(payload + sizeof(NavOrb) + i * sizeof(OrbData));
		memset(g, 0, sizeof(g));
		switch (p->gnssId) {
		case GPS_ID: strcat(g, "GPS"); break;
		case SBAS_ID: strcat(g, "SBAS"); break;
		case Galileo_ID: strcat(g, "Galileo"); break;
		case BeiDou_ID: strcat(g, "BeiDou"); break;
		case IMES_ID: strcat(g, "IMES"); break;
		case QZSS_ID: strcat(g, "QZSS"); break;
		case GLONASS_ID: strcat(g, "GLONASS"); break;
		default: strcat(g, "unknown"); break;
		}
		printf("gnssId %s svId %u, status ", g, p->svId);
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
		printf("; otherOrbDataType ");
		switch(p->otherFlags.Source) {
			case 0: printf("No orbit data available"); break;
			case 1: printf("AssistNowOffline"); break;
			case 2: printf("AssistNowAutonomous"); break;
			default:
				printf("other"); 
				break;
		}
		printf("\n");
	}
  free(payload);
}

void Gnss::navPosEcef() {
	gnssNavPosEcef = *((NavPosEcef *)payload);

	printf("PosECEF iTOW %u, x %dm, y %dm, z %dm +/- %um\n", gnssNavPosEcef.iTOW, gnssNavPosEcef.x / 100, gnssNavPosEcef.y / 100, gnssNavPosEcef.z / 100, gnssNavPosEcef.pAcc / 100);
	free(payload);
}

void Gnss::navPosLlh() {
	gnssNavPosLlh = *((NavPosLlh *)payload);
	
	Latitude lat;
    Longitude lon;
    longLatToDMS(&lat, gnssNavPosLlh.lat);
    longLonToDMS(&lon, gnssNavPosLlh.lon);
	printf("PosLLH iTOW %u, %s %s +/- %d, alt %d, above MSL %d +/- %dm\n", gnssNavPosLlh.iTOW, dmsLatToStr(&lat).c_str(), dmsLonToStr(&lon).c_str(), gnssNavPosLlh.hAcc / 1000, gnssNavPosLlh.alt / 1000, gnssNavPosLlh.altMSL / 1000, gnssNavPosLlh.vAcc / 1000);
  free(payload);
}

void Gnss::navPvt() {
  gnssNavPvt = *((NavPvt *)payload);
  if (gnssNavPvt.validTime.validDate && gnssNavPvt.validTime.validTime && gnssNavPvt.validTime.fullyResolved) {
	char tt[32];
	time_t t = time(NULL);
	tm * time_tm = localtime(&t);
	strftime(tt, 32, "%c", time_tm);
	printf("NatPvt local time: %s\n", tt);

	memset(time_tm, 0, sizeof(tm));
    utcTime = mktime(gps2tm(&(gnssNavPvt.dateTime), time_tm));
	/*struct timeval tv;
	tv.tv_sec = utcTime;
	tc.tv_usec = 0;
    settimeofday(&tv, NULL);*/ //this time is slightly in the past but will be corrected by pps() function 
	                          //if interrupt is attached to an arduino pin where u-blox pps output is connected
							  //alternatively, use timTp() message and set_system_time(utcTime) will be done in pps()
    char ft[9];
    switch (gnssNavPvt.fixType) {
      case NONE: sprintf(ft, "None"); break;
      case DR: sprintf(ft, "DR"); break;
      case TWO_D: sprintf(ft, "2D"); break;
      case THREE_D: sprintf(ft, "3D"); break;
      case GNSS_DR: sprintf(ft, "GNSS+DR"); break;
      case TIME_ONLY: sprintf(ft, "TimeOnly"); break;
    }
    char psm[19];
    switch (gnssNavPvt.flags.psmState) {
      case NOT_ACTIVE: sprintf(psm, "Not Active"); break;
      case ENABLED: sprintf(psm, "Enabled"); break;
      case ACQUISITION: sprintf(psm, "Acquisition"); break;
      case TRACKING: sprintf(psm, "Tracking"); break;
      case POWER_OPTIMIZED_TRACKING: sprintf(psm, "Optimized Tracking"); break;
      case INACTIVE: sprintf(psm, "Inactive"); break;
    }
    char carrSoln[6];
    switch (gnssNavPvt.flags.carrSoln) {
      case NONE: sprintf(carrSoln, "None"); break;
      case FLOAT: sprintf(carrSoln, "Float"); break;
      case FIXED: sprintf(carrSoln, "Fixed"); break;
    }
    Latitude lat;
    Longitude lon;
    longLatToDMS(&lat, gnssNavPvt.lat);
    longLonToDMS(&lon, gnssNavPvt.lon);
    string flags = "FixOK ", yes = "yes", no = "no";
    flags += gnssNavPvt.flags.gnssFixOk ? yes : no;
    flags += "|diffFix ";
    flags += gnssNavPvt.flags.diffSoln ? yes : no;
    flags += "|psm ";
    flags += gnssNavPvt.flags.psmState ? yes : no;
    flags += "|headingValid ";
    flags += gnssNavPvt.flags.headingValid ? yes : no;
    flags += "|carrSoln ";
    flags += gnssNavPvt.flags.carrSoln ? yes : no;
	char str[255];
	printf(" %s %s +/- %um\n", dmsLatToStr(&lat).c_str(), dmsLonToStr(&lon).c_str(), gnssNavPvt.hAcc / 1000);
	printf("iTOW %u, %s +/- %uns, Fix: %s, psm %s, carrSol %s, %s, SVs %u, alt %dm above elipsoid, %dm above MSL +/- %um, pDOP %.2f, SOG %d +/- %um/s, heading %d, motionHeading %d +/- %udeg\n", gnssNavPvt.iTOW, asctime(gmtime(&utcTime)), gnssNavPvt.tAcc, ft, psm, carrSoln, flags.c_str(), gnssNavPvt.numSV, gnssNavPvt.alt / 1000, gnssNavPvt.altMSL / 1000, gnssNavPvt.vAcc / 1000, (double)(gnssNavPvt.pDOP / 100.0), gnssNavPvt.SOG / 1000, gnssNavPvt.sAcc / 1000, gnssNavPvt.heading / 100000, gnssNavPvt.motionHeading / 100000, gnssNavPvt.headAcc / 100000);
  }
  free(payload);
}

void Gnss::navSat() {
  NavSat * navSat = (NavSat *)payload;
  printf("navSat iTOW %u, numSVs %u\n", navSat->iTOW, navSat->numSvs);
  for (uint8_t i = 0; i < navSat->numSvs; i++) {
    SatInfo * satInfo = (SatInfo *)((uint8_t *)navSat + sizeof(NavSat) + i * sizeof(SatInfo));
    char sq[40]; memset(sq, 0, sizeof(sq));
    switch(satInfo->flags.signalQuality) {
      case NO_SIGNAL: sprintf(sq, "None"); break;
      case SEARCHING_SIGNAL: sprintf(sq, "Searching"); break;
      case SIGNAL_ACQUIRED: sprintf(sq, "Acquired"); break;
      case SIGNAL_DETECTED_BUT_UNUSABLE: sprintf(sq, "Detected but unusable"); break;
      case CODE_LOCKED_TIME_SYNCED: sprintf(sq, "Code locked and time synced"); break;
      default: sprintf(sq, "Code and carrier locked and time synced"); break;
    }
    char healthy[8]; memset(healthy, 0, sizeof(healthy));
    switch(satInfo->flags.health)
    {
      case UNKNOWN: sprintf(healthy, "unknown"); break;
      case HEALTHY: sprintf(healthy, "Yes"); break;
      case UNHEALTHY: sprintf(healthy, "No"); break;
    }
    char os[22]; memset(os, 0, sizeof(os));
    switch (satInfo->flags.orbitSource)
    {
      case NONE: sprintf(os, "None"); break;
      case EPHEMERIS: sprintf(os, "Ephimeris"); break;
      case ALMANAC: sprintf(os, "Almanac"); break;
      case ASSIST_NOW_OFFLINE: sprintf(os, "Assist Now Offline"); break;
      case ASSIST_NOW_AUTONOMOUS: sprintf(os, "Assist Now Autonomous"); break;
      default: sprintf(os, "Other"); break;
    }
	char g[8]; memset(g, 0, sizeof(g));
	switch (satInfo->gnssId) {
	case GPS_ID: strcat(g, "GPS"); break;
	case SBAS_ID: strcat(g, "SBAS"); break;
	case Galileo_ID: strcat(g, "Galileo"); break;
	case BeiDou_ID: strcat(g, "BeiDou"); break;
	case IMES_ID: strcat(g, "IMES"); break;
	case QZSS_ID: strcat(g, "QZSS"); break;
	case GLONASS_ID: strcat(g, "GLONASS"); break;
	default: strcat(g, "unknown"); break;
	}
	printf("gnssId %s, svId %u, cno %u, elev %ddeg, azim %ddeg, prRes %dm, signalQuality %s, used %u, healthy %s, diffCorr %u, smoothed %u, orbitSrc %s, ephAvail %u, almAvail %u, anoAvail %u, aopAvail %u, sbas %u, rtcm %u, slas %u, pr %u, cr %u, do %u\n", g, satInfo->svId, satInfo->cno, satInfo->elev, satInfo->azim, satInfo->prRes, sq, satInfo->flags.svUsed, healthy, satInfo->flags.diffCorr, satInfo->flags.smoothed, os, satInfo->flags.ephAvail, satInfo->flags.almAvail, satInfo->flags.anoAvail, satInfo->flags.aopAvail, satInfo->flags.sbasCorrUsed, satInfo->flags.rtcmCorrUsed, satInfo->flags.slasCorrUsed, satInfo->flags.prCorrUsed, satInfo->flags.crCorrUsed, satInfo->flags.doCorrUsed);
  }
  free(payload);
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
	printf("navSbas: iTOW %u, geo %u, mode %s, sys %s, service %s: \n", p->iTOW, p->geo, m, g, r);
	
	for (uint8_t i = 0; i < p->cnt; i++) {
		sv = (SbasSv *)(payload + sizeof(NavSbas) + i * sizeof(SbasSv));
		memset(g, 0, 8);
		memset(r, 0, 40);
		switch (sv->sys) {
			case WAAS: sprintf(g, "WAAS"); break;
			case EGNOS: sprintf(g, "EGNOS"); break;
			case MSAS: sprintf(g, "MSAS"); break;
			case GAGAN: sprintf(g, "GAGAN"); break;
			case GPS_SYS: sprintf(g, "GPS"); break;
			case UNKNOWN_SBAS: sprintf(g, "unknown"); break;
		}
		if (sv->services.ranging) strcat(r, "ranging");
		if (sv->services.corrections) strcat(r, "|corrections");
		if (sv->services.integrity) strcat(r, "|integrity");
		if (sv->services.testMode) strcat(r, "|testMode");
		if (r[0] == 0) sprintf(r, "N/A");
		printf("svId %u, flags %u, status %u, sys %s, service %s, pseudoRange %dcm, ionspherCorr %dcm\n", sv->svId, sv->flags, sv->status, g, r, sv->prc, sv->ic);		
	}
  free(payload);
}

void Gnss::navSlas() {
	SlasSv * sv = NULL;
	NavSlas * p = (NavSlas *)payload;
	char f[40], g[8];
	memset(f, 0, 40);
	if (p->flags.gmsAvailable)  strcat(f, "gmsAvailable");
	if (p->flags.qzssSvAvailable) strcat(f, "|qzssSvAvailable");
	if (p->flags.testMode) strcat(f, "|testMode");
	printf("navSlas: iTOW %u, gmsLon %d x 10^-3 deg, gmsLat %d x 10^-3 deg, code %u, svId %u, flags %s\n", p->iTOW, p->gmsLon, p->gmsLat, p->gmsCode, p->gzssSvId, f);
	for (uint8_t i = 0; i < p->cnt; i++) {
		sv = (SlasSv *)(payload + sizeof(NavSlas) + i * sizeof(SlasSv));
		memset(g, 0, sizeof(g));
		switch (sv->gnssId) {
		case GPS_ID: strcat(g, "GPS"); break;
		case SBAS_ID: strcat(g, "SBAS"); break;
		case Galileo_ID: strcat(g, "Galileo"); break;
		case BeiDou_ID: strcat(g, "BeiDou"); break;
		case IMES_ID: strcat(g, "IMES"); break;
		case QZSS_ID: strcat(g, "QZSS"); break;
		case GLONASS_ID: strcat(g, "GLONASS"); break;
		default: strcat(g, "unknown"); break;
		}

		printf("gnssId %s, svId %u, prc %ucm\n", g, sv->svId, sv->prc);
	}
  free(payload);
}

void Gnss::navStatus() {
	gnssStatus = *((NavStatus *)payload);
	char fix[10], flags[255], fixFlags[255], flags2[255], htime[128];
	memset(fix, 0, sizeof(fix));
	memset(flags, 0, sizeof(flags));
	memset(flags2, 0, sizeof(flags2));
	memset(fixFlags, 0, sizeof(fixFlags));
	memset(htime, 0, sizeof(htime));
	switch (gnssStatus.gpsFix) {
	case NONE: sprintf(fix, "none"); break;
	case DR: sprintf(fix, "DR"); break;
	case TWO_D: sprintf(fix, "2-D"); break;
	case THREE_D: sprintf(fix, "3-D"); break;
	case GNSS_DR: sprintf(fix, "3-D+DR"); break;
	case TIME_ONLY: sprintf(fix, "time-only"); break;
	}

	if (gnssStatus.flags.gpsFixOk) strcat(flags, "gpsFixOk");
	else strcat(flags, "gpsFix invalid");
	if (gnssStatus.flags.diffSoln) strcat(flags, "|diffSoln");
	else strcat(flags, "|no diffSoln");
	if (gnssStatus.flags.towSet) strcat(flags, "|TOW valid");
	else strcat(flags, "|TOW invalid");
	if (gnssStatus.flags.wknSet) strcat(flags, "|week number valid");
	else strcat(flags, "|week number invalid");
	if (gnssStatus.fixFlags.carrSolnValid) strcat(fixFlags, "carrSoln valid");
	else strcat(fixFlags, "carrSoln invalid");
	if (gnssStatus.fixFlags.diffCorr) strcat(fixFlags, "|diffCorr valid");
	else strcat(fixFlags, "|diffCorr invalid");
	switch (gnssStatus.fixFlags.mapMatching) {
	case 0: strcat(fixFlags, "|mapMatching none"); break;
	case 1: strcat(fixFlags, "|mapMatching valid but not used"); break;
	case 2: strcat(fixFlags, "|mapMatching valid and used"); break;
	case 3: strcat(fixFlags, "|mapMatching valid and used, DR enabled"); break;
	}
	switch (gnssStatus.flags2.carrSoln) {
	case 0: strcat(flags2, "carrSoln none"); break;
	case 1: strcat(flags2, "carrSoln with floating ambiguities"); break;
	case 2: strcat(flags2, "carrSoln with fixed ambiguities"); break;
	}
	switch (gnssStatus.flags2.psmState) {
	case 0: strcat(flags2, "|aquisition or psm disabled"); break;
	case 1: strcat(flags2, "|PSM tracking"); break;
	case 2: strcat(flags2, "|PSM power optimized tracking"); break;
	case 3: strcat(flags2, "|PSM inactive"); break;
	}
	switch (gnssStatus.flags2.spoofDetState) {
	case 0: strcat(flags2, "|spoof detection unknown or not activated"); break;
	case 1: strcat(flags2, "|no spoofing indicated"); break;
	case 2: strcat(flags2, "|spoofing indicated"); break;
	case 3: strcat(flags2, "|multiple spoofing indications"); break;
	}
	printf("navStatus iTOW %u, uptime %s, gpsFix %s, timeToFirstFix (time tag) %us, flags %s, fixFlags %s, flags2 %s\n", gnssStatus.iTOW, humanTime(gnssStatus.uptime / 1000, (char *)htime), fix, gnssStatus.timeToFix / 1000, flags, fixFlags, flags2);
	free(payload);
}

void Gnss::navVelEcef() {
	gnssVelEcef = *((NavVelEcef *)payload);
	printf("NavVelEcef iTOW %u, velocityX %.2f, velocityY %.2f, velocityZ %.2f, +/- %.2fkm/h\n", gnssVelEcef.iTOW, (double)(gnssVelEcef.velX) * 36 / 1000, (double)(gnssVelEcef.velY) * 36 / 1000, (double)(gnssVelEcef.velZ) * 36 / 1000, (double)(gnssVelEcef.sAcc) * 36 / 1000);
	free(payload);
}

void Gnss::navVelNed() {
	gnssVelNed = *((NavVelNed *)payload);
	printf("NavVelNed iTOW %u, velocityNorth %.2f, velocityEast %.2f, velocityDown %.2f, speed %.2f, groundSpeed %.2f +/- %.2fkm/h, heading %d +/- %u deg\n", gnssVelNed.iTOW, (double)(gnssVelNed.velNorth) * 36 / 1000, (double)(gnssVelNed.velEast) * 36 / 1000, (double)(gnssVelNed.velDown) * 36 / 1000, (double)(gnssVelNed.speed) * 36 / 1000, (double)(gnssVelNed.groundSpeed) * 36 / 1000, (double)(gnssVelNed.sAcc) * 36 / 1000, gnssVelNed.heading < 0 ? 360 - gnssVelNed.heading / 100000 : gnssVelNed.heading / 100000, gnssVelNed.cAcc / 100000);
	free(payload);
}

void Gnss::navTimeBds() {
	NavTimeBdsGal * p = (NavTimeBdsGal *)payload;
	char f[32];
	memset(f, 0, 32);
	if (p->flags.towValid) strcat(f, "towValid");
	else  strcat(f, "towInvalid");
	if (p->flags.weekValid) strcat(f, "|weekValid");
	else strcat(f, "|weekInvalid");
	if (p->flags.leapSecValid) strcat(f, "|leapSecValid");
	else  strcat(f, "|leapSecInvalid");
	printf("timeBDS: iTOW %ums, tow %u.%09ds +/- %uns, weeks %d, leapSec %d, flags %s\n", p->iTOW, p->fTow < 0 ? p->tow - 1 : p->tow, p->fTow < 0 ? 1000000000 - p->fTow : p->fTow, p->tAcc, p->weeks, p->leapSec, f);
	
  free(payload);
}

void Gnss::navTimeGal() {
	NavTimeBdsGal * p = (NavTimeBdsGal *)payload;
	char f[32];
	memset(f, 0, 32);
	if (p->flags.towValid) strcat(f, "towValid");
	else  strcat(f, "towInvalid");
	if (p->flags.weekValid) strcat(f, "|weekValid");
	else  strcat(f, "|weekInvalid");
	if (p->flags.leapSecValid) strcat(f, "|leapSecValid");
	else  strcat(f, "|leapSecInvalid");
	printf("timeGal: iTOW %ums, tow %u.%09ds +/- %uns, weeks %d, leapSec %d, flags %s\n", p->iTOW, p->fTow < 0 ? p->tow - 1 : p->tow, p->fTow < 0 ? 1000000000 - p->fTow : p->fTow, p->tAcc, p->weeks, p->leapSec, f);
  free(payload);
}

void Gnss::navTimeGlo() {
	NavTimeGlo * p = (NavTimeGlo *)payload;
	char f[20];
	memset(f, 0, 20);
	uint16_t day = p->day, doy = day % 365;
	uint16_t year = 1992 + 4 * p->year + day / 365;
	if (p->flags.todValid) strcat(f, "todValid");
	else strcat(f, "todInvalid");
	if (p->flags.dateValid) strcat(f, "|dateValid");
	else strcat(f, "|dateInvalid");
	printf("timeGlo: iTOW %ums, tod %u.%09ds +/- %uns, dayOfYear %u, year %u, flags %s\n", p->iTOW, p->fTod < 0 ? p->tod - 1 : p->tod, p->fTod < 0 ? 1000000000 - p->fTod : p->fTod, p->tAcc, doy, year, f);
  free(payload);
}

void Gnss::navTimeGps() {
	NavTimeGps * p = (NavTimeGps *)payload;
	char f[32];
	memset(f, 0, 32);
	if (p->flags.towValid) strcat(f, "towValid");
	else  strcat(f, "towInvalid");
	if (p->flags.weekValid) strcat(f, "|weekValid");
	else strcat(f, "|weekInvalid");
	if (p->flags.leapSecValid) strcat(f, "|leapSecValid");
	else strcat(f, "|leapSecInvalid");
	printf("timeGps: iTOW %u.%09ds +/- %uns, weeks %d, leapSec %d, flags %s\n", p->fTow < 0 ? (p->iTOW / 1000) - 1 : p->iTOW / 1000, p->fTow < 0 ? 1000000000 - p->fTow : p->fTow, p->tAcc, p->weeks, p->leapSec, f);
  free(payload);
}

void Gnss::navTimeLs() {
	NavTimeLs * p = (NavTimeLs *)payload;
	char g[17], cs[10], t[128];
	memset(g, 0, 17);
	memset(cs, 0, 10);
	memset(t, 0, sizeof(t));
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

	printf("LeapSecond: iTOW %u, currLS %u, src %s, changeSrc %s, lsChange %d, timeToLs %s, week %u, day %u\n", p->iTOW, p->leapSec, g, cs, p->lsChange, humanTime(p->timeToLs, t), p->gpsWeek, p->gpsDay);
  free(payload);
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
  free(payload);
}

void Gnss::cfgGnss() {
  gnssConf = *((GnssConf *)payload);
  GnssCfg * cfg = NULL;
  
  printf("GnssConf: u-blox numTrkChHw %u, numTrkChUse %u\n", gnssConf.numTrkChHw, gnssConf.numTrkChUse);
  char gnss_id[8], on_off[9], signal[16];
  for (uint8_t i = 0; i < gnssConf.numConfigBlocks; i++) {
	memset(gnss_id, 0, sizeof(gnss_id));
	memset(on_off, 0, sizeof(on_off));
	memset(signal, 0, sizeof(signal));
	cfg = (GnssCfg *)(payload + sizeof(GnssConf) + sizeof(GnssCfg) * i);
    switch (cfg->gnssId) {
      case GPS_ID: 
		  sprintf(gnss_id, "GPS");
		  switch (cfg->flags.sigCfgMask) {
		  case 1: sprintf(signal, "GPS L1C/A");  break;
		  case 0x10: sprintf(signal, "GPS L2C"); break;
		  }
		  break;
      case SBAS_ID: 
		  switch (cfg->flags.sigCfgMask) {
		  case 1: sprintf(signal, "SBAS L1C/A");  break;
		  }
		  sprintf(gnss_id, "SBAS");
		  break;
      case Galileo_ID: 
		  switch (cfg->flags.sigCfgMask) {
		  case 1: sprintf(signal, "Galileo E1");  break;
		  case 0x20: sprintf(signal, "Galileo E5b"); break;
		  }
		  sprintf(gnss_id, "Galileo");
		  break;
      case BeiDou_ID: 
		  switch (cfg->flags.sigCfgMask) {
		  case 1: sprintf(signal, "BeiDou B1I");  break;
		  case 0x10: sprintf(signal, "BeiDou B2I"); break;
		  }
		  sprintf(gnss_id, "BeiDou");
		  break;
      case IMES_ID: 
		  switch (cfg->flags.sigCfgMask) {
		  case 1: sprintf(signal, "IMES L1");  break;
		  }
		  sprintf(gnss_id, "IMES");
		  break;
      case QZSS_ID: 
		  switch (cfg->flags.sigCfgMask) {
		  case 1: sprintf(signal, "QZSS L1C/A");  break;
		  case 4: sprintf(signal, "QZSS L1S"); break;
		  case 0x10: sprintf(signal, "QZSS L2C"); break;
		  }
		  sprintf(gnss_id, "QZSS");
		  break;
      case GLONASS_ID: 
		  switch (cfg->flags.sigCfgMask) {
		  case 1: sprintf(signal, "GLONASS L1");  break;
		  case 0x10: sprintf(signal, "GLONASS L2"); break;
		  }
		  sprintf(gnss_id, "GLONASS");
		  break;
    }
    if (cfg->flags.enabled) sprintf(on_off, "enabled"); 
	else sprintf(on_off, "disabled");
	if (i < maxNumberOfGnss) {
		configuredGnss[i].gnssId = cfg->gnssId;
		configuredGnss[i].minTrkCh = cfg->minTrkCh;
		configuredGnss[i].maxTrkCh = cfg->maxTrkCh;
		configuredGnss[i].flags = cfg->flags;
	}
	printf("GNSS id %s, minCh %u, maxCh %u: %s, sigCfgMask %s\n", gnss_id, cfg->minTrkCh, cfg->maxTrkCh, on_off, signal);
  }
  free(payload);
}

void Gnss::cfgInf() {
  InfoMsgMask mask[6];
  char pr[5], po[8];
  memset(pr, 0, 5);
  memset(po, 0, 8);
  for (uint8_t i = 0; i < 6; i++) {
	  mask[i] = *((InfoMsgMask *)&(payload[i + 4]));
      switch((uint8_t)payload[0]) {
	  case UBX: 
		  sprintf(pr, "UBX"); 
		  gnssCfgInfo[0].protocolId = UBX;
		  gnssCfgInfo[0].mask[i].debug = mask[i].debug;
		  gnssCfgInfo[0].mask[i].error = mask[i].error;
		  gnssCfgInfo[0].mask[i].notice = mask[i].notice;
		  gnssCfgInfo[0].mask[i].test = mask[i].test;
		  gnssCfgInfo[0].mask[i].warning = mask[i].warning;
		  gnssCfgInfo[0].mask[i].reserved = 0;
		  break;
		case NMEA: sprintf(pr, "NMEA"); 
			gnssCfgInfo[1].protocolId = UBX;
			gnssCfgInfo[1].mask[i].debug = mask[i].debug;
			gnssCfgInfo[1].mask[i].error = mask[i].error;
			gnssCfgInfo[1].mask[i].notice = mask[i].notice;
			gnssCfgInfo[1].mask[i].test = mask[i].test;
			gnssCfgInfo[1].mask[i].warning = mask[i].warning;
			gnssCfgInfo[1].mask[i].reserved = 0;
			break;
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
	  printf("cfgInfoMask: %s %s INF-MSG mask: err %u, warn %u, info %u, test %u, debug %u\n", po, pr, mask[i].error, mask[i].warning, mask[i].notice, mask[i].test, mask[i].debug);
  }
  free(payload);
}

void Gnss::cfgLogFilter() {
  gnssLogFilter = *((CfgLogFilter *)payload);
  char flags[70];
  memset(flags, 0, 70);
  if (gnssLogFilter.flags.recordingEnabled) strcat(flags, "recEnabled");  
  else  strcat(flags, "recDisabled");
  if (gnssLogFilter.flags.psmOncePerWakeUpEnabled) strcat(flags, "|psmOncePerWakeUp Enabled");
  else strcat(flags, "|psmOncePerWakeUp Disabled");
  if (gnssLogFilter.flags.applyAllFilterSettings) strcat(flags, "|applyAllFilterSettings Yes");
  else strcat(flags, "|applyAllFilterSettings No");
  printf("cfgLogFilter: minInterval %us, timeThreshold %us, speedThreshold %um/s, positionThreshold %um, flags %s\n", gnssLogFilter.minInterval, gnssLogFilter.timeThreshold, gnssLogFilter.speedThreshold, gnssLogFilter.positionThreshold, flags);
  free(payload);
}

void Gnss::cfgMsg() {
  char ids[9], cl[8], po[5];
  memset(ids, 0, 9);
  memset(cl, 0, 8);
  memset(po, 0, 5);
  if (payloadLength == 8) {
	gnssCfgMsgs = *((CfgMsgs *)payload);
	switch(gnssCfgMsgs.msgClass) {
	case UBX_NAV: 
		sprintf(cl, "UBX-NAV"); 
		switch(gnssCfgMsgs.msgId) {
			case NAV_CLOCK: sprintf(ids, "CLOCK"); break;
			case NAV_DGPS: sprintf(ids, "DGPS"); break;
			case NAV_DOP: sprintf(ids, "DOP"); break;
			case NAV_EOE: sprintf(ids, "EOE"); break;
			case NAV_GEOFENCE: sprintf(ids, "GEOFENCE"); break;
			case NAV_ODO: sprintf(ids, "ODO"); break;
			case NAV_ORB: sprintf(ids, "ORB"); break;
			case NAV_POSECEF: sprintf(ids, "POSECEF"); break;
			case NAV_POSLLH: sprintf(ids, "POSLLH"); break;
			case NAV_PVT: sprintf(ids, "PVT"); break;
			case NAV_RESETODO: sprintf(ids, "RESETODO"); break;
			case NAV_SAT: sprintf(ids, "SAT"); break;
			case NAV_SBAS: sprintf(ids, "SBAS"); break;
			case NAV_SLAS: sprintf(ids, "SLAS"); break;
			case NAV_STATUS: sprintf(ids, "STATUS"); break;
			case NAV_SVINFO: sprintf(ids, "SVINFO"); break;
			case NAV_TIMEBDS: sprintf(ids, "TIMEBDS"); break;
			case NAV_TIMEGAL: sprintf(ids, "TIMEGAL"); break;
			case NAV_TIMEGLO: sprintf(ids, "TIMEGLO"); break;
			case NAV_TIMEGPS: sprintf(ids, "TIMEGPS"); break;
			case NAV_TIMELS: sprintf(ids, "TIMELS"); break;
			case NAV_TIMEUTC: sprintf(ids, "TIMEUTC"); break;
			case NAV_VELECEF: sprintf(ids, "VELECEF"); break;
			case NAV_VELNED: sprintf(ids, "VELNED"); break;
			default: sprintf(ids, "%X", gnssCfgMsgs.msgId);
		}
		break;
	case UBX_TIM: 
		sprintf(cl, "UBX-TIM"); 
		switch(gnssCfgMsgs.msgId) {
			case TIM_TM2: sprintf(ids, "TM2"); break;
			case TIM_TP: sprintf(ids, "TP"); break;
			default: sprintf(ids, "%X", gnssCfgMsgs.msgId);
		}
		break;
	case UBX_NMEA: 
		sprintf(cl, "NMEA"); 
		switch(gnssCfgMsgs.msgId) {
			case NMEA_DTM: sprintf(ids, "DTM"); break;
			case NMEA_GBQ: sprintf(ids, "GBQ"); break;
			case NMEA_GBS: sprintf(ids, "GBS"); break;
			case NMEA_GLQ: sprintf(ids, "GLQ"); break;
			case NMEA_GNQ: sprintf(ids, "GNQ"); break;
			case NMEA_GPQ: sprintf(ids, "GPQ"); break;
			case NMEA_GRS: sprintf(ids, "GRS"); break;
			case NMEA_TXT: sprintf(ids, "TXT"); break;
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
			default: sprintf(ids, "%X", gnssCfgMsgs.msgId);
		}
		break;
	case UBX_PUBX: 
		sprintf(cl, "PUBX"); 
		switch(gnssCfgMsgs.msgId) {
			case PUBX_CONFIG: sprintf(ids, "CONFIG"); break;
			case PUBX_POSITION: sprintf(ids, "POSITION"); break;
			case PUBX_SVSTATUS: sprintf(ids, "SVSTATUS"); break;
			case PUBX_TIME: sprintf(ids, "TIME"); break;
			default: sprintf(ids, "%X", gnssCfgMsgs.msgId);
		}
		break;
		default: sprintf(cl, "%X", gnssCfgMsg.msgClass);
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
		printf("%s cfgMsg %s-%s: rate per navSol %u\n", po, cl, ids, gnssCfgMsgs.rate[portId]);		
	}
  } else {
	gnssCfgMsg = *((CfgMsg *)payload);
	switch(gnssCfgMsg.msgClass) {
	case UBX_NAV: 
		sprintf(cl, "UBX-NAV"); 
		switch(gnssCfgMsg.msgId) {
			case NAV_CLOCK: sprintf(ids, "CLOCK"); break;
			case NAV_DGPS: sprintf(ids, "DGPS"); break;
			case NAV_DOP: sprintf(ids, "DOP"); break;
			case NAV_EOE: sprintf(ids, "EOE"); break;
			case NAV_GEOFENCE: sprintf(ids, "GEOFENCE"); break;
			case NAV_ODO: sprintf(ids, "ODO"); break;
			case NAV_ORB: sprintf(ids, "ORB"); break;
			case NAV_POSECEF: sprintf(ids, "POSECEF"); break;
			case NAV_POSLLH: sprintf(ids, "POSLLH"); break;
			case NAV_PVT: sprintf(ids, "PVT"); break;
			case NAV_RESETODO: sprintf(ids, "RESETODO"); break;
			case NAV_SAT: sprintf(ids, "SAT"); break;
			case NAV_SBAS: sprintf(ids, "SBAS"); break;
			case NAV_SLAS: sprintf(ids, "SLAS"); break;
			case NAV_STATUS: sprintf(ids, "STATUS"); break;
			case NAV_SVINFO: sprintf(ids, "SVINFO"); break;
			case NAV_TIMEBDS: sprintf(ids, "TIMEBDS"); break;
			case NAV_TIMEGAL: sprintf(ids, "TIMEGAL"); break;
			case NAV_TIMEGLO: sprintf(ids, "TIMEGLO"); break;
			case NAV_TIMEGPS: sprintf(ids, "TIMEGPS"); break;
			case NAV_TIMELS: sprintf(ids, "TIMELS"); break;
			case NAV_TIMEUTC: sprintf(ids, "TIMEUTC"); break;
			case NAV_VELECEF: sprintf(ids, "VELECEF"); break;
			case NAV_VELNED: sprintf(ids, "VELNED"); break;
			default: sprintf(ids, "%X", gnssCfgMsg.msgId);
		}
		break;
	case UBX_TIM: 
		sprintf(cl, "UBX-TIM"); 
		switch(gnssCfgMsg.msgId) {
			case TIM_TM2: sprintf(ids, "TM2"); break;
			case TIM_TP: sprintf(ids, "TP"); break;
			default: sprintf(ids, "%X", gnssCfgMsg.msgId);
		}
		break;
	case UBX_NMEA: 
		sprintf(cl, "NMEA"); 
		switch(gnssCfgMsg.msgId) {
			case NMEA_DTM: sprintf(ids, "DTM"); break;
			case NMEA_GBQ: sprintf(ids, "GBQ"); break;
			case NMEA_GBS: sprintf(ids, "GBS"); break;
			case NMEA_GLQ: sprintf(ids, "GLQ"); break;
			case NMEA_GNQ: sprintf(ids, "GNQ"); break;
			case NMEA_GPQ: sprintf(ids, "GPQ"); break;
			case NMEA_GRS: sprintf(ids, "GRS"); break;
			case NMEA_TXT: sprintf(ids, "TXT"); break;
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
			default: sprintf(ids, "%X", gnssCfgMsg.msgId);
		}
		break;
	case UBX_PUBX: 
		sprintf(cl, "PUBX"); 
		switch(gnssCfgMsg.msgId) {
			case PUBX_CONFIG: sprintf(ids, "CONFIG"); break;
			case PUBX_POSITION: sprintf(ids, "POSITION"); break;
			case PUBX_SVSTATUS: sprintf(ids, "SVSTATUS"); break;
			case PUBX_TIME: sprintf(ids, "TIME"); break;
			default: sprintf(ids, "%X", gnssCfgMsg.msgId);
		}
		break;
		default: sprintf(cl, "%X", gnssCfgMsg.msgClass);
	}
    printf("cfgMsg %s-%s: rate per navSol %u\n", cl, ids, gnssCfgMsg.rate);	
  }
  free(payload);
}

void Gnss::cfgNav() {
  gnssNav = *((CfgNav *)payload);
  char dyn[12];
  memset(dyn, 0, 12);
  switch (gnssNav.dynModel) {
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
  switch (gnssNav.utcStandard) {
    case AUTOMATIC: sprintf(utc, "AUTOMATIC"); break;
    case GPS: sprintf(utc, "GPS"); break;
    case GLONASS: sprintf(utc, "GLONASS"); break;
    case BEIDOU: sprintf(utc, "BEIDOU"); break;
    case UTC_ERROR: sprintf(utc, "UTC_ERROR"); break;
  }

  char fm[6];
  memset(fm, 0, 6);
  switch(gnssNav.fixMode) {
  	  case TWO_D_ONLY: sprintf(fm, "2D"); break;
	  case THREE_D_ONLY: sprintf(fm, "3D"); break;
	  case AUTO: sprintf(fm, "AUTO"); break;
	  case FIX_MODE_ERROR: sprintf(fm, "ERROR"); break;
  }
  printf("cfgNav: dynModel %s, utcStandard %s, fixMode %s, fixedAlt %dm, variance %um2, pDOP %u, tDOP %u, pAccMask %u, tAccMask %u\n", dyn, utc, fm, gnssNav.fixedAlt / 100, gnssNav.fixedAltVar / 10000, gnssNav.pDOP / 10, gnssNav.tDOP / 10, gnssNav.pAcc, gnssNav.tAcc);
  printf("cfgNav: cnoThreshold %udBHz, NumSVs %u, staticHoldThreshold %ucm/s, MaxDist %um, minElev %udeg above hor, dgnssTimeout %us\n", gnssNav.cnoThreshold, gnssNav.cnoThresholdNumSVs, gnssNav.staticHoldThreshold, gnssNav.staticHoldMaxDistance, gnssNav.minElev, gnssNav.dgnssTimeout); 
  free(payload);
}

void Gnss::cfgOdo() {
  gnssCfgOdo = *((ODOCfg *)payload);
  char flags[255], profile[9], enabled[8], disabled[9];
  memset(flags, 0, 255);
  memset(profile, 0, 9);
  memset(enabled, 0, 8);
  memset(disabled, 0, 9);

  sprintf(enabled, "enabled");
  sprintf(disabled, "disabled");
  sprintf(flags, "ODO ");
  if (gnssCfgOdo.flags.ODOenabled) strcat(flags, enabled); else strcat(flags, disabled);
  strcat(flags, "|low speed COG filter ");
  if (gnssCfgOdo.flags.COGenabled) strcat(flags, enabled); else strcat(flags, disabled);
  strcat(flags, "|low speed (< 5m/s) filter ");
  if (gnssCfgOdo.flags.outputLPvelocity) strcat(flags, enabled); else strcat(flags, disabled);
  strcat(flags, "|2D headingOfMotion ");
  if (gnssCfgOdo.flags.outputLPcog) strcat(flags, enabled); else strcat(flags, disabled);

  switch (gnssCfgOdo.odoProfile) {
    case RUNNING: sprintf(profile, "RUNNING"); break;
    case CYCLING: sprintf(profile, "CYCLING"); break;
    case SWIMMING: sprintf(profile, "SWIMMING"); break;
    case DRIVING: sprintf(profile, "DRIVING"); break;
    case CUSTOM: sprintf(profile, "CUSTOM"); break;
  }
  printf("ODOcfg - flags: %s, profile %s, COGlowSpeedFilterThresh %um/s, COGmaxAcceptablePosAccuracy %um, velLPgain %u cogLPgain %u\n", flags, profile, gnssCfgOdo.cogMaxSpeed / 10, gnssCfgOdo.cogMaxPosAccuracy, gnssCfgOdo.velLPgain, gnssCfgOdo.cogLPgain);
  free(payload);
}

void Gnss::cfgPm() {
	gnssCfgPm = *((CfgPm *)payload);
	char flags[128];
	memset(flags, 0, 128);
	if (gnssCfgPm.flags.extintSel) strcat(flags, "EXTINT1"); else strcat(flags, "EXTINT0");
	if (gnssCfgPm.flags.extintWake) strcat(flags, "|Wake");
	if (gnssCfgPm.flags.extintBackup) strcat(flags, "|Backup");
	if (gnssCfgPm.flags.extintInactivity) strcat(flags, "|Inactivity");
	if (gnssCfgPm.flags.limitPeakCurr) strcat(flags, "|limitPeakCurr");
	if (gnssCfgPm.flags.waitTimeFix) strcat(flags, "|waitTimeFix");
	if (gnssCfgPm.flags.updateRTC) strcat(flags, "|updateRTC");
	if (gnssCfgPm.flags.updateEph) strcat(flags, "|updateEph");
	if (gnssCfgPm.flags.doNotEnterOff) strcat(flags, "|doNotEnterOff");
	if (gnssCfgPm.flags.mode) strcat(flags, "|PSMCT"); else strcat(flags, "|PSMOO");
	printf("CfgPm: maxStartupStateDuration %us, updatePeriod %ums, searchPeriod %ums, gridOffset %ums, onTime %us, minAcqTime %us, extintInactivity %ums, flags %s\n", gnssCfgPm.maxStartupStateDuration, gnssCfgPm.updatePeriod, gnssCfgPm.searchPeriod, gnssCfgPm.gridOffset, gnssCfgPm.onTime, gnssCfgPm.minAcqTime, gnssCfgPm.extintInactivity, flags);
  free(payload);
}

void Gnss::cfgPms() {
  powerMode = *((PowerMode *)payload);
  char pwr[9];
  memset(pwr, 0, 9);

  switch (powerMode.powerMode)
  {
    case FULL_POWER_MODE: sprintf(pwr, "FULL"); break;
    case BALANCED_POWER_MODE: sprintf(pwr, "BALANCED"); break;
    case INTERVAL_POWER_MODE: sprintf(pwr, "INTERVAL"); break;
    case AGGRESSIVE_1HZ_POWER_MODE: sprintf(pwr, "AGGR_1HZ"); break;
    case AGGRESSIVE_2HZ_POWER_MODE: sprintf(pwr, "AGGR_2HZ"); break;
    case AGGRESSIVE_4HZ_POWER_MODE: sprintf(pwr, "AGGR_4HZ"); break;
    case INVALID_POWER_MODE: sprintf(pwr, "INVALID"); break;
  }
  printf("powerMode %s, period %us, onTime %us\n", pwr, powerMode.period, powerMode.onTime); 
  free(payload);
}

void Gnss::cfgPrt() {
  gnssPort = *((CfgPrt *)payload);
  char id[5], mode[64], clen[2], par[5], sbits[4], inProto[14], outProto[9];
  memset(id, 0, 5);
  memset(mode, 0, 64);
  memset(clen, 0, 2);
  memset(par, 0, 5);
  memset(sbits, 0, 4);
  memset(inProto, 0, 14);
  memset(outProto, 0, 9);

  switch (gnssPort.portId) {
	case DDC: sprintf(id, "DDC"); break;
	case COM1: sprintf(id, "COM1"); break;
	case COM2: sprintf(id, "COM2"); break;
	case USB: sprintf(id, "USB"); break;
	case SPI: sprintf(id, "SPI"); break;
	case RES: sprintf(id, "RES"); break;
  }
  switch (gnssPort.mode.charLen) {
     case FIVE_BIT: sprintf(clen, "5"); break;
     case SIX_BIT: sprintf(clen, "6"); break;
     case SEVEN_BIT: sprintf(clen, "7"); break;
     case EIGHT_BIT: sprintf(clen, "8"); break;
  }
  switch (gnssPort.mode.parity) {
    case PARITY_EVEN: sprintf(par, "even"); break;
    case PARITY_ODD: sprintf(par, "odd"); break;
    case NO_PARITY: sprintf(par, "no"); break;
    case NO_PARITY2: sprintf(par, "no"); break;
  }
  switch (gnssPort.mode.nStopBits) {
    case ONE_STOP_BIT: sprintf(sbits, "1"); break;
    case ONE_AND_HALF_STOP_BIT: sprintf(sbits, "1.5"); break;
    case TWO_STOP_BIT: sprintf(sbits, "2"); break;
    case HALF_STOP_BIT: sprintf(sbits, "0.5"); break;
  }
  if (gnssPort.inProtoMask.inUbx) sprintf(inProto, "Ubx");
  if (gnssPort.inProtoMask.inNmea) strcat(inProto, "|Nmea");
  if (gnssPort.inProtoMask.inRtcm) strcat(inProto, "|Rtcm");
  if (gnssPort.outProtoMask.outUbx) sprintf(outProto, "Ubx");
  if (gnssPort.outProtoMask.outNmea) strcat(outProto, "|Nmea");
  
  sprintf(mode, "%s bit|%s parity|%s stop bit", clen, par, sbits);
  printf("cfgPrt: %s, rate %u baud, mode %s, inProto %s, outProto %s\n", id, (uint32_t)(gnssPort.baudRate), mode, inProto, outProto); 
  free(payload);
}

//UBX-CFG-PWR is deprecated in protocols > 17
void Gnss::cfgPwr() {
}

void Gnss::cfgRate() {
  gnssNavRate = *((NavRate *)payload);
  char timeRef[8];
  memset(timeRef, 0, 8);
  switch (gnssNavRate.timeRef)
  {
    case UTC_TIME: sprintf(timeRef, "UTC"); break;
    case GPS_TIME: sprintf(timeRef, "GPS"); break;
    case GLONASS_TIME: sprintf(timeRef, "GLONASS"); break;
    case BEIDOU_TIME: sprintf(timeRef, "BEIDOU"); break;
    case GALILEO_TIME: sprintf(timeRef, "GALILEO"); break;
  }
  printf("cfgRate: navRate %ums, measurementsPerNavSol %u, timeRef %s\n", gnssNavRate.rate, gnssNavRate.navSolRate, timeRef);
  free(payload);
}

void Gnss::cfgRxm() {
	gnssCfgRxm = *((CfgRxm *)payload);
	printf("cfgRxm mode: ");
	switch(gnssCfgRxm.lpMode) {
		case CONTINUOUS_POWER: printf("CONTINUOUS_POWER\n"); break;
		case POWER_SAVE_MODE: printf("POWER_SAVE_MODE\n"); break;
		case CONTINUOUS_MODE: printf("CONTINUOUS_MODE\n"); break;
	}	
  free(payload);
}

void Gnss::cfgSbas() {
	gnssCfgSbas = *((CfgSbas *)payload);
	char str[64];
	memset(str, 0, 64);
	printf("cfgSbas mode ");
	if (gnssCfgSbas.mode.enabled) strcat(str, "enabled"); else strcat(str, "disabled");
	if (gnssCfgSbas.mode.testMode) strcat(str, "|testMode");
	strcat(str, ", usage ");
	if (gnssCfgSbas.usage.range) strcat(str, "range");
	if (gnssCfgSbas.usage.diffCorr) strcat(str, "|diffCorr");
	if (gnssCfgSbas.usage.integrity) strcat(str, "|integrity");
	printf("%s, maxSbas %u, scanMode1 %u, scanMode2 %u\n", str, gnssCfgSbas.maxSbas, gnssCfgSbas.scanMode1, gnssCfgSbas.scanMode2);
  free(payload);
}

//UBX-CFG-SLAS Not supported in protocols < 19.2
void Gnss::cfgSlas() {
}

void Gnss::cfgTp() {
  gnssTp = *((TimePulse *)payload);
  char flags[255], fp[3], lc[6];
  memset(flags, 0, 255);
  memset(fp, 0, 3);
  memset(lc, 0, 6);
  sprintf(fp, "us");
  sprintf(lc, "2^-32");
  TimePulseFlags f = gnssTp.flags;
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
  printf("timePulse id %u, cableDelay %uns, rfDelay %uns, pulsePeriod %u%s, pulsePeriodLocked %u%s, pulseLen %u%s, pulseLenLocked %u%s, delay %uns, flags %s\n", gnssTp.pulseId, gnssTp.antCableDelay, gnssTp.rfGroupDelay, gnssTp.pulsePeriod, fp, gnssTp.pulsePeriodLocked, fp, gnssTp.pulseLen, lc, gnssTp.pulseLenLocked, lc, gnssTp.userConfigDelay, flags);
  free(payload);
}

void Gnss::cfgGeoFence() {
	gnssGeoFences = *((GeoFences *)payload);
	
	Latitude lat;
    Longitude lon;

	printf("cfgGeofences confLvl %u, pioEnabled %u, pinPolarity %u, pin %u\n", gnssGeoFences.confidenceLevel, gnssGeoFences.pioEnabled, gnssGeoFences.pinPolarity, gnssGeoFences.pin);
	
	numberOfGeoFences = gnssGeoFences.numFences;
	for (uint8_t i = 0; i < numberOfGeoFences; i++) {
		if (numberOfGeoFences < maxNumberOfGeoFences) {
			gnssGeoFence[i] = *((GeoFence *)(payload + sizeof(GeoFences) + i * sizeof(GeoFence)));
			longLatToDMS(&lat, gnssGeoFence[i].lat);
			longLonToDMS(&lon, gnssGeoFence[i].lon);
			printf("%u: %s %s radius %um\n", i, dmsLatToStr(&lat).c_str(), dmsLonToStr(&lon).c_str(), gnssGeoFence[i].radius / 100);
		}
	}
  free(payload);
}

void Gnss::cfgNmea() {
	gnssCfgNmea = *((CfgNmea *)payload);
	char talkerId[14], gsv[8], filter[128], flags[96], gnss[96];
	memset(talkerId, 0, 14);
	memset(gsv, 0, 8);
	memset(filter, 0, 128);
	memset(flags, 0, 96);
	memset(gnss, 0, 96);

	if (gnssCfgNmea.filter.failedFix) strcat(filter, "dispFailedFix");
	if (gnssCfgNmea.filter.invalidFix) strcat(filter, "|dispInvalidFix");
	if (gnssCfgNmea.filter.invalidTime) strcat(filter, "|dispInvalidTime");
	if (gnssCfgNmea.filter.invalidDate) strcat(filter, "|dispInvalidDate");
	if (gnssCfgNmea.filter.invalidCog) strcat(filter, "|dispInvalidCOG");
	if (gnssCfgNmea.filter.gpsOnly) strcat(filter, "|dispGPSonly");
	if (filter[0] == 0) sprintf(filter, "none");
	if (gnssCfgNmea.flags.compat) strcat(flags, "compatMode");
	if (gnssCfgNmea.flags.consider) strcat(flags, "|consideringMode");
	if (gnssCfgNmea.flags.limit82) strcat(flags, "|82charsLimit");
	if (gnssCfgNmea.flags.highPrecision) strcat(flags, "|highPrecisionMode");
	if (gnssCfgNmea.gnssFilter.disableGps) strcat(gnss, "disableGps");
	if (gnssCfgNmea.gnssFilter.disableSbas) strcat(gnss, "|disableSbas");
	if (gnssCfgNmea.gnssFilter.disableQzss) strcat(gnss, "|disableQzss");
	if (gnssCfgNmea.gnssFilter.disableGlonass) strcat(gnss, "|disableGlonass");
	if (gnssCfgNmea.gnssFilter.disableBeidou) strcat(gnss, "|disableBeidou");
	if (gnss[0] == 0) sprintf(gnss, "none");
	switch (gnssCfgNmea.mainTalkerId) {
		case DEFAULT_TALKER_ID: sprintf(talkerId, "default"); break;
		case GP_TALKER_ID: sprintf(talkerId, "GPS|SBAS|QZSS"); break;
		case GL_TALKER_ID: sprintf(talkerId, "GLONASS"); break;
		case GN_TALKER_ID: sprintf(talkerId, "Combo"); break;
		case GA_TALKER_ID: sprintf(talkerId, "Galileo"); break;
		case GB_TALKER_ID: sprintf(talkerId, "BeiDou"); break;
	}
	if (gnssCfgNmea.gsvTalkerId) sprintf(gsv, "main"); else sprintf(gsv, "default");
	printf("cfgNmea nmeaVersion %x, maxSVs %u, displayNonNmeaSVs %u, filter %s, flags %s, Disabled GNSS %s, talkerId %s, gsvTalkerId %s, bdsTalkerId \"%s\"\n", gnssCfgNmea.nmeaVersion, gnssCfgNmea.maxSV, gnssCfgNmea.displayNonNmeaSVs, filter, flags, gnss, talkerId, gsv, gnssCfgNmea.bdsTalkerId);
  free(payload);
}

void Gnss::monVer(uint8_t extensionsNumber) {
  gnssMonVer = *((MonVer *)payload);
  printf("Version sw: %s, hw: %s\n", gnssMonVer.swVersion, gnssMonVer.hwVersion);
  printf("Extensions: \n");
  for (uint8_t i = 0; i < extensionsNumber; i++) printf("%s\n", gnssMonVer.extensions[i]);
  free(payload);
}

void Gnss::monGnss() {
  supportedGnss = *((GnssSupport *)payload);
  char supported[32], def[32], enabled[32];
  memset(supported, 0, 32);
  memset(def, 0, 32);
  memset(enabled, 0, 32);
  if (supportedGnss.supportedGnss.Gps) strcat(supported, "Gps");
  if (supportedGnss.supportedGnss.Glonass) strcat(supported, "|Glonass");
  if (supportedGnss.supportedGnss.BeiDou) strcat(supported, "|BeiDou");
  if (supportedGnss.supportedGnss.Galileo) strcat(supported, "|Galileo");
  if (supportedGnss.defaultGnss.Gps) strcat(def, "Gps");
  if (supportedGnss.defaultGnss.Glonass) strcat(def, "|Glonass");
  if (supportedGnss.defaultGnss.BeiDou) strcat(def, "|BeiDou");
  if (supportedGnss.defaultGnss.Galileo) strcat(def, "|Galileo");
  if (supportedGnss.enabledGnss.Gps) strcat(enabled, "Gps");
  if (supportedGnss.enabledGnss.Glonass) strcat(enabled, "|Glonass");
  if (supportedGnss.enabledGnss.BeiDou) strcat(enabled, "|BeiDou");
  if (supportedGnss.enabledGnss.Galileo) strcat(enabled, "|Galileo");
  printf("supportedGNSS %s, defaultGnss: %s, enabledGnss: %s, simul %hhu\n", supported, def, enabled, supportedGnss.simultaneous);
  free(payload);
}

void Gnss::monPatch() {
	Patch * pp = NULL; 
	MonPatches * p = (MonPatches *)(payload);
	
	char str[32];
	memset(str, 0, 32);
	printf("Patches");
    numberOfPatches = p->numPatches;
	for (uint8_t i = 0; i < numberOfPatches; i++) {
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
		if (i < maxNumberOfPatches) patches[i] = *pp;
		printf(", location %s, comparator %u, address %u, data %u\n", str, pp->comparatorNumber, pp->patchAddress, pp->patchData);
	}
  free(payload);
}

void Gnss::timTm() {
	printf("timTm\n");
	gnssTimTm = *((TimTm *)payload);
	char str[196];
	memset(str, 0, 196);
	if (gnssTimTm.flags.mode) strcat(str, "mode running"); else strcat(str, "mode single");
	if (gnssTimTm.flags.stopped) strcat(str, "|stopped"); else strcat(str, "|armed");
	if (gnssTimTm.flags.newFallingEdge) strcat(str, "|newFallingEdge");
	switch (gnssTimTm.flags.timeBase) {
		case 0: strcat(str, "|Receiver Time Base"); break;
		case 1: strcat(str, "|GNSS Time Base"); break;
		case 2: strcat(str, "|UTC Time Base"); break;
	}
	if (gnssTimTm.flags.utcAvailable) strcat(str, "|utcAvailable"); else strcat(str, "|utcNotAvailable");
	if (gnssTimTm.flags.timeValid) strcat(str, "|timeValid"); else strcat(str, "|timeNotValid");
	if (gnssTimTm.flags.newRisingEdge) strcat(str, "|newRisingEdge");
	printf("timTm: channel %u, count %u, weekNumRising %u, weekNumFalling %u, towRising %u.%ums, towFalling %u.%u, accEst %uns, flags %s\n", gnssTimTm.channel, gnssTimTm.count, gnssTimTm.weekRising, gnssTimTm.weekFalling, gnssTimTm.towRising, gnssTimTm.towSubRising, gnssTimTm.towFalling, gnssTimTm.towSubFalling, gnssTimTm.accEst, str);
  free(payload);
}

void Gnss::timTp() {
  gnssTimeTp = *((TimeTP *)payload);
  time_t utcTimeNext = (uint32_t)(gnssTimeTp.weeks) * ONE_DAY * 7 + (gnssTimeTp.tow / 1000) + GPS_UNIX_DIFF;

  /*time_t ttt = time(NULL);
  tm * gmt = gmtime(&ttt), * lt = localtime(&ttt);
  char gmt_time[32], loc_time[32];
  strftime(gmt_time, 32, "%c", gmt);
  strftime(loc_time, 32, "%c", lt);
  printf("utcTime %lu, time(NULL) %lu, diff %lu\n", utcTime, ttt, utcTime - ttt);
  printf("UTC time: %s, local time: %s\n", gmt_time, loc_time);
  */
  char timeBase[64], timeRef[13], utcSource[15], tt[32];
  memset(timeBase, 0, 64);
  memset(timeRef, 0, 13);
  memset(utcSource, 0, 15);

  if (gnssTimeTp.timeBase.utcBase) {
    switch (gnssTimeTp.timeRef.utcSource) {
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
    switch (gnssTimeTp.timeRef.timeRefGnss) {
      case GPS_TP_TIME: sprintf(timeRef, "GPS"); break;
      case GLONASS_TP_TIME: sprintf(timeRef, "GLONASS"); break;
      case BEIDOU_TP_TIME: sprintf(timeRef, "BEIDOU"); break;
      case UNKNOWN_TP_TIME: sprintf(timeRef, "UNKNOWN_GNSS"); break;
    }
    sprintf(timeBase, "GNSS: %s", timeRef);
    if (gnssTimeTp.timeBase.utc) strcat(timeBase, "|UTC avail"); 
  }
  switch (gnssTimeTp.timeBase.raim) {
    case RAIM_INFO_NOT_AVAILABLE: strcat(timeBase, "|RAIM_INFO_NOT_AVAIL"); break;
    case RAIM_NOT_ACTIVE: strcat(timeBase, "|RAIM_NOT_ACTIVE"); break;
    case RAIM_ACTIVE: strcat(timeBase, "|RAIM_ACTIVE"); break;
  }
  strftime(tt, 32, "%c", localtime(&utcTimeNext));
  printf("timeTP utcTimeNextPPS %s, tow %ums, subTOW %ums 2^-32, quatErr %dps, weeks %u, timeBase %s\n", tt, gnssTimeTp.tow, gnssTimeTp.subTOW, gnssTimeTp.quantErr, gnssTimeTp.weeks, timeBase);
  free(payload);
}

void Gnss::logInfo() {
  gnssLogInfo = *((LogInfo *)payload);
  struct tm tm_time;
  char flags[64], ot[32], nt[32];
  memset(flags, 0, 64);
  memset(ot, 0, 32);
  memset(nt, 0, 32);
  time_t oldest = mktime(gps2tm(&(gnssLogInfo.oldestDateTime), &tm_time));
  strftime(ot, 32, "%c", localtime(&oldest));
  time_t newest = mktime(gps2tm(&(gnssLogInfo.newestDateTime), &tm_time));
  strftime(nt, 32, "%c", localtime(&newest));
  if (gnssLogInfo.flags.enabled) sprintf(flags, "enabled"); else sprintf(flags, "disabled");
  if (gnssLogInfo.flags.inactive) strcat(flags, "|inactive"); else strcat(flags, "|active");
  if (gnssLogInfo.flags.circular) strcat(flags, "|circular"); else strcat(flags, "|non-circular");
  printf("logInfo: fileStoreCapacity %u bytes, maxLogSize %u bytes, logSize %u bytes, numRecords %u, oldest %s, newest %s, flags %s\n", gnssLogInfo.fileStoreCapacity, gnssLogInfo.currentMaxLogSize, gnssLogInfo.currentLogSize, gnssLogInfo.entryCount, ot, nt, flags);
  free(payload);
}

void Gnss::logRetrievePos() {
	gnssLogPos = *((LogRetrievePos *)(payload));
	Latitude lat;
    Longitude lon;
    longLatToDMS(&lat, gnssLogPos.lat);
    longLonToDMS(&lon, gnssLogPos.lon);
	tm tm_time;
	char tt[32];
	memset(tt, 0, 32);
	time_t t = mktime(gps2tm(&(gnssLogPos.dateTime), &tm_time));
	strftime(tt, 32, "%c", localtime(&t));
	printf("idx %u: %s %s %s +/- %um, alt %dm above MSL, speed %um/s, heading %udeg, fixType %u, numSV %u\n", gnssLogPos.index, tt, dmsLatToStr(&lat).c_str(), dmsLonToStr(&lon).c_str(), gnssLogPos.hAcc / 1000, gnssLogPos.altMSL / 1000, gnssLogPos.gSpeed / 1000, gnssLogPos.heading / 100000, gnssLogPos.fixType, gnssLogPos.numSV);
  free(payload);
}

void Gnss::logRetrievePosExtra() {
  tm tm_time;
  char tt[32];
  memset(tt, 0, 32);
  gnssLogOdo = *((LogRetrievePosExtra *)payload); 
  time_t t = mktime(gps2tm(&(gnssLogOdo.dateTime), &tm_time));
  strftime(tt, 32, "%c", localtime(&t));
  printf("idx %u: %s distance %u\n", gnssLogOdo.index, tt, gnssLogOdo.distance);
  free(payload);
}

void Gnss::logRetrieveString() {
  tm tm_time;
  char tt[32];
  memset(tt, 0, 32);
  gnssLogString = *((LogRetrieveString *)payload);
  time_t t = mktime(gps2tm(&(gnssLogString.dateTime), &tm_time)); 
  strftime(tt, 32, "%c", localtime(&t));
  char * ss = (char *)(payload + sizeof(LogRetrieveString));
  printf("idx %u: %s %s\n", gnssLogString.index, tt, ss);
  free(payload);
}

void Gnss::logFindTime() {
  LogFindTimeResponse * res = (LogFindTimeResponse *)payload;
  logIndex = res->index;
  printf("logFindTime response: index %u\n", res->index);
  free(payload);
}

void Gnss::infMsg(const char * infLevel) {
  printf("%s: %s\n", infLevel, (char *)payload);
  free(payload);
}

void Gnss::ackAck() {
	ubxAck = *((UbxAck *)payload);
	switch (ubxAck.classId) {
		case UBX_CFG:
			switch (ubxAck.messageId) {
				case CFG_PRT: cfgPrtOk = true; break;
				case CFG_MSG: cfgMsgOk = true; break;
				case CFG_INF: cfgInfOk = true; break;
				case CFG_RST: cfgRstOk = true; break;
				case CFG_RATE: cfgRateOk = true; break;
				case CFG_CFG: cfgCfgOk = true; break;
				case CFG_RXM: cfgRxmOk = true; break;
				case CFG_SBAS: cfgSbasOk = true; break;
				case CFG_NMEA: cfgNmeaOk = true; break;
				case CFG_ODO: cfgOdoOk = true; break;
				case CFG_NAV5: cfgNavOk = true; break;
				case CFG_TP5: cfgTpOk = true; break;
				case CFG_PM2: cfgPmOk = true; break;
				case CFG_GNSS: cfgGnssOk = true; break;
				case CFG_LOGFILTER: cfgLogfilterOk = true; break;
				case CFG_GEOFENCE: cfgGeofenceOk = true; break;
				case CFG_PMS: cfgPmsOk = true; break;
				case CFG_SLAS: cfgSlasOk = true; break;
			}
			break;
		case UBX_LOG:
			switch (ubxAck.messageId) {
				case LOG_CREATE: logCreateOk = true; break;
				case LOG_ERASE: logEraseOk = true; break;
			}
			break;
		case UBX_NAV:
			switch (ubxAck.messageId) {
			case NAV_RESETODO: resetOdoOk = true; break;
			}
			break;
	}
    if (DEBUG_UBX) printf("ACK_ACK: msgClass %X, msgId %X\n", ubxAck.classId, ubxAck.messageId);
  free(payload);
}

void Gnss::ackNak() {
	ubxNak = *((UbxAck *)payload);
	switch (ubxAck.classId) {
		case UBX_CFG:
			switch (ubxAck.messageId) {
				case CFG_PRT: cfgPrtOk = false; break;
				case CFG_MSG: cfgMsgOk = false; break;
				case CFG_INF: cfgInfOk = false; break;
				case CFG_RST: cfgRstOk = false; break;
				case CFG_RATE: cfgRateOk = false; break;
				case CFG_CFG: cfgCfgOk = false; break;
				case CFG_RXM: cfgRxmOk = false; break;
				case CFG_SBAS: cfgSbasOk = false; break;
				case CFG_NMEA: cfgNmeaOk = false; break;
				case CFG_ODO: cfgOdoOk = false; break;
				case CFG_NAV5: cfgNavOk = false; break;
				case CFG_TP5: cfgTpOk = false; break;
				case CFG_PM2: cfgPmOk = false; break;
				case CFG_GNSS: cfgGnssOk = false; break;
				case CFG_LOGFILTER: cfgLogfilterOk = false; break;
				case CFG_GEOFENCE: cfgGeofenceOk = false; break;
				case CFG_PMS: cfgPmsOk = false; break;
				case CFG_SLAS: cfgSlasOk = false; break;
			}
			break;
		case UBX_LOG:
			switch (ubxAck.messageId) {
				case LOG_CREATE: logCreateOk = false; break;
				case LOG_ERASE: logEraseOk = false; break;
			}
			break;
		case UBX_NAV:
			switch (ubxAck.messageId) {
				case NAV_RESETODO: resetOdoOk = false; break;
			}
			break;
	}
	if (DEBUG_UBX) printf("ACK_NAK: msgClass %X, msgId %X\n", ubxNak.classId, ubxNak.messageId);
  free(payload);
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
  printf("GNGGA %s %s %s, fix type: %u, numSVs: %u, hDOP %.2f, alt %.fm, geoidDiff %.fm, ageDC %us, stId %u\n", tt, dmsLatToStr(&(data.lat)).c_str(), dmsLonToStr(&(data.lon)).c_str(), data.fixType, data.numSV, (double)(data.hDOP), (double)(data.alt), (double)(data.geoidEllipsoidDiff), data.ageDiffCorr, data.diffCorrStationId);
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
	printf("GNGNS: %s %s %s, atl %.fm, fixType[GPS] %c|fixType[GLO] %c, numSV %u, hDOP %.2f, geoidDiff %.fm, AgeDC %uc, stId %u\n", tt, dmsLatToStr(&(data.lat)).c_str(), dmsLonToStr(&(data.lon)).c_str(), (double)(data.alt), data.fixType[0], data.fixType[1], data.numSV, (double)(data.hDOP), (double)(data.geoidEllipsoidDiff), data.ageDiffCorr, data.diffCorrStationId);	
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

void Gnss::nmeaGpq(const char* msgId, const char* talkerId) {
	uint8_t x = 0;
	char s[32], ss[32];

	memset(s, 0, 32);
	memset(ss, 0, 32);

	sprintf(s, "%sGPQ,%s", talkerId, msgId);

	for (uint8_t i = 0; i < strlen(s); i++) {
		x ^= s[i];
	}
	sprintf(ss, "$%s*%X\r\n", s, x);
	if (DEBUG_NMEA) printf("Sending NMEA GPQ package: %s\n", ss);
	if (write(fd, ss, strlen(ss)) != strlen(ss)) {
		printf("poll(): write() error: %s\n", strerror(errno));		
	}
	tcdrain(fd);
}

void Gnss::nmeaGnq(const char* msgId, const char* talkerId) {
	uint8_t x = 0;
	char s[32], ss[32];
	memset(s, 0, 32);
	memset(ss, 0, 32);

	sprintf(s, "%sGNQ,%s", talkerId, msgId);

	for (uint8_t i = 0; i < strlen(s); i++) {
		x ^= s[i];
	}
	sprintf(ss, "$%s*%X\r\n", s, x);
	if (DEBUG_NMEA) printf("Sending NMEA GNQ package: %s\n", ss);
	if (write(fd, ss, strlen(ss)) != strlen(ss)) {
		printf("poll(): write() error: %s\n", strerror(errno));		
	}
	tcdrain(fd);
}

void Gnss::nmeaGlq(const char* msgId, const char* talkerId) {
	uint8_t x = 0;
	char s[32], ss[32];
	memset(s, 0, 32);
	memset(ss, 0, 32);

	sprintf(s, "%sGLQ,%s", talkerId, msgId);

	for (uint8_t i = 0; i < strlen(s); i++) {
		x ^= s[i];
	}
	sprintf(ss, "$%s*%X\r\n", s, x);
	if (DEBUG_NMEA) printf("Sending NMEA GLQ package: %s\n", ss);
	if (write(fd, ss, strlen(ss)) != strlen(ss)) {
		printf("poll(): write() error: %s\n", strerror(errno));
	}
	tcdrain(fd);
}

void Gnss::nmeaGbq(const char* msgId, const char* talkerId) {
	uint8_t x = 0;
	char s[32], ss[32];
	memset(s, 0, 32);
	memset(ss, 0, 32);

	sprintf(s, "%sGBQ,%s", talkerId, msgId);

	for (uint8_t i = 0; i < strlen(s); i++) {
		x ^= s[i];
	}
	sprintf(ss, "$%s*%X\r\n", s, x);
	if (DEBUG_NMEA) printf("Sending NMEA GBQ package: %s\n", ss);
	if (write(fd, ss, strlen(ss)) != strlen(ss)) {
		printf("poll(): write() error: %s\n", strerror(errno));		
	}
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
	printf("pubxPosition: %s %s %s +/- %.fm, alt %.f +/- %.fm above ellipsoid, SOG %.1fkm/h, COG %u, vVel %.2fm/s, hDOP %.2f, vDOP %.2f, tDOP %.2f, ageDC %uc, numSV %u, fixType %s\n", tt, dmsLatToStr(&(data.lat)).c_str(), dmsLonToStr(&(data.lon)).c_str(), (double)(data.hAcc), (double)(data.alt), (double)(data.vAcc), (double)(data.sog), data.cog, (double)(data.vVel), (double)(data.hDOP), (double)(data.vDOP), (double)(data.tDOP), data.ageDiffCorr, data.numSV, data.fixType);
}

void Gnss::pubxConfig(BaudRate rate, InProtoMask inMask, OutProtoMask outMask, Port portId, bool autoBauding) {
	uint8_t x = 0;
	char s[255], ss[255];
	memset(s, 0, 255);
	memset(ss, 0, 255);

	uint16_t im = inMask.inUbx | (inMask.inNmea << 1) | (inMask.inRtcm << 2), om = outMask.outUbx | (outMask.outNmea << 1);
	uint8_t ab = autoBauding ? 1 : 0;
	sprintf(s, "PUBX,41,%u,%.4u,%.4u,%u,%u", (uint8_t)portId, im, om, (uint32_t)rate, ab);

	for (uint8_t i = 0; i < strlen(s); i++ ) {
		x ^= s[i];
	}
	sprintf(ss, "$%s*%X\r\n", s, x);
	if (DEBUG_NMEA) printf("Sending NMEA PUBX_CONFIG package: %s\n", ss);
	if (write(fd, ss, strlen(ss)) != strlen(ss)) {
		printf("poll(): write() error: %s\n", strerror(errno));	
	}
	tcdrain(fd);
}

void Gnss::pubxRate(const char * msgId, uint8_t rateCom1, uint8_t rateCom2, uint8_t rateUsb, uint8_t rateDDC, uint8_t rateSpi) {
	uint8_t x = 0;
	char s[255], ss[255];
	memset(s, 0, 255);
	
	sprintf(s, "PUBX,40,%s,%u,%u,%u,%u,%u,%u", msgId, rateDDC, rateCom1, rateCom2, rateUsb, rateSpi, 0);

	for (uint8_t i = 0; i < strlen(s); i++ ) {
		x ^= s[i];
	}
	sprintf(ss, "$%s*%X\r\n", s, x);
	if (DEBUG_NMEA) printf("Sending NMEA PUBX_RATE package: %s\n", ss);
	if (write(fd, ss, strlen(ss)) != strlen(ss)) {
		printf("poll(): write() error: %s\n", strerror(errno));
	}
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
  printf("PubxTime %s, utcTow: %us, utcWeek: %u, leapSec %u, leapSecSrc %c, clkBias %dns, clkDrift %.3fns/s, tpGran %uns\n", tt, data.utcTow, data.utcWeek, data.leapSec, data.leapSecSrc, data.clkBias, data.clkDrift, data.tpGran);
}

//UBX messages
void Gnss::getVersion() {
  	poll(UBX_MON, MON_VER);
}

uint16_t Gnss::getPatches() {
  	poll(UBX_MON, MON_PATCH);
	return numberOfPatches;
}

bool Gnss::getCfgNav() {
  poll(UBX_CFG, CFG_NAV5);
  return cfgNavOk;
}

DynModel Gnss::getDynamicModel() {
  poll(UBX_CFG, CFG_NAV5);
  return cfgNavOk ? gnssNav.dynModel : MODEL_ERROR;
}

bool Gnss::setDynamicModel(DynModel model) {
  if (gnssNav.dynModel != model) {
	  gnssNav.dynModel = model;
	  memset(&(gnssNav.mask), 0, sizeof(uint16_t));
	  gnssNav.mask.dyn = 1;
	  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)&gnssNav);
  }
  return cfgNavOk;
}

UtcStandard Gnss::getUtcStandard() {
  poll(UBX_CFG, CFG_NAV5);
  return cfgNavOk ? gnssNav.utcStandard : UTC_ERROR;
}

bool Gnss::setUtcStandard(UtcStandard standard) {
  if (gnssNav.utcStandard != standard) {
	  gnssNav.utcStandard = standard;
	  memset(&(gnssNav.mask), 0, sizeof(uint16_t));
	  gnssNav.mask.utcMask = 1;
	  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)&gnssNav);
  }
  return cfgNavOk;
}

FixMode Gnss::getFixMode() {
  poll(UBX_CFG, CFG_NAV5);
  return cfgNavOk ? gnssNav.fixMode : FIX_MODE_ERROR;
}

bool Gnss::setFixMode(FixMode fixMode) {
  poll(UBX_CFG, CFG_NAV5);
  if (!cfgNavOk) return false;
  if (gnssNav.fixMode != fixMode) {
	  gnssNav.fixMode = fixMode;
	  gnssNav.mask.posFixMode = 1;
	  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)&gnssNav);
  }
  return cfgNavOk;
}

bool Gnss::getFixedAlt(FixedAlt * fixedAlt) {
  poll(UBX_CFG, CFG_NAV5);
  if (!cfgNavOk) return false;
  fixedAlt->fixedAlt = gnssNav.fixedAlt;
  fixedAlt->fixedAltVar = gnssNav.fixedAltVar;
  return cfgNavOk;
}

bool Gnss::setFixedAlt(FixedAlt * fixedAlt) {
  poll(UBX_CFG, CFG_NAV5);
  if (!cfgNavOk) return false;
  if (gnssNav.fixedAlt != fixedAlt->fixedAlt || gnssNav.fixedAltVar != fixedAlt->fixedAltVar) {
	  gnssNav.fixedAlt = fixedAlt->fixedAlt;
	  gnssNav.fixedAltVar = fixedAlt->fixedAltVar;
	  gnssNav.mask.posMask = 1;
	  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)&gnssNav);
  }
  return cfgNavOk;
}

bool Gnss::getDop(DOP * dop) {
  poll(UBX_CFG, CFG_NAV5);
  if (!cfgNavOk) return false;
  dop->pDOP = gnssNav.pDOP;
  dop->tDOP = gnssNav.tDOP;
  return cfgNavOk;
}

bool Gnss::setDop(DOP * dop) {
  poll(UBX_CFG, CFG_NAV5);
  if (!cfgNavOk) return false;
  gnssNav.pDOP = dop->pDOP;
  gnssNav.tDOP = dop->tDOP;
  gnssNav.mask.posMask = 1;
  gnssNav.mask.timeMask = 1;
  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)&gnssNav);
  return cfgNavOk;
}

bool Gnss::getAccuracy(Accuracy * acc) {
  poll(UBX_CFG, CFG_NAV5);
  if (!cfgNavOk) return false;
  acc->pAcc = gnssNav.pAcc;
  acc->tAcc = gnssNav.tAcc;
  return cfgNavOk;
}

bool Gnss::setAccuracy(Accuracy * acc) {
  poll(UBX_CFG, CFG_NAV5);
  if (!cfgNavOk) return false;
  gnssNav.pAcc = acc->pAcc;
  gnssNav.tAcc = acc->tAcc;
  gnssNav.mask.posMask = 1;
  gnssNav.mask.timeMask = 1;
  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)&gnssNav);
  return cfgNavOk;
}

bool Gnss::getCnoThreshold(CnoThreshold * cno) {
  poll(UBX_CFG, CFG_NAV5);
  if (!cfgNavOk) return false;
  cno->cnoThreshold = gnssNav.cnoThreshold;
  cno->cnoThresholdNumSVs = gnssNav.cnoThresholdNumSVs;
  return cfgNavOk;
}

bool Gnss::setCnoThreshold(CnoThreshold * cno) {
  poll(UBX_CFG, CFG_NAV5);
  if (!cfgNavOk) return false;
  if (gnssNav.cnoThreshold != cno->cnoThreshold || gnssNav.cnoThresholdNumSVs != cno->cnoThresholdNumSVs) {
	  gnssNav.cnoThreshold = cno->cnoThreshold;
	  gnssNav.cnoThresholdNumSVs = cno->cnoThresholdNumSVs;
	  gnssNav.mask.cnoThreshold = 1;
	  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)&gnssNav);
  }
  return cfgNavOk;
}

bool Gnss::getStaticHoldThresholds(StaticHoldThresholds * thresholds) {
  poll(UBX_CFG, CFG_NAV5);
  if (!cfgNavOk) return false;
  thresholds->staticHoldThreshold = gnssNav.staticHoldThreshold;
  thresholds->staticHoldMaxDistance = gnssNav.staticHoldMaxDistance;
  return cfgNavOk;
}

bool Gnss::setStaticHoldThresholds(StaticHoldThresholds * thresholds) {
  poll(UBX_CFG, CFG_NAV5);
  if (!cfgNavOk) return false;
  if (gnssNav.staticHoldThreshold != thresholds->staticHoldThreshold || gnssNav.staticHoldMaxDistance != thresholds->staticHoldMaxDistance) {
	  gnssNav.staticHoldThreshold = thresholds->staticHoldThreshold;
	  gnssNav.staticHoldMaxDistance = thresholds->staticHoldMaxDistance;
	  gnssNav.mask.staticHoldMask = 1;
	  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)&gnssNav);
  }
  return cfgNavOk;
}

int8_t Gnss::getMinElev() {
  poll(UBX_CFG, CFG_NAV5);
  return cfgNavOk ? gnssNav.minElev : -127;
}

bool Gnss::setMinElev(int8_t minElev) {
  poll(UBX_CFG, CFG_NAV5);
  if (!cfgNavOk) return false;
  gnssNav.minElev = minElev;
  gnssNav.mask.minElev = 1;
  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)&gnssNav);
  return cfgNavOk;
}

uint8_t Gnss::getDgnssTimeout() {
  poll(UBX_CFG, CFG_NAV5);
  return cfgNavOk ? gnssNav.dgnssTimeout : 0;
}

bool Gnss::setDgnssTimeout(int8_t dgnssTimeout) {
  poll(UBX_CFG, CFG_NAV5);
  if (!cfgNavOk) return false;
  if (gnssNav.dgnssTimeout != dgnssTimeout) {
	  gnssNav.dgnssTimeout = dgnssTimeout;
	  gnssNav.mask.dgpsMask = 1;
	  poll(UBX_CFG, CFG_NAV5, sizeof(CfgNav), (uint8_t *)&gnssNav);
  }
  return cfgNavOk;
}

bool Gnss::getCfgInf(Protocol protocolId) {
  poll(UBX_CFG, CFG_INF, sizeof(uint8_t), (uint8_t *)&protocolId);
  return cfgInfOk;
}

bool Gnss::setCfgInf(CfgInfo * cfgInfo, Port portId) {
	switch (cfgInfo->protocolId) {
	case UBX:
		gnssCfgInfo[0].protocolId = cfgInfo->protocolId;
		gnssCfgInfo[0].reserved = 0;
		gnssCfgInfo[0].reserved2 = 0;
		for (int port = 0; port < 6; port++) {
			gnssCfgInfo[0].mask[port].debug = 0;
			gnssCfgInfo[0].mask[port].error = 0;
			gnssCfgInfo[0].mask[port].notice = 0;
			gnssCfgInfo[0].mask[port].test = 0;
			gnssCfgInfo[0].mask[port].warning = 0;
		}
		gnssCfgInfo[0].mask[portId].debug = cfgInfo->mask[portId].debug;
		gnssCfgInfo[0].mask[portId].error = cfgInfo->mask[portId].error;
		gnssCfgInfo[0].mask[portId].notice = cfgInfo->mask[portId].notice;
		gnssCfgInfo[0].mask[portId].test = cfgInfo->mask[portId].test;
		gnssCfgInfo[0].mask[portId].warning = cfgInfo->mask[portId].warning;
		poll(UBX_CFG, CFG_INF, sizeof(CfgInfo), (uint8_t *)&gnssCfgInfo[0]);
		break;
	case NMEA:
		gnssCfgInfo[1].protocolId = cfgInfo->protocolId;
		gnssCfgInfo[1].reserved = 0;
		gnssCfgInfo[1].reserved2 = 0;
		for (int port = 0; port < 6; port++) {
			gnssCfgInfo[1].mask[port].debug = 0;
			gnssCfgInfo[1].mask[port].error = 0;
			gnssCfgInfo[1].mask[port].notice = 0;
			gnssCfgInfo[1].mask[port].test = 0;
			gnssCfgInfo[1].mask[port].warning = 0;
		}
		gnssCfgInfo[1].mask[portId].debug = cfgInfo->mask[portId].debug;
		gnssCfgInfo[1].mask[portId].error = cfgInfo->mask[portId].error;
		gnssCfgInfo[1].mask[portId].notice = cfgInfo->mask[portId].notice;
		gnssCfgInfo[1].mask[portId].test = cfgInfo->mask[portId].test;
		gnssCfgInfo[1].mask[portId].warning = cfgInfo->mask[portId].warning;
		poll(UBX_CFG, CFG_INF, sizeof(CfgInfo), (uint8_t *)&gnssCfgInfo[1]);
		break;
	default: printf("setCfgInf(): unknown protocol Id\n");
	}
  return cfgInfOk;
}

bool Gnss::getCfgPms() {
  poll(UBX_CFG, CFG_PMS);
  return cfgPmsOk;
}

int Gnss::setCfgPms(PowerMode * pm) {
	//poll(UBX_CFG, CFG_PMS);
	//if (!cfgPmsOk) return false;
	//if (powerMode.powerMode != pm->powerMode || powerMode.period != pm->period || powerMode.onTime != pm->onTime) {
	  if (pm->period > 0 && pm->period <= 5) return EINVAL;
	  if ((pm->period > 0 || pm->onTime > 0) && pm->powerMode != INTERVAL_POWER_MODE) return EINVAL;
	  powerMode.powerMode = pm->powerMode;
	  powerMode.period = pm->period;
	  powerMode.onTime = pm->onTime;
	  powerMode.version = 0;
	  powerMode.reserved = 0;
	  poll(UBX_CFG, CFG_PMS, sizeof(PowerMode), (uint8_t *)&powerMode);
  //}
  return cfgPmsOk ? 0 : -1;
}

bool Gnss::getCfgPm() {
  poll(UBX_CFG, CFG_PM2);
  return cfgPmOk;
}

bool Gnss::setCfgPm(CfgPm * cfgPm) {
  poll(UBX_CFG, CFG_PM2);
  if (!cfgPmOk) return false;
  gnssCfgPm.maxStartupStateDuration = cfgPm->maxStartupStateDuration;
  gnssCfgPm.updatePeriod = cfgPm->updatePeriod;
  gnssCfgPm.searchPeriod = cfgPm->searchPeriod;
  //gnssCfgPm.gridOffset = cfgPm->gridOffset;
  gnssCfgPm.onTime = cfgPm->onTime;
  gnssCfgPm.minAcqTime = cfgPm->minAcqTime;
  //gnssCfgPm.extintInactivity = cfgPm->extintInactivity;
  //gnssCfgPm.flags = cfgPm->flags;
  poll(UBX_CFG, CFG_PM2, sizeof(CfgPm), (uint8_t *)&gnssCfgPm);
  return cfgPmOk;
}

bool Gnss::getCfgRxm() {
  poll(UBX_CFG, CFG_RXM);
  return cfgRxmOk;
}

bool Gnss::setCfgRxm(CfgRxmLpMode mode) {
  poll(UBX_CFG, CFG_RXM);
  if (!cfgRxmOk) return false;
  if (gnssCfgRxm.lpMode != mode) {
	gnssCfgRxm.lpMode = mode;
	poll(UBX_CFG, CFG_RXM, sizeof(CfgRxm), (uint8_t *)&gnssCfgRxm);
  }
  return cfgRxmOk;
}

bool Gnss::getCfgPrt(Port portId) {
	poll(UBX_CFG, CFG_PRT, sizeof(uint8_t), (uint8_t *)&portId);
	return cfgPrtOk;
}

bool Gnss::setCfgPrt(CfgPrt * cfgPrt) {
  gnssPort.reserved1 = 0;
  gnssPort.reserved2 = 0;
  gnssPort.mode.reserved = 0;
  gnssPort.mode.reserved2 = 0;
  gnssPort.mode.reserved3 = 0;
  gnssPort.mode.charLen = cfgPrt->mode.charLen;
  gnssPort.mode.parity = cfgPrt->mode.parity;
  gnssPort.mode.nStopBits = cfgPrt->mode.nStopBits;
  gnssPort.flags.reserved2 = 0;
  gnssPort.flags.reserved = 0;
  gnssPort.flags.extendedTimeout = cfgPrt->flags.extendedTimeout;
  gnssPort.baudRate = cfgPrt->baudRate;
  gnssPort.inProtoMask.reserved = 0;
  gnssPort.inProtoMask.inUbx = cfgPrt->inProtoMask.inUbx;
  gnssPort.inProtoMask.inNmea = cfgPrt->inProtoMask.inNmea;
  gnssPort.inProtoMask.inRtcm = cfgPrt->inProtoMask.inRtcm;
  gnssPort.outProtoMask.reserved = 0;
  gnssPort.outProtoMask.outUbx = cfgPrt->outProtoMask.outUbx;
  gnssPort.outProtoMask.outNmea = cfgPrt->outProtoMask.outNmea;
  gnssPort.portId = cfgPrt->portId;
  gnssPort.txReady.en = cfgPrt->txReady.en;
  gnssPort.txReady.pin = cfgPrt->txReady.pin;
  gnssPort.txReady.pol = cfgPrt->txReady.pol;
  gnssPort.txReady.thres = cfgPrt->txReady.thres;
  poll(UBX_CFG, CFG_PRT, sizeof(CfgPrt), (uint8_t *)&gnssPort);
  return cfgPrtOk;
}

bool Gnss::config(ConfMask mask, ConfigType type) {
  ConfigAllDevs conf;
  switch (type)
  {
  	  case DEFAULT_CONFIG: conf.clearMask = mask; memset(&(conf.saveMask), 0, sizeof(ConfMask)); memset(&(conf.loadMask), 0, sizeof(ConfMask)); break;
  	  case SAVE_CONFIG: memset(&(conf.clearMask), 0, sizeof(ConfMask)); conf.saveMask = mask; memset(&(conf.loadMask), 0, sizeof(ConfMask)); break;
  	  case LOAD_CONFIG: memset(&(conf.clearMask), 0, sizeof(ConfMask)); memset(&(conf.saveMask), 0, sizeof(ConfMask)); conf.loadMask = mask; break;
  }
  poll(UBX_CFG, CFG_CFG, sizeof(ConfigAllDevs), (uint8_t *)&conf);
  return cfgCfgOk;
}

bool Gnss::config(ConfMask mask, ConfigType type, Device dev) {
  ConfigDev conf;
  switch (type)
  {
  	  case DEFAULT_CONFIG: conf.clearMask = mask; memset(&(conf.saveMask), 0, sizeof(ConfMask)); memset(&(conf.loadMask), 0, sizeof(ConfMask)); break;
  	  case SAVE_CONFIG: memset(&(conf.clearMask), 0, sizeof(ConfMask)); conf.saveMask = mask; memset(&(conf.loadMask), 0, sizeof(ConfMask)); break;
  	  case LOAD_CONFIG: memset(&(conf.clearMask), 0, sizeof(ConfMask)); memset(&(conf.saveMask), 0, sizeof(ConfMask)); conf.loadMask = mask; break;
  }
  conf.device = dev;
  poll(UBX_CFG, CFG_CFG, sizeof(ConfigDev), (uint8_t *)&conf);
  return cfgCfgOk;
}

bool Gnss::defaultConfig(bool ioPort, bool msgConf, bool infMsg, bool navConf, bool rxmConf, bool senConf, bool rinvConf, bool antConf, bool logConf, bool ftsConf) {
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
  return config(conf, DEFAULT_CONFIG);
}

bool Gnss::saveConfig(bool ioPort, bool msgConf, bool infMsg, bool navConf, bool rxmConf, bool senConf, bool rinvConf, bool antConf, bool logConf, bool ftsConf) {
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
  return config(conf, SAVE_CONFIG);
}

bool Gnss::loadConfig(bool ioPort, bool msgConf, bool infMsg, bool navConf, bool rxmConf, bool senConf, bool rinvConf, bool antConf, bool logConf, bool ftsConf) {
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
  return config(conf, LOAD_CONFIG);
}

bool Gnss::cfgRst(StartType type, ResetMode mode) {
	CfgRst rset;
	rset.navBbrMask = type.mask;
	rset.resetMode = mode;
	poll(UBX_CFG, CFG_RST, sizeof(CfgRst), (uint8_t*)&rset);
	return cfgRstOk;
}

bool Gnss::stopGnss(StartTypes startType) {
	StartType type;
	type.start = startType;
	return cfgRst(type, GNSS_STOP);
}

bool Gnss::startGnss(StartTypes startType) {
	StartType type;
	type.start = startType;
	return cfgRst(type, GNSS_START);
}

bool Gnss::resetGnss(StartTypes startType) {
	StartType type;
	type.start = startType;
	return cfgRst(type, SOFTWARE_RESET_GNSS_ONLY);
}

bool Gnss::reset(bool soft, bool afterShutdown, StartTypes startType) {
	StartType type;
	type.start = startType;
	if (soft) return cfgRst(type, SOFTWARE_RESET);
	else {
		if (afterShutdown) return cfgRst(type, HARDWARE_RESET_AFTER_SHUTDOWN);
		else return cfgRst(type, HARDWARE_RESET);
	}
}

bool Gnss::getCfgMsg(uint8_t msgClass, uint8_t msgId) {
	CfgMsgPoll msgPoll;
	msgPoll.msgClass = msgClass;
	msgPoll.msgId = msgId;
	poll(UBX_CFG, CFG_MSG, sizeof(CfgMsgPoll), (uint8_t *)&msgPoll);
	return cfgMsgOk;
}

bool Gnss::setCfgMsg(uint8_t msgClass, uint8_t msgId, Port port, uint8_t rate) {
  CfgMsgPoll msgPoll;
  msgPoll.msgClass = msgClass;
  msgPoll.msgId = msgId;
  poll(UBX_CFG, CFG_MSG, sizeof(CfgMsgPoll), (uint8_t *)&msgPoll);
  if (!cfgMsgOk) return false;
  if (gnssCfgMsgs.rate[port] != rate) {
	  gnssCfgMsg.msgClass = msgClass;
	  gnssCfgMsg.msgId = msgId;
	  gnssCfgMsg.rate = rate;
	  poll(UBX_CFG, CFG_MSG, sizeof(CfgMsg), (uint8_t *)&gnssCfgMsg);
  }
  return cfgMsgOk;
}

bool Gnss::getCfgOdo() {
  poll(UBX_CFG, CFG_ODO);
  return cfgOdoOk;
}

bool Gnss::setCfgOdo(ODOCfg * odoCfg) {
  //poll(UBX_CFG, CFG_ODO);
  //if (!cfgOdoOk) return false;
  gnssCfgOdo.reserved1 = 0; gnssCfgOdo.reserved2 = 0; gnssCfgOdo.reserved3 = 0;
  gnssCfgOdo.reserved4 = 0; gnssCfgOdo.reserved5 = 0; gnssCfgOdo.reserved6 = 0;
  gnssCfgOdo.version = 0;
  gnssCfgOdo.flags.ODOenabled = odoCfg->flags.ODOenabled;
  gnssCfgOdo.flags.COGenabled = odoCfg->flags.COGenabled;
  gnssCfgOdo.flags.outputLPvelocity = odoCfg->flags.outputLPvelocity;
  gnssCfgOdo.flags.outputLPcog = odoCfg->flags.outputLPcog;
  gnssCfgOdo.odoProfile = odoCfg->odoProfile;
  gnssCfgOdo.cogMaxSpeed = odoCfg->cogMaxSpeed;
  gnssCfgOdo.cogMaxPosAccuracy = odoCfg->cogMaxPosAccuracy;
  gnssCfgOdo.velLPgain = odoCfg->velLPgain; 
  gnssCfgOdo.cogLPgain = odoCfg->cogLPgain;
  poll(UBX_CFG, CFG_ODO, sizeof(ODOCfg), (uint8_t *)&gnssCfgOdo);
  return cfgOdoOk;
}

bool Gnss::getCfgSbas() {
  poll(UBX_CFG, CFG_SBAS);
  return cfgSbasOk;
}

bool Gnss::setCfgSbas(CfgSbas * cfgSbas) {
  poll(UBX_CFG, CFG_SBAS);
  if (!cfgSbasOk) return false;
  gnssCfgSbas.usage = cfgSbas->usage;
  gnssCfgSbas.mode = cfgSbas->mode;
  //gnssCfgSbas.scanMode1 = cfgSbas->scanMode1;
  //gnssCfgSbas.scanMode2 = cfgSbas->scanMode2;
  poll(UBX_CFG, CFG_SBAS, sizeof(CfgSbas), (uint8_t *)&gnssCfgSbas);
  return cfgSbasOk;
}

bool Gnss::getCfgNmea() {
  poll(UBX_CFG, CFG_NMEA);
  return cfgNmeaOk;
}

bool Gnss::setCfgNmea(CfgNmea * cfgNmea) {
  //poll(UBX_CFG, CFG_NMEA);
  //if (!cfgNmeaOk) return false;
  gnssCfgNmea.version = 1;
  gnssCfgNmea.maxSV = cfgNmea->maxSV;
  gnssCfgNmea.filter = cfgNmea->filter;
  gnssCfgNmea.filter.reserved = 0;
  gnssCfgNmea.nmeaVersion = cfgNmea->nmeaVersion;
  gnssCfgNmea.flags = cfgNmea->flags;
  gnssCfgNmea.flags.reserved = 0;
  gnssCfgNmea.gnssFilter = cfgNmea->gnssFilter;
  gnssCfgNmea.gnssFilter.reserved = 0;
  gnssCfgNmea.gnssFilter.reserved2 = 0;
  gnssCfgNmea.displayNonNmeaSVs = cfgNmea->displayNonNmeaSVs;
  gnssCfgNmea.mainTalkerId = cfgNmea->mainTalkerId;
  gnssCfgNmea.gsvTalkerId = cfgNmea->gsvTalkerId;
  gnssCfgNmea.bdsTalkerId[0] = cfgNmea->bdsTalkerId[0];
  gnssCfgNmea.bdsTalkerId[1] = cfgNmea->bdsTalkerId[1];
  gnssCfgNmea.reserved = 0;
  gnssCfgNmea.reserved2 = 0;
  poll(UBX_CFG, CFG_NMEA, sizeof(CfgNmea), (uint8_t *)&gnssCfgNmea);
  return cfgNmeaOk;
}

//LOG messages
bool Gnss::getCfgLogFilter() {
  poll(UBX_CFG, CFG_LOGFILTER);
  return cfgLogfilterOk;
}

bool Gnss::setCfgLogFilter(CfgLogFilter * cfgLogFilter) {
  gnssLogFilter.version = 1;
  gnssLogFilter.minInterval = cfgLogFilter->minInterval;
  gnssLogFilter.timeThreshold = cfgLogFilter->timeThreshold;
  gnssLogFilter.speedThreshold = cfgLogFilter->speedThreshold;
  gnssLogFilter.positionThreshold = cfgLogFilter->positionThreshold;
  gnssLogFilter.flags = cfgLogFilter->flags;
  poll(UBX_CFG, CFG_LOGFILTER, sizeof(CfgLogFilter), (uint8_t*)&gnssLogFilter);
  return cfgLogfilterOk;
}

bool Gnss::enableLogging(uint16_t interval) {
	gnssLogFilter.flags.recordingEnabled = 1;
	gnssLogFilter.flags.psmOncePerWakeUpEnabled = 0;
	gnssLogFilter.flags.applyAllFilterSettings = 1;
	gnssLogFilter.minInterval = 0;
	gnssLogFilter.positionThreshold = 0;
	gnssLogFilter.speedThreshold = 0;
	gnssLogFilter.timeThreshold = interval;
	return setCfgLogFilter(&gnssLogFilter);
}

bool Gnss::disableLogging() {
	gnssLogFilter.flags.recordingEnabled = 0;
	gnssLogFilter.flags.psmOncePerWakeUpEnabled = 0;
	gnssLogFilter.flags.applyAllFilterSettings = 0;
	gnssLogFilter.minInterval = 0;
	gnssLogFilter.positionThreshold = 0;
	gnssLogFilter.speedThreshold = 0;
	gnssLogFilter.timeThreshold = 0;
	return setCfgLogFilter(&gnssLogFilter);
}

void Gnss::getLogInfo() {
  poll(UBX_LOG, LOG_INFO);
}

bool Gnss::createLog(LogSize logSize, uint32_t userDefinedSize, bool circular) {
  LogCreate logCreate;
  logCreate.version = 0;
  logCreate.circular = circular ? 1 : 0;
  logCreate.reserved = 0;
  logCreate.logSize = logSize;
  logCreate.userDefinedSize = userDefinedSize;
  poll(UBX_LOG, LOG_CREATE, sizeof(LogCreate), (uint8_t *)&logCreate);
  return logCreateOk;
}

bool Gnss::eraseLog() {
  poll(UBX_LOG, LOG_ERASE);
  return logEraseOk;
}

uint32_t Gnss::logFind(DateTime dateTime) {
	LogFindTimeRequest req;
	LogFindTimeResponse * res = NULL;
	req.version = 0;
	req.type = 0;
	req.reserved = 0;
	req.dateTime = dateTime;
	req.reserved2 = 0;
	poll(UBX_LOG, LOG_FINDTIME, sizeof(LogFindTimeRequest), (uint8_t *)&req);
	return logIndex;
}

void Gnss::logMsg(const char * msg, uint8_t len) {
  poll(UBX_LOG, LOG_STRING, len, (uint8_t *)msg);
}

void  Gnss::logRetrieve(uint32_t index, uint32_t count) {
  struct {
	uint32_t index;
	uint32_t count; //256 max!
	uint8_t version;
	uint16_t reserved;
	uint8_t reserved2;
  }  __attribute__((packed)) msg;
  msg.index = index;
  msg.count = count;
  msg.version = 0;
  msg.reserved = 0;
  msg.reserved2 = 0;
  poll(UBX_LOG, LOG_RETRIEVE, sizeof(msg), (uint8_t *)&msg);
  //ACK-NAK will be sent after the log messages, 
  //especially NAK is sent when there are more than 256 records to retrieve after first 256 records are sent to indicate that
  //logRetrieve command should be repeated with the new index!
}

void Gnss::getSupportedGnss() {
  poll(UBX_MON, MON_GNSS);
}

bool Gnss::getGnss() {
  poll(UBX_CFG, CFG_GNSS);
  return cfgGnssOk;
}

int Gnss::setGnss(MajorGnss gnss, bool enableSBAS, bool enableIMES) {
  poll(UBX_CFG, CFG_GNSS);
  if (!cfgGnssOk) return -1;
  for (uint8_t i = 0; i < gnssConf.numConfigBlocks; i++) {
	  switch (configuredGnss[i].gnssId) {
		  case GPS_ID: configuredGnss[i].flags.enabled = gnss.Gps; break;
		  case SBAS_ID: configuredGnss[i].flags.enabled = enableSBAS; break; 
		  case Galileo_ID: configuredGnss[i].flags.enabled = gnss.Galileo; break;
		  case BeiDou_ID: configuredGnss[i].flags.enabled = gnss.BeiDou; break;
		  case IMES_ID: configuredGnss[i].flags.enabled = enableIMES; break;
		  case QZSS_ID: configuredGnss[i].flags.enabled = gnss.Gps; break;//QZSS should be enabled or disabled togather with GPS
		  case GLONASS_ID: configuredGnss[i].flags.enabled = gnss.Glonass; break;
	  }
  }
  uint8_t * buf;
  buf = (uint8_t *)malloc(sizeof(GnssConf) + sizeof(GnssCfg) * gnssConf.numConfigBlocks);
  if (buf == NULL) { printf("setGnss(): out of memory\n"); return ENOMEM;}
  memcpy(buf, &gnssConf, sizeof(GnssConf));
  memcpy(buf + sizeof(GnssConf), &configuredGnss, sizeof(GnssCfg) * gnssConf.numConfigBlocks);
  poll(UBX_CFG, CFG_GNSS, sizeof(GnssConf) + sizeof(GnssCfg) * gnssConf.numConfigBlocks, (uint8_t *)buf);
  free(buf);
  return cfgGnssOk ? 0 : -1;
}

bool Gnss::getNavRate() {
  poll(UBX_CFG, CFG_RATE);
  return cfgRateOk;
}

bool Gnss::setNavRate(NavRate * navRate) {
  poll(UBX_CFG, CFG_RATE);
  if (!cfgRateOk) return false;
  gnssNavRate = *((NavRate*)payload);
  gnssNavRate.rate = navRate->rate; //ms
  gnssNavRate.navSolRate = navRate->navSolRate; //cycles - number of measurements for each NavSol, max 127.
  gnssNavRate.timeRef = navRate->timeRef;
  poll(UBX_CFG, CFG_RATE, sizeof(NavRate), (uint8_t *)&gnssNavRate);
  return cfgRateOk;
}

bool Gnss::getTimePulse() {
  poll(UBX_CFG, CFG_TP5);
  return cfgTpOk;
}

bool Gnss::setTimePulse(TimePulse * tp) {
  poll(UBX_CFG, CFG_TP5);
  if (!cfgTpOk) return false;
  gnssTimePulse = *((TimePulse*)payload);
  gnssTimePulse.pulsePeriod = tp->pulsePeriod;
  gnssTimePulse.pulseLen = tp->pulseLen; 
  gnssTimePulse.pulsePeriodLocked = tp->pulsePeriodLocked;
  gnssTimePulse.pulseLenLocked = tp->pulseLenLocked; 
  gnssTimePulse.userConfigDelay = tp->userConfigDelay; 
  gnssTimePulse.flags.active = tp->flags.active;
  gnssTimePulse.flags.lockGnssFreq = tp->flags.lockGnssFreq;
  gnssTimePulse.flags.lockedOtherSet = tp->flags.lockedOtherSet;
  gnssTimePulse.flags.isFreq = tp->flags.isFreq;
  gnssTimePulse.flags.isLength = tp->flags.isLength;
  gnssTimePulse.flags.alignToTOW = tp->flags.alignToTOW;
  gnssTimePulse.flags.gridUtcGnss = tp->flags.gridUtcGnss;
  gnssTimePulse.flags.syncMode = tp->flags.syncMode;
  poll(UBX_CFG, CFG_TP5, sizeof(TimePulse), (uint8_t *)&gnssTimePulse);
  return cfgTpOk;
}

void Gnss::getNavPvt() {
  poll(UBX_NAV, NAV_PVT);
}

void Gnss::getNavSat() {
  poll(UBX_NAV, NAV_SAT);
}

void Gnss::getNavTimeUtc() {
  poll(UBX_NAV, NAV_TIMEUTC);
}

void Gnss::getNavClock() {
  poll(UBX_NAV, NAV_CLOCK);
}

void Gnss::getNavDgps() {
  poll(UBX_NAV, NAV_DGPS);
}

void Gnss::getNavDop() {
  poll(UBX_NAV, NAV_DOP);
}

void Gnss::getNavGeofence() {
  poll(UBX_NAV, NAV_GEOFENCE);
}

bool Gnss::getCfgGeofences() {
  poll(UBX_CFG, CFG_GEOFENCE);
  return cfgGeofenceOk;
}

bool Gnss::setCfgGeofence(GeoFence * geofence, uint8_t confidenceLevel) {
  poll(UBX_CFG, CFG_GEOFENCE);
  if (!cfgGeofenceOk) return false;
  CfgGeofences cfgFences;
  cfgFences.geoFences.version = gnssGeoFences.version;
  cfgFences.geoFences.numFences = 1;
  cfgFences.geoFences.confidenceLevel = confidenceLevel;
  cfgFences.geoFences.reserved = gnssGeoFences.reserved;//0x20
  cfgFences.geoFences.pioEnabled = gnssGeoFences.pioEnabled;//0
  cfgFences.geoFences.pinPolarity = gnssGeoFences.pinPolarity;//0
  cfgFences.geoFences.pin = gnssGeoFences.pin;//0
  cfgFences.geoFences.reserved2 = gnssGeoFences.reserved2;//0x20
  cfgFences.geoFence.lat = geofence->lat;
  cfgFences.geoFence.lon = geofence->lon;
  cfgFences.geoFence.radius = geofence->radius;
  poll(UBX_CFG, CFG_GEOFENCE, sizeof(CfgGeofences), (uint8_t *)&cfgFences);
  return cfgGeofenceOk;
}

void Gnss::getNavOdo() {
  poll(UBX_NAV, NAV_ODO);
}

bool Gnss::resetOdo() {
  poll(UBX_NAV, NAV_RESETODO);
  return resetOdoOk;
}

void Gnss::getNavOrb() {
  poll(UBX_NAV, NAV_ORB);
}

void Gnss::getNavPosEcef() {
	poll(UBX_NAV, NAV_POSECEF);
}

void Gnss::getNavPosLlh() {
  poll(UBX_NAV, NAV_POSLLH);
}

void Gnss::getNavSbas() {
  poll(UBX_NAV, NAV_SBAS);
}

void Gnss::getNavSlas() {
  poll(UBX_NAV, NAV_SLAS);
}

void Gnss::getNavTimeBds() {
  poll(UBX_NAV, NAV_TIMEBDS);
}

void Gnss::getNavTimeGal() {
  poll(UBX_NAV, NAV_TIMEGAL);
}

void Gnss::getNavTimeGps() {
  poll(UBX_NAV, NAV_TIMEGPS);
}

void Gnss::getNavTimeGlo() {
  poll(UBX_NAV, NAV_TIMEGLO);
}

void Gnss::getNavTimeLs() {
  poll(UBX_NAV, NAV_TIMELS);
}

void Gnss::getTimTm() {
  poll(UBX_TIM, TIM_TM2);
}

void Gnss::getTimTp() {
  poll(UBX_TIM, TIM_TP);
}

void Gnss::getNavStatus() {
	poll(UBX_NAV, NAV_STATUS);
}

void Gnss::getNavVelEcef() {
	poll(UBX_NAV, NAV_VELECEF);
}

void Gnss::getNavVelNed() {
	poll(UBX_NAV, NAV_VELNED);
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
			case 8: data->clkDrift = stof(tim[i], nullptr); break;
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
	  const uint32_t l1 = 10000000;
	  const uint32_t l2 = 1000000;
	  const double l3 = 100000;
	  if (lat < 0) {
		latitude->NS = 'S';
		l = -lat;
	  }
      else 
	  {
		latitude->NS = 'N';      
		l = lat;
	  }
   	  latitude->deg = l / l1;
      l %= l1;
      latitude->min = l * 6 / l2;
      latitude->sec = l * 36.0 / l3 - latitude->min * 60.0;
}

int32_t dmsToLongLat(Latitude * latitude) {
	const uint32_t l1 = 10000000;
	const uint32_t l2 = 1000000;
	const double l3 = 100000;

	int32_t lat = latitude->deg * l1 + latitude->min * l2 / 6 + latitude->sec * l3 / 36.0;	  
	if (latitude->NS == 'S') lat = -lat;
	return lat;
}

void longLonToDMS(Longitude * longitude, int32_t lon) {
      uint32_t l;
	  const uint32_t l1 = 10000000;
	  const uint32_t l2 = 1000000;
	  const double l3 = 100000;
	  if (lon < 0) {
		longitude->EW = 'W';
		l = -lon;
	  }
      else 
	  {
		longitude->EW = 'E';      
		l = lon;
	  }
	  longitude->deg = l / l1;
      l %= l1;
      longitude->min = l * 6 / l2;
      longitude->sec = l * 36.0 / l3 - longitude->min * 60.0;
}

int32_t dmsToLongLon(Longitude * longitude) {
      const uint32_t l1 = 10000000;
	  const uint32_t l2 = 1000000;
	  const double l3 = 100000;
	  int32_t lon = longitude->deg * l1 + longitude->min * l2 / 6 + longitude->sec * l3 / 36.0;
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

char * humanTime(uint32_t time, char * htime) {
	if (htime) {
		uint16_t days = time / 86400;
		uint8_t hours = (time % 86400) / 3600;
		uint8_t minutes = (time % 3600) / 60;
		uint8_t seconds = time % 60;
		if (days) sprintf(htime, "%u days %u hours %u minutes %u seconds", days, hours, minutes, seconds);
		else if (hours) sprintf(htime, "%u hours %u minutes %u seconds", hours, minutes, seconds);
		else if (minutes) sprintf(htime, "%u minutes %u seconds", minutes, seconds);
		else if (seconds) sprintf(htime, "%u seconds", seconds);
		return htime;
	}
	else return NULL;
}

void delay(int sec) {
	time_t t = time(NULL);	
	while (time(NULL) - t < sec) {
		sleep(sec);
		continue;
	}
}

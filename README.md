# Gnss
Arduino library for u-blox M8 UBX-M8030 GNSS (GPS GLONASS Galileo BeiDou) chip 

This is probably the most complete u-blox library for Arduino. It supports both protocols: NMEA and UBX. It implements the majority of the functions up to and including the protocol version 18.

The library provides two Gnss class templates: Gnss < class > and Gnss < class, class > where class could be HardwareSerial or SoftwareSerial. The first class template parameter is for the communication with a Gnss chip. It defaults to COM1 port but other ports may also be also supported. I haven't tested them as my TOPGNSS board only comes with COM1 broken out. The second class template parameter is for the output. It can be anything that has serial-like input and works with either HardwareSerial or SoftwareSerial. I tested it with an Arduino console and an ESP8266 WiFi module.
  
The first class template is very light because it does not include any textual output. It only provides empty function prototypes. You would need to fill them with whatever you need to do with the Gnss output. 

The second class template actually implements these functions. Therefore, it is much heavier because of numerous strings, which I tried to keep in the flash memory. They print out the parsed output from Gnss using the second class template parameter. 

Because of very small UART buffer in Arduino board, you need to select UART speed for communication with GNSS and other devices carefully. I found that 9600 baud is ok for GNSS but the output device should have higher speed. I used 115200 for ESP8266 and Arduino console via HardwareSerial. Because GNSS sends lots of data, if you don't read them out in time from the Arduino buffer, the following GNSS data will overwrite the unread one and you'll get lots of checksum errors.
  
Please refer to the official u-blox M8 GNSS product description u-blox8-M8_ReceiverDescrProtSpec_(UBX-13003221)_Public.pdf for understanding its featues and functions.

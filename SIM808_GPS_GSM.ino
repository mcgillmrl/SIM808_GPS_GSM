#include<stdio.h>
#include<string.h>
#define ADAFRUIT_FONA_DEBUG
#include "GenericSIM808.h"
#define DEBUG true


int power_on=9;
char replybuffer[250];

// Use this for FONA 800 and 808s
GenericSIM808 sim808 = GenericSIM808(power_on);
uint8_t type;

//gps data;
bool gps_forwarding = false;
int gps_msg_count = 0;
char gpsdata[120];
long last_msg_time=millis();
int stat=0;
float timestamp=20191111191056.000;
float lat=45.534430;
float lon=-73.611353;
float alt=63.500;
float speed_over_ground=0.0;
float course_over_ground=0;
int fix_mode=0;
float hdop=0;
float pdop=0;
float vdop=0;
int sat_in_view=0;
int sat_used=0;
int CN0_max=0;


void setup()
{
  SerialUSB.begin(1000000);
  while (!SerialUSB) {
    ; // wait for serial port to connect
  }
  sim808_init();
}

void loop()
{
    // check gps status
    checkGPS();

    if(sim808.available()){
      while (sim808.available()) {
         SerialUSB.write(sim808.read());
      }
    }
             
    if ((gps_msg_count % 30) == 1){
      SerialUSB.println("Forwarding GPS data to remote server");
      if (gps_forwarding)
        sim808.enableGPSNMEA(0);
      send_http_get_request();
      if (gps_forwarding)
        sim808.enableGPSNMEA(1);
    }
}


void sim808_init(){
  Serial1.begin(115200);
  if (!sim808.begin_(Serial1)) {
    SerialUSB.println(F("Couldn't find SIM808"));
    while (1);
  }
  delay(1000);
  
  // Print module IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = sim808.getIMEI(imei);
  if (imeiLen > 0) {
    SerialUSB.print(F("Module IMEI: ")); SerialUSB.println(imei);
  }

  gsm_connect();
  gps_power_on();
}

void switch_power(){
  pinMode(power_on, OUTPUT);
  digitalWrite(power_on, HIGH);
  delay(3000);
  digitalWrite(power_on, LOW);
  delay(1000); 
}

bool gps_power_on(){
  // turn gps on
  SerialUSB.print("Turning GPS on...");
  while(!sim808.enableGPS(true)){
    delay(1000);
  }
  SerialUSB.println(" OK");
  delay(1000);
}

bool gsm_connect(){
  // set GPRS network settings
  sim808.setGPRSNetworkSettings(F("hologram"), F(""), F(""));

  // wait until network is up
  SerialUSB.print(F("Waiting for network registration..."));
  uint8_t n = sim808.getNetworkStatus();
  while (n != 1 && n != 5){
    delay(1000);
    n = sim808.getNetworkStatus();
  }
  SerialUSB.println(" OK");

  // wait until gprs is successfully enabled
  SerialUSB.print(F("Enabling GPRS..."));
  while(!sim808.enableGPRS(true)){
    delay(1000);
  }
  SerialUSB.println(F(" OK"));
}

void checkGPS(){
  if(millis() - last_msg_time > 1000){
    int8_t stat = sim808.GPSstatus(); 
    switch (stat){
      case 0:
        gps_power_on(); 
        break;
      case 1:
        if (gps_forwarding){
          SerialUSB.println("Disabled GPS forwarding");
          sim808.enableGPSNMEA(0);
          gps_forwarding = false;
        }
        delay(1000);
      case 2:
      case 3:
        // start forwarding
        if (!gps_forwarding){
          SerialUSB.println("Forwarding GPS data to host");
          sim808.enableGPSNMEA(1);
          gps_forwarding = true;
        }
        // read gps
        last_msg_time = millis();
        sim808.getGPS(0, gpsdata, 120);
        process_gps_msg(gpsdata);
        gps_msg_count++;
        break;
      default:
        break;
    }
  }
}

uint16_t GSMLoc(){
  uint16_t returncode;
  while(!sim808.getGSMLoc(&returncode, replybuffer, 250)){
    delay(1000);
  }
  SerialUSB.print("Return code: ");
  SerialUSB.println(returncode);
  SerialUSB.println(replybuffer);
  return returncode;
}

void process_gps_msg(char* data){
  char *ptr = strtok(data, ":,");
  int i = 0;
  while(ptr != NULL){
    switch(i){
      case 1:
          stat = atoi(ptr);
      case 2:
          timestamp = atof(ptr);
      case 3:
          lat = atof(ptr);
      case 4:
          lon = atof(ptr);
      case 5:
          alt = atof(ptr);
      case 6:
          speed_over_ground = atof(ptr);
      case 7:
          course_over_ground = atof(ptr);
      case 8:
          fix_mode = atoi(ptr);
      case 9:
          hdop = atof(ptr);
      case 10:
          pdop = atof(ptr);
      case 11:
          vdop = atof(ptr);
      case 12:
          sat_in_view = atoi(ptr);
      case 13:
          sat_used = atoi(ptr);
      case 14:
          CN0_max = atoi(ptr);
    }
    i++;
    ptr = strtok(NULL, ":,");
  }
}

void send_http_get_request(){
  uint16_t statuscode;
  uint16_t datalen = 0;
  String url = F("http://132.206.74.128:8000");
  url += "?lat=";
  url += lat;
  url += "&lon=";
  url += lon;
  url += "&timestamp=";
  url += timestamp;
  if (!sim808.HTTP_GET_start((char *)url.c_str(), &statuscode, &datalen)){
    SerialUSB.print(F("GET request failed. Status: "));
    SerialUSB.println(statuscode);
  }
  SerialUSB.println(F("\n****\n"));
  while (datalen > 0) {
    while (sim808.available()) {
      char c = sim808.read();

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
      loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
      UDR0 = c;
#else
      SerialUSB.write(c);
#endif
      datalen--;
      if (!datalen) 
        break;
    }
  }
  SerialUSB.println(F("\n****\n"));
  sim808.HTTP_POST_end();
}


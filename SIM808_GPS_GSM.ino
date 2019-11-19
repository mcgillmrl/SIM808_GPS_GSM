#include "PPMReader.h"

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

//remote code
PPMReader ppmReader(3, digitalPinToInterrupt(3), false);
//Infrared-----------------------------------------------------------------------
#define IR_SEND_PWM_PIN 9
#include "IRLibAll.h"
//Create a receiver object to listen on pin 2
IRrecvPCI myReceiver(5);
//Create a decoder object 
IRdecode myDecoder;
IRsend mySender;
uint32_t lastValueSent;
volatile unsigned long lastIRTime = 0;  // the last time the output pin was toggled
//NeoPixels
#include "FastLED.h"
#define NUM_LEDS 10
CRGB leds[NUM_LEDS];

void setup()
{
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  for(int t = 0; t<1000; t++)
  {digitalWrite(4, HIGH);
  delay(200);
  digitalWrite(4, LOW);
  delay(200);
  }

  
  //remote code
  ppmReader.start();
  //Infrared-----------------------------------------------------------------------
  myReceiver.enableIRIn(); // Start the receiver
  //NeoPixel
  FastLED.addLeds<NEOPIXEL, 6>(leds, NUM_LEDS);

          for(int j=0; j<3; j++)
    {
    for(int i=0; i<3; i++)
    {
      leds[i] = CRGB::White; FastLED.show(); delay(100);
//      leds[i] = CRGB::Black; FastLED.show(); delay(100);
    }
    for(int i=0; i<10; i++)
    {
      leds[i] = CRGB::Red; FastLED.show(); delay(100);
//      leds[i] = CRGB::Black; FastLED.show(); delay(100);
    }
    for(int i=0; i<10; i++)
    {
      leds[i] = CRGB::Green; FastLED.show(); delay(100);
//      leds[i] = CRGB::Black; FastLED.show(); delay(100);
    }
    for(int i=0; i<10; i++)
    {
      leds[i] = CRGB::Blue; FastLED.show(); delay(100);
//      leds[i] = CRGB::Black; FastLED.show(); delay(100);
    }
    }


  
  SerialUSB.begin(1000000);
  SerialUSB.write("hello Juavis");
//  while (!SerialUSB) {
//    ; // wait for serial port to connect
//  }
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

    SerialUSB.print("\n");
    SerialUSB.print("R,");

    for(int l=0;l<8;l++)
    {
      SerialUSB.print((int)ppmReader.get(l));
      SerialUSB.print(",");
      //Serial.print(testf[l]);Serial.print(", ");
    }
    SerialUSB.print("\n");

  //------IR RECEIVER---------------------------------------------------------------
  //Continue looping until you get a complete signal received
  if (myReceiver.getResults()) {
    myDecoder.decode();           //Decode it
    //myDecoder.dumpResults(true);  //Now print results. Use false for less detail
    myReceiver.enableIRIn();      //Restart receiver
    //Serial.println(myDecoder.value);
    receivedValue(myDecoder.value);
  }
//    delay(50);
}

void receivedValue(uint32_t value)
{ 
  
  uint32_t deltaT = (millis() - lastIRTime); 
  lastIRTime = millis();
  //Serial.println(deltaT);
  if (deltaT > 500) {
  switch(value) {
        case 0xc0:  //u
//          flashColor(pixels.Color(0,150,0));
          doBeep(2200);
          SerialUSB.write("u\n");
          break;
        case 0xc1:  //d
//          flashColor(pixels.Color(150,0,0));
          doBeep(1800);
          SerialUSB.write("d\n");
          break;
        case 0xc2:  //l
//          flashColor(pixels.Color(150,150,0));
          doBeep(1900);
          SerialUSB.write("l\n");
          break;
        case 0xc3:  //c
//          flashColor(pixels.Color(0,0,150));
          doBeep(2000);
          SerialUSB.write("c\n");
          break;
        case 0xc4:  //r
//          flashColor(pixels.Color(0,150,150));
          doBeep(2100);
          SerialUSB.write("r\n");
          break;
    }
    sendIR(value);
    //doBeep(4000);
    
  }
}

void sendIR(uint32_t code)
{
  myReceiver.disableIRIn();
  //mySender.send(SONY,0xa8bca, 20);
  SerialUSB.print("Code:");
  SerialUSB.print(code);
  SerialUSB.print("\n");
  mySender.send(SONY,code, 8);
  lastValueSent = code;
  myReceiver.enableIRIn(); // Re-enable receiver
}


void doBeep(int freq) {

}

void setColor(uint32_t c) {// uint8_t wait

}

void flashColor(uint32_t c) {

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
        //        delay(1000);
        break; 
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


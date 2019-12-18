#include "PPMReader.h"
#include <stdio.h>
#include <string.h>
#include "GenericSIM808.h"

#define RC_IN 2
#define RELAY_CLK 3
#define RELAY_D 4
#define IR_RX 5
#define TOP_LEDS 6
#define INTERNAL_LEDS 7
#define SPEAKER 8
#define IR_TX 9
#define SIM808_POWER_ON 9
#define NUM_LEDS 10

#include "IRLibAll.h"
#include "FastLED.h"

// gps global variables
int gps_msg_count = 0;
char gpsdata[120];
long last_msg_time = millis();
int stat = 0;
float timestamp = 20191111191056.000;
float lat = 45.534430;
float lon = -73.611353;
float alt = 63.500;
float speed_over_ground = 0.0;
float course_over_ground = 0;
int fix_mode = 0;
float hdop = 0;
float pdop = 0;
float vdop = 0;
int sat_in_view = 0;
int sat_used = 0;
int CN0_max = 0;

// IR remote global variables
uint32_t lastValueSent;
volatile unsigned long lastIRTime = 0; // the last time the output pin was toggled

// RC global variables
String inputString = ""; // a String to hold incoming data

//NeoPixels
CRGB leds[NUM_LEDS];

// SIM808 GPS GSM module
GenericSIM808 sim808 = GenericSIM808(SIM808_POWER_ON);

// RC remote reader
PPMReader ppmReader(RC_IN, digitalPinToInterrupt(RC_IN), false);

//Infrared remote
IRrecvPCI myReceiver(IR_RX); // receiver
IRdecode myDecoder;          // decoder
IRsend mySender;             // sender

void setup()
{
  delay(1000);

  setup_pins();

  // init serial
  SerialUSB.begin(115200);

  // start RC remote
  ppmReader.start();

  // start IR remote
  myReceiver.enableIRIn(); // Start the receiver

  // start LEDS
  FastLED.addLeds<NEOPIXEL, TOP_LEDS>(leds, NUM_LEDS);
  led_strip_start_pattern();

  // init GPS GSM module
  sim808_init();
  playBoot();
}

uint32_t i = 0;
void loop()
{
  // check gps status
  checkGPS();

  // print gps data, if available

  if (sim808.available())
  {
    if (sim808.available())
    {
      SerialUSB.println(sim808.readStringUntil('\n'));
    }
  }

  // periodically send gps message to http server
  if (gps_msg_count % 30 == 0 && sim808.GPRSstate())
  {
    SerialUSB.println("Forwarding GPS data to remote server");
    sim808.enableGPSNMEA(0);
    send_http_get_request();
    sim808.enableGPSNMEA(1);
  }

  // Output received RC values
  SerialUSB.print("\n");
  SerialUSB.print("R,");

  for (int l = 0; l < 8; l++)
  {
    SerialUSB.print((int)ppmReader.get(l));
    SerialUSB.print(",");
    //Serial.print(testf[l]);Serial.print(", ");
  }
  SerialUSB.print("\n");

  // Process IR inputs
  if (myReceiver.getResults())
  {
    myDecoder.decode(); //Decode it
    //myDecoder.dumpResults(true);  //Now print results. Use false for less detail
    myReceiver.enableIRIn(); //Restart receiver
    //Serial.println(myDecoder.value);
    receivedValue(myDecoder.value);
  }

  // Process serial commands from host
  process_host_commands();
}

void receivedValue(uint32_t value)
{

  uint32_t deltaT = (millis() - lastIRTime);
  lastIRTime = millis();
  //Serial.println(deltaT);
  if (deltaT > 500)
  {
    switch (value)
    {
    case 0xc0: //u
      doBeep(2200);
      SerialUSB.write("u\n");
      switch_relay(1);
      break;
    case 0xc1: //d
      doBeep(1800);
      SerialUSB.write("d\n");
      switch_relay(0);
      break;
    case 0xc2: //l
      doBeep(1900);
      SerialUSB.write("l\n");
      break;
    case 0xc3: //c
      doBeep(2000);
      SerialUSB.write("c\n");
      break;
    case 0xc4: //r
      doBeep(2100);
      SerialUSB.write("r\n");
      break;
    }
    delay(200);
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
  mySender.send(SONY, code, 8);
  lastValueSent = code;
  myReceiver.enableIRIn(); // Re-enable receiver
}

void doBeep(int freq)
{
  tone(SPEAKER, freq, 100);
}

void process_host_commands()
{
  if (SerialUSB.available())
  {
    inputString = SerialUSB.readStringUntil('\n');
  }
}

void setColor(uint32_t c)
{
  // TODO
}

void flashColor(uint32_t c)
{
  // TODO
}

void setup_pins()
{
  pinMode(RC_IN, INPUT);

  digitalWrite(RELAY_CLK, HIGH);
  pinMode(RELAY_CLK, OUTPUT);
  digitalWrite(RELAY_D, HIGH);
  pinMode(RELAY_D, OUTPUT);

  pinMode(IR_RX, INPUT);
  pinMode(IR_TX, OUTPUT);

  pinMode(TOP_LEDS, OUTPUT);
  pinMode(INTERNAL_LEDS, OUTPUT);

  pinMode(SPEAKER, OUTPUT);
}

void sim808_init()
{
  Serial1.begin(115200);
  if (!sim808.begin_(Serial1))
  {
    SerialUSB.println(F("Couldn't find SIM808"));
  }
  delay(1000);

  // Print module IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = sim808.getIMEI(imei);
  if (imeiLen > 0)
  {
    SerialUSB.print(F("Module IMEI: "));
    SerialUSB.println(imei);
  }

  gsm_connect(10);
  gps_power_on(5);
}

void switch_sim808_power()
{
  pinMode(SIM808_POWER_ON, OUTPUT);
  digitalWrite(SIM808_POWER_ON, HIGH);
  delay(3000);
  digitalWrite(SIM808_POWER_ON, LOW);
  delay(1000);
}

bool gps_power_on(int n_retries)
{
  // turn gps on
  SerialUSB.print("Turning GPS on");
  int retries = n_retries;
  while (retries > 0)
  {
    SerialUSB.print(".");
    if (sim808.enableGPS(true))
    {
      SerialUSB.println(" OK");
      switch_all_leds(0x101000, 6, 9);
      break;
    }
    retries--;

    switch_all_leds(CRGB::DarkRed, 6, 9);
    delay(1000);
    switch_all_leds(CRGB::Black, 6, 9);
  }
  if (retries == 0)
  {
    SerialUSB.println(" FAIL. Retrying later...");
    return false;
  }
  return true;
}

bool gsm_connect(int n_retries)
{
  // set GPRS network settings
  sim808.setGPRSNetworkSettings(F("hologram"), F(""), F(""));

  // wait until network is up
  switch_all_leds(CRGB::Black, 0, 10);
  SerialUSB.print(F("Waiting for network registration "));
  uint8_t n = sim808.getNetworkStatus();
  int retries = n_retries;
  while (retries > 0)
  {
    SerialUSB.print(n);
    SerialUSB.print(".");
    if (n == 1 || n == 5)
    {
      SerialUSB.println(" OK");
      switch_all_leds(0x101000, 0, 3);
      break;
    }
    retries--;
    switch_all_leds(CRGB::DarkRed, 0, 3);
    delay(500);
    switch_all_leds(CRGB::Black, 0, 3);
    n = sim808.getNetworkStatus();
  }
  if (retries == 0)
  {
    SerialUSB.println(" FAIL. Retying later...");
    return false;
  }

  // wait until gprs is successfully enabled
  SerialUSB.print(F("Enabling GPRS"));
  retries = n_retries;
  sim808.enableGPRS(true);
  while (retries > 0)
  {
    SerialUSB.print(".");
    if (sim808.enableGPRS(true))
    {
      SerialUSB.println(" OK");
      switch_all_leds(0x101000, 3, 6);
      break;
    }
    retries--;
    switch_all_leds(CRGB::DarkRed, 3, 6);
    delay(500);
    switch_all_leds(CRGB::Black, 3, 6);
  }
  if (retries == 0)
  {
    SerialUSB.println(" FAIL. Retying later...");
    return false;
  }
  return true;
}

void checkGPS()
{
  if (millis() - last_msg_time > 999)
  {
    int8_t stat = sim808.GPSstatus();
    switch (stat)
    {
    case 0:
      gps_power_on(1);
      // enable gps forwarding on serial port
      sim808.enableGPSNMEA(1);
      break;
    case 1:
      last_msg_time = millis();
      gps_msg_count++;
      switch_all_leds(0x101000, 0, 10);
      sim808.enableGPSNMEA(1);
      break;
    case 2:
    case 3:
      // start forwarding
      // read gps
      if (!sim808.GPRSstate())
        gsm_connect(1);
      sim808.getGPS(0, gpsdata, 120);
      last_msg_time = millis();
      gps_msg_count++;
      process_gps_msg(gpsdata);
      switch_all_leds(CRGB::Green, 0, 10);
      sim808.enableGPSNMEA(1);
      break;
    default:
      break;
    }
  }
}

uint16_t GSMLoc()
{
  char replybuffer[250];
  uint16_t returncode;
  while (!sim808.getGSMLoc(&returncode, replybuffer, 250))
  {
    delay(100);
  }
  SerialUSB.print("Return code: ");
  SerialUSB.println(returncode);
  SerialUSB.println(replybuffer);
  return returncode;
}

void process_gps_msg(char *data)
{
  char *ptr = strtok(data, ":,");
  int i = 0;
  while (ptr != NULL)
  {
    switch (i)
    {
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

void send_http_get_request()
{
  uint16_t statuscode;
  uint16_t datalen = 0;
  String url = F("http://132.206.74.128:8000/gps_put");
  url += "?lat=";
  url += lat;
  url += "&lon=";
  url += lon;
  url += "&timestamp=";
  url += timestamp;
  if (!sim808.HTTP_GET_start((char *)url.c_str(), &statuscode, &datalen))
  {
    SerialUSB.print(F("GET request failed. Status: "));
    SerialUSB.println(statuscode);
  }
  SerialUSB.println(F("\n****\n"));
  while (datalen > 0)
  {
    while (sim808.available())
    {
      char c = sim808.read();
      SerialUSB.write(c);
      datalen--;
      if (!datalen)
        break;
    }
  }
  SerialUSB.println(F("\n****\n"));
  sim808.HTTP_POST_end();
}

void led_strip_start_pattern()
{
  for (int j = 0; j < 3; j++)
  {
    for (int i = 0; i < 3; i++)
    {
      leds[i] = 0x999999;
      FastLED.show();
      delay(10);
    }
    for (int i = 0; i < 10; i++)
    {
      leds[i] = CRGB::DarkRed;
      FastLED.show();
      delay(10);
    }
    for (int i = 0; i < 10; i++)
    {
      leds[i] = CRGB::DarkGreen;
      FastLED.show();
      delay(10);
    }
    for (int i = 0; i < 10; i++)
    {
      leds[i] = CRGB::DarkBlue;
      FastLED.show();
      delay(10);
    }
  }
}

void switch_all_leds(CRGB color, int i0, int n)
{
  for (int i = i0; i < n; i++)
  {
    leds[i] = color;
  }
  FastLED.show();
}

void leds_ok(CRGB color, int i0, int n)
{
  switch_all_leds(color, i0, n);
  delay(1000);
  switch_all_leds(CRGB::Black, i0, n);
  delay(1000);
  switch_all_leds(color, i0, n);
}

void setColors(uint8_t lr, uint8_t lg, uint8_t lb, uint8_t rr, uint8_t rg, uint8_t rb)
{ // uint8_t wait
  leds[0].red = lr;
  leds[0].green = lg;
  leds[0].blue = lb;
  leds[1].red = rr;
  leds[1].green = rg;
  leds[1].blue = rb;
  FastLED.show();
}

void switch_relay(int val)
{

  if (val)
  {
    //switch robot on
    digitalWrite(RELAY_D, LOW);
    digitalWrite(RELAY_CLK, LOW);
    digitalWrite(RELAY_CLK, HIGH);
  }
  else
  {
    //switch robot off
    digitalWrite(RELAY_D, HIGH);
    digitalWrite(RELAY_CLK, LOW);
    digitalWrite(RELAY_CLK, HIGH);
    switch_all_leds(CRGB::Black, 0, 10);
  }
}

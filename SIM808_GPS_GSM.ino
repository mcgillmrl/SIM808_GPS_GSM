#include<stdio.h>
#include<string.h>
#define DEBUG true
int pon=9;
//gps data;
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
  //while (!SerialUSB) {
  //  ; // wait for serial port to connect
  //}
  Serial1.begin(57600);
  delay(1000);
  reset();
  delay(1000);
  reset();
  SerialUSB.println("Waiting for 3 seconds to boot up");
  delay(3000);
  SerialUSB.println("Initializing GSM");
  gsm_connect();
  SerialUSB.println("Initializing GPS");
  gps_power_on();
  sendData("AT+CNTP?",3000,DEBUG);
}

void loop()
{
    // read GPS
    String response = sendData( "AT+CGNSINF",1000,DEBUG);
    process_gps_msg((char*) response.c_str());
    send_http_get_request();
}

void reset(){
  pinMode(pon, OUTPUT);
  digitalWrite(pon, HIGH);
  delay(3000);
  digitalWrite(pon, LOW);
  delay(1000); 
}

bool gps_power_on(){
     sendData( "AT+CGNSPWR=1",1000,DEBUG);
     sendData("AT+CGNSSEQ=RMC",1000,DEBUG);
}

bool gsm_connect(){
     SerialUSB.println("Checking network registration");
     sendData("AT+CREG?",3000,DEBUG);     
     SerialUSB.println("Attaching to GSM service");
     sendData("AT+CMGF=1",1000,DEBUG);
     sendData("AT+CGATT=1",1000,DEBUG);
     SerialUSB.println("Setting APN to hologram");
     sendData("AT+CSTT=hologram",3000,DEBUG);
     SerialUSB.println("Bring up wireless connection");
     sendData("AT+CIICR",3000,DEBUG);
     SerialUSB.println("Getting currently assigned IP");
     sendData("AT+CIFSR",3000,DEBUG);
     
}

void process_gps_msg(char* data){
  char *ptr = strtok(data, ":,");
  int i = 0;
  while(ptr != NULL){
    switch(i){
      case 2:
          stat = atoi(ptr);
      case 3:
          timestamp = atof(ptr);
      case 4:
          lat = atof(ptr);
      case 5:
          lon = atof(ptr);
      case 6:
          alt = atof(ptr);
      case 7:
          speed_over_ground = atof(ptr);
      case 8:
          course_over_ground = atof(ptr);
      case 9:
          fix_mode = atoi(ptr);
      case 10:
          hdop = atof(ptr);
      case 11:
          pdop = atof(ptr);
      case 12:
          vdop = atof(ptr);
      case 13:
          sat_in_view = atoi(ptr);
      case 14:
          sat_used = atoi(ptr);
      case 15:
          CN0_max = atoi(ptr);
    }
    i++;
    ptr = strtok(NULL, ":,");
  }
}

bool send_http_get_request(){
  sendData("AT+HTTPINIT",1000,DEBUG);
  sendData("AT+HTTPPARA=\"CID\",1",1000,DEBUG);
  sendData("AT+HTTPPARA=\"UA\",\"AQUA5\"",1000,DEBUG);
  // http request
  Serial1.print("AT+HTTPPARA=");
  Serial1.print("\"URL\",\"");
  Serial1.print("http://132.206.74.128:8000"); // URL of location server
  Serial1.print("?lat=");
  Serial1.print(lat);
  Serial1.print("&lon=");
  Serial1.print(lon);
  Serial1.print("&alt=");
  Serial1.print(alt);
  Serial1.print("&time=");
  Serial1.print(timestamp);
  sendData("\"",1000,DEBUG);
  sendData("AT+HTTPACTION=0",1000,DEBUG);
  sendData("AT+HTTPREAD",1000,DEBUG);
  sendData("AT+HTTPCLOSE",1000,DEBUG);
}


String sendData(String command, const int timeout, boolean debug)
{
    String response = "";    
    Serial1.println(command); 
    long int time = millis();   
    while( (time+timeout) > millis())
    {
      while(Serial1.available())
      {       
        char c = Serial1.read(); 
        response+=c;
      }  
    }    
    if(debug)
    {
      SerialUSB.print(response);
    }    
    return response;
}

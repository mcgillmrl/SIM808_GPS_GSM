#include "Adafruit_FONA.h"

class GenericSIM808 : public Adafruit_FONA
{
public:
  GenericSIM808(int8_t r) : Adafruit_FONA(r){};

  boolean begin_(Stream &port)
  {
    mySerial = &port;

    // first check if on
    int16_t timeout = 3000;
    bool device_on = false;
    while (timeout > 0)
    {
      while (mySerial->available())
        mySerial->read();
      device_on = sendCheckReply(F("AT"), ok_reply);
      if (device_on)
        break;
      while (mySerial->available())
        mySerial->read();
      device_on = sendCheckReply(F("AT"), F("AT"));
      if (device_on)
        break;
      delay(500);
      timeout -= 500;
    }

    pinMode(_rstpin, OUTPUT);
    if (device_on)
    {
      SerialUSB.println(F("Device is on, powering off"));
      digitalWrite(_rstpin, HIGH);
      delay(2000);
      digitalWrite(_rstpin, LOW);
      delay(1000);
    }
    SerialUSB.println(F("Powering device on"));
    digitalWrite(_rstpin, HIGH);
    delay(2000);
    digitalWrite(_rstpin, LOW);
    delay(1000);

    DEBUG_PRINTLN(F("Attempting to open comm with ATs"));
    // give 7 seconds to reboot
    timeout = 7000;

    while (timeout > 0)
    {
      while (mySerial->available())
        mySerial->read();
      if (sendCheckReply(F("AT"), ok_reply))
        break;
      while (mySerial->available())
        mySerial->read();
      if (sendCheckReply(F("AT"), F("AT")))
        break;
      delay(500);
      timeout -= 500;
    }

    if (timeout <= 0)
    {
#ifdef ADAFRUIT_FONA_DEBUG
      DEBUG_PRINTLN(F("Timeout: No response to AT... last ditch attempt."));
#endif
      sendCheckReply(F("AT"), ok_reply);
      delay(100);
      sendCheckReply(F("AT"), ok_reply);
      delay(100);
      sendCheckReply(F("AT"), ok_reply);
      delay(100);
    }

    // turn off Echo!
    sendCheckReply(F("ATE0"), ok_reply);
    delay(100);

    if (!sendCheckReply(F("ATE0"), ok_reply))
    {
      return false;
    }

    // set error messages to be verbose
    sendCheckReply(F("AT+CMEE=2"), ok_reply);

    // turn on hangupitude
    sendCheckReply(F("AT+CVHU=0"), ok_reply);

    delay(100);
    flushInput();

    DEBUG_PRINT(F("\t---> "));
    DEBUG_PRINTLN("ATI");

    mySerial->println("ATI");
    readline(500, true);

    DEBUG_PRINT(F("\t<--- "));
    DEBUG_PRINTLN(replybuffer);

    if (prog_char_strstr(replybuffer, (prog_char *)F("SIM808 R14")) != 0)
    {
      _type = FONA808_V2;
    }
    else if (prog_char_strstr(replybuffer, (prog_char *)F("SIM808 R13")) != 0)
    {
      _type = FONA808_V1;
    }
    else if (prog_char_strstr(replybuffer, (prog_char *)F("SIM800 R13")) != 0)
    {
      _type = FONA800L;
    }
    else if (prog_char_strstr(replybuffer, (prog_char *)F("SIMCOM_SIM5320A")) != 0)
    {
      _type = FONA3G_A;
    }
    else if (prog_char_strstr(replybuffer, (prog_char *)F("SIMCOM_SIM5320E")) != 0)
    {
      _type = FONA3G_E;
    }

    if (_type == FONA800L)
    {
      // determine if L or H

      DEBUG_PRINT(F("\t---> "));
      DEBUG_PRINTLN("AT+GMM");

      mySerial->println("AT+GMM");
      readline(500, true);

      DEBUG_PRINT(F("\t<--- "));
      DEBUG_PRINTLN(replybuffer);

      if (prog_char_strstr(replybuffer, (prog_char *)F("SIM800H")) != 0)
      {
        _type = FONA800H;
      }
    }

#if defined(FONA_PREF_SMS_STORAGE)
    sendCheckReply(F("AT+CPMS=" FONA_PREF_SMS_STORAGE "," FONA_PREF_SMS_STORAGE "," FONA_PREF_SMS_STORAGE), ok_reply);
#endif

    return true;
  };
};

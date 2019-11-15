# Code for the GPS GSM module based on the SIM808 chipset

# Installation
Get the [Arduino IDE](https://www.arduino.cc/en/main/software) and install the [Adafruit FONA library](https://learn.adafruit.com/adafruit-fona-mini-gsm-gprs-cellular-phone-module/arduino-test). This version assumes you'll be using the ATSAMD21 Cortex-M0 microcontroller (Arduino Zero or equivalent)

After installing the FONA library, you may try flashing your microcontroller.

If it hangs when running, it is possible that the debug prints are being sent to the serial port in use by the SIM808. To fix
this, you need to change the the [DebugStream definition](https://github.com/adafruit/Adafruit_FONA/blob/4a556e180fdeefb9518fac160a8abf915ae8c92c/includes/platform/FONAPlatStd.h#L49) to
    
    #define DebugStream		SerialUSB

if you want to see debug information being printed. Alternatively, you can just disable debug prints by removing the definition of [ADAFRUIT_FONA_DEBUG](https://github.com/adafruit/Adafruit_FONA/blob/4a556e180fdeefb9518fac160a8abf915ae8c92c/includes/FONAConfig.h#L31):

    //#define ADAFRUIT_FONA_DEBUG

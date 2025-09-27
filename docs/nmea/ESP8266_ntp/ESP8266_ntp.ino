#include <WiFiManager.h>
#include "NTPClient.h"

WiFiManager wifiManager;
WiFiUDP ntpUDP;

const unsigned long utcOffset = 0;
const unsigned long maxRetryDelay = 60000;

unsigned long attemptCounter = 0;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffset);

void setup() {
  Serial.begin(9600);
  wifiManager.setClass("invert");

  if (!wifiManager.autoConnect("Hodiki", "24diyclock24")) {
    Serial.println("wifiManager.autoConnect() failed.");
    ESP.restart();
  }

  timeClient.begin();
}

void loop() {
  if (!timeClient.forceUpdate() || !timeClient.isTimeSet()) {
    Serial.println("timeClient.forceUpdate() failed.");

    unsigned long delayValue = (++attemptCounter) * 1000;
    if (delayValue > maxRetryDelay) {
      delayValue = maxRetryDelay;
    }

    delay(delayValue);
    return;
  }

  attemptCounter = 0;

  if (timeClient.getSeconds() < 59) {
    bool addOneSecond = false;
    int msec = timeClient.getMilliseconds();
    // Wait until the beginning of the next second
    if (msec > 0 && msec < 1000) {
      unsigned long syncValue = 1000 - msec;
      delay(syncValue);
      addOneSecond = true;
      // String temp = "added second. msec=" + String(msec);
      // Serial.println(temp);
    }

    String msg = makeNMEAString(addOneSecond);
    Serial.println(msg);

    // The board will be powered off once the synchronization is complete
    delay(60 * 60 * 1000);
  }
}

String makeNMEAString(bool addOneSecond) {
  time_t epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime((time_t *)&epochTime);

  int day = ptm->tm_mday;
  int month = ptm->tm_mon + 1;
  int year = ptm->tm_year + 1900;

  String time = getFormattedTime(timeClient, addOneSecond);
  
  char date[7];
  snprintf(date, sizeof(date), "%02d%02d%02d", day, month, year % 100);

  /*
  https://docs.novatel.com/OEM7/Content/Logs/GPRMC.htm

  Recommended minimum specific GPS/Transit data
  eg1. $GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62
  eg2. $GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68

            225446       Time of fix 22:54:46 UTC
            A            Navigation receiver warning A = OK, V = warning
            4916.45,N    Latitude 49 deg. 16.45 min North
            12311.12,W   Longitude 123 deg. 11.12 min West
            000.5        Speed over ground, Knots
            054.7        Course Made Good, True
            191194       Date of fix  19 November 1994
            020.3,E      Magnetic variation 20.3 deg East
            *68          mandatory checksum
  */

  String msg = "GPRMC," + time + ".00,A,,,,,,," + date + ",,,";
  byte crc = calculateCRC(msg);

  char crcString[7];
  snprintf(crcString, sizeof(crcString), "%02X", crc);

  return "$" + msg + "*" + crcString;
}

String getFormattedTime(NTPClient timeClient, bool addOneSecond) {
  unsigned long rawTime = timeClient.getEpochTime();
  unsigned long hours = (rawTime % 86400L) / 3600;
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);

  unsigned long minutes = (rawTime % 3600) / 60;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);

  unsigned long seconds = rawTime % 60;
  if (addOneSecond) {
    seconds += 1;
  }
  String secondStr = seconds < 10 ? "0" + String(seconds) : String(seconds);

  return hoursStr + minuteStr + secondStr;
}

byte calculateCRC(String nmea) {
  byte result = 0;
  
  for (byte i = 0; i < nmea.length(); i++) {
    result = result ^ nmea[i];
  }

  return result;
}

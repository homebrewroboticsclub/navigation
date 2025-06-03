#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#include <QMC5883LCompass.h>

#define GPS_RX_PIN 5  // Connect GPS TX to this pin (Arduino RX)
#define GPS_TX_PIN 6  // Connect GPS RX to this pin (Arduino TX)

QMC5883LCompass compass;

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);  // RX, TX

TinyGPSPlus gps;


void setup() {
  Serial.begin(115200);   // USB Serial for data transmission to ROS
  gpsSerial.begin(9600);  // GPS speed (usually 9600)

  compass.init();
  compass.setCalibrationOffsets(-101.00, -446.00, 747.00);
  compass.setCalibrationScales(0.79, 0.95, 1.45);
}


void loop() {
  // Reading GPS from SoftwareSerial
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // If there is an update to GPS coordinates, send it
  if (gps.location.isUpdated()) {
    float lat = gps.location.lat();
    float lon = gps.location.lng();
    float alt = gps.altitude.meters();

    Serial.print("GPS,");
    Serial.print(lat, 6);
    Serial.print(",");
    Serial.print(lon, 6);
    Serial.print(",");
    Serial.print(alt, 2);
    Serial.println();
  }

  // Reading magnetometer
  int mx, my, mz, ma;
  compass.read();
  mx = compass.getX();
  my = compass.getY();
  mz = compass.getZ();
  ma = compass.getAzimuth();
  // Sending magnetometer data (in microteslas)
  Serial.print("MAG,");
  Serial.print(mx);
  Serial.print(",");
  Serial.print(my);
  Serial.print(",");
  Serial.print(mz);
  Serial.print(",");
  Serial.print(ma);
  Serial.println();

  delay(100);  // approximately 10 Hz
  
}

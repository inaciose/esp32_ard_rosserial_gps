#define ENABLE_GPS 

#include <Arduino.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "wifi.h"

IPAddress ip;

#define LED_BUILTIN 2
int led_time = 500;

#ifdef ENABLE_GPS
  #include <TinyGPS++.h>
  #define RXD2 16
  #define TXD2 17
  
  TinyGPSPlus gps;

  double gps_lat = 0;
  double gps_lng = 0;

  unsigned int gps_time = 0;
  double gps_speed = 0;
  double gps_altitude = 0;
  double gps_course = 0;
  unsigned int gps_satellites = 0;
  int gps_hdop = 0;
  int gps_pdop = 0;
  int gps_vdop = 0;
  int gps_status = 1;


  TinyGPSCustom pdop(gps, "GPGSA", 15); // $GPGSA sentence, 15th element
  TinyGPSCustom vdop(gps, "GPGSA", 17); // $GPGSA sentence, 17th element

  TinyGPSCustom totalGPGSVMessages(gps, "GPGSV", 1); // $GPGSV sentence, first element
  TinyGPSCustom messageNumber(gps, "GPGSV", 2);      // $GPGSV sentence, second element
  TinyGPSCustom satsInView(gps, "GPGSV", 3);         // $GPGSV sentence, third element
  TinyGPSCustom satNumber[4]; // to be initialized later
  TinyGPSCustom elevation[4];
  TinyGPSCustom azimuth[4];
  TinyGPSCustom snr[4];

  int GPGSVtotalMessages;
  int GPGSVcurrentMessage;

  static const int MAX_SATELLITES = 40;

  struct {
    bool active;
    int elevation;
    int azimuth;
    int snr;
  } sats[MAX_SATELLITES];

  // ROS GPSStatus MSG array storage
  int sat_used_prn[MAX_SATELLITES];
  int sat_prn[MAX_SATELLITES];
  int sat_elevation[MAX_SATELLITES];
  int sat_azimuth[MAX_SATELLITES];
  int sat_snr[MAX_SATELLITES];
  int sat_visible_count;

  // ROS published message flag
  int msg_published = 1;


#endif

// bof:ROS
#include <ros.h>

#ifdef ENABLE_GPS
  #include <gps_common/GPSFix.h>
  #include <gps_common/GPSStatus.h>
  char gps_frameid[] = "gps_frame";
#endif

// HARDCODED
// Set the rosserial socket server IP address
IPAddress server(192,168,1,60);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
// eof:ROS

#ifdef ENABLE_GPS
  // gps publisher

  gps_common::GPSStatus gps_status_msg;
  gps_common::GPSFix gps_msg;
  ros::Publisher gpsPublisher("gps", &gps_msg);

  #define GPS_PUB_TIMER 1000

  void gpsRosPub() {
    static long unsigned int gpsPubTimer = 0;

    //
    // PUBLISH GPS DATA (only on timer)
    //

    if(millis() >= gpsPubTimer) {
      gpsPubTimer = millis() + GPS_PUB_TIMER;

      gps_status_msg.header.stamp = nh.now();
      gps_status_msg.satellites_used = gps_satellites;
      if(gps_status != 2) {
        gps_status_msg.status = -1;
      } else {
        gps_status_msg.status = 0;
      }

      msg_published = 1;
      
      gps_status_msg.satellites_visible = sat_visible_count;
      
      // limit the satellite_visible array itens (due to buffer limitations)
      int sat_visible_count_tmp = sat_visible_count;
      if(sat_visible_count_tmp > 8) sat_visible_count_tmp = 8;

      // update satellite_visible count (array item)
      gps_status_msg.satellite_visible_prn_length = sat_visible_count_tmp;
      gps_status_msg.satellite_visible_z_length = sat_visible_count_tmp;
      gps_status_msg.satellite_visible_azimuth_length = sat_visible_count_tmp;
      gps_status_msg.satellite_visible_snr_length = sat_visible_count_tmp;

      gps_msg.header.stamp = nh.now();
      gps_msg.latitude = gps_lat;
      gps_msg.longitude = gps_lng;
      gps_msg.altitude = gps_altitude;
      gps_msg.track = gps_course;
      gps_msg.speed = gps_speed;
      gps_msg.time = gps_time;
      gps_msg.hdop = gps_hdop;
      gps_msg.vdop = gps_vdop;
      gps_msg.pdop = gps_pdop;

      gps_msg.status = gps_status_msg;

      gpsPublisher.publish( &gps_msg);
    }
  }

  void gpsRead() {
    //
    // READ GPS DATA
    //
    while (Serial2.available() > 0)
      if (gps.encode(Serial2.read())) {
        if (gps.location.isValid()) {
          gps_lat = gps.location.lat();         // double > f64
          gps_lng = gps.location.lng();         // double > f64
          gps_time = gps.time.value();          // u32 > f64
          gps_speed = gps.speed.mps();          // double > f64
          gps_altitude = gps.altitude.meters(); // double > f64
          gps_course = gps.course.deg();        // double > f64
          gps_satellites = gps.satellites.value(); // u32 >
          gps_hdop = gps.hdop.value();          // i32 > f64 

          Serial.print(gps_lat, 6);
          Serial.print(F(","));
          Serial.println(gps_lng, 6);

          gps_status = 2;

          gps_pdop = atoi(pdop.value());
          gps_vdop = atoi(vdop.value());

        } else {
          Serial.println(F("gps invalid, wait for sattelites"));
          gps_status = 1;
        }

        if (totalGPGSVMessages.isUpdated()) {
          for (int i=0; i<4; ++i) {
            int no = atoi(satNumber[i].value());
            // Serial.print(F("SatNumber is ")); Serial.println(no);
            if (no >= 1 && no <= MAX_SATELLITES) {
              sats[no-1].elevation = atoi(elevation[i].value());
              sats[no-1].azimuth = atoi(azimuth[i].value());
              sats[no-1].snr = atoi(snr[i].value());
              sats[no-1].active = true;
            }
          }
        }

        GPGSVtotalMessages = atoi(totalGPGSVMessages.value());
        GPGSVcurrentMessage = atoi(messageNumber.value());

        if (GPGSVtotalMessages == GPGSVcurrentMessage && (msg_published || !sat_visible_count)) {
          msg_published = 0;
          sat_visible_count = 0;
          for (int i=0; i<MAX_SATELLITES; ++i)
            if (sats[i].active) {
              sat_prn[sat_visible_count] = i+1;
              sat_elevation[sat_visible_count] = sats[i].elevation;
              sat_azimuth[sat_visible_count] = sats[i].azimuth;
              sat_snr[sat_visible_count] = sats[i].snr;
              sat_visible_count++;
            }

          for (int i=0; i<MAX_SATELLITES; ++i)
            sats[i].active = false;
        }

      }

    if (millis() > 5000 && gps.charsProcessed() < 10) {
      gps_status = 0;
      Serial.println(F("No GPS detected: check wiring."));
      //while(true);
    }
  }

  void gpsRosSetup() {
	  nh.advertise(gpsPublisher);
    
    // Initialize all the uninitialized TinyGPSCustom objects
    for (int i=0; i<4; ++i) {
      satNumber[i].begin(gps, "GPGSV", 4 + 4 * i); // offsets 4, 8, 12, 16
      elevation[i].begin(gps, "GPGSV", 5 + 4 * i); // offsets 5, 9, 13, 17
      azimuth[i].begin(  gps, "GPGSV", 6 + 4 * i); // offsets 6, 10, 14, 18
      snr[i].begin(      gps, "GPGSV", 7 + 4 * i); // offsets 7, 11, 15, 19
    }

    //
    // GPSStatus MSG default contents
    // http://docs.ros.org/en/api/gps_common/html/msg/GPSStatus.html
    //

    gps_status_msg.header.frame_id = gps_frameid;
    // # Satellites used in solution
    gps_status_msg.satellites_used = 0;
    // pointer to array alocated memory
    gps_status_msg.satellite_used_prn = sat_used_prn;
    // array len (this should be updated)
    gps_status_msg.satellite_used_prn_length = 0;

    // # Satellites visible
    gps_status_msg.satellites_visible = 0;
    // pointers to array alocated memory
    gps_status_msg.satellite_visible_prn = sat_prn;
    gps_status_msg.satellite_visible_z = sat_elevation;
    gps_status_msg.satellite_visible_azimuth = sat_azimuth;
    gps_status_msg.satellite_visible_snr = sat_snr;
    // arrays len (this should be updated)
    gps_status_msg.satellite_visible_prn_length = 0;
    gps_status_msg.satellite_visible_z_length = 0;
    gps_status_msg.satellite_visible_azimuth_length = 0;
    gps_status_msg.satellite_visible_snr_length = 0;    
    // # Measurement status
    gps_status_msg.status = -1;
    gps_status_msg.motion_source = 1;
    gps_status_msg.orientation_source = 0;
    gps_status_msg.position_source = 1;

    //
    // GPSFix MSG default contents
    // http://docs.ros.org/en/api/gps_common/html/msg/GPSFix.html
    //

    gps_msg.header.frame_id = gps_frameid;
    gps_msg.latitude = 0;
    gps_msg.longitude = 0;
    gps_msg.altitude = 0;
    gps_msg.track = 0;
    gps_msg.speed = 0;
    gps_msg.climb = 0;
    gps_msg.pitch = 0;
    gps_msg.roll = 0;
    gps_msg.dip = 0;
    gps_msg.time = 0;
    gps_msg.gdop = 0;
    gps_msg.pdop = 0;
    gps_msg.hdop = 0;
    gps_msg.vdop = 0;

    // ## Uncertainty of measurement, 95% confidence
    // default not set
  }
#endif

void blinkLedBuiltin() {
	static boolean ledstate = 0;
	static long unsigned int ledTimer = 0;
	if(millis() >= ledTimer) {
		ledTimer = millis() + led_time;
		ledstate = !ledstate;
		digitalWrite(LED_BUILTIN, ledstate);
	}
}

void setup() {

  // led config
  pinMode(LED_BUILTIN, OUTPUT);

  // serial config
  Serial.begin(115200);
  Serial.println("Booting");

  #ifdef ENABLE_GPS
    // serial link to GPS
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  #endif

  // WIFI

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
      delay(500);
      Serial.print(F("."));
  }

  ip = WiFi.localIP();

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);

    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(ip);

	// bof:ROS
  Serial.print("ROS IP address: ");
  Serial.println(server);

	// Set the connection to rosserial socket server
	nh.getHardware()->setConnection(server, serverPort);
	nh.initNode();

	// Another way to get IP
	Serial.print("ROS IP = ");
	Serial.println(nh.getHardware()->getLocalIP());

  #ifdef ENABLE_GPS
    gpsRosSetup();
  #endif
 
}

void loop() {
  ArduinoOTA.handle();

  #ifdef ENABLE_GPS
    gpsRead();
    gpsRosPub();
  #endif

  blinkLedBuiltin();

  if (WiFi.status() == WL_CONNECTED) {
    nh.spinOnce();
  }

}
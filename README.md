# GHG_Planter
This is the embedded device code for my ESET 420 capstone project.

## Main Hardware Components
 - Arduino Nano 33 IoT
 - 12V Peristaltic Pump x2
 - Adafruit HX8357/FT5336 LCD
 - Photoresistors x4
 - DHT11 Humidity Sensor
 - Capacitive Soil Moisture Sensors x2

## Main Software Functionality
 - Collect sensor data (light level, humidity, soil moisture)
 - Automatic Watering Mode (based on soil moisture)
 - Scheduled Watering Mode (based on schedule set by user)
 - Connect to WiFi for accurate time
 - MQTT communication for iOS app
 - Display sensor data on LCD
 - Change water settings with touchscreen/LCD

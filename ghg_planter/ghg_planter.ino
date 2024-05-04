#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <DHT11.h>
#include <Adafruit_HX8357.h>
#include <Adafruit_FT5336.h>

#define MQTT_USER "Arduino"
#define MQTT_PASS "Arduino"
#define MQTT_BROKER "a9faa51c.ala.dedicated.aws.emqxcloud.com"
#define MQTT_PORT 8883 // ssl/tls port == 8883
#define DATA_TOPIC "ghg/topic"
#define APP_TOPIC "ghg/topic2"
#define SETTINGS_TOPIC "ghg/topic3"

#define FT5336_MAXTOUCHES 1
Adafruit_FT5336 ctp = Adafruit_FT5336();
#define TFT_CS      14 //A0
#define TFT_RST     15 //A1
#define TFT_DC      16 //A2
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC);

// Pinout
#define SM1 A3
#define SM2 A6
#define HM 21 // A7
#define PR1 6
#define PR2 5
#define PR3 7
#define PR4 8
// #define PR5 7
// #define PR6 8
#define WP 2

/* GLOBALS */
WiFiSSLClient wifi;
MqttClient mqtt(wifi);
DHT11 dht11(HM);
static short humidity;
static short light[4]; // down to 4 from 6 now
static int moisture[2];

/* Approximate ADC values for moisture sensor (taken from https://lastminuteengineers.com/capacitive-soil-moisture-sensor-arduino)
 * Open Air: 590
 * Dry Soil: 380
 * Ideal Soil Moisture: 277 - 380
 * Soil just watered: 277
 * In cup of water: 273

 Upon further testing, above values were incorrect for our sensors with each value being higher in reality
*/
const int moisture_thresh = 600; // estimate value
const long flowrate = 126/60; // estimate value

#define MAX_CHARS 32
#define KEY_SZ 40
static unsigned long last_touch;
static uint8_t screen = 0;
static uint16_t c_x[33];
static uint16_t c_y;
static char ssid[MAX_CHARS+1];
static char pass[MAX_CHARS+1];
static char key_input[MAX_CHARS+1];
static uint8_t input_idx;
static enum KEY_STATE {
  NORMAL,
  SHIFT,
  SYMBOL
} key_state;
static enum INPUT_DATA {
  SSID,
  PASS
} typed_input;
static enum MODE {
  AUTO,
  SCHEDULED
} mode;
const char nums[] = "1234567890";
const uint16_t num_x1 = 40, num_x2 = (uint16_t)HX8357_TFTHEIGHT-num_x1;
const uint16_t num_y1 = 110, num_y2 = num_y1+KEY_SZ;
const char row1[] = "qwertyuiopQWERTYUIOP%\\|=[]<>{}";
const uint16_t row1_x1 = 40, row1_x2 = (uint16_t)HX8357_TFTHEIGHT-row1_x1;
const uint16_t row1_y1 = 150, row1_y2 = row1_y1+KEY_SZ;
const char row2[] = "asdfghjklASDFGHJKL@#$_&-+()";
const uint16_t row2_x1 = 60, row2_x2 = (uint16_t)HX8357_TFTHEIGHT-row2_x1;
const uint16_t row2_y1 = 190, row2_y2 = row2_y1+KEY_SZ;
const char row3[] = "zxcvbnmZXCVBNM*\"\':;!?";
const uint16_t row3_x1 = 100, row3_x2 = (uint16_t)HX8357_TFTHEIGHT-row3_x1;
const uint16_t row3_y1 = 230, row3_y2 = row3_y1+KEY_SZ;
const uint16_t row4_x1 = 100, row4_x2 = (uint16_t)HX8357_TFTHEIGHT-row4_x1;
const uint16_t row4_y1 = 270, row4_y2 = row4_y1+KEY_SZ;

static short schedule[7];
const char* const days[] = {"M", "T", "W", "Th", "F", "S", "Su"};
const char* const hours[] = { "012", "0123456789" }; // REMEMBER 2ND BOX CANNOT BE ABOVE 3 ONCE 1ST BOX IS 2
const char* const minutes[] = { "012345", "0123456789" };
static short count_h1, count_h2, count_m1, count_m2;
static char selected_time[4] = { hours[0][count_h1], hours[1][count_h2], minutes[0][count_m1], minutes[1][count_m2] };
static unsigned long mins_since;
static unsigned long scheduled_time;

static short selected_amount = 0;
const unsigned int dispense[] = { 236, 2*236, 3*236};

static unsigned long watered_at;
static unsigned int estimated_water = 5000;

const uint16_t word_offset = 15;
const uint16_t char_offset = 19;

static char assembled_data[8][10];
static char data_msg[100];
static char assembled_settings[10][4];
static char settings_msg[100];

static short initial_setup = 0;
static short save_settings = 0;

void setup() {
  // Serial init for testing
  Serial.begin(9600);
  //while(!Serial);

  // Pin setup
  pinMode(PR1, INPUT);
  pinMode(PR2, INPUT);
  pinMode(PR3, INPUT);
  pinMode(PR4, INPUT);
  // pinMode(PR5, INPUT);
  // pinMode(PR6, INPUT);
  pinMode(HM, INPUT);
  pinMode(WP, OUTPUT);
  
  // Init LCD
  init_LCD();
  
  // Settings screen & touch inputs
  initial_setup = 1;
}

void loop() {
  mqtt.poll(); // keep signal alive

  if (initial_setup){
    unsigned long poll_timer;
    screen = 1;
    displaySettings();
    initial_setup = 0;
    while (!save_settings){
      monitor_LCD();
      if (millis() - poll_timer > 100){ // 100 ms
        mqtt.poll();
        poll_timer = millis();
      }
    }
  }

  // update readings, display, and send mqtt
  static unsigned long send_timer;
  if (millis() - send_timer >= 10000){ // 10 seconds
    read_sensors();
    displayData();
      // format data
    package_data();
    // format settings
    package_settings();

    if (mqtt.connected()){ // only attempt send if mqtt connected
      //send data
      sendMessage(DATA_TOPIC, data_msg);
      //send settings
      sendMessage(SETTINGS_TOPIC, settings_msg);
    }
    send_timer = millis();
  }
  
  if (screen == 1){
    while (!save_settings){
      unsigned long poll_timer;
      monitor_LCD();
      if (millis() - poll_timer >= 100){ // 100 ms
        mqtt.poll();
        poll_timer = millis();
      }
    }
  }
  else{
    unsigned long poll_timer;
    if (millis() - poll_timer >= 100){ // 100 ms
      mqtt.poll();
      poll_timer = millis();
    }
    monitor_LCD();
  }
  
  unsigned long start;
  if (millis() - start >= 60000){ // only check water status every min
    mqtt.poll();

    // need to ensure wifi time works
    short tries;
    for (tries = 5; tries > 0; tries--){
      if (WiFi.getTime()) break;
      delay(300);
    }

    if (tries){ // skip time checks if can't contact time server
      if (save_settings){
        // check if needs watering switch
        if (mode == AUTO){
          if (moisture[0] > moisture_thresh || moisture[1] > moisture_thresh){
            if (watered_at != 0){
              if (WiFi.getTime() - watered_at >= 36*60*60){ // over 36 hours
                pump_water(dispense[selected_amount]);
                // take new readings
                read_sensors();
              }
            }
            else{
              pump_water(dispense[selected_amount]);
              // take new readings
              read_sensors();
            }
          }
        }
        // IF I WANT TO DO CST TIME INSTEAD OF UNIX, I MUST SUBTRACT 5 HOURS FROM EPOCH
        else if (mode == SCHEDULED){
          // day of week == (floor(epoch/86400) + 3) % 7 with 0=Monday, 1=Tuesday, etc
          if (schedule[(((WiFi.getTime()-(5*3600)) / 86400) + 3) % 7]){ // day of week
            // minutes since midnight == floor(epoch/3600) % 24 * 60 + floor(epoch/60) % 60
            mins_since = ((((WiFi.getTime()-(5*3600)) / 3600) % 24) * 60) + ((WiFi.getTime()-(5*3600)) / 60) % 60;
            // convert input time to minutes since midnight to compare
            if (mins_since == scheduled_time){
              pump_water(dispense[selected_amount]);
              // take new readings
              read_sensors();
            }
          }
        }
        start = millis();
      }
    }
  }
}

void wifi_init(char* ssid, char* pass){
  if (strlen(ssid) < 1){
    Serial.println("Enter your WiFi SSID: ");
    while (Serial.available() == 0);
    Serial.readStringUntil('\n').toCharArray(ssid, MAX_CHARS+1);
    Serial.print(ssid);
    Serial.println("Enter your WiFi password: ");
    while (Serial.available() == 0);
    Serial.readStringUntil('\n').toCharArray(pass, MAX_CHARS+1);
    Serial.print(pass);  
  }
  Serial.print("Attempting to connect to WPA SSID: "); Serial.println(ssid);

  short tries;
  for (tries = 5; tries > 0; tries--){
    if (WiFi.begin(ssid, pass) != WL_CONNECTED){
      Serial.print("."); // failed, retry
      delay(1000);
    }
    else{
      Serial.println();
      Serial.println("You're connected to the network"); Serial.println();
      break;
    }
  }
  if (! tries){
    Serial.println();
    Serial.println("Unable to connect to network.");
  }
}

void mqtt_init(){
  mqtt.setId("smart_planter");
  mqtt.setUsernamePassword(MQTT_USER, MQTT_PASS);
  
  Serial.println("Attempting to connect to the MQTT broker: "); Serial.println(MQTT_BROKER);

  if (! mqtt.connect(MQTT_BROKER, MQTT_PORT)) {
    Serial.print("MQTT connection failed! Error Code = "); Serial.println(mqtt.connectError());
    delay(200);
  }
  else{
    Serial.println("You're connected to the MQTT broker!");
    mqtt.onMessage(onMessage);
    mqtt.subscribe(APP_TOPIC, 0);
  }
}

char* package_data(){
  // convert to string
  light[0] ? strcpy(assembled_data[0], "L") : strcpy(assembled_data[0], "D");
  light[1] ? strcpy(assembled_data[1], "L") : strcpy(assembled_data[1], "D");
  light[2] ? strcpy(assembled_data[2], "L") : strcpy(assembled_data[2], "D");
  light[3] ? strcpy(assembled_data[3], "L") : strcpy(assembled_data[3], "D");
  moisture[0] < moisture_thresh ? strcpy(assembled_data[4], "W") : strcpy(assembled_data[4], "D");
  moisture[1] < moisture_thresh ? strcpy(assembled_data[5], "W") : strcpy(assembled_data[5], "D");
  itoa(humidity, assembled_data[6], 10);
  //ultoa(watered_at, assembled_data[7], 10);

  // combine string
  data_msg[0] = '\0';
  strcat(data_msg, assembled_data[0]); strcat(data_msg, ",");
  strcat(data_msg, assembled_data[1]); strcat(data_msg, ",");
  strcat(data_msg, assembled_data[2]); strcat(data_msg, ",");
  strcat(data_msg, assembled_data[3]); strcat(data_msg, ",");
  strcat(data_msg, assembled_data[4]); strcat(data_msg, ",");
  strcat(data_msg, assembled_data[5]); strcat(data_msg, ",");
  strcat(data_msg, assembled_data[6]); //strcat(data_msg, ",");
  //strcat(data_msg, assembled_data[7]);
}

char* package_settings(){
  // convert to string
  itoa(selected_amount, assembled_settings[0], 10);
  itoa(mode, assembled_settings[1], 10);
  itoa(schedule[0], assembled_settings[2], 10);
  itoa(schedule[1], assembled_settings[3], 10);
  itoa(schedule[2], assembled_settings[4], 10);
  itoa(schedule[3], assembled_settings[5], 10);
  itoa(schedule[4], assembled_settings[6], 10);
  itoa(schedule[5], assembled_settings[7], 10);
  itoa(schedule[6], assembled_settings[8], 10);
  itoa(scheduled_time, assembled_settings[9], 10);

  // combine string
  settings_msg[0] = '\0';
  strcat(settings_msg, assembled_settings[0]); strcat(settings_msg, ",");
  strcat(settings_msg, assembled_settings[1]); strcat(settings_msg, ",");
  strcat(settings_msg, assembled_settings[2]); strcat(settings_msg, ",");
  strcat(settings_msg, assembled_settings[3]); strcat(settings_msg, ",");
  strcat(settings_msg, assembled_settings[4]); strcat(settings_msg, ",");
  strcat(settings_msg, assembled_settings[5]); strcat(settings_msg, ",");
  strcat(settings_msg, assembled_settings[6]); strcat(settings_msg, ",");
  strcat(settings_msg, assembled_settings[7]); strcat(settings_msg, ",");
  strcat(settings_msg, assembled_settings[8]); strcat(settings_msg, ",");
  strcat(settings_msg, assembled_settings[9]);
}

void unpackage_settings(char* settings){
  int values[10];

  char* token = strtok(settings, ",");
  for (int i = 0; i<10; i++){
    if (token != NULL){
      values[i] = atoi(token);
      token = strtok(NULL, ",");
    }
  }

  selected_amount = values[0];
  mode = (MODE)values[1];
  for (int i = 2; i<9; i++){
    schedule[i-2] = values[i];
  }
  
  selected_time[0] = hours[0][values[9] / 60 / 10];
  selected_time[1] = hours[1][values[9] / 60 % 10];
  selected_time[2] = minutes[0][values[9] % 60 / 10];
  selected_time[3] = minutes[1][values[9] % 60 % 10];
}

void sendMessage(String topic, String message){
    Serial.print("Sending msg to topic: "); Serial.println(topic);
    Serial.print(message);

    mqtt.beginMessage(topic);
    mqtt.print(message);
    mqtt.endMessage();

    Serial.println();
}

void onMessage(int messageSize){
  char message[100];
  char repeat[100];
  const char* topic = mqtt.messageTopic().c_str();
  Serial.print("Received a message on topic '");
  Serial.print(topic);
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  int count = 0;
  while (mqtt.available()) {
    if (count < 50){
      message[count] = (char)mqtt.read();
      count++;
    }
  }
  message[count] = '\0';
  
  Serial.println(message);
  Serial.println();

  // copy message but without null terminator to send to app
  char* ptr = message;
  char* ptr2 = repeat;
  while(*ptr != '\0'){
    *ptr2++ = *ptr++;
  }

  if (! strcmp(topic, APP_TOPIC)){
    unpackage_settings(message);
    if (mqtt.connected()){
      sendMessage(SETTINGS_TOPIC, repeat);
    }
  }
}

void read_sensors(){
  moisture[0] = analogRead(SM1);
  moisture[1] = analogRead(SM2);

  humidity = dht11.readHumidity();
  if (humidity != DHT11::ERROR_CHECKSUM && humidity != DHT11::ERROR_TIMEOUT);
  else humidity = -1;
  //Serial.println(DHT11::getErrorString(humidity));

  light[0] = digitalRead(PR1);
  light[1] = digitalRead(PR2);
  light[2] = digitalRead(PR3);
  light[3] = digitalRead(PR4);
  // light[4] = digitalRead(PR5);
  // light[5] = digitalRead(PR6);
}

void pump_water(unsigned long int amount){
  tft.fillScreen(HX8357_BLACK);
  tft.setTextSize(4);
  tft.setCursor(0, 0);
  tft.print("WATERING...");
  tft.setTextSize(2);

  digitalWrite(WP, HIGH);
  unsigned long start = millis();
  while (millis() - start < amount/flowrate*1000){
    unsigned long poll_timer;
    if (millis() - poll_timer >= 100){
      mqtt.poll();
      poll_timer = millis();
    }
  }
  digitalWrite(WP, LOW);
  estimated_water -= amount;

  short tries;
  for (tries = 5; tries > 0; tries--){
    if (WiFi.getTime()) break;
    delay(300);
  }
  if (tries) watered_at = WiFi.getTime();
}

void init_LCD(){
  tft.begin();
  tft.setRotation(3);

  if (! ctp.begin(FT53XX_DEFAULT_ADDR, &Wire)) {
    Serial.println("Couldn't start FT5336 touchscreen controller");
    while (1) delay(10);
  }

  Serial.println("Capacitive touchscreen started");
  tft.fillScreen(HX8357_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(HX8357_WHITE);
}

void monitor_LCD(){
  uint8_t touches = ctp.touched();
  if (! touches){
    return;
  }
  if (millis() - last_touch < 200){
     return;
  }
  last_touch = millis();

  TS_Point ps[FT5336_MAXTOUCHES];
  ctp.getPoints(ps, FT5336_MAXTOUCHES);

  // for (int i=0; i<FT5336_MAXTOUCHES; i++) {
  //   if (ps[i].z == 0) continue;
  //   Serial.print("("); Serial.print(ps[i].x);
  //   Serial.print(", "); Serial.print(ps[i].y);
  //   Serial.print(")\t");
  // }
  // Serial.println();

  if (screen){
    if (ps[0].y < 80 && ps[0].x < 40){
      Serial.println("Exiting menu...");
      screen--;
      screen ? displaySettings() : (scheduled_time = (((int)selected_time[0]-'0')*10 + (int)selected_time[1]-'0') * 60 + (((int)selected_time[2]-'0')*10 + (int)selected_time[3]-'0'), save_settings=1, displayData());
      key_input[0] = '\0';
      input_idx = 0;
    }
    if (screen == 1){
      //wifi
      if (ps[0].y > HX8357_TFTHEIGHT-130-240 && ps[0].y < HX8357_TFTHEIGHT-130 && ps[0].x > 25 && ps[0].x < 65){
        Serial.println("Displaying Keyboard...");
        screen = 2;
        displayKeyboard();
        drawKeyboard();
        // set cursor for echo input
        tft.setCursor(52, 72);
        typed_input = SSID;
      }      
      //pass
      else if (ps[0].y > HX8357_TFTHEIGHT-130-240 && ps[0].y < HX8357_TFTHEIGHT-130 && ps[0].x > 75 && ps[0].x < 115){
        Serial.println("Displaying Keyboard...");
        screen = 2;
        displayKeyboard();
        drawKeyboard();
        // set cursor for echo input
        tft.setCursor(52, 72);
        typed_input = PASS;
      }
      
      //connect
      else if (ps[0].y > 10 && ps[0].y < 10+90 && ps[0].x > 75 && ps[0].x < 115){
        Serial.println("Connecting...");
        tft.drawRect(130, 25, 240, 40, HX8357_YELLOW);
        wifi_init(ssid, pass);
        if (WiFi.status() == WL_CONNECTED){
          tft.drawRect(130, 25, 240, 40, HX8357_GREEN);
          mqtt_init();
        }
        else{
          tft.drawRect(130, 25, 240, 40, HX8357_RED);
        }
      }
      
      //dispense
      else if (ps[0].x > 148-10 && ps[0].x < 148+10){
        if (ps[0].y > 10+88+88 && ps[0].y < 10+88+88+88){
          tft.fillCircle(219+20+48+5+15, 140+8, 8, HX8357_BLACK);
          tft.fillCircle(219+20+48+5+15+20+48+5+15, 140+8, 8, HX8357_BLACK);
          tft.fillCircle(219, 140+8, 6, HX8357_WHITE);
          Serial.println("Set amount to 1 cup.");
          selected_amount = 0;
        }
        else if (ps[0].y > 10+88 && ps[0].y < 10+88+88){
          tft.fillCircle(219, 140+8, 8, HX8357_BLACK);
          tft.fillCircle(219+20+48+5+15+20+48+5+15, 140+8, 8, HX8357_BLACK);
          tft.fillCircle(219+20+48+5+15, 140+8, 6, HX8357_WHITE);
          Serial.println("Set amount to 2 cups.");
          selected_amount = 1;
        }
        else if (ps[0].y > 10 && ps[0].y < 10+88){
          tft.fillCircle(219, 140+8, 8, HX8357_BLACK);
          tft.fillCircle(219+20+48+5+15, 140+8, 8, HX8357_BLACK);
          tft.fillCircle(219+20+48+5+15+20+48+5+15, 140+8, 6, HX8357_WHITE);
          Serial.println("Set amount to 3 cups.");
          selected_amount = 2;
        }
      }

      //mode
      else if (ps[0].x > 178-10 && ps[0].x < 178+10){
        if (ps[0].y > 80 && ps[0].y < 80+96+40){
          tft.fillCircle(195, 170+8, 8, HX8357_BLACK);
          tft.fillCircle(195+20+48+15, 170+8, 6, HX8357_WHITE); // DEFAULT
          Serial.println("Set mode to SCHEDULED.");
          mode = SCHEDULED;
        }
        else if (ps[0].y > 80+96+40+5 && ps[0].y < 80+96+40+5+48+40){
          tft.fillCircle(195+20+48+15, 170+8, 8, HX8357_BLACK);
          tft.fillCircle(195, 170+8, 6, HX8357_WHITE); // DEFAULT
          Serial.println("Set mode to AUTO.");
          mode = AUTO;
        }
      }

      //days
      else if (ps[0].x > 220-15 && ps[0].x < 220-15+40){
        int idx;
        if (ps[0].y > 5 && ps[0].y < 280+70+5){
          idx = (280+70+5-ps[0].y)/(40+10);
          schedule[idx] ^= 1;
          Serial.print("Toggled \"");
          Serial.print(days[idx]);
          Serial.println("\".");
          if (schedule[idx]){
            for (int i = 2; i < 7; i++){
             tft.drawRect(130+50*idx+i, 220-15+i, 40-2*i, 40-2*i, HX8357_BLUE);
            }
          }
          else{
            for (int i = 2; i < 7; i++){
             tft.drawRect(130+50*idx+i, 220-15+i, 40-2*i, 40-2*i, HX8357_BLACK);
            }
          }
        }                                      
      }

      //time
      else if (ps[0].x > 270-15 && ps[0].x < 270-15+40){
        if (ps[0].y > 280+70-40 && ps[0].y < 280+70){
          tft.setCursor(72+48+10+15, 270);
          tft.fillRect(tft.getCursorX(), tft.getCursorY(), 15, 15, HX8357_BLACK);
          count_h1 < strlen(hours[0])-1 ? count_h1++ : count_h1=0;
          tft.print(hours[0][count_h1]);
          selected_time[0] = hours[0][count_h1];
          Serial.print("Selected time set to \"");
          Serial.print(selected_time[0]);
          Serial.print(selected_time[1]);
          Serial.print(':');
          Serial.print(selected_time[2]);
          Serial.print(selected_time[3]);
          Serial.print("\".");
        }
        else if (ps[0].y > 280+70-40-10-40 && ps[0].y < 280+70-40-10){
          tft.setCursor(72+48+10+50+15, 270);
          tft.fillRect(tft.getCursorX(), tft.getCursorY(), 15, 15, HX8357_BLACK);
          if (count_h1 == 2){ // only go up to 23:59
            count_h2 < strlen(hours[1])-7 ? count_h2++ : count_h2=0;  
          }
          else{
            count_h2 < strlen(hours[1])-1 ? count_h2++ : count_h2=0;
          }
          tft.print(hours[1][count_h2]);
          selected_time[1] = hours[1][count_h2];
          Serial.print("Selected time set to \"");
          Serial.print(selected_time[0]);
          Serial.print(selected_time[1]);
          Serial.print(':');
          Serial.print(selected_time[2]);
          Serial.print(selected_time[3]);
          Serial.print("\".");          
        }
        else if (ps[0].y > 280+70-40-10-40-10-20-40 && ps[0].y < 280+70-40-10-40-10-20){
          tft.setCursor(72+48+10+50+70+15, 270);
          tft.fillRect(tft.getCursorX(), tft.getCursorY(), 15, 15, HX8357_BLACK);
          count_m1 < strlen(minutes[0])-1 ? count_m1++ : count_m1=0;
          tft.print(minutes[0][count_m1]);
          selected_time[2] = minutes[0][count_m1];
          Serial.print("Selected time set to \"");
          Serial.print(selected_time[0]);
          Serial.print(selected_time[1]);
          Serial.print(':');
          Serial.print(selected_time[2]);
          Serial.print(selected_time[3]);
          Serial.print("\".");        
        }
        else if (ps[0].y > 80+70-40-10-40-10-20-40-10-40 && ps[0].y < 280+70-40-10-40-10-20-40-10){
          tft.setCursor(72+48+10+50+70+50+15, 270);
          tft.fillRect(tft.getCursorX(), tft.getCursorY(), 15, 15, HX8357_BLACK);
          count_m2 < strlen(minutes[1])-1 ? count_m2++ : count_m2=0;
          tft.print(minutes[1][count_m2]);
          selected_time[3] = minutes[1][count_m2];
          Serial.print("Selected time set to \"");
          Serial.print(selected_time[0]);
          Serial.print(selected_time[1]);
          Serial.print(':');
          Serial.print(selected_time[2]);
          Serial.print(selected_time[3]);
          Serial.print("\".");          
        }      
      }
    }

    if (screen == 2){
      // keyboard inputs
      if (input_idx < MAX_CHARS){
        char c;
        if (ps[0].x > num_y1 && ps[0].x < 150){ // number row
          if (ps[0].y < HX8357_TFTHEIGHT-40 && ps[0].y > 40){
            c = nums[(HX8357_TFTHEIGHT-40-ps[0].y)/KEY_SZ];
            key_input[input_idx++] = c;
            key_input[input_idx] = '\0';
            c_x[input_idx-1] = tft.getCursorX(); c_y = tft.getCursorY();
            tft.print(c);
          }
        }
        else if (ps[0].x > row1_y1 && ps[0].x < row1_y2){ // row 1 letters
          if (ps[0].y < HX8357_TFTHEIGHT-40 && ps[0].y > 40){
            c = row1[(HX8357_TFTHEIGHT-40-ps[0].y)/KEY_SZ + 10*key_state];
            key_input[input_idx++] = c;
            key_input[input_idx] = '\0';
            c_x[input_idx-1] = tft.getCursorX(); c_y = tft.getCursorY();
            tft.print(c);
          }
        }
        else if (ps[0].x > row2_y1 && ps[0].x < row2_y2){ // row 2 letters
          if (ps[0].y < HX8357_TFTHEIGHT-40-20 && ps[0].y > 40+20){
            c = row2[(HX8357_TFTHEIGHT-40-20-ps[0].y)/KEY_SZ + 9*key_state];
            key_input[input_idx++] = c;
            key_input[input_idx] = '\0';
            c_x[input_idx-1] = tft.getCursorX(); c_y = tft.getCursorY();
            tft.print(c);
          }
        }
        else if (ps[0].x > row3_y1 && ps[0].x < row3_y2){ // row 3 letters
          if (ps[0].y < HX8357_TFTHEIGHT-40-40-20 && ps[0].y > 40+40+20){
            c = row3[(HX8357_TFTHEIGHT-40-40-20-ps[0].y)/KEY_SZ + 7*key_state];
            key_input[input_idx++] = c;
            key_input[input_idx] = '\0';
            c_x[input_idx-1] = tft.getCursorX(); c_y = tft.getCursorY();
            tft.print(c);
          }
        }
        else if (ps[0].x > row4_y1){ // row 4
          if (ps[0].y < HX8357_TFTHEIGHT-100 && ps[0].y > HX8357_TFTHEIGHT-100-40){
            c = ',';
            key_input[input_idx++] = c;
            key_input[input_idx] = '\0';
            c_x[input_idx-1] = tft.getCursorX(); c_y = tft.getCursorY();
            tft.print(c);
          }
          else if (ps[0].y < HX8357_TFTHEIGHT-100-40 && ps[0].y > HX8357_TFTHEIGHT-100-40-200){
            c = ' ';
            key_input[input_idx++] = c;
            key_input[input_idx] = '\0';
            c_x[input_idx-1] = tft.getCursorX(); c_y = tft.getCursorY();
            tft.print(c);                
          }
          else if (ps[0].y < HX8357_TFTHEIGHT-100-40-200 && ps[0].y > HX8357_TFTHEIGHT-100-40-200-40){
            c = '.';
            key_input[input_idx++] = c;
            key_input[input_idx] = '\0';
            c_x[input_idx-1] = tft.getCursorX(); c_y = tft.getCursorY();
            tft.print(c);
          }
        }
      }
      // special keys (shift, symbols, enter, backspace)
      if (ps[0].x > row2_y1 && ps[0].x < row2_y2){ // row 2
        if (ps[0].y > HX8357_TFTHEIGHT-row2_x1 && ps[0].y < HX8357_TFTHEIGHT-row2_x1+KEY_SZ){ // shift
          key_state = key_state==SHIFT ? NORMAL : SHIFT;
          c_x[input_idx] = tft.getCursorX(); c_y = tft.getCursorY();
          drawKeyboard();
          tft.setCursor(c_x[input_idx], c_y);
        }
        else if (ps[0].y > HX8357_TFTHEIGHT-row2_x2-KEY_SZ && ps[0].y < HX8357_TFTHEIGHT-row2_x2){ // backspace
          if (input_idx){
            tft.setCursor(c_x[input_idx-1], c_y);
            tft.setTextColor(HX8357_BLACK);
            tft.print(key_input[input_idx-1]);
            //Serial.println(key_input);
            key_input[input_idx-1] = '\0';
            tft.setCursor(c_x[input_idx-1], c_y);
            tft.setTextColor(HX8357_WHITE);
            input_idx--;
          }
        }
      }
      else if (ps[0].x > row3_y1 && ps[0].x < row3_y2){ // row 3
        if (ps[0].y > HX8357_TFTHEIGHT-row3_x1 && ps[0].y < HX8357_TFTHEIGHT-row3_x1+KEY_SZ+20){ // symbol
          key_state = key_state==SYMBOL ? NORMAL : SYMBOL;
          c_x[input_idx] = tft.getCursorX(); c_y = tft.getCursorY();
          drawKeyboard();
          tft.setCursor(c_x[input_idx], c_y);
        }
        else if (ps[0].y > HX8357_TFTHEIGHT-row3_x2-60 && ps[0].y < HX8357_TFTHEIGHT-row3_x2){ // enter
          Serial.println("Exiting menu...");
          screen--;
          input_idx = 0;
          switch(typed_input){
            case SSID:
              Serial.println("WiFi SSID input stored.");
              strcpy(ssid, key_input);
              break;
            case PASS:
              Serial.println("Password input stored.");
              strcpy(pass, key_input);
              break;
            default:
              Serial.println("Keyboard input not stored.");
              break;
          }
          key_input[0] = '\0';
          displaySettings();
        }
      }
    }
  }
  else{
    if (ps[0].y > 10 && ps[0].y < 120 && ps[0].x > 10 && ps[0].x < 80) {
        Serial.println("Displaying Settings...");
        screen = 1;
        displaySettings();
    }
  }
}

void displayData(){
  tft.fillScreen(HX8357_BLACK);
  //wifi status
  tft.setCursor(20, 20);
  tft.setTextColor(HX8357_WHITE);
  tft.setTextSize(2);
  tft.print("WiFi Status: ");
  WiFi.status() == WL_CONNECTED ? tft.print("Connected") : tft.print("Disconnected");
  //settings button
  tft.fillRect(HX8357_TFTHEIGHT-120-10, 10, 120, 80, HX8357_BLUE);
  tft.setCursor(480-60-45-10, 40);
  tft.setTextColor(HX8357_WHITE);
  tft.println("Settings");
  //refill warning
  if (estimated_water <= 1000){
    tft.fillRect(HX8357_TFTHEIGHT-100, HX8357_TFTWIDTH-100, 90, 90, HX8357_RED);
    tft.setCursor(HX8357_TFTHEIGHT-88, HX8357_TFTWIDTH-70);
    tft.print("REFILL");
    tft.setCursor(tft.getCursorX()-60, tft.getCursorY()+20);
    tft.print("TANK");
  }
  //light level
  tft.fillCircle(40, 140, 10, HX8357_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(40+20, 133);
  tft.print("Light Level: ");
  tft.setCursor(tft.getCursorX()+5, tft.getCursorY()-25);
  tft.print("BL");
  tft.setCursor(tft.getCursorX()-24, tft.getCursorY()+20);
  tft.drawRect(tft.getCursorX(), tft.getCursorY(), 24, 24, HX8357_WHITE);
  if (light[0]){
    tft.fillRect(tft.getCursorX()+1, tft.getCursorY()+1, 22, 22, HX8357_YELLOW);
  }
  else{
    tft.fillRect(tft.getCursorX()+1, tft.getCursorY()+1, 22, 22, HX8357_BLACK);
  }
  tft.setCursor(tft.getCursorX()+30, tft.getCursorY()-20);
  tft.print("BR");
  tft.setCursor(tft.getCursorX()-24, tft.getCursorY()+20);
  tft.drawRect(tft.getCursorX(), tft.getCursorY(), 24, 24, HX8357_WHITE);
  if (light[1]){
    tft.fillRect(tft.getCursorX()+1, tft.getCursorY()+1, 22, 22, HX8357_YELLOW);
  }
  else{
    tft.fillRect(tft.getCursorX()+1, tft.getCursorY()+1, 22, 22, HX8357_BLACK);
  }
  tft.setCursor(tft.getCursorX()+30, tft.getCursorY()-20);
  tft.print("FL");
  tft.setCursor(tft.getCursorX()-24, tft.getCursorY()+20);
  tft.drawRect(tft.getCursorX(), tft.getCursorY(), 24, 24, HX8357_WHITE);
  if (light[2]){
    tft.fillRect(tft.getCursorX()+1, tft.getCursorY()+1, 22, 22, HX8357_YELLOW);
  }
  else{
    tft.fillRect(tft.getCursorX()+1, tft.getCursorY()+1, 22, 22, HX8357_BLACK);
  }
  tft.setCursor(tft.getCursorX()+30, tft.getCursorY()-20);
  tft.print("FR");
  tft.setCursor(tft.getCursorX()-24, tft.getCursorY()+20);
  tft.drawRect(tft.getCursorX(), tft.getCursorY(), 24, 24, HX8357_WHITE);
  if (light[3]){
    tft.fillRect(tft.getCursorX()+1, tft.getCursorY()+1, 22, 22, HX8357_YELLOW);
  }
  else{
    tft.fillRect(tft.getCursorX()+1, tft.getCursorY()+1, 22, 22, HX8357_BLACK);
  }

  //humidity
  tft.fillCircle(40, 140+40, 10, HX8357_BLUE);
  tft.setCursor(40+20, 133+40);
  tft.print("Humidity: "); // humidity variable
  tft.print(humidity);
  tft.print("%");

  //soil moisture
  tft.fillCircle(40, 140+80, 10, HX8357_MAGENTA);
  tft.setCursor(40+20, 133+80);
  tft.print("Soil Moisture: "); // soil moisture variable
  tft.setCursor(tft.getCursorX()+10+5, tft.getCursorY()-20);
  tft.print("L");
  tft.setCursor(tft.getCursorX()-24, tft.getCursorY()+20);
  moisture[0] < moisture_thresh ? tft.print("WET") : tft.print("DRY");
  tft.setCursor(tft.getCursorX()+30, tft.getCursorY()-20);
  tft.print("R");
  tft.setCursor(tft.getCursorX()-24, tft.getCursorY()+20);
  moisture[1] < moisture_thresh ? tft.print("WET") : tft.print("DRY");
}

void displaySettings(){
  tft.fillScreen(HX8357_BLACK);
  save_settings = 0;

  //exit button
  tft.fillRect(HX8357_TFTHEIGHT-80, 0, 80, 40, HX8357_BLUE);
  tft.setCursor(HX8357_TFTHEIGHT-80+19, 13);
  tft.print("EXIT");

  //wifi
  tft.setCursor(10, 40);
  tft.print("WiFi: ");
  tft.drawRect(tft.getCursorX()+48, tft.getCursorY()-15, 240, 40, HX8357_WHITE);
  tft.setCursor(tft.getCursorX()+48+10, 40);
  for (int i = 0; i < strlen(ssid) && i < 15; i++){
    tft.print(ssid[i]);
  }
  if (strlen(ssid) > 15){
    tft.print('.');
    tft.print('.');
    tft.print('.');
  }
  //pass
  tft.setCursor(10, 90);
  tft.print("Password: ");
  tft.drawRect(tft.getCursorX(), tft.getCursorY()-15, 240, 40, HX8357_WHITE);
  tft.setCursor(tft.getCursorX()+10, 90);
  for (int count = 0; count < strlen(pass); count++){
    tft.print('*');
  }
  if (strlen(pass) > 15){
    tft.print('.');
    tft.print('.');
    tft.print('.');  
  }
  //connect
  tft.fillRect(130+240+10, 90-15, 90, 40, HX8357_GREEN);
  tft.setCursor(130+240+10+5, 90);
  tft.setTextColor(HX8357_BLACK);
  tft.print("Connect");
  tft.setTextColor(HX8357_WHITE);

  //dispense
  tft.setCursor(10, 140);
  tft.print("Dispense Amount: ");
  tft.fillCircle(tft.getCursorX()+5, tft.getCursorY()+8, 10, HX8357_WHITE);
  tft.fillCircle(tft.getCursorX()+5, tft.getCursorY()+8, 8, HX8357_BLACK);
  tft.setCursor(tft.getCursorX()+5+20, tft.getCursorY());
  tft.print("1");
  tft.setCursor(tft.getCursorX()+5, tft.getCursorY());
  tft.print("cup");
  tft.fillCircle(tft.getCursorX()+15, tft.getCursorY()+8, 10, HX8357_WHITE);
  tft.fillCircle(tft.getCursorX()+15, tft.getCursorY()+8, 8, HX8357_BLACK);
  tft.setCursor(tft.getCursorX()+15+20, tft.getCursorY());
  tft.print("2");
  tft.setCursor(tft.getCursorX()+5, tft.getCursorY());
  tft.print("cup");
  tft.fillCircle(tft.getCursorX()+15, tft.getCursorY()+8, 10, HX8357_WHITE);
  tft.fillCircle(tft.getCursorX()+15, tft.getCursorY()+8, 8, HX8357_BLACK);
  tft.setCursor(tft.getCursorX()+15+20, tft.getCursorY());
  tft.print("3");
  tft.setCursor(tft.getCursorX()+5, tft.getCursorY());
  tft.print("cup");
  switch (selected_amount){
    case 0: // DEFAULT
      tft.fillCircle(219+20+48+5+15, 140+8, 8, HX8357_BLACK);
      tft.fillCircle(219+20+48+5+15+20+48+5+15, 140+8, 8, HX8357_BLACK);
      tft.fillCircle(219, 140+8, 6, HX8357_WHITE);
      break;
    case 1:
      tft.fillCircle(219, 140+8, 8, HX8357_BLACK);
      tft.fillCircle(219+20+48+5+15+20+48+5+15, 140+8, 8, HX8357_BLACK);
      tft.fillCircle(219+20+48+5+15, 140+8, 6, HX8357_WHITE);
      break;
    case 2:
      tft.fillCircle(219, 140+8, 8, HX8357_BLACK);
      tft.fillCircle(219+20+48+5+15, 140+8, 8, HX8357_BLACK);
      tft.fillCircle(219+20+48+5+15+20+48+5+15, 140+8, 6, HX8357_WHITE);
      break;
  }

  //watering mode
  tft.setCursor(10, 170);
  tft.print("Watering Mode: ");
  tft.fillCircle(tft.getCursorX()+5, tft.getCursorY()+8, 10, HX8357_WHITE);
  tft.fillCircle(tft.getCursorX()+5, tft.getCursorY()+8, 8, HX8357_BLACK);
  tft.setCursor(tft.getCursorX()+5+20, tft.getCursorY());
  tft.print("Auto");
  tft.fillCircle(tft.getCursorX()+15, tft.getCursorY()+8, 10, HX8357_WHITE);
  tft.fillCircle(tft.getCursorX()+15, tft.getCursorY()+8, 8, HX8357_BLACK);
  tft.setCursor(tft.getCursorX()+15+20, tft.getCursorY());
  tft.print("Schedule");
  switch (mode){
    case AUTO: // DEFAULT
      tft.fillCircle(195+20+48+15, 170+8, 8, HX8357_BLACK);
      tft.fillCircle(195, 170+8, 6, HX8357_WHITE);    
      break;
    case SCHEDULED:
      tft.fillCircle(195, 170+8, 8, HX8357_BLACK);
      tft.fillCircle(195+20+48+15, 170+8, 6, HX8357_WHITE);
      break;
  }

  //schedule
  //days
  tft.setCursor(10, 220);
  tft.print("Schedule: ");
  for (int i=0; i<7; i++){
    tft.drawRect(130+50*i, 220-15, 40, 40, HX8357_WHITE);
    i==3 || i==6 ? tft.setCursor(130+50*i+10, 220) : tft.setCursor(130+50*i+15, 220);
    tft.print(days[i]);
    
    if (schedule[i]){
      for (int j = 2; j < 7; j++){
        tft.drawRect(130+50*i+j, 220-15+j, 40-2*j, 40-2*j, HX8357_BLUE);
      }
    }
    else{
      for (int j = 2; j < 7; j++){
        tft.drawRect(130+50*i+j, 220-15+j, 40-2*j, 40-2*j, HX8357_BLACK);
      }
    }
  }

  //time
  tft.setCursor(10, 270);
  tft.print("Time: ");
  tft.drawRect(72+48+10, 270-15, 40, 40, HX8357_WHITE);
  tft.setCursor(72+48+10+15, 270);
  tft.fillRect(tft.getCursorX(), tft.getCursorY(), 15, 15, HX8357_BLACK);
  tft.print(selected_time[0]);
  tft.drawRect(72+48+10+50, 270-15, 40, 40, HX8357_WHITE);
  tft.setCursor(72+48+10+50+15, 270);
  tft.fillRect(tft.getCursorX(), tft.getCursorY(), 15, 15, HX8357_BLACK);
  tft.print(selected_time[1]);
  tft.drawRect(72+48+10+50+70, 270-15, 40, 40, HX8357_WHITE);
  tft.setCursor(72+48+10+50+70+15, 270);
  tft.fillRect(tft.getCursorX(), tft.getCursorY(), 15, 15, HX8357_BLACK);
  tft.print(selected_time[2]);
  tft.drawRect(72+48+10+50+70+50, 270-15, 40, 40, HX8357_WHITE);
  tft.setCursor(72+48+10+50+70+50+15, 270);
  tft.fillRect(tft.getCursorX(), tft.getCursorY(), 15, 15, HX8357_BLACK);
  tft.print(selected_time[3]);
  //am/pm
  // tft.fillCircle(72+48+50+70+50+50+15, 270+5, 10, HX8357_WHITE);
  // tft.fillCircle(72+48+50+70+50+50+15, 270+5, 8, HX8357_BLACK);
  // tft.setCursor(72+48+50+70+50+50+15+20, 270);
  // tft.print("AM");
  // tft.fillCircle(72+48+50+70+50+50+15+20+36+10, 270+5, 10, HX8357_WHITE);
  // tft.fillCircle(72+48+50+70+50+50+15+20+36+10, 270+5, 8, HX8357_BLACK);
  // tft.setCursor(72+48+50+70+50+50+15+20+36+10+20, 270);
  // tft.print("PM");
  // colon
  tft.setCursor(72+48+10+50+48, 270-5);
  tft.setTextSize(3);
  tft.print(':');
  tft.setTextSize(2);
}

void displayKeyboard(){
  tft.fillScreen(HX8357_BLACK);

  //exit button
  tft.fillRect(HX8357_TFTHEIGHT-80, 0, 80, 40, HX8357_BLUE);
  tft.setCursor(HX8357_TFTHEIGHT-80+19, 13);
  tft.print("EXIT");

  //input echo box
  tft.drawRect(40,40,400,60,HX8357_WHITE);
  tft.drawLine(52, 92, 428, 92, HX8357_WHITE);

  //set cursor for echo input
  tft.setCursor(52, 72);
}

void drawKeyboard(){
  //numbers
  for (int i=0; i<10;i++){
    tft.drawRect(num_x1+KEY_SZ*i,num_y1,KEY_SZ,KEY_SZ,HX8357_WHITE);
    tft.setCursor(num_x1+KEY_SZ*i+char_offset, num_y1+char_offset);
    tft.print(nums[i]);
  }
  //row1
  for (int i=0; i<10;i++){
    tft.drawRect(row1_x1+KEY_SZ*i,row1_y1,KEY_SZ,KEY_SZ,HX8357_WHITE);
    tft.setCursor(row1_x1+KEY_SZ*i+char_offset, row1_y1+char_offset);
    if (key_state){
      tft.fillRect(row1_x1+1+KEY_SZ*i, row1_y1+1, KEY_SZ-2, KEY_SZ-2, HX8357_BLACK);
      tft.print(row1[i+10*key_state]);
    }
    else{
      tft.fillRect(row1_x1+1+KEY_SZ*i, row1_y1+1, KEY_SZ-2, KEY_SZ-2, HX8357_BLACK);
      tft.print(row1[i]);
    }
  }
  //row2
  //shift
  tft.drawRect(row2_x1-40, row2_y1, KEY_SZ, KEY_SZ, HX8357_WHITE);
  tft.fillRect(row2_x1-40+13, row2_y1+18, 15, 15, HX8357_WHITE);
  tft.fillTriangle(row2_x1-40+9, row2_y1+18, row2_x1-40+20, row2_y1+7, row2_x1-40+31, row2_y1+18, HX8357_WHITE);
  for (int i=0; i<9;i++){
    tft.drawRect(row2_x1+KEY_SZ*i,row2_y1,KEY_SZ,KEY_SZ,HX8357_WHITE);
    tft.setCursor(row2_x1+KEY_SZ*i+char_offset, row2_y1+char_offset);
    if (key_state){
      tft.fillRect(row2_x1+1+KEY_SZ*i, row2_y1+1, KEY_SZ-2, KEY_SZ-2, HX8357_BLACK);
      tft.print(row2[i+9*key_state]);
    }
    else{
      tft.fillRect(row2_x1+1+KEY_SZ*i, row2_y1+1, KEY_SZ-2, KEY_SZ-2, HX8357_BLACK);
      tft.print(row2[i]);
    }
  }
  //backspace
  tft.drawRect(HX8357_TFTHEIGHT-60, row2_y1, KEY_SZ, KEY_SZ, HX8357_WHITE);
  tft.fillRect(HX8357_TFTHEIGHT-60+18, row2_y1+13, 15, 15, HX8357_WHITE);
  tft.fillTriangle(HX8357_TFTHEIGHT-60+18, row2_y1+9, HX8357_TFTHEIGHT-60+18-11, row2_y1+20, HX8357_TFTHEIGHT-60+18, row2_y1+31, HX8357_WHITE);
  //row3
  //symbols
  tft.drawRect(row3_x1-60, row3_y1, 60, 40, HX8357_WHITE);
  tft.setCursor(row3_x1-60+word_offset, row3_y1+word_offset);
  tft.print("#$@");
  for (int i=0; i<7;i++){
    tft.drawRect(row3_x1+KEY_SZ*i,row3_y1,KEY_SZ,KEY_SZ,HX8357_WHITE);
    tft.setCursor(row3_x1+KEY_SZ*i+char_offset, row3_y1+char_offset);
    if (key_state){
      tft.fillRect(row3_x1+1+KEY_SZ*i, row3_y1+1, KEY_SZ-2, KEY_SZ-2, HX8357_BLACK);
      tft.print(row3[i+7*key_state]);
    }
    else{
      tft.fillRect(row3_x1+1+KEY_SZ*i, row3_y1+1, KEY_SZ-2, KEY_SZ-2, HX8357_BLACK);
      tft.print(row3[i]);
    }
  }
  //enter
  tft.drawRect(HX8357_TFTHEIGHT-100, row3_y1, 60, 40, HX8357_WHITE);
  tft.setCursor(HX8357_TFTHEIGHT-100+1, row3_y1+word_offset);
  tft.print("ENTER");

  //row4
  //comma
  tft.drawRect(row4_x1, row4_y1, KEY_SZ, KEY_SZ, HX8357_WHITE);
  tft.setCursor(row4_x1+char_offset, row4_y1+char_offset);
  tft.print(',');
  //space
  tft.drawRect(row4_x1+40, row4_y1, KEY_SZ*5, KEY_SZ, HX8357_WHITE);
  tft.setCursor(row4_x1+40+60+10, row4_y1+word_offset);
  tft.print("SPACE");
  //period
  tft.drawRect(row4_x1+40+KEY_SZ*5, row4_y1, KEY_SZ, KEY_SZ, HX8357_WHITE);
  tft.setCursor(row4_x1+40+KEY_SZ*5+char_offset, row4_y1+char_offset);
  tft.print('.');
}
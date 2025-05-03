
#include <Arduino.h>
#include <lvgl.h>
#include "ui/ui.h"
#include "lovyanGfxSetup.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <PubSubClient.h>


#define SD_CS_PIN 10
#define WIFI_CREDENTIALS_FILE "/wifi_credentials.txt"

typedef struct sensor {
  int id;
  float temp;
  float hum;
  int readingId;
} sensor;
sensor PTaSHTsensor;

typedef struct parameter {
  float temp1, temp2, temp3, temp4;
  float hum1, hum2, hum3, hum4;
  String time1, time2, time3, time4;
} parameter;
parameter FVparameter;

HardwareSerial mySerial(0);
TaskHandle_t WiFiTaskHandle = NULL;
TaskHandle_t SensorTaskHandle = NULL;
TaskHandle_t MQTTTaskHandle = NULL;

const char* mqtt_server = "116.108.82.46";
const char* mqtt_topic = "window";

WiFiClient espClient;
PubSubClient client(espClient);

#define TFT_HOR_RES SCREEN_WIDTH
#define TFT_VER_RES SCREEN_HEIGHT

/* LVGL draws into this buffer, 1/10 screen size usually works well. The size is in bytes. */
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))

uint32_t draw_buf[DRAW_BUF_SIZE / 4];

LGFX tft;

#if LV_USE_LOG != 0
void my_print(lv_log_level_t level, const char *buf)
{
  LV_UNUSED(level);
  Serial.println(buf);
  Serial.flush();
}
#endif

/* LVGL calls it when a rendered image needs to copied to the display. */
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
  uint32_t w = lv_area_get_width(area);
  uint32_t h = lv_area_get_height(area);
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.writePixels((lgfx::rgb565_t *)px_map, w * h);
  tft.endWrite();

  /* Call it to tell LVGL you are ready. */
  lv_disp_flush_ready(disp);
}

/* Read the touchpad. */
void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data)
{
  uint16_t touchX, touchY;
  bool touched = tft.getTouch(&touchX, &touchY);

  if (!touched)
  {
    data->state = LV_INDEV_STATE_RELEASED;
  }
  else
  {
    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = touchX;
    data->point.y = touchY;
#if 0
    Serial.printf("x: %03d, y: %03d\n", data->point.x, data->point.y);
#endif
  }
}

/** Set tick routine needed for LVGL internal timings **/
static uint32_t my_tick_get_cb (void) { return millis(); }

void ReceiveSensorDataFromSlave(void *parameter) {
  while (true) {
    while (mySerial.available()) { 
        char input[512];
        int len = mySerial.readBytesUntil('}', input, sizeof(input) - 2);
        input[len] = '}'; 
        input[len + 1] = '\0';

        StaticJsonDocument<512> jsonRecvData;
        DeserializationError error = deserializeJson(jsonRecvData, input);

        if (!error) {
            String type = jsonRecvData["type"].as<String>();
            if (type == "sensor") {    
                PTaSHTsensor.id = jsonRecvData["id"].as<String>().toInt();
                PTaSHTsensor.temp = jsonRecvData["temp"];
                PTaSHTsensor.hum = jsonRecvData["hum"];
                PTaSHTsensor.readingId = jsonRecvData["readingId"].as<String>().toInt();

                Serial.print("Received sensor data:\n");
                Serial.print("ID: "); Serial.println(PTaSHTsensor.id);
                Serial.print("Temp: "); Serial.println(PTaSHTsensor.temp);
                Serial.print("Hum: "); Serial.println(PTaSHTsensor.hum);
                Serial.print("Reading ID: "); Serial.println(PTaSHTsensor.readingId);

                String tempString = String("PV: ") + String(PTaSHTsensor.temp) + "°C";
                String humString = String("PV: ") + String(PTaSHTsensor.hum) + "%";
                lv_label_set_text(ui_pvtemp, tempString.c_str());
                lv_label_set_text(ui_pvhumi, humString.c_str());
            }

            // Kiểm tra nếu nhận được dữ liệu giai đoạn
            if (type == "stage_info") {
              String stage = jsonRecvData["stage"].as<String>();
              float temp = jsonRecvData["temp"].as<float>();
              float hum = jsonRecvData["hum"].as<float>();
              int timeLeft = jsonRecvData["time_left"].as<int>();

              // Hiển thị thông tin về giai đoạn
              Serial.print("Received stage info:\n");
              Serial.print("Stage: "); Serial.println(stage.c_str());
              Serial.print("Temperature: "); Serial.println(temp);
              Serial.print("Humidity: "); Serial.println(hum);
              Serial.print("Time left: "); Serial.println(timeLeft);

              // Cập nhật thông tin giai đoạn lên màn hình
              String stageString = String("Process: ") + stage;
              lv_label_set_text(ui_giaidoan, stageString.c_str());

              String fvTempString = String("FV: ") + String(temp) + "°C";
              lv_label_set_text(ui_fvtemp, fvTempString.c_str());

              String fvHumString = String("FV: ") + String(hum) + "%";
              lv_label_set_text(ui_fvhumi, fvHumString.c_str());

              String timeElapsedString = String("Time elapsed: ") + String(timeLeft) + "s";
              lv_label_set_text(ui_timeelapsed, timeElapsedString.c_str());
          }
        } else {
            Serial.println("Error parsing JSON");
        }
    }
    vTaskDelay(100); // Delay to prevent blocking other tasks
  }
}

bool StartState = false;

void ui_event_Startbnt(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    if(event_code == LV_EVENT_CLICKED) {
        // Đặt cờ trạng thái StartState thành true (hoặc trạng thái bạn cần)
        StartState = true;
        //printf("StartState: %d\n", StartState);
        
    }
    sendParameterToSlave("StartState");
}

void ui_event_okbnt(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if (event_code == LV_EVENT_RELEASED) {
        // Lấy thông tin từ các textarea
        FVparameter.temp1 = atof(lv_textarea_get_text(ui_tempsetting1));
        FVparameter.hum1 = atof(lv_textarea_get_text(ui_humdsetting1));
        FVparameter.time1 = String(lv_textarea_get_text(ui_timesetting1));

        FVparameter.temp2 = atof(lv_textarea_get_text(ui_tempsetting2));
        FVparameter.hum2 = atof(lv_textarea_get_text(ui_humdsetting2));
        FVparameter.time2 = String(lv_textarea_get_text(ui_timesetting2));

        FVparameter.temp3 = atof(lv_textarea_get_text(ui_tempsetting3));
        FVparameter.hum3 = atof(lv_textarea_get_text(ui_humdsetting3));
        FVparameter.time3 = String(lv_textarea_get_text(ui_timesetting3));

        FVparameter.temp4 = atof(lv_textarea_get_text(ui_tempsetting4));
        FVparameter.hum4 = atof(lv_textarea_get_text(ui_humdsetting4));
        FVparameter.time4 = String(lv_textarea_get_text(ui_timesetting4));

        // In thông số của tất cả các giai đoạn
        // printf("Giai doan 1 (Say khu am) - Nhiệt độ: %s, Độ ẩm: %s, Thời gian: %s\n", temperature1, humidity1, time1);
        // printf("Giai doan 2 (Len men) - Nhiệt độ: %s, Độ ẩm: %s, Thời gian: %s\n", temperature2, humidity2, time2);
        // printf("Giai doan 3 (On dinh) - Nhiệt độ: %s, Độ ẩm: %s, Thời gian: %s\n", temperature3, humidity3, time3);
        // printf("Giai doan 4 (Bao quan) - Nhiệt độ: %s, Độ ẩm: %s, Thời gian: %s\n", temperature4, humidity4, time4);

        sendParameterToSlave("parameter");

        // Xóa màn hình hiện tại (ui_Screen2)
        _ui_screen_delete(&ui_Screen2);

        // Chuyển sang màn hình ui_Screen3
        _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_MOVE_TOP, 500, 0, &ui_Screen3_screen_init);
    }
}

void sendParameterToSlave( const String &type) {
  StaticJsonDocument<512> doc;
  doc ["type"] = type; 
  
  if (type == "parameter"){
    JsonObject stage1 = doc.createNestedObject("Say khu am");
    stage1["temp"] = FVparameter.temp1;
    stage1["hum"] = FVparameter.hum1;
    stage1["time"] = FVparameter.time1;

    // Giai đoạn 2
    JsonObject stage2 = doc.createNestedObject("Len men");
    stage2["temp"] = FVparameter.temp2;
    stage2["hum"] = FVparameter.hum2;
    stage2["time"] = FVparameter.time2;

    // Giai đoạn 3
    JsonObject stage3 = doc.createNestedObject("On dinh");
    stage3["temp"] = FVparameter.temp3;
    stage3["hum"] = FVparameter.hum3;
    stage3["time"] = FVparameter.time3;

    // Giai đoạn 4
    JsonObject stage4 = doc.createNestedObject("Bao quan");
    stage4["temp"] = FVparameter.temp4;
    stage4["hum"] = FVparameter.hum4;
    stage4["time"] = FVparameter.time4;
  } 
  
  else if (type == "StartState") {
    doc["Start"] = StartState;
    StartState = false;
  }

  // Chuyển JSON thành chuỗi
  String json_data;
  serializeJson(doc, json_data);
  mySerial.print(json_data);  // Gửi dữ liệu qua UART
  // Gửi dữ liệu qua UART
  // Serial.println("Sending sensor data to Slave...");
  // Serial.println(json_data);  // In dữ liệu JSON ra Serial để debug
}

// Hàm gửi dữ liệu sensor đến MQTT
void sendSensorDataToMQTT() {
  if (client.connected()) {
    String payload = String("{\"temp\":") + String(PTaSHTsensor.temp) + ", \"hum\":" + String(PTaSHTsensor.hum) + "}";
    client.publish(mqtt_topic, payload.c_str());
    Serial.println("Sensor data sent to MQTT broker: " + payload);
  }
}

void mqttConnectTask(void *parameter) {
  // Khởi tạo client MQTT một lần duy nhất
  client.setServer(mqtt_server, 1883);
  
  while (true) {
    // Kiểm tra kết nối WiFi trước
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("MQTT: Waiting for WiFi connection...");
      vTaskDelay(5000 / portTICK_PERIOD_MS);  // Chờ 5s trước khi thử lại
      continue;
    }

    // Xử lý kết nối MQTT
    if (!client.connected()) {
      Serial.println("MQTT: Attempting to connect...");
      
      String clientId = "ESP32Client-" + String(random(0xffff), HEX);
      
      if (client.connect(clientId.c_str())) {
        Serial.println("MQTT: Connected!");
        client.subscribe(mqtt_topic);
      } else {
        Serial.print("MQTT: Connection failed, rc=");
        Serial.print(client.state());
        Serial.println(", retrying in 5 seconds...");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        continue;
      }
    }

    // Gửi dữ liệu định kỳ
    static uint32_t lastSendTime = 0;
    uint32_t now = millis();
    
    if (now - lastSendTime >= 10000) {  // Mỗi 10 giây
      if (WiFi.status() == WL_CONNECTED && client.connected()) {
        sendSensorDataToMQTT();
      }
      lastSendTime = now;
    }

    // Duy trì kết nối MQTT
    client.loop();
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Giảm tải CPU
  }
}

char prev_wifi_ssid[32] = "";
char prev_wifi_password[32] = "";

void saveWiFiCredentials(const char* ssid, const char* password) {
  if (strcmp(wifi_ssid, prev_wifi_ssid) != 0 || strcmp(wifi_password, prev_wifi_password) != 0) {
    Serial.println("New WiFi credentials entered!");
    strncpy(prev_wifi_ssid, wifi_ssid, sizeof(prev_wifi_ssid) - 1);
    strncpy(prev_wifi_password, wifi_password, sizeof(prev_wifi_password) - 1);
  } else {
    // Dữ liệu không thay đổi
    Serial.println("WiFi credentials have not changed.");
  }

  File file = SD.open("/wifi_credentials.txt", FILE_WRITE);

  if (file) {
    file.print(ssid);
    file.print("\n");
    file.print(password);
    file.close();
    Serial.println("WiFi credentials saved to SD card.");
  } else {
    Serial.println("Failed to open file for writing.");
  }
  
}

void readWiFiCredentials() {
  
  File file = SD.open("/wifi_credentials.txt", FILE_READ);

  if (file) {
    String ssid = file.readStringUntil('\n');
    String password = file.readStringUntil('\n');

    ssid.trim(); // Xóa khoảng trắng đầu và cuối
    password.trim(); // Xóa khoảng trắng đầu và cuối

    if (ssid.length() > 0 && password.length() > 0) {
      ssid.toCharArray(wifi_ssid, sizeof(wifi_ssid));
      password.toCharArray(wifi_password, sizeof(wifi_password));

    } else {
      Serial.println("Invalid WiFi credentials in file.");
    }

    file.close();

    } else {
      Serial.println("Failed to open file for reading.");
    }

  Serial.print("SSID: ");
  Serial.println(wifi_ssid);
  Serial.print("Password: ");
  Serial.println(wifi_password);
}

/** WiFi erase function stored in EEPROM **/
void clearWiFiCredentials() {
  // Xóa file chứa thông tin WiFi trên SD card
  if (SD.exists("/wifi_credentials.txt")) {
    SD.remove("/wifi_credentials.txt");
    Serial.println("WiFi credentials cleared!");
  }
}

/** Wifi config with LVGL */
void wifiTask(void *parameter) {
  const char* ssid = (const char*)parameter;
  const char* password = wifi_password;

  while (true) {  // Vòng lặp vô hạn để thử lại kết nối
    // Ngắt kết nối WiFi cũ và thiết lập lại
    WiFi.disconnect(true);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    int timeout = 20; // 20 lần thử, mỗi lần 500ms => tổng 10 giây
    while (WiFi.status() != WL_CONNECTED && timeout > 0) {
      vTaskDelay(500 / portTICK_PERIOD_MS);
      Serial.print(".");
      timeout--;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
      lv_label_set_text(ui_statuswf, "Connected");
      lv_label_set_text(ui_wifiname, WiFi.SSID().c_str());
      saveWiFiCredentials(ssid, password);
      
      // Giữ task WiFi chạy để kiểm tra kết nối định kỳ
      while (WiFi.status() == WL_CONNECTED) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);  // Kiểm tra mỗi 10 giây
      }
      Serial.println("WiFi disconnected!");
    } else {
      Serial.println("\nConnect fail! Retrying in 10 seconds...");
      lv_label_set_text(ui_statuswf, "Wrong WiFi");
      lv_label_set_text(ui_wifiname, "No WiFi");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      vTaskDelay(10000 / portTICK_PERIOD_MS);  // Chờ 10 giây trước khi thử lại
    }
  }
}


/** Function to input SSID and password from the keyboard */
void connectWifi(lv_timer_t * timer) {

  if (WiFiTaskHandle != NULL) return;
  
  
  const char* ssid = get_wifi_ssid();
  if (*ssid) {
      xTaskCreate(
          wifiTask,      // WiFi Task
          "WiFiTask",    
          4048,          // RAM For task (4KB)
          (void*)ssid,   // Input parameter (SSID)
          1,             // Priority (1 = low)
          &WiFiTaskHandle     
      );
  }
}

void setup()
{
  // Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, 44, 43); // RX_PIN, TX_PIN là các chân UART0

  lv_init();

  tft.begin();
  tft.setRotation(0);
  tft.setBrightness(255);

  /* Set a tick source so that LVGL will know how much time elapsed. */
  lv_tick_set_cb((lv_tick_get_cb_t)millis);

  /* Register print function for debugging. */
#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

  /* Create a display. */
  lv_display_t *disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);
  lv_display_set_flush_cb(disp, my_disp_flush);
  lv_display_set_buffers(disp, draw_buf, NULL, sizeof(draw_buf), LV_DISPLAY_RENDER_MODE_PARTIAL);

  /* Initialize the (dummy) input device driver. */
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); /*Touchpad should have POINTER type*/
  lv_indev_set_read_cb(indev, my_touchpad_read);

#if 1
  /* Create a simple label. */
  lv_obj_t *label = lv_label_create( lv_scr_act() );
  lv_label_set_text( label, "Hello Arduino, I'm LVGL!" );
  lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );
#endif

lv_tick_set_cb( my_tick_get_cb );

ui_init();

if (!SD.begin(SD_CS_PIN)) {
  Serial.println("Failed to initialize SD card.");
  return;
}
Serial.println("SD card initialized.");

  //clearWiFiCredentials();
  readWiFiCredentials();

  if (strlen(wifi_ssid) > 0 && strlen(wifi_password) > 0) {
    Serial.println("Attempting WiFi connection...");
    if (WiFiTaskHandle == NULL || eTaskGetState(WiFiTaskHandle) == eDeleted) {
      xTaskCreate(
        wifiTask,          // Hàm thực thi
        "WiFiTask",        // Tên task
        4096,              // Kích thước stack
        (void*)wifi_ssid,  // Tham số
        1,                 // Độ ưu tiên (thấp hơn task sensor)
        &WiFiTaskHandle    // Handle
      );
    }
  } else {
    Serial.println("No WiFi credentials found");
  }

/** lv timer for run task */
lv_timer_t* WifiTask = lv_timer_create(connectWifi, 5000, NULL);

xTaskCreate(ReceiveSensorDataFromSlave, "SensorDataTask", 4096, NULL, 2, &SensorTaskHandle);

if (MQTTTaskHandle == NULL) {
  xTaskCreate(mqttConnectTask, "MQTTConnectTask", 4096, NULL, 0, &MQTTTaskHandle);
}

Serial.println("Setup done");
}

void loop()
{
  lv_task_handler(); /* Let LVGL do its work. */
  delay(5);
}
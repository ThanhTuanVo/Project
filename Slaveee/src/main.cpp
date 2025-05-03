#include <HardwareSerial.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_SHT31.h>

// Định nghĩa các thông số cho Board
#define MAX31865_CS  12
#define MAX31865_SDI 10
#define MAX31865_SDO 11
#define MAX31865_CLK 16
#define SDA_PIN 36
#define SCL_PIN 35

#define BOARD_ID 1
HardwareSerial mySerial(1);

#define RREF      430.0
#define RNOMINAL  100.0

Adafruit_MAX31865 pt100 = Adafruit_MAX31865(MAX31865_CS, MAX31865_SDI, MAX31865_SDO, MAX31865_CLK);

// ----------------- Cảm biến SHT3x -------------------
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// Cấu trúc dữ liệu cảm biến và tham số
float pt100_temp;
float sht_hum;


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

bool StartState = false;
unsigned long stageStartTime = 0;
int currentStage = 0;
bool stageInProgress = false;

// Hàm nhận dữ liệu từ Master qua UART
void receiveDataFromMaster(void *parameter) {
  while (true) {
    while (mySerial.available() > 0) {
      char input[512];
      int len = mySerial.readBytes(input, sizeof(input) - 1);
      input[len] = '\0'; // Thêm ký tự kết thúc chuỗi
      Serial.println("Received raw data:");
      Serial.println(input);
      StaticJsonDocument<512> doc;
      DeserializationError error = deserializeJson(doc, input);

      if (!error) {
        String type = doc["type"].as<String>();
        if (type == "parameter") {
          
          JsonObject stage1 = doc["Say khu am"];
          FVparameter.temp1 = stage1["temp"];
          FVparameter.hum1 = stage1["hum"];
          FVparameter.time1 = stage1["time"].as<String>();

          // Giai đoạn 2
          JsonObject stage2 = doc["Len men"];
          FVparameter.temp2 = stage2["temp"];
          FVparameter.hum2 = stage2["hum"];
          FVparameter.time2 = stage2["time"].as<String>();

          // Giai đoạn 3
          JsonObject stage3 = doc["On dinh"];
          FVparameter.temp3 = stage3["temp"];
          FVparameter.hum3 = stage3["hum"];
          FVparameter.time3 = stage3["time"].as<String>();

          // Giai đoạn 4
          JsonObject stage4 = doc["Bao quan"];
          FVparameter.temp4 = stage4["temp"];
          FVparameter.hum4 = stage4["hum"];
          FVparameter.time4 = stage4["time"].as<String>();

          Serial.println("Received parameter data:");
          Serial.printf("Say khu am - Temp: %.2f°C, Hum: %.2f%%, Time: %s\n", FVparameter.temp1, FVparameter.hum1, FVparameter.time1.c_str());
          Serial.printf("Len men - Temp: %.2f°C, Hum: %.2f%%, Time: %s\n", FVparameter.temp2, FVparameter.hum2, FVparameter.time2.c_str());
          Serial.printf("On dinh - Temp: %.2f°C, Hum: %.2f%%, Time: %s\n", FVparameter.temp3, FVparameter.hum3, FVparameter.time3.c_str());
          Serial.printf("Bao quan - Temp: %.2f°C, Hum: %.2f%%, Time: %s\n", FVparameter.temp4, FVparameter.hum4, FVparameter.time4.c_str());
        }
        else if (type == "StartState"){
          StartState = doc["Start"];
          stageStartTime = millis();
          stageInProgress = true;

        }
        Serial.println();
      } else {
        Serial.println("Unknown type received.");
      }
    }
    vTaskDelay(2000); // Delay để tránh bị block
  }
}

void runStages() {
  if (!stageInProgress) return; // Nếu không trong trạng thái thực hiện giai đoạn thì thoát

  unsigned long currentTime = millis();
  StaticJsonDocument<512> doc;

  // Thêm type vào JSON
  doc["type"] = "stage_info"; // Đánh dấu kiểu dữ liệu là "stage_info"

  switch (currentStage) {
    case 0: // Giai đoạn Say khu am
      if (currentTime - stageStartTime >= FVparameter.time1.toInt() * 1000) {
        Serial.println("End of 'Say khu am' stage");
        currentStage++;
        stageStartTime = currentTime; // Reset timer for next stage
      } else {
        // Thêm thông tin về giai đoạn
        doc["stage"] = "Say khu am";
        doc["temp"] = FVparameter.temp1;
        doc["hum"] = FVparameter.hum1;
        doc["time_left"] = (FVparameter.time1.toInt() * 1000 - (currentTime - stageStartTime)) / 1000;

        if (pt100_temp < FVparameter.temp1) {
          digitalWrite(48, HIGH);  // Bật LED nếu nhiệt độ hiện tại thấp hơn nhiệt độ cài đặt
        } else {
          digitalWrite(48, LOW);   // Tắt LED nếu nhiệt độ đã đạt yêu cầu
        }

        String output;
        ArduinoJson::serializeJson(doc, output);
        mySerial.print(output); // Gửi dữ liệu JSON qua UART
        Serial.println("Running 'Say khu am' stage...");
        Serial.printf("Stage: Say khu am, Temp: %.2f°C, Hum: %.2f%%, Time left: %d seconds\n", FVparameter.temp1, FVparameter.hum1, doc["time_left"]);
      }
      break;
    
    case 1: // Giai đoạn Len men
      if (currentTime - stageStartTime >= FVparameter.time2.toInt() * 1000) {
        Serial.println("End of 'Len men' stage");
        currentStage++;
        stageStartTime = currentTime;
      } else {
        // Thêm thông tin về giai đoạn
        doc["stage"] = "Len men";
        doc["temp"] = FVparameter.temp2;
        doc["hum"] = FVparameter.hum2;
        doc["time_left"] = (FVparameter.time2.toInt() * 1000 - (currentTime - stageStartTime)) / 1000;

        if (pt100_temp < FVparameter.temp1) {
          digitalWrite(48, HIGH);  // Bật LED nếu nhiệt độ hiện tại thấp hơn nhiệt độ cài đặt
        } else {
          digitalWrite(48, LOW);   // Tắt LED nếu nhiệt độ đã đạt yêu cầu
        }

        String output;
        ArduinoJson::serializeJson(doc, output);
        mySerial.print(output); // Gửi dữ liệu JSON qua UART
        Serial.println("Running 'Len men' stage...");
        Serial.printf("Stage: Len men, Temp: %.2f°C, Hum: %.2f%%, Time left: %d seconds\n", FVparameter.temp2, FVparameter.hum2, doc["time_left"]);
      }
      break;

    case 2: // Giai đoạn On dinh
      if (currentTime - stageStartTime >= FVparameter.time3.toInt() * 1000) {
        Serial.println("End of 'On dinh' stage");
        currentStage++;
        stageStartTime = currentTime;
      } else {
        // Thêm thông tin về giai đoạn
        doc["stage"] = "On dinh";
        doc["temp"] = FVparameter.temp3;
        doc["hum"] = FVparameter.hum3;
        doc["time_left"] = (FVparameter.time3.toInt() * 1000 - (currentTime - stageStartTime)) / 1000;

        if (pt100_temp < FVparameter.temp1) {
          digitalWrite(48, HIGH);  // Bật LED nếu nhiệt độ hiện tại thấp hơn nhiệt độ cài đặt
        } else {
          digitalWrite(48, LOW);   // Tắt LED nếu nhiệt độ đã đạt yêu cầu
        }

        String output;
        ArduinoJson::serializeJson(doc, output);
        mySerial.print(output); // Gửi dữ liệu JSON qua UART
        Serial.println("Running 'On dinh' stage...");
        Serial.printf("Stage: On dinh, Temp: %.2f°C, Hum: %.2f%%, Time left: %d seconds\n", FVparameter.temp3, FVparameter.hum3, doc["time_left"]);
      }
      break;

    case 3: // Giai đoạn Bao quan
      if (currentTime - stageStartTime >= FVparameter.time4.toInt() * 1000) {
        Serial.println("End of 'Bao quan' stage");
        currentStage = 0; 
        stageInProgress = false; // All stages completed
      } else {
        // Thêm thông tin về giai đoạn
        doc["stage"] = "Bao quan";
        doc["temp"] = FVparameter.temp4;
        doc["hum"] = FVparameter.hum4;
        doc["time_left"] = (FVparameter.time4.toInt() * 1000 - (currentTime - stageStartTime)) / 1000;

        if (pt100_temp < FVparameter.temp1) {
          digitalWrite(48, HIGH);  // Bật LED nếu nhiệt độ hiện tại thấp hơn nhiệt độ cài đặt
        } else {
          digitalWrite(48, LOW);   // Tắt LED nếu nhiệt độ đã đạt yêu cầu
        }

        String output;
        ArduinoJson::serializeJson(doc, output);
        mySerial.print(output); // Gửi dữ liệu JSON qua UART
        Serial.println("Running 'Bao quan' stage...");
        Serial.printf("Stage: Bao quan, Temp: %.2f°C, Hum: %.2f%%, Time left: %d seconds\n", FVparameter.temp4, FVparameter.hum4, doc["time_left"]);
      }
      break;

    default:
      break;
  }
}

void runStagesTask(void *parameter) {
  while (true) {
    runStages();  // Gọi runStages mỗi lần trong task

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay 1 giây giữa mỗi lần gọi
  }
}

// Hàm gửi dữ liệu cảm biến qua UART
void sendSensorDataToMaster(void *parameter) {
  while (true) {
    // Đọc dữ liệu từ cảm biến trong task này
    PTaSHTsensor.id = BOARD_ID;
    PTaSHTsensor.readingId++;

    pt100_temp = pt100.temperature(RNOMINAL, RREF);
    sht_hum = sht31.readHumidity();

    if (isnan(pt100_temp)) {
      Serial.println("Lỗi đọc PT100!");
      pt100_temp = 0;
    }
    if (isnan(sht_hum)) {
      Serial.println("Lỗi đọc SHT31!");
      sht_hum = 0;
    }

    PTaSHTsensor.temp = pt100_temp;
    PTaSHTsensor.hum = sht_hum;

    // Tạo JSON cho dữ liệu cảm biến DHT
    StaticJsonDocument<200> doc;
    doc["type"] = "sensor";
    doc["id"] = PTaSHTsensor.id;
    doc["temp"] = PTaSHTsensor.temp;
    doc["hum"] = PTaSHTsensor.hum;
    doc["readingId"] = PTaSHTsensor.readingId;

    String output;
    ArduinoJson::serializeJson(doc, output);
    mySerial.print(output); // Gửi dữ liệu JSON qua UART

    Serial.print("Sent sensor data to master: ");
    Serial.println(output);

    vTaskDelay(5000); // Gửi dữ liệu mỗi 2 giây (giống như interval trước)
  }
}

void setup() {
  Wire.begin(36, 35); // SDA: GPIO 36, SCL: GPIO 35
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, 18, 17); // RX: GPIO 18, TX: GPIO 17

  pinMode(48, OUTPUT); // Đặt GPIO48 là đầu ra (LED)

  if (!pt100.begin(MAX31865_3WIRE)) {
    Serial.println(" Không tìm thấy cảm biến PT100!");
    while (1) delay(1);
  }

  if (!sht31.begin(0x44)) {
    Serial.println(" Không tìm thấy cảm biến SHT3x!");
    while (1) delay(1);
  }
  // Tạo task để nhận dữ liệu từ Master
  xTaskCreate(receiveDataFromMaster, "ReceiveData", 4096, NULL, 3, NULL);

  // Tạo task để gửi dữ liệu cảm biến đến Master
  xTaskCreate(sendSensorDataToMaster, "SendData", 2048, NULL, 2, NULL);

  // Tạo task để chạy các giai đoạn
  xTaskCreate(runStagesTask, "RunStages", 4096, NULL, 1, NULL);
}

void loop() {

  delay(5);
}
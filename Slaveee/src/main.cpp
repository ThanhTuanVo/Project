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

        if (type == "StartState"){
          StartState = doc["Start"];
          if (StartState) {
            Serial.println("Start state received: true");
            // gọi chương trình hoạt động
          } else {
            Serial.println("Start state received: false");
          }

        }
        Serial.println();
      } else {
        Serial.println("Unknown type received.");
      }
    }
    vTaskDelay(2000); // Delay để tránh bị block
  }
}

// Hàm gửi dữ liệu cảm biến qua UART
void sendSensorDataToMaster(void *parameter) {
  while (true) {
    // Đọc dữ liệu từ cảm biến trong task này
    PTaSHTsensor.id = BOARD_ID;
    PTaSHTsensor.readingId++;

    float pt100_temp = pt100.temperature(RNOMINAL, RREF);
    float sht_hum = sht31.readHumidity();

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
    serializeJson(doc, output);
    mySerial.print(output); // Gửi dữ liệu JSON qua UART

    Serial.print("Sent sensor data to master: ");
    Serial.println(output);

    vTaskDelay(5000); // Gửi dữ liệu mỗi 2 giây (giống như interval trước)
  }
}

void setup() {
  Wire.begin(36, 35); // SDA: GPIO 48, SCL: GPIO 47
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, 18, 17); // RX: GPIO 18, TX: GPIO 17

  if (!pt100.begin(MAX31865_3WIRE)) {
    Serial.println(" Không tìm thấy cảm biến PT100!");
    while (1) delay(1);
  }

  if (!sht31.begin(0x44)) {
    Serial.println(" Không tìm thấy cảm biến SHT3x!");
    while (1) delay(1);
  }
  // Tạo task để nhận dữ liệu từ Master
  xTaskCreate(receiveDataFromMaster, "ReceiveData", 4096, NULL, 1, NULL);

  // Tạo task để gửi dữ liệu cảm biến đến Master
  xTaskCreate(sendSensorDataToMaster, "SendData", 2048, NULL, 2, NULL);
}

void loop() {

  delay(5);
}

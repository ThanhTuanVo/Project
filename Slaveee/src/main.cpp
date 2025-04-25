#include <HardwareSerial.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <ArduinoJson.h>

// Định nghĩa các thông số cho Board
#define BOARD_ID 1
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

HardwareSerial mySerial(1);

// Cấu trúc dữ liệu cảm biến và tham số
typedef struct sensor {
    int id;
    float temp;
    float hum;
    int readingId;
} sensor;
sensor DHTsensor;

typedef struct parameter {
  float temp;
  float hum;
  String time;
} parameter;
parameter FVparameter;

// Hàm nhận dữ liệu từ Master qua UART
void receiveDataFromMaster(void *parameter) {
  while (true) {
    while (mySerial.available() > 0) {
      char input[128];
      int len = mySerial.readBytesUntil('}', input, sizeof(input) - 2);
      input[len] = '}';
      input[len + 1] = '\0';

      StaticJsonDocument<128> doc;
      DeserializationError error = deserializeJson(doc, input);

      if (!error) {
        String type = doc["type"].as<String>();
        if (type == "parameter") {
          FVparameter.temp = doc["temp"];
          FVparameter.hum = doc["hum"];
          FVparameter.time = doc["time"].as<String>();

          Serial.print("Received parameter: ");
          Serial.print("Temp: " + String(FVparameter.temp) + "°C, ");
          Serial.print("Hum: " + String(FVparameter.hum) + "%, ");
          Serial.println("Time: " + FVparameter.time);
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
    DHTsensor.id = BOARD_ID;
    DHTsensor.readingId++;

    DHTsensor.temp = dht.readTemperature();
    DHTsensor.hum = dht.readHumidity();

    if (isnan(DHTsensor.temp) || isnan(DHTsensor.hum)) {
      Serial.println("Failed to read from DHT sensor!");
      DHTsensor.temp = 0;
      DHTsensor.hum = 0;
    } else {
      Serial.println("Temp: " + String(DHTsensor.temp) + "°C, Hum: " + String(DHTsensor.hum) + "%");
    }

    // Tạo JSON cho dữ liệu cảm biến DHT
    StaticJsonDocument<200> doc;
    doc["type"] = "sensor";
    doc["id"] = DHTsensor.id;
    doc["temp"] = DHTsensor.temp;
    doc["hum"] = DHTsensor.hum;
    doc["readingId"] = DHTsensor.readingId;

    String output;
    serializeJson(doc, output);
    mySerial.print(output); // Gửi dữ liệu JSON qua UART

    Serial.print("Sent sensor data to master: ");
    Serial.println(output);

    vTaskDelay(5000); // Gửi dữ liệu mỗi 2 giây (giống như interval trước)
  }
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, 18, 17); // RX: GPIO 18, TX: GPIO 17
  dht.begin();

  // Tạo task để nhận dữ liệu từ Master
  xTaskCreate(receiveDataFromMaster, "ReceiveData", 2048, NULL, 1, NULL);

  // Tạo task để gửi dữ liệu cảm biến đến Master
  xTaskCreate(sendSensorDataToMaster, "SendData", 2048, NULL, 2, NULL);
}

void loop() {

  delay(5);
}

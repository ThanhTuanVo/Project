#include <HardwareSerial.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <ArduinoJson.h>



/** Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc) **/
#define BOARD_ID 1

/** Digital pin connected to the DHT sensor **/ 
#define DHTPIN 4
/** Loại cảm biến sử dụng: Nếu cần thay từ DHT22 sang DHT11, thay đổi ở đây **/
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

HardwareSerial mySerial(1);
//-----------------------------------------------------------------------------------
/** Cấu trúc dữ liệu nhận trạng thái công tắc từ Master **/
typedef struct sw_state {
  bool switch_states[5];  // Sử dụng phần tử đầu tiên để điều khiển LED
} sw_state;
sw_state ledData;
//-----------------------------------------------------------------------------------
/** Cấu trúc dữ liệu cảm biến **/
typedef struct sensor {
    int id;
    float temp;
    float hum;
    int readingId;
} sensor;
sensor DHTsensor;

unsigned long previousMillis = 0;  // Lưu thời gian lần gửi dữ liệu trước
const long interval = 2000;        // Khoảng thời gian gửi dữ liệu (2 giây)
unsigned int readingId = 0;
/////////////////////////////////////////////////////////////////////////////////////////////////////

/** Hàm gửi dữ liệu dạng JSON qua master*/
void SendSensorDataToMaster() {
  // Tạo JSON cho dữ liệu cảm biến DHT
  StaticJsonDocument<200> doc;

  doc["type"] = "sensor"; // Loại dữ liệu là cảm biến
  doc["id"] = DHTsensor.id;
  doc["temp"] = DHTsensor.temp;
  doc["hum"] = DHTsensor.hum;
  doc["readingId"] = DHTsensor.readingId;

  // Gửi JSON qua UART
  String output;
  serializeJson(doc, output);
  mySerial.print(output);  // Gửi dữ liệu JSON qua UART

  // In dữ liệu ra Serial Monitor để kiểm tra
  Serial.print("Sent sensor data to master: ");
  Serial.println(output);
}

/** Hàm đọc nhiệt độ và độ ẩm từ cảm biến DHT11 **/
void readTempHumi(sensor &DHTdata) {
  DHTdata.temp = dht.readTemperature();
  DHTdata.hum = dht.readHumidity();

  if (isnan(DHTdata.temp) || isnan(DHTdata.hum)) {
      Serial.println("Failed to read from DHT sensor!");
      DHTdata.temp = 0;
      DHTdata.hum = 0;
  } else {
      Serial.println("Temp: " + String(DHTdata.temp) + "°C, Hum: " + String(DHTdata.hum) + "%");
  }
}



void setup() {
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, 18, 17); // RX: GPIO 18, TX: GPIO 17
  dht.begin();

  
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    DHTsensor.id = BOARD_ID;
    readTempHumi(DHTsensor);
    DHTsensor.readingId = readingId++;

    // Gửi dữ liệu cảm biến dưới dạng JSON
    SendSensorDataToMaster();
     // In dữ liệu ra Serial Monitor để debug
    Serial.print("Sent data: ");
    
  }
}
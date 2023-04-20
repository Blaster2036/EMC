#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <utility/imumaths.h>
#include <math.h>
#include <SPI.h>
#include <LoRa.h>

#define BNO055_SAMPLERATE_DELAY_MS (50)

#define SEALEVELPRESSURE_HPA (1020) // 1020 hPa tested SantoDomingo Provided by Accuweather

// define the pins used by the transceiver module
#define CS 5
#define RST 14
#define G0 27
#define RX 16
#define TX 17

#define SerialMonbaudRate 115200
#define GPSSerialBaudRate 9600
#define BNOExtCrystal true
#define TransmitterFrequency 866E6 // SET FREQUENCY: 433E6 for Asia, 866E6 for Europe, 915E6 for North America
#define SyncWord 0xFA
#define SetTransmitterPower 18 // Min 6 <---> Max 20

// Invoke libraries
Adafruit_BNO055 IMU = Adafruit_BNO055();
Adafruit_BME280 bme;
Gps gpsModule;

// Variables

float imuTemp = 0.0, accX = 0.0, accY = 0.0, accZ = 0.0, gyroX = 0.0, gyroY = 0.0,
      gyroZ = 0.0, quatW = 0.0, quatX = 0.0, quatY = 0.0, quatZ = 0.0;

float *ptr;
String msg = "";
String str = "";
String gpsStr = "";
unsigned long previousMillis = 0UL;
unsigned long interval = 50UL;
int counter = 0;

// Prototype functions
void transmit(String msg, int counter);
void BNO055();
float *BME280();
String gpsData();

void BNO055()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis > interval)
  {
    imu::Vector<3> acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> magnet = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Quaternion quat = IMU.getQuat();

    imuTemp = IMU.getTemp();
    accX = acc.x();
    accY = acc.y();
    accZ = acc.z();
    gyroX = gyro.x();
    gyroY = gyro.y();
    gyroZ = gyro.y();
    quatW = quat.w();
    quatX = quat.x();
    quatY = quat.y();
    quatZ = quat.z();

    ptr = BME280(); // trae la data del BME referenciado a través del puntero *ptr y debajo se extrae el contenido del arreglo
    gpsStr = gpsData();
    msg = gpsStr + "," +
          (String)ptr[0] + "," +
          (String)ptr[1] + "," +
          (String)ptr[2] + "," +
          (String)ptr[3] + "," +
          (String)imuTemp + "," +
          (String)accX + "," +
          (String)accY + "," +
          (String)accZ + "," +
          (String)gyroX + "," +
          (String)gyroY + "," +
          (String)gyroZ + "," +
          (String)quatW + "," +
          (String)quatX + "," +
          (String)quatY + "," +
          (String)quatZ;
    transmit(msg, counter);
    counter = counter + 1;
    previousMillis = currentMillis;
    Serial.println(msg);
  }
}

float *BME280()
{
  static float msgBME280[4];
  float temp = bme.readTemperature();
  float pressure = bme.readPressure();
  float elevation = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float humidity = bme.readHumidity();
  msgBME280[0] = temp;
  msgBME280[1] = pressure / 1000;
  msgBME280[2] = elevation;
  msgBME280[3] = humidity;
  return msgBME280;
}

String gpsData()
{
  GpsRmcData data = gpsModule.getGpsData(false);
  if (data.validData)
  {
    str = String(data.day) + "," +
          String(data.month) + "," +
          String(data.year) + "," +
          String(data.hour) + "," +
          String(data.minute) + "," +
          String(data.second) + "," +
          String(data.longitude) + "," +
          String(data.ns) + "," +
          String(data.latitude) + "," +
          String(data.ew) + "," +
          String(data.speed);
  }
  else
  {
    str = "N/A";
  }

  return str;
}

void transmit(String msg, int counter)
{
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.print(",");
  LoRa.print(counter);
  LoRa.endPacket();
}

void setup()
{
  Serial.begin(SerialMonbaudRate);
  bme.begin(BME280_ADDRESS);           // BME280 INIT
  IMU.begin();                         // BNO055 INIT
  IMU.setExtCrystalUse(BNOExtCrystal); // External Crystal IMU

  Serial1.begin(GPSSerialBaudRate, SERIAL_8N1, RX, TX); // GPS PIN DECLARATION
  gpsModule.begin(&Serial1);                            // GPS INIT

  LoRa.setPins(CS, RST, G0); // LORA PIN DECLARATION

  while (!LoRa.begin(TransmitterFrequency))
  {
    Serial.println("Establishing RF Connection...");
    delay(500);
  }
  Serial.println("RF Connection Established...");
  LoRa.setSyncWord(SyncWord); // SET SYNCWORD
  LoRa.setTxPower(SetTransmitterPower);
}

void loop()
{
  /*uint8_t system, gyroc, accel, magnetc = 0;
  IMU.getCalibration(&system, &gyroc, &accel, &magnetc);*/
  BNO055();
  /*
  if (gyroc > 1 && accel > 1 && magnetc > 1)
  {           // Espera a que todos los sensores del IMU estén calibrados (0 siendo el min y 3 el max)
    BNO055(); // Llama a la función del BNO055 para tirar los cálculos
  }
  else
  {
    Serial.println("Calibrando: ");
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyroc, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.println(magnetc, DEC);
  }*/
}
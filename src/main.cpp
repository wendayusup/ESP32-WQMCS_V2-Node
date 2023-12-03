// Author       : Wenda Yusup
// Project      : Water Quality Monitoring and Controlling Systems
// Code Version : 2.0
// Last Update  : 25/11/2023




/*---------LIBRARY--------*/
#include <Arduino.h>
#include <SPI.h>
#include <DHT.h>
#include <Wire.h>
#include <WiFi.h>
#include <RTClib.h>
#include <OneWire.h>
#include <WiFiManager.h>
#include <ModbusMaster.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>


/*---------OUTPUT--------*/
#define LED 12    // Pin LED 12
#define RELAY 14  // Pin RELAY 14
#define BUZZER 23 // Pin BUZZER 23


/*---------SENSOR--------*/
#define PH_PIN 32 // Pin Sensor PH4502C

#define DO_PIN 33 // Pin Sensor DO

#define DHTPIN 15 // Pin 15

#define TRIG 5 // Pin Sensor JSN_SR04T
#define ECHO 18

#define ORP_PIN 35    // pin 35
#define ArrayLenth 40 // times of collection
#define VREF 3300     // VREF (mv)
#define VREF1 3.00    // VREF (mv)
#define OFFSET -1113  // zero drift voltage
#define ADC_RES 4095  // ADC Resolution

#define DHTTYPE DHT22  // Tipe sensor DHT (DHT22)
#define ONE_WIRE_BUS 4 // Pin Sensor DS18B20
#define MSG_BUFFER_SIZE (50)

unsigned long awal = millis();

RTC_DS3231 rtc;
WiFiManager wm;
ModbusMaster node;
WiFiClient WqmcsClient;
DHT dht(DHTPIN, DHTTYPE);
OneWire onewire(ONE_WIRE_BUS);
PubSubClient client(WqmcsClient);
DallasTemperature ds18b20(&onewire);
LiquidCrystal_I2C lcd(0x27, 20, 4);
HardwareSerial SerialPort(1); // RX, TX

String project = "WQMCS";
String id = "WQ01";
String clientID = "WQMCS" + id;

const char *ssid = "Setting";
const char *password = "admin1234";

const char *mqtt_server = "103.187.146.142";
unsigned long lastMsg = millis();
unsigned long awalNO3 = millis();
unsigned long awalDO = millis();
unsigned long awalJSN = 0;
unsigned long awalDS = millis();
unsigned long awalPH = millis();
unsigned long awalORP = millis();
double orpValue;
double TeganganPh;
int Korp;
int orp;
int orpppm;

int i = 0;
int Secs = 0;
int sum = 0;
int motorValue = 0;
int LCD_Milis = 0;
int LCD_Count = 0;
long duration;
int distance;
int jarak = 0;
int value = 0;
int nilai_analog_PH;
int orpArray[ArrayLenth];
int orpArrayIndex = 0;
int intervalsensorNO3 = 500;
int intervalpublic = 1000;
int intervalsensorDO = 100;
int intervalsensorJSN = 250;
int intervalsensor4502C = 300;
int intervalsensorORP = 50;
int interval = 250;
bool readtemp = true;
bool readdistance = false;
bool readdo = false;
bool readpH = false;
bool readwatertemp = false;
bool readORP = false;
bool readno3 = false;

float Po = 0;
float PH_step;
float NO3Val = 0;
float PH_4 = 3.00;
float PH_7 = 2.40;
float humidity = 0;
float heatIndex = 0;
float temperature = 0;
String timeNow = "";
String dateNow = "";
String statusWiFi = "";
String statusMQTT = "";
String motorStatus = "";
int motorStatusWeb = 0;
// bool lockWiFi = false;
char msg[MSG_BUFFER_SIZE];
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//==================ORP================
double avergearray(int *arr, int number)
{
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0)
  {
    printf("Error number for the array to avraging!/n");
    return 0;
  }
  if (number < 5)
  { // less than 5, calculated directly statistics
    for (i = 0; i < number; i++)
    {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  }
  else
  {
    if (arr[0] < arr[1])
    {
      min = arr[0];
      max = arr[1];
    }
    else
    {
      min = arr[1];
      max = arr[0];
    }
    for (i = 2; i < number; i++)
    {
      if (arr[i] < min)
      {
        amount += min; // arr<min
        min = arr[i];
      }
      else
      {
        if (arr[i] > max)
        {
          amount += max; // arr>max
          max = arr[i];
        }
        else
        {
          amount += arr[i]; // min<=arr<=max
        }
      }
    }
    avg = (double)amount / (number - 2);
  }
  return avg;
}

//=================DO===================
#define TWO_POINT_CALIBRATION 0
#define READ_TEMP (25) // Current water temperature ℃, Or temperature sensor function
#define CAL1_V (1805)  // mv
#define CAL1_T (25)    // ℃
#define CAL2_V (1300)  // mv
#define CAL2_T (15)    // ℃
const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};
uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;
int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

//- - - - - - - - - NO3- - - - - - - - -
float hexToFloat(uint16_t val1, uint16_t val2)
{
  uint32_t regVal = (val2 << 16) | val1;
  float result;
  memcpy(&result, &regVal, sizeof(regVal));

  return result;
}

uint16_t val_reg0;
uint16_t val_reg1;

void lcdPrint(String text0, String text1, String text2, String text3)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(text0);
  lcd.setCursor(0, 1);
  lcd.print(text1);
  lcd.setCursor(0, 2);
  lcd.print(text2);
  lcd.setCursor(0, 3);
  lcd.print(text3);
}

// void setup_wifi()
//{
//   delay(10);
//   // We start by connecting to a WiFi network
//   Serial.println();
//   Serial.print("Connecting to ");
//   Serial.println(ssid);
//
//   WiFi.begin(ssid, password);
//
//   while (WiFi.status() != WL_CONNECTED)
//   {
//     delay(500);
//     Serial.print(".");
//   }
//
//   randomSeed(micros());
//
//   Serial.println("");
//   Serial.println("WiFi connected");
//   Serial.println("IP address: ");
//   Serial.println(WiFi.localIP());
// }

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientID = "WQMCS" + id;
    clientID += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientID.c_str()))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "Water Quality Monitoring and Controll Systems");
      // ... and resubscribe
      client.subscribe("inTopic");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void buzz(int delayTime, int repeat)
{
  for (int i = 0; i < repeat; i++)
  {
    tone(BUZZER, 2000);
    delay(delayTime);
    tone(BUZZER, 0);
    delay(delayTime);
  }
}

void sensorDO()
{
  Temperaturet = (uint8_t)READ_TEMP;
  ADC_Raw = analogRead(DO_PIN);
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;

  Serial.println("Sensor DO    : " + String(float(readDO(ADC_Voltage, Temperaturet)) / 1000));
  // Serial.println( "DO :" +  ADC_Raw);
}

void sensorJSNSR04T()
{

  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(33);
  digitalWrite(TRIG, LOW);

  duration = pulseIn(ECHO, HIGH);
  distance = (duration * 0.034 / 2);
  i++;
  sum += distance;
  if (i == 3)
  {
    // Serial.println("Nilai Rata-rata adalah : " + (String)((sum / 3) + 5));
    jarak = ((sum / 3) + 2);
    i = 0;
    sum = 0;
  }
  Serial.print("Sensor Jarak : ");
  Serial.print(jarak);
  Serial.println(" Cm");

  if (jarak <= 40)
  {
    digitalWrite(LED, HIGH);
    digitalWrite(RELAY, HIGH);
    digitalWrite(BUZZER, HIGH);
    delay(100);
    digitalWrite(BUZZER, LOW);
    delay(100);
    motorStatus = "Hidup";
    motorStatusWeb = 1;
    Serial.println(String() + "Solenoid     : " + motorStatus);
  }
  else
  {
    digitalWrite(LED, LOW);
    digitalWrite(RELAY, LOW);
    digitalWrite(BUZZER, LOW);
    motorStatus = "Mati";
    motorStatusWeb = 0;
    Serial.println(String() + "Solenoid     : " + motorStatus);
  }
}

void sensorDS18B20()
{
  ds18b20.requestTemperatures();
  Serial.print("Suhu Air     : ");
  Serial.print(ds18b20.getTempCByIndex(0));
  Serial.println(" .C ");
}

void sensorPH4502C()
{
  int nilai_analog_PH = analogRead(PH_PIN);
  // Serial.print("Nilai ADC Ph: ");
  // Serial.println(nilai_analog_PH);
  TeganganPh = 3.3 / 4095.0 * nilai_analog_PH;
  // Serial.print("Tegangan Ph: ");
  // Serial.println(TeganganPh, 3);

  PH_step = (PH_4 - PH_7) / 3;
  Po = 7.00 + (((PH_7 - TeganganPh) / PH_step) - 7.00);

  if (Po > 5)
  {
    Po += 0.5;
  }

  Serial.print("Sensor pH    : ");
  Serial.println(Po, 2);
}

void sensorORP()
{
  static unsigned long orpTimer = millis(); // analog sampling interval
  static unsigned long printTime = millis();
  if (millis() >= orpTimer)
  {
    orpTimer = millis() + 20;
    orpArray[orpArrayIndex++] = analogRead(ORP_PIN); // read an analog value every 20ms
    if (orpArrayIndex == ArrayLenth)
    {
      orpArrayIndex = 0;
    }
    orpValue = ((30 * (double)VREF1 * 1000) - (75 * avergearray(orpArray, ArrayLenth) * VREF1 * 1000 / 4095)) / 75 - OFFSET;
  }
  if (millis() >= printTime)
  {
    printTime = millis() + 800;
    orp = (int)orpValue;
    Korp = orp / 300;
    orpppm = orp / Korp;
    Serial.print("Sensor ORP   : ");
    Serial.print(orpppm);
    Serial.println(" ppm");
  }
}
void Dht22()
{
  float humidity = dht.readHumidity() - 36.50;
  float temperature = dht.readTemperature() - 1.8;
  if (isnan(humidity) || isnan(temperature))
  {
    Serial.println("Gagal membaca data dari sensor DHT22!");
    return;
  }
  float hic = dht.computeHeatIndex(temperature, humidity, false);

  Serial.print(F("Humidity    : "));
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print(F("Temperature : "));
  Serial.print(temperature);
  Serial.println(F(".C "));
  Serial.print(F("Heat Index  : "));
  Serial.print(hic);
  Serial.println(F(".C "));
  Serial.println("");
}

void NO3()
{
  uint8_t result;

  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readHoldingRegisters(1, 4);

  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    val_reg0 = node.getResponseBuffer(0);
    val_reg1 = node.getResponseBuffer(1);

    NO3Val = hexToFloat(val_reg0, val_reg1);

    Serial.print("Sensor NO3-  : ");
    Serial.println(NO3Val);
  }
  else
  {
    Serial.print("Sensor  NO3- : ");
    Serial.println("NaN");
    ;
  }
  delay(1000);
}

class Task
{
private:
  unsigned long previousMillis; // Waktu terakhir tugas dieksekusi
  unsigned long interval;       // Interval waktu antara eksekusi tugas
  void (*lcdUpdate)();          // Pointer ke fungsi tugas

public:
  Task(unsigned long interval, void (*lcdUpdateParam)())
  {
    this->interval = interval;
    this->lcdUpdate = lcdUpdateParam;
    previousMillis = 0;
  }

  // Fungsi untuk menjalankan tugas jika sudah waktunya
  void run()
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis;
      lcdUpdate();
    }
  }
};

Task dataUpdate(1500, []()
                {
                  DateTime now = rtc.now(); // get the current time
                  // = = = = = = = = = = = = = variable feed = = = = = = = = = = = = =

                  // sensorJSNSR04T();
                  // motorValue = digitalRead(RELAY);
                  // motorStatus = (motorValue == 0) ? "Mati" : "Hidup";

                  // NO3();

                  // = = = = = = = = = = = = = DHT11 = = = = = = = = = = = = =
                  humidity = dht.readHumidity();
                  temperature = dht.readTemperature();
                  heatIndex = dht.computeHeatIndex(temperature, humidity, false);
                  //  timeNow = String(now.hour(), DEC) + ":" + String(now.minute(), DEC) + ":" + String(now.second(), DEC);
                  //  dateNow = String(now.day(), DEC) + "/" + String(now.month(), DEC) + "/" + String(now.year(), DEC);
                });

Task lcdUpdate(1500, []()
               {
  // = = = = = = = = = = = = = LCD Print = = = = = = = = = = = = =
  // Start Milis for print LCD every 2000ms
  if (millis() - LCD_Milis >= 2200) {
    LCD_Milis = millis();
    if (LCD_Count == 0) {
      lcdPrint("   WATER QUALITY   ", "     MONITORING     ", "        AND         ", "CONTROLLING SYSTEMS");
      LCD_Count = 1;
    }
    else if (LCD_Count == 1) {
      lcdPrint("Waktu     :" + String (timeNow), "Tanggal   :" + String (dateNow), "Suhu Lingk:" + String(temperature - 1.8) + " C", "Humidity  :" + String(humidity - 36.50) + " %");
      LCD_Count = 2;
    }
    else if (LCD_Count == 2) {
      lcdPrint("O R P  :" + String ((int)orpppm) + "  ppm", "DO     :" + String(float(readDO(ADC_Voltage, Temperaturet)) / 1000) + " ppm", "NO3-   :" + String (NO3Val) + " ppm", "pH     :" + String() + (Po) + " pH");
      LCD_Count = 3;
    }
    else if (LCD_Count == 3) {
      lcdPrint("Suhu Air  :" + String(ds18b20.getTempCByIndex(0)) + " C", "Jarak Air :" + String(jarak) + " cm", "SOLENOID  :" + String(motorStatus), "");
      LCD_Count = 1;
    }
  } });

void setup()
{
  Serial.begin(9600);
  SerialPort.begin(9600, SERIAL_8N1, 27, 19);
  dht.begin();
  ds18b20.begin();
  node.begin(1, SerialPort);
  //  setup_wifi();
  client.setServer(mqtt_server, 1883);
  lcd.init();
  lcd.backlight();

  lcdPrint("   WATER QUALITY   ", "     MONITORING     ", "         AND        ", "CONTROLLING SYSTEMS");
  buzz(100, 2);
  buzz(500, 1);
  delay(3000);
  String wifiAP = "WQMCS/" + id;
  lcdPrint("     " + String(wifiAP.c_str()), " Masuk ke Jaringan", "        WiFi", "    192.168.4.1");

  bool res;
  res = wm.autoConnect(wifiAP.c_str());
  if (!res)
  {
    Serial.println("Failed to connect");
    lcdPrint("WiFi Connect", "Failed", "", "");
    ESP.restart();
  }
  else
  {
    // if you get here you have connected to the WiFi
    Serial.println("connected...yeey :)");
    lcdPrint("WiFi", "Connected", "", "");
    buzz(100, 4);
    buzz(700, 1);
  }

  if (!rtc.begin())
  {
    Serial.println("RTC module is NOT found");
    lcdPrint("     RTC Module", "    is NOT Found", "", "");
    Serial.flush();
    while (1)
      ;
  }

  pinMode(LED, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(PH_PIN, INPUT);

  digitalWrite(RELAY, LOW);
  digitalWrite(LED, LOW);
  digitalWrite(BUZZER, LOW);
}

void loop()
{
  unsigned long sekarang = millis();

  if (sekarang - awal >= 5000)
  {
    Dht22();
    // i += 1;
    // Serial.println(String() + "Data ke " + i);
  }

  if (millis() - awal >= 2000)
  {
    awal = millis();
    sensorJSNSR04T();
    sensorDO();
    sensorDS18B20();
    sensorORP();
    NO3();
  }

  if (millis() - awalJSN >= 180000)

  {
    awalJSN = millis();
    sensorPH4502C();
    // Serial.println(String() + "Data Ke" + i);
  }


  DateTime now = rtc.now();

  timeNow = String(now.hour(), DEC) + ":" + String(now.minute(), DEC) + ":" + String(now.second(), DEC);
  dateNow = String(now.day(), DEC) + "/" + String(now.month(), DEC) + "/" + String(now.year(), DEC);
  delay(1000);
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature))
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    lcdPrint("   Failed to Read", "  from DHT Sensor", "", "");
    return;
  }
  float hic = dht.computeHeatIndex(temperature, humidity, false);
  // unsigned long sekarang = millis();

  unsigned long now2 = millis();
  if (now2 - lastMsg > 1000)
  {
    ++value;
    Serial.println(msg);
    String PayLoad = "WQ01," + (String)(temperature - 1.8) + "," + (String)(humidity - 36.50) + "," +
                     (String)orpppm + "," + (String)(float(readDO(ADC_Voltage, Temperaturet)) / 1000) + "," +
                     (String)Po + "," + (String)ds18b20.getTempCByIndex(0) + "," + (String)(jarak) + "," +
                     (String)NO3Val + "," + (String)(motorStatusWeb) + ",#";
    client.publish("WQ01", PayLoad.c_str());
  }

  dataUpdate.run();
  lcdUpdate.run();
}
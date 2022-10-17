#if defined(ESP32)
#include <WiFi.h>
#include <FirebaseESP32.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#endif
//จอและไฟล์รูปภาพ
#include <SPI.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include "jpeg1.h"
#include "jpeg2.h"
#include "jpeg3.h"

//เล่นเสียง
#include "SoundData.h" //เสียงระวังข้างหน้า
#include "SoundData2.h" //เสียงโปรดช่วยเหลือ
#include "XT_DAC_Audio.h"
//หัวใจออกซิเจน
#include <time.h>
#include <Wire.h>
#include "MAX30105.h" //sparkfun MAX3010X library
#include "heartRate.h"
//อุณหภูมิ
#include <Adafruit_MLX90614.h>
//ระยะ
#include <Wire.h>
#include <VL53L0X.h>

//รหัสไวไฟ
String device = "device001";
#define SSID        "BANK"
#define PASSWORD    "10086010"
//IP ฐานข้อมูล
#define FIREBASE_HOST "esp1-146ec-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "TUnpCwP8iv2XcKfDIvW8Rnf3eIY9S3MPNrvdbDjT"
//จอแสดงผล
TFT_eSPI tft = TFT_eSPI();       // Invoke custom library
//หัวใจออกซิเจน
MAX30105 particleSensor;
//อุณหภูมิ
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
//ระยะ
VL53L0X sensor;
//หัวใจ/ออกซิเจน
double avered = 0; double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 50;//calculate SpO2 by this sampling interval 100
//ออกซิเจน
double ESpO2 = 95.0;//initial value of estimated SpO2
double FSpO2 = 0.7; //filter factor for estimated SpO2
double frate = 0.95; //low pass filter for IR/red LED value to eliminate AC component
#define TIMETOBOOT 3000 // wait for this time(msec) to output SpO2
#define SCALE 88.0 //adjust to display heart beat and SpO2 in the same scale
#define SAMPLING 5 //if you want to see heart beat more precisely , set SAMPLING to 1
#define FINGER_ON 3000 // if red signal is lower than this , it indicates your finger is not on the sensor
#define MINIMUM_SPO2 80.0
//เสียง
XT_Wav_Class ForceWithYou(Force);     // create an object of type XT_Wav_Class that is used by
XT_Wav_Class second_voiceWithYou(second_voice);
// the dac audio class (below), passing wav data as parameter.
XT_DAC_Audio_Class DacAudio(26, 0);   // Create the main player class object.
// Use GPIO 26, one of the 2 DAC pins and timer 0
//ลูปการทำงาน
TaskHandle_t Task0;

//หัวใจ/ออกซิเจน
const byte RATE_SIZE = 5; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
uint32_t ir, red , green;
#define USEFIFO
int oxygen;
int r = 0;

float temp;//อุณหภูมิ

int selec_dsp = 0;//เงื่อนไขการเช็คหน้าทำงาน

int phase;//ระยะ
int warn = 600;//ระยะเตือนเริ่มต้น

//เซนเซอร์ความเร่ง
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

//เวลาของฐานข้อมูล
unsigned long previousMillis = 0;
unsigned long previousMillisA = 0;
unsigned long previousMillisB = 0;
unsigned long previousMillisC = 0;
unsigned long previousMillism = 0;

//ฐานข้อมูล
FirebaseData firebaseData;

//ลำโพง
int buzzer = 2;
//การนับทำงาน
int h = 0;
int m = 0;
int sec = 0;

//เมนู
int button_menu = 35;
int upButton = 34;
int downButton = 33;
int selectButton = 32;
int menu = 1;//เมนูเลื่อน
int menu_n = 0;//ออกเมนู

//GPS
#include <TinyGPS++.h>
#include <HardwareSerial.h>
float latitude , longitude;
float latA = 0;
float lngA = 0;
String  lat_str , lng_str;
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
/**************************************************************************************************/
void setup()
{
  pinMode(buzzer, OUTPUT); //ลำโพงbuzzer
  pinMode(upButton, INPUT);//สวิตซ์กดขึ้น ขา 32
  pinMode(downButton, INPUT);//สวิตซ์กดลง ขา 35
  pinMode(selectButton, INPUT);//สวิตซ์กดตกลง ขา 34
  pinMode(button_menu, INPUT);//สวิตซ์กดเข้าเมนู  ขา 33

  Serial.begin(115200);
  //จอ TFT LCD 3.5
  tft.init();
  tft.setRotation(3);
  //ฟังก์ชั่นรูปแสดงบนจอ TFT
  tft.setSwapBytes(true);
  tft.pushImage(0, 0, 480, 343, welcome1);//แสดงหน้า welcome
  delay(3000);

  //ไวไฟ
  tft.fillScreen(tft.color24to16(0x6666FF));//จอหน้าไวไฟ
  WiFi.begin(SSID, PASSWORD);
  Serial.printf("WiFi connecting to %s\n",  SSID);
  //แสดงการเชื่อมต่อไวไฟ
  tft.setTextColor(TFT_RED);//สีข้อความ
  tft.setTextSize(2);//ขนาดข้อความ
  tft.setCursor(20, 20);
  tft.print("WiFi connecting to ");
  tft.setCursor(250, 20);
  tft.print(SSID);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(400);
    tft.fillRect(20, 50, 480, 20, tft.color24to16(0x6666FF));//ปิดข้อความ
    tft.setCursor(20, 50);//แสดงการเชื่อมต่อไวไฟ
    tft.print("connecting to wifi ...");
  }
  Serial.printf("\nWiFi connected\nIP : ");
  Serial.println(WiFi.localIP());
  tft.setCursor(20, 80); //แสดงการเชื่อมต่อไวไฟ
  tft.print("IP : ");
  tft.setTextColor(0x99FF33);  
  tft.setCursor(70, 80); //แสดงการเชื่อมต่อไวไฟ
  tft.print(WiFi.localIP());
  delay(6000);

  //หน้าพื้นหลังแสดงค่าบนจอ TFT LCD 
  tft.pushImage(0, 0, 480, 343, test_2);

  Serial.println(F("Initializing..."));//หัวใจออกซิเจน
  while (!particleSensor.begin(Wire, 100000)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
    delay(1000);
  }
  //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 0x7F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  particleSensor.enableDIETEMPRDY();

  mlx.begin();//อุณหภูมิ

  Wire.begin();//ระยะ
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  sensor.startContinuous();

  Wire.begin();//แกน
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);//ฐานข้อมูล
  Firebase.reconnectWiFi(true);

  //GPS
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

  xTaskCreatePinnedToCore(loop_T0, "Task0", 10000, NULL, 0, NULL, 0);
}
/**************************************************************************************************/
void loop_T0(void * parameter)
{
  while (1)
  {
    if (selec_dsp == 0)
    {
      tft.fillRect(0, 250, 40, 80, tft.color24to16(0xfff486));//ปิดค่าอุณหภูมิทีเกิน
      //แสดงค่า ระยะ
      tft.fillRect(200, 25, 80, 20, tft.color24to16(0xfff486));//ปิดข้อความ
      tft.setTextColor(TFT_RED);//สีข้อความ
      tft.setTextSize(3);//ขนาดข้อความ
      tft.setCursor(210, 25);
      tft.print(phase);
      if (phase <= warn + 3) //โชว์ข้อความการเตือนของระยะบนจอ
      {
        tft.fillRect(0, 60, 480, 95, tft.color24to16(0x76bcff));  //ปิดข้อความแจ้งเตือนตรงกลาง
        tft.setTextColor(tft.color24to16(0xFFFF00));
        tft.setTextSize(5); //ขนาดข้อความ
        tft.drawString("Be Careful", 100, 110);
      }
      else if (phase <= warn)
      {
        //ฐานข้อมูล
        Firebase.setString(firebaseData, "/" + device + "/" + "phase", String(phase));
      }
      else
      {
        tft.fillRect(0, 60, 480, 95, tft.color24to16(0x76bcff));  //ปิดข้อความแจ้งเตือนตรงกลาง
        tft.setTextColor(tft.color24to16(0xFFFF00));
        tft.setTextSize(5); //ขนาดข้อความ
        tft.drawString("BALANCE", 140, 120);
      }
      while (phase < warn + 1) //แจ้งเตือนเสียงระยะ
      {
        DacAudio.FillBuffer();                // Fill the sound buffer with data
        if (ForceWithYou.Playing == false)    // if not playing,
          DacAudio.Play(&ForceWithYou);       // play it, this will cause it to repeat and repeat...
      }

      //แสดงค่า อุณหภูมิ
      //ร่างกายปกติ >> 35.4 – 37.4 องศาเซลเซียส มีไข้ต่ำ >> 37.5 – 38.4 องศาเซลเซียส
      tft.fillRect(380, 250, 100, 20, tft.color24to16(0xfff486));
      tft.setTextColor(TFT_RED);//สีข้อความ
      tft.setTextSize(3);//ขนาดข้อความ
      tft.setCursor(380, 250);
      tft.print(temp);
      //ขึ้นฐานข้อมูล อุณหภูมิ
      if ( 34 < temp && temp < 37.4) {
        Firebase.setString(firebaseData, "/" + device + "/" + "temp", String(temp));
      } else {}

      while ( temp > 38.5)//อุณหภูมิสูง
      {
        digitalWrite(buzzer, HIGH);
      }

      //แสดงค่า แกน
      tft.fillRect(0, 20, 110, 40, tft.color24to16(0x76bcff));//ปิดให้ค่าลัน
      tft.setTextColor(tft.color24to16(0xFFFFFF));//สีข้อความ
      tft.setTextSize(1);//ขนาดข้อความ
      tft.drawString("ACC X: ", 10, 20);//ข้อความ
      tft.setCursor(45, 20);
      tft.print(AcX);
      tft.drawString("ACC Y: ", 10, 30);//ข้อความ
      tft.setCursor(45, 30);
      tft.print(AcY);
      tft.drawString("ACC Z: ", 10, 40);//ข้อความ
      tft.setCursor(45, 40);
      tft.print(AcZ);

      //เช็คการยืน/เดิน/แจ้งเตือน
      if (AcX < - 8000 || AcX > 8000) //การล้มข้างหน้าและหลัง
      {
        Serial.println("ACC X : fall in front or back");
        tft.fillRect(420, 2, 58, 60, tft.color24to16(0x76bcff));
        tft.pushImage(420, 2, 58, 60, fall); //รูปคนล้ม
        Firebase.setString(firebaseData, "/" + device + "/" + "acx", String(AcX));
        while (AcX < - 8000 || AcX > 8000) {
          //เสียงโปรดช่วยเหลือ
          DacAudio.FillBuffer();                // Fill the sound buffer with data
          if (second_voiceWithYou.Playing == false)    // if not playing,
            DacAudio.Play(&second_voiceWithYou);       // play it, this will cause it to repeat and repeat...
        }
      }
      else if (AcY < - 8000 || AcY > 8000)  //การล้มซ้ายและขวา
      {
        Serial.println("ACC Y : falling sideways");
        Firebase.setString(firebaseData, "/" + device + "/" + "acy", String(AcY));
        tft.pushImage(420, 2, 58, 60, fall); //รูปคนล้ม
        while (AcY < - 8000 || AcY > 8000) {
          //เสียงโปรดช่วยเหลือ
          DacAudio.FillBuffer();                // Fill the sound buffer with data
          if (second_voiceWithYou.Playing == false)    // if not playing,
            DacAudio.Play(&second_voiceWithYou);       // play it, this will cause it to repeat and repeat...

        }
      }
      else if (AcZ <  -8000) //การล้มแบบคว่ำหน้า
      {
        Serial.println("ACC Z : fall upside down");
        Firebase.setString(firebaseData, "/" + device + "/" + "acz", String(AcZ));
        tft.pushImage(420, 2, 58, 60, fall); //รูปคนล้ม
        while (AcZ <  -8000) {
          //เสียงโปรดช่วยเหลือ
          DacAudio.FillBuffer();                // Fill the sound buffer with data
          if (second_voiceWithYou.Playing == false)    // if not playing,
            DacAudio.Play(&second_voiceWithYou);       // play it, this will cause it to repeat and repeat...
        }
      }
      else //สมดุล
      { tft.fillRect(420, 2, 58, 60, tft.color24to16(0x76bcff));
        tft.pushImage(420, 2, 58, 60, stand);//รูปคนยืน
      }


      //นับเวลาการทำงาน
      unsigned long currentMillism = millis();
      if (currentMillism - previousMillism >= 1000) {
        sec++;
        if (sec >= 60) {
          m++;
          sec = 0;
        }
        if (m >= 60) {
          h++;
          m = 0;
        }
        if (h >= 24) {
          h = 0;
        }
        previousMillism = currentMillism;
      }
      tft.fillRect(0, 0, 120, 20, tft.color24to16(0x76bcff));//ปิดการนับเวลาทำงาน
      tft.setTextColor(tft.color24to16(0xFF0066));//สีข้อความ
      tft.setTextSize(1);//ขนาดข้อความ
      tft.drawString("Time: ", 10, 10);   //ข้อความ
      tft.setCursor(40, 10);
      tft.print(h);
      tft.drawString(" : ", 50, 10);      //ข้อความ
      tft.setCursor(60, 10);
      tft.print(m);
      tft.drawString(" : ", 70, 10);      //ข้อความ
      tft.setCursor(80, 10);
      tft.print(sec);


      //การเลื่อนของเมนู
      if (digitalRead(button_menu) == 1) {
        menu_n = 1;
        while (menu_n == 1) {
          updateMenu();
          if (digitalRead(downButton) == 1) {
            menu++;
            updateMenu();
            delay(100);
          }
          if (digitalRead(upButton) == 1) {
            menu--;
            updateMenu();
            delay(100);
          }
          if (digitalRead(selectButton) == 1) {
            executeAction();
            updateMenu();
            delay(100);
          }
          delay(500);//delay(1000);
        }
      }

      //ไม่มีการวางนิ้ววัดหัวใจ
      tft.setTextColor(TFT_RED);//สีข้อความ
      tft.setTextSize(2);//ขนาดข้อความ
      tft.setCursor(30, 230);
      tft.print("No finger");

      delay(100);
    }
    else if (selec_dsp == 1) //วัดและแสดงค่าหัวใจออกซิเจน
    {
      tft.fillRect(30, 230, 120, 20, tft.color24to16(0xfff486));//ไม่มีนิ้วมาวัดหัวใจ
      tft.fillRect(20, 250, 100, 30, tft.color24to16(0xfff486));//วัดหัวใจ
      tft.fillRect(20, 250, 20, 60, tft.color24to16(0xfff486));//นับเวลาหัวใจ
      tft.setTextColor(TFT_RED);//สีข้อความ
      tft.setTextSize(4.5);//ขนาดข้อความ
      tft.setCursor(60, 250);
      tft.print(beatAvg);
      //นับเวลา
      tft.setTextSize(2);//ขนาดข้อความ
      tft.setCursor(20, 250);
      tft.print(r);

      tft.fillRect(190, 250, 120, 30, tft.color24to16(0xfff486));
      tft.setTextSize(4.5);//ขนาดข้อความ
      tft.setCursor(200, 250);
      tft.print(oxygen);//oxygen

      /*      tft.fillRect(200, 25, 80, 20, tft.color24to16(0xfff486));//ปิดข้อความ
            tft.setTextSize(3);//ขนาดข้อความ
            tft.setCursor(210, 25);
            tft.print(phase);

            tft.fillRect(380, 250, 100, 20, tft.color24to16(0xfff486));
            tft.setTextSize(3);//ขนาดข้อความ
            tft.setCursor(380, 250);
            tft.print(temp);

            tft.fillRect(0, 20, 110, 40, tft.color24to16(0x76bcff));//ปิดให้ค่าลัน
            tft.setTextColor(tft.color24to16(0xFFFFFF));//สีข้อความ
            tft.setTextSize(1);//ขนาดข้อความ
            tft.drawString("ACC X: ", 10, 20);//ข้อความ
            tft.setCursor(45, 20);
            tft.print(AcX);
            tft.drawString("ACC Y: ", 10, 30);//ข้อความ
            tft.setCursor(45, 30);
            tft.print(AcY);
            tft.drawString("ACC Z: ", 10, 40);//ข้อความ
            tft.setCursor(45, 40);
            tft.print(AcZ);*/

      if (r >= 15)//ขึ้นฐานข้อมูล
      {
        Firebase.setString(firebaseData, "/" + device + "/" + "oxy", String(oxygen));
        Firebase.setString(firebaseData, "/" + device + "/" + "bpm", String(beatAvg));
        while (r > 16)//วางนิ้วครบ 14 วินาทีสั่งให้เอานิ้วออก
        {
          digitalWrite(buzzer, HIGH);
        }
      }
      delay(100);
    }
  }
}

/**************************************************************************************************/
void loop()
{
  read_bpm();
  read_spo2();
  read_phase();
  read_axg();
  read_temp();


  //การเช็ควางนิ้ว
  if (ir >= FINGER_ON)
  {
    selec_dsp = 1;
    Serial.println("r : " + String(r) + " ir : " + String(ir) + " BPM  : " + String(beatAvg) + " Oxygen : " + String(oxygen));
    Serial.println(" % Temp = " + String(temp) + " *C phase = " + String(phase));
    Serial.println("AcX : " + String(AcX) + " AcY : " + String(AcY) + " AcZ = : " + String(AcZ));
  }
  else
  {
    selec_dsp = 0;
    r = 0;
    digitalWrite(buzzer, LOW);
  }


  //gps
  if (SerialGPS.available() > 0)
  {
    if (gps.encode(SerialGPS.read()))
    {
      if (gps.location.isValid())
      {
        latitude = gps.location.lat();
        lat_str = String(latitude , 6);
        longitude = gps.location.lng();
        lng_str = String(longitude , 6);
        //Serial.print("Latitude/Longitude : ");
        //Serial.println(lat_str+" "+lng_str);
        lat_str = latA;
        lng_str = lngA;
      }
      //delay(1000);
    }
  }


  //ขึ้นฐานข้อมูล
  unsigned long currentMillis = millis();//10000
  if (currentMillis - previousMillis  >= 6000) {
    Firebase.setString(firebaseData, "/" + device + "/" + "phase", String(phase));
    previousMillis = currentMillis;
  }
  /* //ขึ้นฐานข้อมูล
    unsigned long currentMillisA = millis();
    if (currentMillisA - previousMillisA  >= 30000) {
     Firebase.setString(firebaseData, "/" + device + "/" + "temp", String(temp));
     previousMillisA = currentMillisA;
    }*/
  //ขึ้นฐานข้อมูล
  unsigned long currentMillisB = millis();
  if (currentMillisB - previousMillisB  >= 40000) {
    Firebase.setString(firebaseData, "/" + device + "/" + "lat_gps", String(latA));
    Firebase.setString(firebaseData, "/" + device + "/" + "lng_gps", String(lngA));
    previousMillisB = currentMillisB;
  }



}//LOOP
/**************************************************************************************************/
//อ่านค่า อัตราการเต้นหัวใจ
void read_bpm()
{
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
      r++;
    }
  }
}

//อ่านค่า อุณหภูมิ
void read_temp()
{
  temp = mlx.readObjectTempC();
}

//อ่านค่า ออกซิเจน
void read_spo2()
{

  double fred, fir;
  double SpO2 = 0; //raw SpO2 before low pass filtered

#ifdef USEFIFO
  particleSensor.check(); //Check the sensor, read up to 3 samples

  while (particleSensor.available())
  { //do we have new data
#ifdef MAX30105
    red = particleSensor.getFIFORed(); //Sparkfun's MAX30105
    ir = particleSensor.getFIFOIR();  //Sparkfun's MAX30105
#else
    red = particleSensor.getFIFOIR(); //why getFOFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
    ir = particleSensor.getFIFORed(); //why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
#endif

    i++;
    fred = (double)red;
    fir = (double)ir;
    avered = avered * frate + (double)red * (1.0 - frate);//average red level by low pass filter
    aveir = aveir * frate + (double)ir * (1.0 - frate); //average IR level by low pass filter
    sumredrms += (fred - avered) * (fred - avered); //square sum of alternate component of red level
    sumirrms += (fir - aveir) * (fir - aveir);//square sum of alternate component of IR level

    if (ir < FINGER_ON) ESpO2 = MINIMUM_SPO2; //indicator for finger detached

    if ((i % SAMPLING) == 0)
    { //slow down graph plotting speed for arduino Serial plotter by thin out
      if (millis() > TIMETOBOOT)
      {
        if (ir < FINGER_ON)
          ESpO2 = MINIMUM_SPO2; //indicator for finger detached
        //float temperature = particleSensor.readTemperatureF();
        if (ESpO2 <= -1)
        {
          ESpO2 = 0;
        }

        if (ESpO2 > 100)
        {
          ESpO2 = 100;
        }
        oxygen = ESpO2;//แปลงให้เป็นจำนวนเต็ม
      }
    }

    if ((i % Num) == 0)
    {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      // Serial.println(R);
      SpO2 = -23.3 * (R - 0.4) + 100;
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;//low pass filter
      if (ESpO2 >= 100) {
        ESpO2 = 100;
      }
      //  Serial.print(SpO2);Serial.print(",");Serial.println(ESpO2);
      sumredrms = 0.0; sumirrms = 0.0; i = 0;
      break;
    }
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }
#endif
}

//อ่านค่า ระยะ
void read_phase()
{
  phase = sensor.readRangeContinuousMillimeters();
}

//อ่านค่า แกน
void read_axg()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
}


//ส่วนของเมนู
void updateMenu() {
  switch (menu) {
    case 0:
      menu = 1;
      break;
    case 1:
      tft.fillRect(0, 50, 480, 140, tft.color24to16(0x76bcff));  //ปิดข้อความแจ้งเตือนตรงกลาง สี 76bcff
      tft.setTextSize(3);
      tft.setTextColor(TFT_RED);
      tft.setCursor(70, 110);
      tft.print("   Phase : 300 mm");
      tft.setCursor(70, 140);
      tft.print("   Phase : OFF");
      tft.setCursor(70, 170);
      tft.print("   Exit");
      tft.setTextColor(0x99FF33);
      tft.setCursor(70, 80);
      tft.print("> Phase : 1000 mm");
      break;
    case 2:
      tft.fillRect(0, 50, 480, 140, tft.color24to16(0x76bcff));  //ปิดข้อความแจ้งเตือนตรงกลาง
      tft.setTextSize(3);
      tft.setTextColor(TFT_RED);
      tft.setCursor(70, 80);
      tft.print("   Phase : 1000 mm");
      tft.setCursor(70, 140);
      tft.print("   Phase : OFF");
      tft.setCursor(70, 170);
      tft.print("   Exit");
      tft.setTextColor(0x99FF33);
      tft.setCursor(70, 110);
      tft.print("> Phase : 300 mm");
      break;
    case 3:
      tft.fillRect(0, 50, 480, 140, tft.color24to16(0x76bcff));  //ปิดข้อความแจ้งเตือนตรงกลาง
      tft.setTextSize(3);
      tft.setTextColor(TFT_RED);
      tft.setCursor(70, 80);
      tft.print("   Phase : 1000 mm");
      tft.setCursor(70, 110);
      tft.print("   Phase : 300 mm");
      tft.setCursor(70, 170);
      tft.print("   Exit");
      tft.setTextColor(0x99FF33);
      tft.setCursor(70, 140);
      tft.print("> Phase : OFF");
      break;
    case 4:
      tft.fillRect(0, 50, 480, 140, tft.color24to16(0x76bcff));  //ปิดข้อความแจ้งเตือนตรงกลาง
      tft.setTextSize(3);
      tft.setTextColor(TFT_RED);
      tft.setCursor(70, 80);
      tft.print("   Phase : 1000 mm");
      tft.setCursor(70, 110);
      tft.print("   Phase : 300 mm");
      tft.setCursor(70, 140);
      tft.print("   Phase : OFF");
      tft.setTextColor(0x99FF33);
      tft.setCursor(70, 170);
      tft.print("> Exit");
      break;
    case 5:
      menu = 5;
      break;
  }
}

void executeAction() {
  switch (menu) {
    case 1:
      action1();
      break;
    case 2:
      action2();
      break;
    case 3:
      action3();
      break;
    case 4:
      action4();
      break;
  }
}

void action1() {
  tft.fillRect(0, 50, 480, 140, tft.color24to16(0x76bcff));  //ปิดข้อความแจ้งเตือนตรงกลาง
  tft.setTextSize(4);
  tft.setTextColor(TFT_RED);
  tft.setCursor(110, 100);
  tft.print("> Executing #1 1000 mm");
  warn = 1000;
  delay(1500);
}
void action2() {
  tft.fillRect(0, 50, 480, 140, tft.color24to16(0x76bcff));  //ปิดข้อความแจ้งเตือนตรงกลาง
  tft.setTextSize(4);
  tft.setTextColor(TFT_RED);
  tft.setCursor(110, 100);
  tft.print("> Executing #2 300 mm");
  warn = 300;
  delay(1500);
}
void action3() {
  tft.fillRect(0, 50, 480, 140, tft.color24to16(0x76bcff));  //ปิดข้อความแจ้งเตือนตรงกลาง
  tft.setTextSize(4);
  tft.setTextColor(TFT_RED);
  tft.setCursor(110, 100);
  tft.print("> Executing #3 OFF");
  warn = 0;
  delay(1500);
}
void action4() {
  tft.fillRect(0, 50, 480, 140, tft.color24to16(0x76bcff));  //ปิดข้อความแจ้งเตือนตรงกลาง
  tft.setTextSize(4);
  tft.setTextColor(TFT_RED);
  tft.setCursor(150, 100);
  tft.print("> Exit <");
  delay(1500);
  menu_n = 0;
}

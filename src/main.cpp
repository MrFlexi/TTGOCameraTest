#include <Arduino.h>


#include <WiFi.h>
#include <Wire.h>
#include "esp_camera.h"
/***************************************
 *  Board select
 **************************************/

    #define T_Camera_PLUS_VERSION


#include "select_pins.h"

/***************************************
 *  Function
 **************************************/

//When there is BME280, set the reading time here
#define ENABLE_BEM280 1
#define DEFAULT_MEASUR_MILLIS   3000        /* Get sensor time by default (ms)*/

//When using timed sleep, set the sleep time here
#define uS_TO_S_FACTOR 1000000              /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5                    /* Time ESP32 will go to sleep (in seconds) */


/***************************************
 *  WiFi
 **************************************/
#define WIFI_SSID   "MrFlexi"
#define WIFI_PASSWD "Linde-123"

WiFiClient client;


/***************************************
 *  Netcup Server
 **************************************/
String serverName = "api.szaroletta.de";   // REPLACE WITH YOUR Raspberry Pi IP ADDRESS
String serverPath = "/upload";     // The default serverPath should be upload.php
const int serverPort = 5000;

String macAddress = "";
String ipAddress = "";

extern void startCameraServer();


#if defined(ENABLE_BEM280)
// Depend BME280 library ,See https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
#include "SparkFunBME280.h"
BME280 sensor;
String temp, pressure, altitude, humidity;
#endif


#if defined(ENABLE_TFT)
// Depend TFT_eSPI library ,See  https://github.com/Bodmer/TFT_eSPI
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();
#endif


String sendPhoto() {
  String getAll;
  String getBody;

  Serial.println("Smile.....");
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }
  
  Serial.println("Connecting to server: " + serverName);

  if (client.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");    
    String head = "--MrFlexi\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--MrFlexi--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;
  
    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=MrFlexi");
    client.println();
    client.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0; n<fbLen; n=n+1024) {
      if (n+1024 < fbLen) {
        client.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        client.write(fbBuf, remainder);
      }
    }   
    client.print(tail);
    
    esp_camera_fb_return(fb);
    
    int timoutTimer = 10000;
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + timoutTimer) > millis()) {
      Serial.print(".");
      delay(100);      
      while (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (getAll.length()==0) { state=true; }
          getAll = "";
        }
        else if (c != '\r') { getAll += String(c); }
        if (state==true) { getBody += String(c); }
        startTimer = millis();
      }
      if (getBody.length()>0) { break; }
    }
    Serial.println();
    client.stop();
    Serial.println(getBody);
  }
  else {
    getBody = "Connection to " + serverName +  " failed.";
    Serial.println(getBody);
  }
  return getBody;
}

bool setupSensor()
{
#if defined(ENABLE_BEM280)
    bool status = sensor.beginI2C();
    if (!status)return false;
    sensor.setMode(MODE_SLEEP); //Sleep for now
    sensor.setFilter(1); //0 to 4 is valid. Filter coefficient. See 3.4.4
    sensor.setStandbyTime(0); //0 to 7 valid. Time between readings. See table 27.

    sensor.setTempOverSample(1); //0 to 16 are valid. 0 disables temp sensing. See table 24.
    sensor.setPressureOverSample(1); //0 to 16 are valid. 0 disables pressure sensing. See table 23.
    sensor.setHumidityOverSample(1); //0 to 16 are valid. 0 disables humidity sensing. See table 19.

    sensor.setMode(MODE_SLEEP); //MODE_SLEEP, MODE_FORCED, MODE_NORMAL is valid. See 3.3
#endif
    return true;
}

void readSensor()
{
#if defined(ENABLE_BEM280)

    static long lastMillis;

    if (millis() - lastMillis < DEFAULT_MEASUR_MILLIS)return;
    sensor.setMode(MODE_FORCED); //Wake up sensor and take reading

    long startTime = millis();
    while (sensor.isMeasuring() == false) ; //Wait for sensor to start measurment
    while (sensor.isMeasuring() == true) ; //Hang out while sensor completes the reading
    long endTime = millis();

    //Sensor is now back asleep but we get get the data
    Serial.print(" Measure time(ms): ");
    Serial.print(endTime - startTime);

    Serial.print(" Humidity: ");
    humidity  = sensor.readFloatHumidity();
    Serial.print(humidity);

    Serial.print(" Pressure: ");
    pressure = sensor.readFloatPressure();
    Serial.print(pressure);

    Serial.print(" Alt: ");
    //Serial.print(sensor.readFloatAltitudeMeters(), 1);
    altitude = String(sensor.readFloatAltitudeFeet());
    Serial.print(altitude);

    Serial.print(" Temp: ");
    // Serial.print(sensor.readTempF(), 2);
    temp = String(sensor.readTempC());
    Serial.print(temp);

    Serial.println();

#endif  /*ENABLE_BEM280*/
}



bool deviceProbe(uint8_t addr)
{
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

bool setupDisplay()
{

#if defined(ENABLE_TFT)
#if defined(T_Camera_PLUS_VERSION)
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("TFT_eSPI", tft.width() / 2, tft.height() / 2);
    tft.drawString("LilyGo Camera Plus", tft.width() / 2, tft.height() / 2 + 20);
    pinMode(TFT_BL_PIN, OUTPUT);
    digitalWrite(TFT_BL_PIN, HIGH);
#endif

#endif
    return true;
}


void loopDisplay()
{
#if defined(SSD130_MODLE_TYPE)
    if (ui.update()) {
#endif  /*SSD130_MODLE_TYPE*/

#if defined(BUTTON_1)
        button.tick();
#endif  /*BUTTON_1*/

#if defined(SSD130_MODLE_TYPE)
    }
#elif defined(ENABLE_TFT)

        // ***

#endif  /*SSD130_MODLE_TYPE & ENABLE_TFT*/
}


bool setupPower()
{
#if defined(ENABLE_IP5306)
#define IP5306_ADDR 0X75
#define IP5306_REG_SYS_CTL0 0x00
    if (!deviceProbe(IP5306_ADDR))return false;
    bool en = true;
    Wire.beginTransmission(IP5306_ADDR);
    Wire.write(IP5306_REG_SYS_CTL0);
    if (en)
        Wire.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
    else
        Wire.write(0x35); // 0x37 is default reg value
    return Wire.endTransmission() == 0;

#endif


    return true;
}

#if defined(SDCARD_CS_PIN)
#include <SD.h>
#endif
bool setupSDCard()
{
    /*
        T-CameraPlus Board, SD shares the bus with the LCD screen.
        It does not need to be re-initialized after the screen is initialized.
        If the screen is not initialized, the initialization SPI bus needs to be turned on.
    */
    // SPI.begin(TFT_SCLK_PIN, TFT_MISO_PIN, TFT_MOSI_PIN);

#if defined(SDCARD_CS_PIN)
    if (!SD.begin(SDCARD_CS_PIN)) {
        tft.setTextColor(TFT_RED);
        tft.drawString("SDCard begin failed", tft.width() / 2, tft.height() / 2 - 20);
        tft.setTextColor(TFT_WHITE);
        return false;
    } else {
        String cardInfo = String(((uint32_t)SD.cardSize() / 1024 / 1024));
        tft.setTextColor(TFT_GREEN);
        tft.drawString("SDcardSize=[" + cardInfo + "]MB", tft.width() / 2, tft.height() / 2 + 92);
        tft.setTextColor(TFT_WHITE);

        Serial.print("SDcardSize=[");
        Serial.print(cardInfo);
        Serial.println("]MB");
    }
#endif
    return true;
}


bool setupCamera()
{
    camera_config_t config;

#if defined(Y2_GPIO_NUM)
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    //init with high specs to pre-allocate larger buffers
    if (psramFound()) {
        Serial.printf("psram found");
        config.frame_size = FRAMESIZE_SXGA;
        config.jpeg_quality = 10;
        config.fb_count = 2;
    } else {
        Serial.printf("NO psram found");
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }
#endif

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    //initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);//flip it back
        s->set_brightness(s, 1);//up the blightness just a bit
        s->set_saturation(s, -2);//lower the saturation
    }
    //drop down frame size for higher initial frame rate
    //s->set_framesize(s, FRAMESIZE_QVGA);

    return true;
}

void setupNetwork()
{
    macAddress = "LilyGo-CAM-";
    WiFi.begin(WIFI_SSID, WIFI_PASSWD);
    Serial.print("Wifi connecting...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    ipAddress = WiFi.localIP().toString();
    macAddress += WiFi.macAddress().substring(0, 5);


    tft.drawString("ipAddress:", tft.width() / 2, tft.height() / 2 + 50);
    tft.drawString(ipAddress, tft.width() / 2, tft.height() / 2 + 72);

}

void setupButton()
{

}



void setup()
{

    Serial.begin(115200);

#if defined(I2C_SDA) && defined(I2C_SCL)
    Wire.begin(I2C_SDA, I2C_SCL);
#endif

    bool status;
    status = setupDisplay();
    Serial.print("setupDisplay status "); Serial.println(status);

    status = setupSDCard();
    Serial.print("setupSDCard status "); Serial.println(status);

    status = setupPower();
    Serial.print("setupPower status "); Serial.println(status);

    status = setupSensor();
    Serial.print("setupSensor status "); Serial.println(status);

    status = setupCamera();
    Serial.print("setupCamera status "); Serial.println(status);
    if (!status) {
        delay(10000); esp_restart();
    }

    setupButton();

    setupNetwork();

    startCameraServer();

    Serial.print("Camera Ready! Use 'http://");
    Serial.print(ipAddress);
    Serial.println("' to connect");
    sendPhoto();
}

void loop()
{
    loopDisplay();
}

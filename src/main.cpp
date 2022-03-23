#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include "esp_camera.h"
#include "SPIFFS.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <TJpg_Decoder.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
/***************************************
 *  Board select
 *  goto Pio libs include 
 *  file user_setup_select.h   
 *          comment #include <User_Setup.h>  
 *          uncomment #include <User_Setups/Setup44_TTGO_CameraPlus.h> 
 **************************************/

#define T_Camera_PLUS_VERSION
#include "select_pins.h"

#define GFXFF 1
#define FSB9 &FreeSerifBold9pt7b

/***************************************
 *  Function
 **************************************/

#define DEFAULT_MEASUR_MILLIS 3000 /* Get sensor time by default (ms)*/

// When using timed sleep, set the sleep time here
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 5        /* Time ESP32 will go to sleep (in seconds) */

/***************************************
 *  WiFi
 **************************************/
#define WIFI_SSID "MrFlexi"
#define WIFI_PASSWD "Linde-123"

WiFiClient client;

/***************************************
 *  Netcup Server
 **************************************/
String serverName = "api.szaroletta.de";
String serverPath = "/add";
const int serverPort = 5000;

String macAddress = "";
String ipAddress = "";

extern void startCameraServer();

// Depend TFT_eSPI library ,See  https://github.com/Bodmer/TFT_eSPI
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();

void setup_filesystem()
{

    unsigned int totalBytes = SPIFFS.totalBytes();
    unsigned int usedBytes = SPIFFS.usedBytes();
    float freeKBytes = 0;

    //---------------------------------------------------------------
    // Mounting File System SPIFFS
    //---------------------------------------------------------------
    ESP_LOGI(TAG, "Mounting SPIFF Filesystem");
    // External File System Initialisation
    if (!SPIFFS.begin())
    {
        ESP_LOGE(TAG, "An Error has occurred while mounting SPIFFS");
        return;
    }
    File root = SPIFFS.open("/");
    File file = root.openNextFile();

    while (file)
    {
        ESP_LOGI(TAG, "%s", file.name());
        file = root.openNextFile();
    }

    freeKBytes = (totalBytes - usedBytes) / 1024;
    Serial.println("SPIFF File sistem info.");
    Serial.print("Total space: ");
    Serial.print(totalBytes / 1024);
    Serial.println(" KBytes");
    Serial.print("Free space: ");
    Serial.print(freeKBytes);
    Serial.println(" kBytes");
    Serial.println();
}

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap)
{
    // Stop further decoding as image is running off bottom of screen
    if (y >= tft.height())
        return 0;

    // This function will clip the image block rendering automatically at the TFT boundaries
    tft.pushImage(x, y, w, h, bitmap);

    // This might work instead if you adapt the sketch to use the Adafruit_GFX library
    // tft.drawRGBBitmap(x, y, bitmap, w, h);

    // Return 1 to decode next block
    return 1;
}

String sendPhoto()
{
    String getAll;
    String getBody;

    camera_fb_t *fb = NULL;
    fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("Camera capture failed");
    }

    if (client.connect(serverName.c_str(), serverPort))
    {
        //Serial.println("Connection successful!");
        String head = "--MrFlexi\r\nContent-Disposition: form-data; name=\"image\"; filename=\"image.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
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
        Serial.println("Start transfer");
        for (size_t n = 0; n < fbLen; n = n + 1024)
        {
            if (n + 1024 < fbLen)
            {
                client.write(fbBuf, 1024);
                fbBuf += 1024;
            }
            else if (fbLen % 1024 > 0)
            {
                size_t remainder = fbLen % 1024;
                client.write(fbBuf, remainder);
            }
        }
        //Serial.println("Tail");
        client.print(tail);

        esp_camera_fb_return(fb);

        int timoutTimer = 10000;
        long startTimer = millis();
        boolean state = false;

        Serial.println("Second");
        while ((startTimer + timoutTimer) > millis())
        {
            Serial.print(".");
            delay(50);
            while (client.available())
            {
                char c = client.read();
                if (c == '\n')
                {
                    if (getAll.length() == 0)
                    {
                        state = true;
                    }
                    getAll = "";
                }
                else if (c != '\r')
                {
                    getAll += String(c);
                }
                if (state == true)
                {
                    getBody += String(c);
                }
                startTimer = millis();
            }
            if (getBody.length() > 0)
            {
                break;
            }
        }
        client.stop();
        Serial.println(getBody);
    }
    else
    {
        getBody = "Connection to " + serverName + " failed.";
        Serial.println(getBody);
    }
    return getBody;
}

bool deviceProbe(uint8_t addr)
{
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

bool setupDisplay()
{
    tft.init();
    tft.setRotation(180);
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("TFT_eSPI", tft.width() / 2, tft.height() / 2);
    tft.drawString("LilyGo Camera Plus", tft.width() / 2, tft.height() / 2 + 20);
    tft.drawString("Push Foto to Netcup", tft.width() / 2, tft.height() / 2 + 30);
    pinMode(TFT_BL_PIN, OUTPUT);
    digitalWrite(TFT_BL_PIN, HIGH);
    return true;
}


bool setupPower()
{
#if defined(ENABLE_IP5306)
#define IP5306_ADDR 0X75
#define IP5306_REG_SYS_CTL0 0x00
    if (!deviceProbe(IP5306_ADDR))
        return false;
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

//  Camera functions
camera_fb_t *capture()
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    fb = esp_camera_fb_get();
    return fb;
}

void parsingResult(String response)
{
    Serial.println();
    Serial.println(response);
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, response);
    JsonArray array = doc.as<JsonArray>();
    int yPos = 4;
    for (JsonVariant v : array)
    {
        JsonObject object = v.as<JsonObject>();
        const char *description = object["description"];
        float score = object["score"];
        String label = "";
        label += description;
        label += ":";
        label += score;

        tft.drawString(label, 8, yPos);
        yPos += 16;
    }
}

void postImage(camera_fb_t *fb)
{
    HTTPClient client;
    client.begin("http://api.szaroletta.de:5000/add");
    client.addHeader("Content-Type", "image/jpeg");
    int httpResponseCode = client.POST(fb->buf, fb->len);
    if (httpResponseCode == 200)
    {
        String response = client.getString();
        parsingResult(response);
    }
    else
    {
        //Error
        tft.drawString("Check Your Server!!!", 8, 4);
        Serial.println("HTTP Post Image error");
        Serial.println(client.getString());
    }

    client.end();
}

bool setupSDCard()
{
    /*
        T-CameraPlus Board, SD shares the bus with the LCD screen.
        It does not need to be re-initialized after the screen is initialized.
        If the screen is not initialized, the initialization SPI bus needs to be turned on.
    */
    // SPI.begin(TFT_SCLK_PIN, TFT_MISO_PIN, TFT_MOSI_PIN);

#if defined(SDCARD_CS_PIN)
    if (!SD.begin(SDCARD_CS_PIN))
    {
        tft.setTextColor(TFT_RED);
        tft.drawString("SDCard begin failed", tft.width() / 2, tft.height() / 2 - 20);
        tft.setTextColor(TFT_WHITE);
        return false;
    }
    else
    {
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
    //config.pixel_format = PIXFORMAT_RGB565;
    config.pixel_format = PIXFORMAT_JPEG;
    // init with high specs to pre-allocate larger buffers
    if (psramFound())
    {
        Serial.println("Psram found");
        //config.frame_size = FRAMESIZE_VGA;
        config.frame_size = FRAMESIZE_240X240;
        config.jpeg_quality = 10;
        config.fb_count = 2;
    }
    else
    {
        Serial.println("NO psram found");
        //config.frame_size = FRAMESIZE_VGA;
        config.frame_size = FRAMESIZE_240X240;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }
#endif

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    

    s->set_hmirror(s,1);
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID)
    {
        Serial.println("Camera Sensor PID OV3660");
        s->set_vflip(s, 1);       // flip it back
        s->set_brightness(s, 1);  // up the blightness just a bit
        s->set_saturation(s, -2); // lower the saturation
    }
    // drop down frame size for higher initial frame rate
    // s->set_framesize(s, FRAMESIZE_QVGA);

    return true;
}

// https://github.com/0015/ThatProject/blob/master/ESP32CAM_Projects/ESP32_CAM_AICamera_Labelling/ESP32_CAM_ARDUINO/ESP32_CAM_ARDUINO.ino

void showImage()
{
    camera_fb_t *fb = capture();
    if (!fb || fb->format != PIXFORMAT_JPEG)
    {
        Serial.println("Camera capture failed");
        esp_camera_fb_return(fb);
        return;
    }
    else
    {
        TJpgDec.drawJpg(0, 0, (const uint8_t *)fb->buf, fb->len);
        esp_camera_fb_return(fb);
    }
}

void setupNetwork()
{
    macAddress = "LilyGo-CAM-";
    WiFi.begin(WIFI_SSID, WIFI_PASSWD);
    Serial.print("Wifi connecting...");
    while (WiFi.status() != WL_CONNECTED)
    {
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

void sendImage()
{
    camera_fb_t *fb = capture();
    if (!fb || fb->format != PIXFORMAT_JPEG)
    {
        Serial.println("Camera capture failed");
        esp_camera_fb_return(fb);
        return;
    }
    else
    {
        TJpgDec.drawJpg(0, 0, (const uint8_t *)fb->buf, fb->len);

        tft.drawString("Wifi Connecting!", 8, 4, GFXFF);

        if (WiFi.status() == WL_CONNECTED)
        {
            //tft.drawString("Wifi Connected!", 8, 4, GFXFF);
            TJpgDec.drawJpg(0, 0, (const uint8_t *)fb->buf, fb->len);
            postImage(fb);
        }
        else
        {
            tft.drawString("Check Wifi credential!", 8, 4);
        }
        esp_camera_fb_return(fb);
    }
}

void setupButton()
{
}

void setup()
{

#if defined(I2C_SDA) && defined(I2C_SCL)
    Wire.begin(I2C_SDA, I2C_SCL);
#endif

    TJpgDec.setJpgScale(1);
    TJpgDec.setSwapBytes(true);
    TJpgDec.setCallback(tft_output);

    Serial.begin(115200);

    setupNetwork();
    setup_filesystem();
    setupDisplay();
    setupSDCard();
    setupPower();
    setupCamera();
    
    //startCameraServer();
    //Serial.print("Camera Ready! Use 'http://");
    //Serial.print(ipAddress);
    //Serial.println("' to connect");
    // sendPhoto();
}

void loop()
{
    showImage();
    sendPhoto();
    //sendImage();
    delay(5000);
}

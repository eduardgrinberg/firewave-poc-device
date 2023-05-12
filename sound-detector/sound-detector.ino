#include <driver/i2s.h>
#include <WiFi.h>

#include "SD.h"
#include "SPI.h"

// Set these to your desired credentials.
const char* wifiSsid     = "Pixel_JeniaY";
const char* wifiPassword = "willbeok";

// Use WiFiClient class to create TCP connections
WiFiClient client;
const int destPort = 6000;
// const char* destIp = "3.78.56.42";
//const char* destIp = "192.168.0.114";
const char* destIp = "192.168.105.95";
const char* FILE_NAME = "/tmp.raw";

int ledPin = 13;

// M5Stcikc status led
#define LED_BUILTIN 10   // Set the GPIO pin where you connected your test LED or comment this line out if your dev board has a built-in LED
#define NUM_SAMPLES 256
#define BYTES_PER_SAMPLE 2
#define SAMPLE_RATE 44100
#define SEGMENT_LENGTH 4
#define PIN_CLK     33
#define PIN_DATA    32
#define READ_LEN    (BYTES_PER_SAMPLE * NUM_SAMPLES)


char BUFFER[READ_LEN] = {0};

void mic_record_task(void *arg) {
    size_t bytesread;
    size_t totalbytes;
    delete_file();
    while (1) {
      i2s_read(I2S_NUM_0, BUFFER, READ_LEN, &bytesread, (20 / portTICK_RATE_MS));
      // Serial.printf("%d Bytes read, total %d, total seconds %d \n", bytesread, totalbytes, bytes_to_sec(totalbytes));
      if(bytesread > 0){
        if(append_to_file(BUFFER, bytesread)) {
          totalbytes += bytesread;
        }
        if (bytes_to_sec(totalbytes) >= SEGMENT_LENGTH) {
          Serial.printf("File reached %d bytes and %d seconds\n", totalbytes, bytes_to_sec(totalbytes));
          send_file();
          delete_file();
          totalbytes = 0;
        }
      }
    }
}

bool append_to_file(const char* BUFFER, size_t len) {
  File file = SD.open(FILE_NAME, FILE_APPEND);

  if(!file){
      Serial.println("Failed to open file for appending");
      return false;
  }

  bool res = file.write((uint8_t*)BUFFER, len);
  file.close();

  return res;
}

void delete_file(){
  Serial.println("Deleting temp file");
  SD.remove(FILE_NAME);
}

void send_file(){
  Serial.println("Sending file to server.");
  static uint8_t buffer[1024];
  int bytesRead = 0;
  File file = SD.open(FILE_NAME);
  
  Serial.printf("File size %d \n", file.size());

  connectToServer(destIp, destPort);

  if (client.connected()) {
    Serial.println("Connected.");

    while(file.available()) {
      bytesRead = file.read(buffer, sizeof(buffer));
      client.write(buffer, bytesRead);
    } 
    file.close(); 
    Serial.println("File sent successfully.");
  }
  else {
    Serial.println("No connection to server.");
  }  
}

int bytes_to_sec(size_t bytes){
  return bytes / SAMPLE_RATE / BYTES_PER_SAMPLE ;
}

void i2sInit() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate = SAMPLE_RATE, //44100
        .bits_per_sample =
            I2S_BITS_PER_SAMPLE_16BIT,  // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 1, 0)
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
#else
        .communication_format = I2S_COMM_FORMAT_I2S,
#endif
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count    = 2,
        .dma_buf_len      = NUM_SAMPLES,//128
    };

    i2s_pin_config_t pin_config;

#if (ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 3, 0))
    pin_config.mck_io_num = I2S_PIN_NO_CHANGE;
#endif

    pin_config.bck_io_num   = I2S_PIN_NO_CHANGE;
    pin_config.ws_io_num    = PIN_CLK;
    pin_config.data_out_num = I2S_PIN_NO_CHANGE;
    pin_config.data_in_num  = PIN_DATA;

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO); //44100
}

void connectToServer(const char* ip, int port) {
  if (!client.connected()) {
    Serial.println("Connecting to server.");

    client.connect(ip, port);

    if (!client.connected()) {
      Serial.println("Failed to connect to server");
    }  
  } 
}

void connectToWiFi(const char* ssid, const char* password) {
  if (WiFi.status() != WL_CONNECTED){ 
    Serial.println("Connecting to WiFi.");

    WiFi.begin(ssid, password); 

    while (WiFi.status() != WL_CONNECTED){
      Serial.print(".");
      delay(1000);
    }

    Serial.println("\nWiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void mountSDCard(){
    Serial.println("Mounting SD Card");
    
    int res = SD.begin(4);

    if(!SD.begin(4)){
        Serial.println("Card Mount Failed");
    }

    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }
}

void setup() {
    Serial.begin(9600);
    delay(1000);

    mountSDCard();
   
    pinMode(ledPin, OUTPUT);

    i2sInit();
    xTaskCreate(mic_record_task, "mic_record_task", 4096, NULL, 1, NULL);
}

void loop() {
  // if (ifSending){
  //   digitalWrite(ledPin, !digitalRead(10));
  // }else if (ifConnected){
  //   digitalWrite(ledPin, LOW);
  // }else{
  //   digitalWrite(ledPin, HIGH);
  // }
  
  connectToWiFi(wifiSsid, wifiPassword);

  vTaskDelay(500 / portTICK_RATE_MS); 
}



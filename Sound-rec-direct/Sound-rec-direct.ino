#include <driver/i2s.h>
#include <WiFi.h>

// Set these to your desired credentials.
//const char* ssid     = "Yurkovsky";
//const char* password = "Varsha2908";
const char* ssid     = "EGrinberg";
const char* password = "00000000";
// const char* ssid     = "ABC";
// const char* password = "06041985";



// Use WiFiClient class to create TCP connections
WiFiClient client;
const int httpPort = 65432;

int ledPin = 13;
// send microphone sample to wifi
bool ifConnected = false;
bool ifSending = true;
bool buttonPressed = false;
// M5Stcikc status led
#define LED_BUILTIN 10   // Set the GPIO pin where you connected your test LED or comment this line out if your dev board has a built-in LED
#define NUM_SAMPLES 512
#define BYTES_PER_SAMPLE 4
#define SAMPLE_RATE 44100
#define PIN_CLK     33
#define PIN_DATA    32
#define READ_LEN    (BYTES_PER_SAMPLE * NUM_SAMPLES)


uint8_t BUFFER[READ_LEN] = {0};

void mic_record_task(void *arg) {
    size_t bytesread;
    while (1) {
        i2s_read(I2S_NUM_0, (char *)BUFFER, READ_LEN, &bytesread,(20 / portTICK_RATE_MS));
        if (bytesread!=READ_LEN){
          printf("i2s%d\n",int(bytesread));
        }
        if ((ifConnected) && (ifSending)){
          client.write(BUFFER,bytesread);
        }
    }
}

void i2sInit() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate = SAMPLE_RATE, //44100
        .bits_per_sample =
            I2S_BITS_PER_SAMPLE_32BIT,  // is fixed at 12bit, stereo, MSB
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
    i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO); //44100
}

void setup() {
    Serial.begin(9600);
    delay(10);

    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    
    pinMode(ledPin, OUTPUT);
    //pinMode(A0,INPUT);
    i2sInit();
    xTaskCreate(mic_record_task, "mic_record_task", 2048, NULL, 1, NULL);
}

void loop() {
  // M5.update();
  // if (M5.BtnA.wasPressed()){
  //   ifSending = !ifSending;
  // }
  if (ifSending){
    digitalWrite(ledPin, !digitalRead(10));
  }else if (ifConnected){
    digitalWrite(ledPin, LOW);
  }else{
    digitalWrite(ledPin, HIGH);
  }
    if (WiFi.status() != WL_CONNECTED){ //check connection to network
      ifConnected = false;
      Serial.println("Dropped from network, attempting to recconect");
      WiFi.begin(ssid, password); //connect to network

      // print the ip address
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
    }else if (!client.connected()) {
      // try to connect to the target ip
      if (!client.connect("172.20.10.4", httpPort)) {
        Serial.println("connection to client failed");
        ifConnected = false;
        // try to recconnect to network
        //WiFi.disconnect();
      }else{
        ifConnected = true;
      }   
    }
    int batlevel = analogRead(A0); 
    Serial.println(batlevel);
    vTaskDelay(500 / portTICK_RATE_MS);  // otherwise the main task wastes half                                    // of the cpu cycles
}

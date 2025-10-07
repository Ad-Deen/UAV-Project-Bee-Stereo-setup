#include "esp_camera.h"
#include "ESP32_OV5640_AF.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// ---------------- Wi-Fi & UDP ----------------
const char* ssid = "Hive";
const char* password = "12345678";
const char* receiverIP = "192.168.4.2";
const uint16_t receiverPort = 12346;  // change for camera2
// const uint16_t receiverPort = 12345;  // change for camera1
WiFiUDP udp;
const unsigned int localPort = 54320;

// ---------------- Camera config ----------------
#define PWDN_GPIO_NUM    38
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5
#define Y9_GPIO_NUM      16
#define Y8_GPIO_NUM      17
#define Y7_GPIO_NUM      18
#define Y6_GPIO_NUM      12
#define Y5_GPIO_NUM      10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM      11
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM    13

OV5640 ov5640 = OV5640();
const int MAX_UDP_PACKET_SIZE = 1400;  // safe chunk size 1024, max limit 1472 , MTU = 1500 for a single IP
uint8_t packet[6 + MAX_UDP_PACKET_SIZE];  // header + payload

// ---------------- FreeRTOS shared frame ----------------
camera_fb_t* shared_fb = nullptr;
SemaphoreHandle_t fbReady;
static uint16_t frame_id = 0;

// ---------------- Camera Task (Core 1) ----------------
void cameraTask(void* param) {
    while (true) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb && fb->format == PIXFORMAT_JPEG) {
            if (xSemaphoreTake(fbReady, portMAX_DELAY) == pdTRUE) {
                if (shared_fb) esp_camera_fb_return(shared_fb);  // free old frame
                shared_fb = fb;
                xSemaphoreGive(fbReady);
            }
        } else if (fb) {
            esp_camera_fb_return(fb);
        }
        delay(1);
    }
}

// ---------------- UDP Task (Core 0) ----------------
void udpTask(void* param) {
    while (true) {
        if (xSemaphoreTake(fbReady, portMAX_DELAY) == pdTRUE) {
            if (shared_fb) {
                int total_len = shared_fb->len;
                // Handling DMA overflow crashes gracefully --------------------------------------------
                // if (total_len > 13000) {
                //     Serial.printf("Frame too large (%d bytes), dropping\n", total_len);
                //     esp_camera_fb_return(shared_fb);
                //     shared_fb = nullptr;
                //     xSemaphoreGive(fbReady);
                //     continue; // skip sending this frame
                // }
            //--------------------------------------------------------------------------

                int chunks = (total_len + MAX_UDP_PACKET_SIZE - 1) / MAX_UDP_PACKET_SIZE;
                Serial.print("Image size ->");
                Serial.println(total_len);
                Serial.print(" bytes & Number of chunks ->");
                Serial.println(chunks);
                Serial.print(" bytes & frmaeID ->");
                Serial.println(frame_id);
                for (int i = 0; i < chunks; i++) {
                    int chunk_size = (i == chunks - 1) ? (total_len - i * MAX_UDP_PACKET_SIZE) : MAX_UDP_PACKET_SIZE;
                    packet[0] = (frame_id >> 8) & 0xFF;
                    packet[1] = frame_id & 0xFF;
                    packet[2] = (i >> 8) & 0xFF;
                    packet[3] = i & 0xFF;
                    packet[4] = (chunk_size >> 8) & 0xFF;
                    packet[5] = chunk_size & 0xFF;
                    memcpy(packet + 6, shared_fb->buf + i * MAX_UDP_PACKET_SIZE, chunk_size);

                    udp.beginPacket(receiverIP, receiverPort);
                    udp.write(packet, chunk_size + 6);
                    udp.endPacket();
                }

                esp_camera_fb_return(shared_fb);
                shared_fb = nullptr;
                if(frame_id < 5){
                    frame_id++;
                }else {
                    frame_id=0;
                }

            }
            xSemaphoreGive(fbReady);
        }
        delay(1); // small yield
    }
}

// ---------------- Setup ----------------
void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");

    fbReady = xSemaphoreCreateBinary();
    xSemaphoreGive(fbReady);

    // Camera init
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM; config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 20;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("Camera init failed!");
        return;
    }

    sensor_t* sensor = esp_camera_sensor_get();
    ov5640.start(sensor);
    // if (ov5640.focusInit() == 0) {
    //     Serial.println("OV5640_Focus_Init Successful!");
    // }

    // if (ov5640.autoFocusMode() == 0) {
    //     Serial.println("OV5640_Auto_Focus Successful!");
    // }

    udp.begin(localPort);

    // Create tasks
    xTaskCreatePinnedToCore(cameraTask, "CameraTask", 16384, NULL, 2, NULL, 1);  // Core 1
    xTaskCreatePinnedToCore(udpTask, "UdpTask", 16384, NULL, 1, NULL, 0);        // Core 0
}

void loop() {
    // Nothing needed here, tasks handle everything
}

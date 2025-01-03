#include <Arduino.h>
#include "esp_camera.h"
#include "camera.h"

camera_config_t cameraConfig;

void initConfig(camera_config_t& c){
    c.ledc_channel = LEDC_CHANNEL_0;
    c.ledc_timer = LEDC_TIMER_0;
    c.pin_d0 = Y2_GPIO_NUM;
    c.pin_d1 = Y3_GPIO_NUM;
    c.pin_d2 = Y4_GPIO_NUM;
    c.pin_d3 = Y5_GPIO_NUM;
    c.pin_d4 = Y6_GPIO_NUM;
    c.pin_d5 = Y7_GPIO_NUM;
    c.pin_d6 = Y8_GPIO_NUM;
    c.pin_d7 = Y9_GPIO_NUM;
    c.pin_xclk = XCLK_GPIO_NUM;
    c.pin_pclk = PCLK_GPIO_NUM;
    c.pin_vsync = VSYNC_GPIO_NUM;
    c.pin_href = HREF_GPIO_NUM;
    c.pin_sccb_sda = SIOD_GPIO_NUM;
    c.pin_sccb_scl = SIOC_GPIO_NUM;
    c.pin_pwdn = PWDN_GPIO_NUM;
    c.pin_reset = RESET_GPIO_NUM;
    c.xclk_freq_hz = 16500000;
    c.frame_size = FRAMESIZE_HVGA;//FRAMESIZE_VGA;
    c.pixel_format = PIXFORMAT_JPEG; // for streaming
    c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.jpeg_quality = 7;
    c.fb_count = 1;
}

void cameraSetup(){
    initConfig(cameraConfig);
    esp_err_t err = esp_camera_init(&cameraConfig);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t * s = esp_camera_sensor_get();
    s->set_vflip(s, 0);        //1-Upside down, 0-No operation
    s->set_hmirror(s, 0);      //1-Reverse left and right, 0-No operation
    s->set_brightness(s, 1);   //up the brightness just a bit
    s->set_saturation(s, -1);  //lower the saturation
}

bool cameraCapture(CAMERA_CAPTURE& cap){
    cap.jpgBuff=NULL;
    cap.jpgBuffLen=0;
    cap.fb = esp_camera_fb_get();
    
    if (!cap.fb) {
        Serial.print("Camera capture failed.");
        return false;
    }

    if(cap.fb->format != PIXFORMAT_JPEG){
        bool jpeg_converted = frame2jpg(cap.fb, 80, &cap.jpgBuff, &cap.jpgBuffLen);
        if(!jpeg_converted){
            Serial.print("JPEG compression failed");
            esp_camera_fb_return(cap.fb);
            cap.fb=NULL;
            return false;
        }
    } else {
        cap.jpgBuffLen = cap.fb->len;
        cap.jpgBuff = cap.fb->buf;
    }
    return true;
}

void cameraCaptureCleanup(CAMERA_CAPTURE& cap){
    if (cap.fb){
        if (cap.fb->format != PIXFORMAT_JPEG){
            free(cap.jpgBuff);
        }
        esp_camera_fb_return(cap.fb);
    }
    cap.jpgBuff=NULL;
    cap.jpgBuffLen=0;
    cap.fb=NULL;
}
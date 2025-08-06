# English Translation

English | [中文](./README_cn.md)

## M5_Camera_Examples

### Description

All examples in this library are developed based on the ESP-IDF platform, designed for communication between TimerCAM and host computers or data interaction with various cloud services. The demos in this library include the following functions: cloud storage using Ali-OSS, communication with AWS-related cloud services, real-time camera data transmission via HTTP, file transfer to host computers using SMB servers, wake-up function demonstration, and wireless transmission function demonstration.

TimerCAM is an ESP32-based camera module featuring an ESP32 chip with 8MB PSRAM onboard. It uses a 3-megapixel camera (OV3660) with a 66.5° field of view, capable of capturing photos at up to 1600 x 1200 resolution. With status indicators, it emphasizes ultra-low power design. Through the RTC (BM8563), it can achieve timed sleep and wake-up functions, reducing sleep current to as low as 2μA. The board includes a battery interface for users to connect external power supplies. The module supports WiFi image transmission and USB port debugging, with a bottom HY2.0-4P port for connecting other peripherals.

### IDF Version Support

- Adapted for ESP-IDF v5.0 and above

### Project Structure

```
├── common
│ ├── battery -> Battery output control and voltage monitoring
│ ├── bm8563 -> RTC time control and irq wakeup setting
│ ├── esp32-camera -> ESP32_Camera control
│ ├── i2c_manager -> I2C device manager
│ ├── libsmb2 -> SMB2 protocol library
│ └── m5stack_camera -> M5_Camera control
├── idf
│ ├── Ali-OSS -> Alibaba Cloud OSS service
│ ├── AWS-S3-PUT -> AWS S3 bucket service
│ ├── http-stream -> Data upload via HTTP protocol
│ ├── SMB-PUT -> Real-time photo file upload via SMB protocol
│ ├── Wake-up -> Device wake-up
│ └── wireless-send -> Wireless data transmission
```

A Python script has also been written to run on the server side to handle TCP responses in the wireless-send function.

### Product Details

For more information about product specifications and usage, please visit [this product documentation](https://docs.m5stack.com/en/unit/timercam).
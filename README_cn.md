## M5_Camera_Examples

[English](./README.md) | 中文

### 描述

本库中的全部案例都基于 ESP-IDF 平台开发，用于 TimerCAM 和上位机进行通讯或云端各种服务进行数据交互。该库中的 demo 有以下功能： 使用 Ali - OSS 进行数据的云存储，与 AWS 相关的云服务进行通讯，使用 http 进行摄像头数据的实时传递，用 SMB 服务器进行上位机的文件传输，唤醒功能演示，无线传输功能演示。

TimerCAM 是一款基于 ESP32 的摄像头模块，集成 ESP32 芯片，板载 8M PSRAM ，采用 300 万像素的摄像头（OV3660）可视角 66.5° ，最高可实现拍摄 1600 x 1200 分辨率的照片，带有状态指示灯，主打超低功耗设计，通过 RTC(BM8563) 可实现定时休眠与唤醒，休眠电流可降低至 2μA ，板上预留电池接口，用户可自行接入电池供电。模块支持WiFi图像传输和USB端口调试，底部HY2.0-4P端口输出，可连接其他外设。

### IDF 版本支持

- 已为 ESP-IDF v5.0 以上的版本进行了适配

### 项目结构

```
.
├── common
│   ├── battery -> Battery ouput control and voltage monitoring 
│   ├── bm8563 -> RTC time control and irq wakeup setting
│   ├── esp32-camera -> ESP32_Camera control
│   ├── i2c_manager -> I2C device manager
│   ├── libsmb2 -> SMB2 protocol library 
│   └── m5stack_camera -> M5_Camera control
├── idf
│   ├── Ali-OSS - > 阿里云 OSS 服务
│   ├── AWS-S3-PUT ->  AWS S3 存储桶服务
│   ├── http-stream -> 用 HTTP 协议上传数据
│   ├── SMB-PUT -> 用 SMB 协议实时上传照片文件
│   ├── Wake-up -> 唤醒设备
│   └── wireless-send -> 无线发送数据
```

同时编写了一个 python 脚本，可在服务器端运行，满足 wireless-send 中的 tcp 响应。

### 产品详情

如需要了解产品的各项参数或使用也可访问 [此产品文档](https://docs.m5stack.com/zh_CN/unit/timercam)。


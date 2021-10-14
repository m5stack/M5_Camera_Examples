## Wireless-send

Use M5Stack Fire as the receiver, the project is based on [ESP32_ScreenShotReceiver](https://github.com/lovyan03/ESP32_ScreenShotReceiver).

### Protocol reference
```C
byte[] rgbValues = { 0 };
ms.WriteByte(0x4A); // prefix "JPG" 3Byte
ms.WriteByte(0x50);
ms.WriteByte(0x47);
ms.WriteByte(0);    // data len 4Byte
ms.WriteByte(0);
ms.WriteByte(0);
ms.WriteByte(0);
bmp.Save(ms, _jpgEncoder, _encParams);
ms.Capacity = (int)ms.Length;
rgbValues = ms.GetBuffer();
{
    UInt32 len = (UInt32)(ms.Length - 7);
    rgbValues[3] = (byte)(len & 0xFF);
    rgbValues[4] = (byte)((len >> 8) & 0xFF);
    rgbValues[5] = (byte)((len >> 16) & 0xFF);
    rgbValues[6] = (byte)((len >> 24) & 0xFF);
    tcp.setData(rgbValues);
}
```

### Protocol code
```C
protocol_header[0] = 0x4a;
protocol_header[1] = 0x50;
protocol_header[2] = 0x47;
protocol_header[3] = _jpg_buf_len & 0xff;
protocol_header[4] = (_jpg_buf_len >> 8) & 0xff;;
protocol_header[5] = (_jpg_buf_len >> 16) & 0xff;;
protocol_header[6] = (_jpg_buf_len >> 24) & 0xff;;

err = send(sock, protocol_header, sizeof(protocol_header), 0);
if (err < 0) {
    ESP_LOGE(TAG, "Error occurred during sending protocol header errno %d", errno);
    break;
}

err = send(sock, _jpg_buf, _jpg_buf_len, 0);
if (err < 0) {
    ESP_LOGE(TAG, "Error occurred during sending image data errno %d", errno);
    break;
}
```
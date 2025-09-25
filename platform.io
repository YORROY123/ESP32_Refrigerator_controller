; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino

; 監控序列埠的鮑率，需與程式碼中的 Serial.begin(115200) 匹配
monitor_speed = 115200

; 函式庫相依性
lib_deps =
    ; Modbus Master 函式庫，直接從 GitHub 下載 (使用作者更新後的 URL)
    https://github.com/4-20ma/ModbusMaster.git

    ; 用於處理 JSON 資料的函式庫 (這是為了解決編譯錯誤而新增的)
    bblanchon/ArduinoJson@^6.0

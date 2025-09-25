#include <ModbusMaster.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- BLE UUIDs (與 index.html 相同) ---
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

// --- 全域變數 ---
int defogoffset = -8;
int defogofftemp = 2;
int targetTemp = -25;
int controlTemp = 4;
int eev_opening = 400;
int defrostInterval = 4 * 60 * 60 * 1000;
int defrostCompressorSpeed = 60;
int defrostT1 = 10 * 60 * 1000;
int DEFROST_HEATER = 18;
int defrostT2 = 3 * 60 * 1000;
int16_t eevOpening_set = 400;
float SHDMin = 0;
float SHDMax = 3;
int EEVStep = 10;
int EEVTimeDelay = 1;
int errorCode = 0;
HardwareSerial &ModbusSerial = Serial2;
ModbusMaster node;

struct ModbusData {
    int16_t count, systemMode, errorCode, indoorFanState, indoorFanSpeedCmd, indoorFanSpeedFeedback, outdoorFanState, outdoorFanSpeedCmd, outdoorFanSpeedFeedback, compressorState, compressorSpeedCmd, compressorSpeedFeedback, eevState, eevOpening, doorSensor1, doorSensor2, indoorFanRelay, outdoorFanRelay, defrostRelay1, defrostRelay2, defogRelay, lightRelay1, lightRelay2, fourWayValveRelay, faultRelay, upperControlStart, indoorFanMod, outdoorFanMod;
    float targetTemperature, ambientTemperature, storageTemperature1, storageTemperature2, returnAirTemperature, evapCoilTemperature, liquidLineTemperature, expansionValveOutTemperature, cpSuctionTemperature, SHD_temperature, cpDischargeTemperature, doorFrameTemperature, sht30Temperature, sht30Humidity, highPressure, lowPressure;
};
ModbusData modbusValues;
const uint16_t MODBUS_REGISTER_COUNT = 45;
unsigned long sysTimer, dataUpdateTimer, startTimer, controlTimer, heater_end, defrost_start, eevSetDelay, outdoorFanErrorTimer, jsonSendTimer;
uint8_t runState = 0;
bool firstRun = true, indoorFanOn = false, f_defrost = false;
int defState = 0;
bool outdoorFanErrorDetected = false, outdoorFanStopped = false;
int16_t lastOutdoorFanState = 0;
bool outdoorFanShouldRun = false;

// 函式原型
void handleBleCommand(const std::string& value);
void sendParamsAsJson();
void sendStatusAsJson();
void readAllModbusRegisters();
void writeModbusRegister(uint16_t address, uint16_t value);
void writeSystemMode(int16_t mode);
void writeTargetTemperature(int16_t temp);
void writeIndoorFanSpeedCmd(int16_t speed);
void writeOutdoorFanSpeedCmd(int16_t speed);
void writeCompressorSpeedCmd(int16_t speed);
void writeEevOpening(int16_t opening);
void writeDefrostRelay1(int16_t state);
void writeDefrostRelay2(int16_t state);
void writeDefogRelay(int16_t state);
void writeLightRelay1(int16_t state);
void writeLightRelay2(int16_t state);
void writeFourWayValveRelay(int16_t state);
void writeFaultRelay(int16_t state);
void writeUpperControlStart(int16_t state);
void writeIndoorFanMod(int16_t state);
void writeOutdoorFanMod(int16_t state);
void eevControl();
void defrostProcess();
void indoorFanControl();
void lightControl();
void defogControl();
void temperatureErrorControl();
void outdoorFanErrorControl();
void eevloop();


// BLE 伺服器回呼
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
        deviceConnected = true;
        Serial.println("裝置已連線");
    }
    void onDisconnect(BLEServer *pServer) {
        deviceConnected = false;
        Serial.println("裝置已斷線");
        BLEDevice::startAdvertising();
        Serial.println("重新開始廣播");
    }
};

// BLE 特徵回呼
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            Serial.print("收到來自 BLE 的指令: ");
            Serial.println(value.c_str());
            handleBleCommand(value);
        }
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("------------------------------------------");
    Serial.println("正在運行 v7 - 換行符最終修正版");
    Serial.println("------------------------------------------");

    BLEDevice::init("ESP32_Refrigerator");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    BLEDevice::startAdvertising();
    Serial.println("BLE 廣播已啟動");

    ModbusSerial.begin(9600, SERIAL_8N1, 16, 17);
    node.begin(1, ModbusSerial);
    Serial.println("Modbus Master initialized.");
    
    unsigned long initialMillis = millis();
    sysTimer = dataUpdateTimer = startTimer = controlTimer = defrost_start = outdoorFanErrorTimer = jsonSendTimer = initialMillis;
}

void loop() {
    unsigned long currentMillis = millis();
    if (errorCode == 0) {
        if (currentMillis - sysTimer > 1000) {
            sysTimer = currentMillis;
            switch (runState) {
                case 0:
                    if (firstRun) {
                        Serial.println("開機程序: 第一次運行");
                        eevOpening_set = eev_opening;
                        startTimer = currentMillis + 5 * 1000;
                        firstRun = false;
                    }
                    if (currentMillis >= startTimer) {
                        Serial.println("開機程序: 5秒結束，開始運行");
                        runState = 1;
                        writeCompressorSpeedCmd(75);
                        writeOutdoorFanSpeedCmd(1400);
                        outdoorFanShouldRun = true;
                        indoorFanOn = true;
                    }
                    break;
                case 1:
                    firstRun = true;
                    if (modbusValues.storageTemperature1 < targetTemp && modbusValues.compressorSpeedCmd != 40) {
                        writeCompressorSpeedCmd(40);
                    } else if (modbusValues.storageTemperature1 > (targetTemp + controlTemp) && modbusValues.compressorSpeedCmd != 75) {
                        writeCompressorSpeedCmd(75);
                    }
                    if (currentMillis - eevSetDelay > (unsigned long)EEVTimeDelay * 60 * 1000) {
                        eevControl();
                    }
                    break;
                case 2:
                    defrostProcess();
                    break;
            }
            if ((currentMillis - defrost_start) > (unsigned long)defrostInterval) {
                if(runState != 2) Serial.println("時間到達，進入除霜模式");
                runState = 2;
            }
        }
    } else {
         if (errorCode == 1) {
            errorCode = 2;
            writeCompressorSpeedCmd(0);
            writeOutdoorFanSpeedCmd(0);
            writeIndoorFanSpeedCmd(0);
            writeLightRelay1(0);
            writeLightRelay2(0);
            writeFourWayValveRelay(0);
            writeDefrostRelay1(0);
            writeDefrostRelay2(0);
            writeDefogRelay(0);
            writeFaultRelay(1);
            eevOpening_set = 0;
            firstRun = true;
            runState = 0;
            outdoorFanShouldRun = false;
        }
    }

    if (currentMillis - dataUpdateTimer > 1000) {
        dataUpdateTimer = currentMillis;
        readAllModbusRegisters();
        defogControl();
        lightControl();
        indoorFanControl();
        temperatureErrorControl();
        outdoorFanErrorControl();
    }

    if (deviceConnected && (currentMillis - jsonSendTimer > 2000)) {
        jsonSendTimer = currentMillis;
        sendParamsAsJson();
        delay(50); 
        sendStatusAsJson();
    }

    eevloop();
}

// ★★★ 核心修正：發送 JSON 後加上換行符 \n ★★★
void sendJsonWithDelimiter(const String& jsonString, const char* logPrefix) {
    if (jsonString.length() < 512) {
      String payload = jsonString + "\n"; // 加上換行符
      pCharacteristic->setValue(payload.c_str());
      pCharacteristic->notify();
      Serial.print(logPrefix);
      Serial.println(jsonString);
    } else {
      Serial.printf("%s JSON 過長，發送失敗！\n", logPrefix);
    }
}

void sendParamsAsJson() {
    StaticJsonDocument<512> doc;
    doc["type"] = "params";
    doc["defogoffset"] = defogoffset;
    doc["defogofftemp"] = defogofftemp;
    doc["targetTemp"] = targetTemp;
    doc["controlTemp"] = controlTemp;
    doc["eev_opening"] = eev_opening;
    doc["defrostInterval"] = defrostInterval / (60 * 1000);
    doc["defrostCompressorSpeed"] = defrostCompressorSpeed;
    doc["defrostT1"] = defrostT1 / (60 * 1000);
    doc["DEFROST_HEATER"] = DEFROST_HEATER;
    doc["defrostT2"] = defrostT2 / (60 * 1000);
    doc["SHDMin"] = SHDMin;
    doc["SHDMax"] = SHDMax;
    doc["EEVStep"] = EEVStep;
    
    String output;
    serializeJson(doc, output);
    sendJsonWithDelimiter(output, "發送 Params JSON (v7): ");
}

void sendStatusAsJson() {
    StaticJsonDocument<1024> doc;
    doc["type"] = "status";
    
    JsonObject modbus = doc.createNestedObject("modbusValues");
    modbus["storageTemperature1"] = modbusValues.storageTemperature1;
    modbus["storageTemperature2"] = modbusValues.storageTemperature2;
    modbus["evapCoilTemperature"] = modbusValues.evapCoilTemperature;
    modbus["SHD_temperature"] = modbusValues.SHD_temperature;
    modbus["compressorSpeedFeedback"] = modbusValues.compressorSpeedFeedback;
    modbus["eevOpening"] = modbusValues.eevOpening;
    modbus["returnAirTemperature"] = modbusValues.returnAirTemperature;
    modbus["cpDischargeTemperature"] = modbusValues.cpDischargeTemperature;
    modbus["highPressure"] = modbusValues.highPressure;
    modbus["lowPressure"] = modbusValues.lowPressure;
    modbus["systemMode"] = modbusValues.systemMode;
    modbus["errorCode"] = modbusValues.errorCode;

    String output;
    serializeJson(doc, output);
    sendJsonWithDelimiter(output, "發送 Status JSON (v7): ");
}

void handleBleCommand(const std::string& value) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, value);
    if (error) {
        Serial.print("JSON 解析失敗: ");
        Serial.println(error.c_str());
        return;
    }

    if (doc.containsKey("param")) {
        const char *param = doc["param"];
        float val = doc["value"];

        if (strcmp(param, "defogoffset") == 0) defogoffset = val;
        else if (strcmp(param, "defogofftemp") == 0) defogofftemp = val;
        else if (strcmp(param, "targetTemp") == 0) targetTemp = val;
        else if (strcmp(param, "controlTemp") == 0) controlTemp = val;
        else if (strcmp(param, "eev_opening") == 0) eev_opening = val;
        else if (strcmp(param, "defrostInterval") == 0) defrostInterval = val * 60 * 1000;
        else if (strcmp(param, "defrostCompressorSpeed") == 0) defrostCompressorSpeed = val;
        else if (strcmp(param, "defrostT1") == 0) defrostT1 = val * 60 * 1000;
        else if (strcmp(param, "DEFROST_HEATER") == 0) DEFROST_HEATER = val;
        else if (strcmp(param, "defrostT2") == 0) defrostT2 = val * 60 * 1000;
        else if (strcmp(param, "SHDMin") == 0) SHDMin = val;
        else if (strcmp(param, "SHDMax") == 0) SHDMax = val;
        else if (strcmp(param, "EEVStep") == 0) EEVStep = val;
        
        Serial.printf("參數 %s 已更新為 %.1f\n", param, val);

    } else if (doc.containsKey("command")) {
        const char *command = doc["command"];
        if (strcmp(command, "force_defrost") == 0) {
            defrost_start = millis() - defrostInterval;
            Serial.println("收到強制除霜指令");
        }
    }
}

void readAllModbusRegisters() {
  uint8_t result;
  result = node.readHoldingRegisters(0, MODBUS_REGISTER_COUNT);
  if (result == node.ku8MBSuccess) {
    modbusValues.count = (int16_t)node.getResponseBuffer(0);
    modbusValues.systemMode = (int16_t)node.getResponseBuffer(1);
    modbusValues.targetTemperature = (int16_t)node.getResponseBuffer(2) * 0.1;
    modbusValues.errorCode = (int16_t)node.getResponseBuffer(3);
    modbusValues.indoorFanState = (int16_t)node.getResponseBuffer(4);
    modbusValues.indoorFanSpeedCmd = (int16_t)node.getResponseBuffer(5);
    modbusValues.indoorFanSpeedFeedback = (int16_t)node.getResponseBuffer(6);
    modbusValues.outdoorFanState = (int16_t)node.getResponseBuffer(7);
    modbusValues.outdoorFanSpeedCmd = (int16_t)node.getResponseBuffer(8);
    modbusValues.outdoorFanSpeedFeedback = (int16_t)node.getResponseBuffer(9);
    modbusValues.compressorState = (int16_t)node.getResponseBuffer(10);
    modbusValues.compressorSpeedCmd = (int16_t)node.getResponseBuffer(11);
    modbusValues.compressorSpeedFeedback = (int16_t)node.getResponseBuffer(12);
    modbusValues.eevState = (int16_t)node.getResponseBuffer(13);
    modbusValues.eevOpening = (int16_t)node.getResponseBuffer(14);
    modbusValues.ambientTemperature = (int16_t)node.getResponseBuffer(16) * 0.1;
    modbusValues.storageTemperature1 = (int16_t)node.getResponseBuffer(17) * 0.1;
    modbusValues.storageTemperature2 = (int16_t)node.getResponseBuffer(18) * 0.1;
    modbusValues.returnAirTemperature = (int16_t)node.getResponseBuffer(19) * 0.1;
    modbusValues.evapCoilTemperature = (int16_t)node.getResponseBuffer(20) * 0.1;
    modbusValues.liquidLineTemperature = (int16_t)node.getResponseBuffer(21) * 0.1;
    modbusValues.expansionValveOutTemperature = (int16_t)node.getResponseBuffer(22) * 0.1;
    modbusValues.cpSuctionTemperature = (int16_t)node.getResponseBuffer(23) * 0.1;
    modbusValues.SHD_temperature = ((int16_t)node.getResponseBuffer(23) * 0.1) - ((int16_t)node.getResponseBuffer(22) * 0.1);
    modbusValues.cpDischargeTemperature = (int16_t)node.getResponseBuffer(24) * 0.1;
    modbusValues.doorFrameTemperature = (int16_t)node.getResponseBuffer(25) * 0.1;
    modbusValues.sht30Temperature = (int16_t)node.getResponseBuffer(26) * 0.1;
    modbusValues.sht30Humidity = (int16_t)node.getResponseBuffer(27) * 0.1;
    modbusValues.highPressure = (int16_t)node.getResponseBuffer(28) * 0.1;
    modbusValues.lowPressure = (int16_t)node.getResponseBuffer(29) * 0.1;
    modbusValues.doorSensor1 = (int16_t)node.getResponseBuffer(30);
    modbusValues.doorSensor2 = (int16_t)node.getResponseBuffer(31);
    modbusValues.indoorFanRelay = (int16_t)node.getResponseBuffer(32);
    modbusValues.outdoorFanRelay = (int16_t)node.getResponseBuffer(33);
    modbusValues.defrostRelay1 = (int16_t)node.getResponseBuffer(34);
    modbusValues.defrostRelay2 = (int16_t)node.getResponseBuffer(35);
    modbusValues.defogRelay = (int16_t)node.getResponseBuffer(36);
    modbusValues.lightRelay1 = (int16_t)node.getResponseBuffer(37);
    modbusValues.lightRelay2 = (int16_t)node.getResponseBuffer(38);
    modbusValues.fourWayValveRelay = (int16_t)node.getResponseBuffer(39);
    modbusValues.faultRelay = (int16_t)node.getResponseBuffer(40);
    modbusValues.upperControlStart = (int16_t)node.getResponseBuffer(42);
    modbusValues.indoorFanMod = (int16_t)node.getResponseBuffer(43);
    modbusValues.outdoorFanMod = (int16_t)node.getResponseBuffer(44);
  } else {
    Serial.printf("Modbus 讀取失敗: %02X\n", result);
  }
}

void writeModbusRegister(uint16_t address, uint16_t value) {
  uint8_t result;
  result = node.writeSingleRegister(address, value);
  if (result != node.ku8MBSuccess) {
    Serial.printf("寫入 Modbus 位址 %d 失敗: %02X\n", address, result);
  }
}

void writeSystemMode(int16_t mode) { writeModbusRegister(1, (uint16_t)mode); }
void writeTargetTemperature(int16_t temp) { writeModbusRegister(2, (uint16_t)temp); }
void writeIndoorFanSpeedCmd(int16_t speed) { writeModbusRegister(5, (uint16_t)speed); }
void writeOutdoorFanSpeedCmd(int16_t speed) { writeModbusRegister(8, (uint16_t)speed); }
void writeCompressorSpeedCmd(int16_t speed) { writeModbusRegister(11, (uint16_t)speed); }
void writeEevOpening(int16_t opening) { writeModbusRegister(14, (uint16_t)opening); }
void writeDefrostRelay1(int16_t state) { writeModbusRegister(34, (uint16_t)state); }
void writeDefrostRelay2(int16_t state) { writeModbusRegister(35, (uint16_t)state); }
void writeDefogRelay(int16_t state) { writeModbusRegister(36, (uint16_t)state); }
void writeLightRelay1(int16_t state) { writeModbusRegister(37, (uint16_t)state); }
void writeLightRelay2(int16_t state) { writeModbusRegister(38, (uint16_t)state); }
void writeFourWayValveRelay(int16_t state) { writeModbusRegister(39, (uint16_t)state); }
void writeFaultRelay(int16_t state) { writeModbusRegister(40, (uint16_t)state); }
void writeUpperControlStart(int16_t state) { writeModbusRegister(42, (uint16_t)state); }
void writeIndoorFanMod(int16_t state) { writeModbusRegister(43, (uint16_t)state); }
void writeOutdoorFanMod(int16_t state) { writeModbusRegister(44, (uint16_t)state); }

void eevloop() {
  if (eevOpening_set != modbusValues.eevOpening) {
    unsigned long currentMillis = millis();
    if (currentMillis - eevSetDelay > 20000) {
        eevSetDelay = currentMillis;
        writeEevOpening(eevOpening_set);
    }
  }
}

void eevControl() {
    if (modbusValues.SHD_temperature < SHDMin && eevOpening_set > 50) {
        eevOpening_set -= EEVStep;
        if (eevOpening_set < 50) eevOpening_set = 50;
    } else if (modbusValues.SHD_temperature > SHDMax && eevOpening_set < 500) {
        eevOpening_set += EEVStep;
        if (eevOpening_set > 500) eevOpening_set = 500;
    }
}

void defrostProcess() {
  if (!f_defrost) {
    defrost_start = millis();
    f_defrost = true;
    writeCompressorSpeedCmd(defrostCompressorSpeed);
    indoorFanOn = false;
    writeIndoorFanSpeedCmd(0);
    writeOutdoorFanSpeedCmd(0);
    outdoorFanShouldRun = false;
    writeFourWayValveRelay(1);
    writeDefrostRelay1(1);
    writeDefrostRelay2(1);
    writeEevOpening(0);
    eevOpening_set = 0;
    defState = 0;
  } else {
    unsigned long lasted = millis() - defrost_start;
    switch (defState) {
    case 0:
      if (lasted > (unsigned long)defrostT1) {
        defState = 2;
        heater_end = millis();
      }
      if (modbusValues.evapCoilTemperature > DEFROST_HEATER && (modbusValues.fourWayValveRelay != 0)) {
        writeCompressorSpeedCmd(0);
        writeDefrostRelay1(0);
        writeDefrostRelay2(0);
        writeFourWayValveRelay(0);
      }
      break;
    case 2:
      if (millis() - heater_end > (unsigned long)defrostT2) {
        f_defrost = false;
        runState = 0;
        defState = 0;
        firstRun = true;
        outdoorFanErrorDetected = false;
        outdoorFanStopped = false;
      }
    }
  }
}

void indoorFanControl() {
  if (indoorFanOn) {
    if (modbusValues.evapCoilTemperature < 0.0) {
      if (modbusValues.indoorFanSpeedCmd != 1500) {
        writeIndoorFanSpeedCmd(1500);
      }
    } else if (modbusValues.evapCoilTemperature > (modbusValues.storageTemperature1 - 2.0)) {
      if (modbusValues.indoorFanSpeedCmd != 0) {
        writeIndoorFanSpeedCmd(0);
      }
    }
  } else if (modbusValues.indoorFanSpeedCmd != 0) {
    writeIndoorFanSpeedCmd(0);
  }
}

void lightControl() {
  if (modbusValues.doorSensor1 == 0 || modbusValues.doorSensor2 == 0) {
    if (modbusValues.lightRelay1 != 1) {
      writeLightRelay1(1);
    }
  } else {
    if (modbusValues.lightRelay1 != 0) {
      writeLightRelay1(0);
    }
  }
}

void defogControl() {
  double T = modbusValues.sht30Temperature;
  double RH = modbusValues.sht30Humidity;
  if (RH > 0) {
    double Pws = exp(53.67957 - 6743.769 / (273.15 + T) - 4.845 * log(273.15 + T)) / 10;
    double Pw = Pws * RH / 100;
    if (Pw > 0) { // 再加一層保護，避免 log(0)
        double LN = log(Pw);
        double Tdew = 6.54 + 14.562 * LN + 0.7398 * pow(LN, 2) + 0.09486 * pow(LN, 3) + 0.4569 * pow(Pw, 0.1984);
        if ((modbusValues.doorFrameTemperature + defogoffset) >= (Tdew + defogofftemp)) {
          if (modbusValues.defogRelay != 0) {
            writeDefogRelay(0);
          }
        } else if ((modbusValues.doorFrameTemperature + defogoffset) <= Tdew) {
          if (modbusValues.defogRelay != 1) {
            writeDefogRelay(1);
          }
        }
    }
  }
}

void temperatureErrorControl() {
  if (modbusValues.cpDischargeTemperature > 80) {
    errorCode = 1;
  }
}

void outdoorFanErrorControl() {
  if (modbusValues.outdoorFanState == 16 && lastOutdoorFanState != 16 && outdoorFanShouldRun && !outdoorFanErrorDetected) {
    Serial.println("偵測到外風扇狀態16，開始錯誤處理程序");
    outdoorFanErrorDetected = true;
    outdoorFanStopped = false;
    outdoorFanErrorTimer = millis();
    writeOutdoorFanSpeedCmd(0);
  }

  lastOutdoorFanState = modbusValues.outdoorFanState;

  if (outdoorFanErrorDetected) {
    unsigned long elapsedTime = millis() - outdoorFanErrorTimer;
    if (!outdoorFanStopped && elapsedTime >= 10000) {
      Serial.println("外風扇錯誤處理：10秒後重新啟動");
      writeOutdoorFanSpeedCmd(1400);
      outdoorFanStopped = true;
      outdoorFanErrorTimer = millis();
    } else if (outdoorFanStopped && elapsedTime >= 10000) {
      Serial.println("外風扇錯誤處理完成");
      outdoorFanErrorDetected = false;
      outdoorFanStopped = false;
    }
  }
}


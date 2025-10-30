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

// --- BLE UUIDs ---
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic;
BLEServer *pServer = NULL;
BLEService *pService = NULL; 
BLEAdvertising *pAdvertising = NULL;
bool deviceConnected = false;
unsigned long lastBleActivity = 0;

// ★★★ 新增：用於安全重置 BLE 服務的非阻塞狀態機 ★★★
enum BleResetState {
    BLE_STATE_IDLE,
    BLE_STATE_STOP_ADV,
    BLE_STATE_REMOVE_SERVICE,
    BLE_STATE_CREATE_SERVICE,
    BLE_STATE_START_ADV
};
volatile BleResetState g_bleResetState = BLE_STATE_IDLE;
unsigned long g_bleResetTimer = 0;


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
bool modbusOk = false;
bool shouldSendParams = true;
HardwareSerial &ModbusSerial = Serial2;
ModbusMaster node;

struct ModbusData {
    int16_t count, systemMode, errorCode, indoorFanState, indoorFanSpeedCmd, indoorFanSpeedFeedback, outdoorFanState, outdoorFanSpeedCmd, outdoorFanSpeedFeedback, compressorState, compressorSpeedCmd, compressorSpeedFeedback, eevState, eevOpening, doorSensor1, doorSensor2, indoorFanRelay, outdoorFanRelay, defrostRelay1, defrostRelay2, defogRelay, lightRelay1, lightRelay2, fourWayValveRelay, faultRelay, upperControlStart, indoorFanMod, outdoorFanMod;
    float targetTemperature, ambientTemperature, storageTemperature1, storageTemperature2, returnAirTemperature, evapCoilTemperature, liquidLineTemperature, expansionValveOutTemperature, cpSuctionTemperature, SHD_temperature, cpDischargeTemperature, doorFrameTemperature, sht30Temperature, sht30Humidity, highPressure, lowPressure;
};
ModbusData modbusValues;
const uint16_t MODBUS_REGISTER_COUNT = 45;
unsigned long sysTimer, dataUpdateTimer, startTimer, controlTimer, heater_end, defrost_start, eevSetDelay, outdoorFanErrorTimer, jsonSendTimer;
unsigned long modbusRetryTimer;
uint8_t runState = 0;
bool firstRun = true, indoorFanOn = false, f_defrost = false;
int defState = 0;
bool outdoorFanErrorDetected = false, outdoorFanStopped = false;
int16_t lastOutdoorFanState = 0;
bool outdoorFanShouldRun = false;

// 函式原型
void create_ble_services(); 
void handleBleCommand(const std::string& value);
void sendParamsAsJson();
void sendStatusAsJson();
bool readAllModbusRegisters();
void writeModbusRegister(uint16_t address, uint16_t value);
void eevControl();
void defrostProcess();
void indoorFanControl();
void lightControl();
void defogControl();
void temperatureErrorControl();
void outdoorFanErrorControl();
void eevloop();
void writeCompressorSpeedCmd(int16_t speed);
void writeOutdoorFanSpeedCmd(int16_t speed);
void writeEevOpening(int16_t opening);
void writeFourWayValveRelay(int16_t state);
void writeDefrostRelay1(int16_t state);
void writeDefrostRelay2(int16_t state);
void writeIndoorFanSpeedCmd(int16_t speed);
void writeLightRelay1(int16_t state);
void writeDefogRelay(int16_t state);


// BLE 伺服器回呼
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        lastBleActivity = millis();
        Serial.println(">>> 藍牙裝置已連線 <<<");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        if (g_bleResetState == BLE_STATE_IDLE) {
            g_bleResetState = BLE_STATE_STOP_ADV;
        }
        Serial.println(">>> 藍牙裝置已斷線，將觸發 GATT 服務重建流程。 <<<");
    }
};

// BLE 特徵回呼
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        lastBleActivity = millis();
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            handleBleCommand(value);
        }
    }
};

void create_ble_services() {
    pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    Serial.println("GATT 服務已建立。");
}

void setup() {
    Serial.begin(115200);
    Serial.println("------------------------------------------");
    Serial.println("正在運行 v10.3 - 並行非阻塞GATT重建最終穩定版");
    Serial.println("------------------------------------------");

    BLEDevice::init("ESP32_Refrigerator");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    create_ble_services(); 
    
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("藍牙服務已初始化並開始廣播。");

    ModbusSerial.begin(9600, SERIAL_8N1, 16, 17);
    node.begin(1, ModbusSerial);
    Serial.println("Modbus Master initialized.");
    
    unsigned long initialMillis = millis();
    sysTimer = dataUpdateTimer = startTimer = controlTimer = defrost_start = outdoorFanErrorTimer = jsonSendTimer = modbusRetryTimer = initialMillis;
}

// ★★★ 新增：獨立的非阻塞 GATT 重建處理函式 ★★★
void handleGattReset() {
    unsigned long currentMillis = millis();
    switch(g_bleResetState) {
        case BLE_STATE_STOP_ADV:
            Serial.println("GATT 重建 1/4: 停止廣播...");
            pAdvertising->stop();
            g_bleResetTimer = currentMillis + 200; // 等待 200ms
            g_bleResetState = BLE_STATE_REMOVE_SERVICE;
            break;
        case BLE_STATE_REMOVE_SERVICE:
            if (currentMillis >= g_bleResetTimer) {
                Serial.println("GATT 重建 2/4: 移除舊服務...");
                pServer->removeService(pService);
                g_bleResetTimer = currentMillis + 100; // 等待 100ms
                g_bleResetState = BLE_STATE_CREATE_SERVICE;
            }
            break;
        case BLE_STATE_CREATE_SERVICE:
             if (currentMillis >= g_bleResetTimer) {
                Serial.println("GATT 重建 3/4: 建立新服務...");
                create_ble_services();
                g_bleResetTimer = currentMillis + 100; // 等待 100ms
                g_bleResetState = BLE_STATE_START_ADV;
            }
            break;
        case BLE_STATE_START_ADV:
            if (currentMillis >= g_bleResetTimer) {
                Serial.println("GATT 重建 4/4: 重啟廣播...");
                pAdvertising->start();
                g_bleResetState = BLE_STATE_IDLE; // 完成
                Serial.println("GATT 重建完成，系統恢復正常。");
            }
            break;
        default:
            break;
    }
}


void loop() {
    unsigned long currentMillis = millis();

    // ★★★ 核心修正：在迴圈頂部處理 BLE 重置，但不阻塞 ★★★
    if (g_bleResetState != BLE_STATE_IDLE) {
        handleGattReset();
    }
    
    // 心跳看門狗
    if (deviceConnected && (currentMillis - lastBleActivity > 20000)) {
        Serial.println("BLE 心跳超時，觸發 GATT 服務重建...");
        lastBleActivity = currentMillis;
        if (g_bleResetState == BLE_STATE_IDLE) {
            g_bleResetState = BLE_STATE_STOP_ADV;
        }
    }

    // ★★★ 核心修正：確保冰箱主邏輯在 BLE 重置期間也能繼續運行 ★★★
    if (g_bleResetState == BLE_STATE_IDLE) { // 只有在 BLE 非重置狀態下才輪詢 Modbus
        if (!modbusOk) {
            if (currentMillis - modbusRetryTimer > 10000) {
                 modbusRetryTimer = currentMillis;
                 Serial.println("正在嘗試重新連接 Modbus...");
                 modbusOk = readAllModbusRegisters();
            }
        } else {
            if (currentMillis - dataUpdateTimer > 1000) {
                dataUpdateTimer = currentMillis;
                if (!readAllModbusRegisters()) {
                    modbusOk = false;
                    modbusRetryTimer = currentMillis;
                }
            }
        }
    }


    // 只有在 Modbus 連線正常時，才執行設備控制邏輯
    if (modbusOk) {
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
        
        defogControl();
        lightControl();
        indoorFanControl();
        temperatureErrorControl();
        outdoorFanErrorControl();
    } 

    // BLE 資料傳輸
    if (deviceConnected && g_bleResetState == BLE_STATE_IDLE && (currentMillis - jsonSendTimer > 1000)) {
        jsonSendTimer = currentMillis;
        if (shouldSendParams) {
            sendParamsAsJson();
        } else {
            sendStatusAsJson();
        }
        shouldSendParams = !shouldSendParams;
    }

    // EEV 控制
    if (modbusOk) {
        eevloop();
    }
}

void sendJsonWithDelimiter(const String& jsonString, const char* logPrefix) {
    if (pServer->getConnectedCount() == 0) {
        return;
    }
    if (jsonString.length() < 180) {
      String payload = jsonString + "\n";
      pCharacteristic->setValue(payload.c_str());
      pCharacteristic->notify();
    } else {
      Serial.printf("%s JSON 過長 (%d bytes)，可能發送失敗！\n", logPrefix, jsonString.length());
    }
}

void sendParamsAsJson() {
    StaticJsonDocument<256> doc;
    doc["t"] = "p"; doc["do"] = defogoffset; doc["dot"] = defogofftemp;
    doc["tt"] = targetTemp; doc["ct"] = controlTemp; doc["eo"] = eev_opening;
    doc["di"] = defrostInterval / (60 * 1000); doc["dcs"] = defrostCompressorSpeed;
    doc["dt1"] = defrostT1 / (60 * 1000); doc["dh"] = DEFROST_HEATER;
    doc["dt2"] = defrostT2 / (60 * 1000); doc["shm"] = SHDMin;
    doc["shM"] = SHDMax; doc["es"] = EEVStep;
    String output;
    serializeJson(doc, output);
    sendJsonWithDelimiter(output, "發送 Params JSON: ");
}

void sendStatusAsJson() {
    StaticJsonDocument<384> doc;
    doc["t"] = "s"; 
    JsonObject mv = doc.createNestedObject("mv");
    mv["st1"] = modbusValues.storageTemperature1; mv["st2"] = modbusValues.storageTemperature2;
    mv["ect"] = modbusValues.evapCoilTemperature; mv["shd"] = modbusValues.SHD_temperature;
    mv["csf"] = modbusValues.compressorSpeedFeedback; mv["eev"] = modbusValues.eevOpening;
    mv["rat"] = modbusValues.returnAirTemperature; mv["cdt"] = modbusValues.cpDischargeTemperature;
    mv["hp"] = modbusValues.highPressure; mv["lp"] = modbusValues.lowPressure;
    mv["sm"] = modbusValues.systemMode;
    doc["mbc"] = modbusOk; doc["err"] = errorCode; 
    String output;
    serializeJson(doc, output);
    sendJsonWithDelimiter(output, "發送 Status JSON: ");
}

void handleBleCommand(const std::string& value) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, value);
    if (error) { Serial.print("JSON 解析失敗: "); Serial.println(error.c_str()); return; }
    
    if (doc.containsKey("param")) {
        Serial.print("收到參數更新指令: "); Serial.println(value.c_str());
        const char *param = doc["param"]; float val = doc["value"];
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
    } else if (doc.containsKey("cmd")) { 
        const char* cmd = doc["cmd"];
        if(strcmp(cmd, "ping") == 0) {
            // 收到心跳
        }
    } else if (doc.containsKey("command")) {
        Serial.print("收到功能指令: "); Serial.println(value.c_str());
        const char *command = doc["command"];
        if (strcmp(command, "force_defrost") == 0 && modbusOk) {
            defrost_start = millis() - defrostInterval;
            Serial.println("收到強制除霜指令");
        } 
    }
}

bool readAllModbusRegisters() {
  uint8_t result = node.readHoldingRegisters(0, MODBUS_REGISTER_COUNT);
  if (result == node.ku8MBSuccess) {
    if (!modbusOk) Serial.println("Modbus 通訊已恢復。");
    modbusValues.count = (int16_t)node.getResponseBuffer(0); modbusValues.systemMode = (int16_t)node.getResponseBuffer(1);
    modbusValues.targetTemperature = (int16_t)node.getResponseBuffer(2) * 0.1; modbusValues.errorCode = (int16_t)node.getResponseBuffer(3);
    modbusValues.indoorFanState = (int16_t)node.getResponseBuffer(4); modbusValues.indoorFanSpeedCmd = (int16_t)node.getResponseBuffer(5);
    modbusValues.indoorFanSpeedFeedback = (int16_t)node.getResponseBuffer(6); modbusValues.outdoorFanState = (int16_t)node.getResponseBuffer(7);
    modbusValues.outdoorFanSpeedCmd = (int16_t)node.getResponseBuffer(8); modbusValues.outdoorFanSpeedFeedback = (int16_t)node.getResponseBuffer(9);
    modbusValues.compressorState = (int16_t)node.getResponseBuffer(10); modbusValues.compressorSpeedCmd = (int16_t)node.getResponseBuffer(11);
    modbusValues.compressorSpeedFeedback = (int16_t)node.getResponseBuffer(12); modbusValues.eevState = (int16_t)node.getResponseBuffer(13);
    modbusValues.eevOpening = (int16_t)node.getResponseBuffer(14); modbusValues.ambientTemperature = (int16_t)node.getResponseBuffer(16) * 0.1;
    modbusValues.storageTemperature1 = (int16_t)node.getResponseBuffer(17) * 0.1; modbusValues.storageTemperature2 = (int16_t)node.getResponseBuffer(18) * 0.1;
    modbusValues.returnAirTemperature = (int16_t)node.getResponseBuffer(19) * 0.1; modbusValues.evapCoilTemperature = (int16_t)node.getResponseBuffer(20) * 0.1;
    modbusValues.liquidLineTemperature = (int16_t)node.getResponseBuffer(21) * 0.1; modbusValues.expansionValveOutTemperature = (int16_t)node.getResponseBuffer(22) * 0.1;
    modbusValues.cpSuctionTemperature = (int16_t)node.getResponseBuffer(23) * 0.1;
    modbusValues.SHD_temperature = ((int16_t)node.getResponseBuffer(23) * 0.1) - ((int16_t)node.getResponseBuffer(22) * 0.1);
    modbusValues.cpDischargeTemperature = (int16_t)node.getResponseBuffer(24) * 0.1; modbusValues.doorFrameTemperature = (int16_t)node.getResponseBuffer(25) * 0.1;
    modbusValues.sht30Temperature = (int16_t)node.getResponseBuffer(26) * 0.1; modbusValues.sht30Humidity = (int16_t)node.getResponseBuffer(27) * 0.1;
    modbusValues.highPressure = (int16_t)node.getResponseBuffer(28) * 0.1; modbusValues.lowPressure = (int16_t)node.getResponseBuffer(29) * 0.1;
    modbusValues.doorSensor1 = (int16_t)node.getResponseBuffer(30); modbusValues.doorSensor2 = (int16_t)node.getResponseBuffer(31);
    modbusValues.indoorFanRelay = (int16_t)node.getResponseBuffer(32); modbusValues.outdoorFanRelay = (int16_t)node.getResponseBuffer(33);
    modbusValues.defrostRelay1 = (int16_t)node.getResponseBuffer(34); modbusValues.defrostRelay2 = (int16_t)node.getResponseBuffer(35);
    modbusValues.defogRelay = (int16_t)node.getResponseBuffer(36); modbusValues.lightRelay1 = (int16_t)node.getResponseBuffer(37);
    modbusValues.lightRelay2 = (int16_t)node.getResponseBuffer(38); modbusValues.fourWayValveRelay = (int16_t)node.getResponseBuffer(39);
    modbusValues.faultRelay = (int16_t)node.getResponseBuffer(40); modbusValues.upperControlStart = (int16_t)node.getResponseBuffer(42);
    modbusValues.indoorFanMod = (int16_t)node.getResponseBuffer(43);
    modbusValues.outdoorFanMod = (int16_t)node.getResponseBuffer(44);
    return true;
  } else {
    if (modbusOk) Serial.printf("Modbus 讀取失敗: %02X。進入 unconnect 模式。\n", result);
    memset(&modbusValues, 0, sizeof(modbusValues));
    return false;
  }
}

void writeModbusRegister(uint16_t address, uint16_t value) {
  if (!modbusOk) return;
  Serial.printf("Modbus 寫入: 位址=%d, 值=%d ... ", address, value);
  uint8_t result = node.writeSingleRegister(address, value);
  if (result == node.ku8MBSuccess) { Serial.println("成功"); } 
  else { Serial.printf("失敗! Code: %02X\n", result); modbusOk = false; }
}

void writeCompressorSpeedCmd(int16_t speed) { writeModbusRegister(11, (uint16_t)speed); }
void writeOutdoorFanSpeedCmd(int16_t speed) { writeModbusRegister(8, (uint16_t)speed); }
void writeEevOpening(int16_t opening) { writeModbusRegister(14, (uint16_t)opening); }
void writeFourWayValveRelay(int16_t state) { writeModbusRegister(39, (uint16_t)state); }
void writeDefrostRelay1(int16_t state) { writeModbusRegister(34, (uint16_t)state); }
void writeDefrostRelay2(int16_t state) { writeModbusRegister(35, (uint16_t)state); }
void writeIndoorFanSpeedCmd(int16_t speed) { writeModbusRegister(5, (uint16_t)speed); }
void writeLightRelay1(int16_t state) { writeModbusRegister(37, (uint16_t)state); }
void writeDefogRelay(int16_t state) { writeModbusRegister(36, (uint16_t)state); }


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
        eevOpening_set -= EEVStep; if (eevOpening_set < 50) eevOpening_set = 50;
    } else if (modbusValues.SHD_temperature > SHDMax && eevOpening_set < 500) {
        eevOpening_set += EEVStep; if (eevOpening_set > 500) eevOpening_set = 500;
    }
}

void defrostProcess() {
  if (!f_defrost) {
    defrost_start = millis(); f_defrost = true; writeCompressorSpeedCmd(defrostCompressorSpeed);
    indoorFanOn = false; writeIndoorFanSpeedCmd(0); writeOutdoorFanSpeedCmd(0);
    outdoorFanShouldRun = false; writeFourWayValveRelay(1); writeDefrostRelay1(1);
    writeDefrostRelay2(1); writeEevOpening(0); eevOpening_set = 0; defState = 0;
  } else {
    unsigned long lasted = millis() - defrost_start;
    switch (defState) {
    case 0:
      if (lasted > (unsigned long)defrostT1) { defState = 2; heater_end = millis(); }
      if (modbusValues.evapCoilTemperature > DEFROST_HEATER && (modbusValues.fourWayValveRelay != 0)) {
        writeCompressorSpeedCmd(0); writeDefrostRelay1(0); writeDefrostRelay2(0); writeFourWayValveRelay(0);
      }
      break;
    case 2:
      if (millis() - heater_end > (unsigned long)defrostT2) {
        f_defrost = false; runState = 0; defState = 0; firstRun = true;
        outdoorFanErrorDetected = false; outdoorFanStopped = false;
      }
    }
  }
}

void indoorFanControl() {
  if (indoorFanOn) {
    if (modbusValues.evapCoilTemperature < 0.0) {
      if (modbusValues.indoorFanSpeedCmd != 1500) writeIndoorFanSpeedCmd(1500);
    } else if (modbusValues.evapCoilTemperature > (modbusValues.storageTemperature1 - 2.0)) {
      if (modbusValues.indoorFanSpeedCmd != 0) writeIndoorFanSpeedCmd(0);
    }
  } else if (modbusValues.indoorFanSpeedCmd != 0) { writeIndoorFanSpeedCmd(0); }
}

void lightControl() {
  if (modbusValues.doorSensor1 == 0 || modbusValues.doorSensor2 == 0) {
    if (modbusValues.lightRelay1 != 1) writeLightRelay1(1);
  } else {
    if (modbusValues.lightRelay1 != 0) writeLightRelay1(0);
  }
}

void defogControl() {
  double T = modbusValues.sht30Temperature; double RH = modbusValues.sht30Humidity;
  if (RH > 0) {
    double Pws = exp(53.67957 - 6743.769 / (273.15 + T) - 4.845 * log(273.15 + T)) / 10;
    double Pw = Pws * RH / 100;
    if (Pw > 0) {
        double LN = log(Pw);
        double Tdew = 6.54 + 14.562 * LN + 0.7398 * pow(LN, 2) + 0.09486 * pow(LN, 3) + 0.4569 * pow(Pw, 0.1984);
        if ((modbusValues.doorFrameTemperature + defogoffset) >= (Tdew + defogofftemp)) {
          if (modbusValues.defogRelay != 0) writeDefogRelay(0);
        } else if ((modbusValues.doorFrameTemperature + defogoffset) <= Tdew) {
          if (modbusValues.defogRelay != 1) writeDefogRelay(1);
        }
    }
  }
}

void temperatureErrorControl() {
  if (modbusValues.cpDischargeTemperature > 80) { if (errorCode == 0) errorCode = 1; }
}

void outdoorFanErrorControl() {
  if (modbusValues.outdoorFanState == 16 && lastOutdoorFanState != 16 && outdoorFanShouldRun && !outdoorFanErrorDetected) {
    Serial.println("偵測到外風扇狀態16，開始錯誤處理程序");
    outdoorFanErrorDetected = true; outdoorFanStopped = false; outdoorFanErrorTimer = millis();
    writeOutdoorFanSpeedCmd(0);
  }
  lastOutdoorFanState = modbusValues.outdoorFanState;
  if (outdoorFanErrorDetected) {
    unsigned long elapsedTime = millis() - outdoorFanErrorTimer;
    if (!outdoorFanStopped && elapsedTime >= 10000) {
      Serial.println("外風扇錯誤處理：10秒後重新啟動");
      writeOutdoorFanSpeedCmd(1400); outdoorFanStopped = true; outdoorFanErrorTimer = millis();
    } else if (outdoorFanStopped && elapsedTime >= 10000) {
      Serial.println("外風扇錯誤處理完成");
      outdoorFanErrorDetected = false; outdoorFanStopped = false;
    }
  }
}


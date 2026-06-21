#include "BLE.h"

BLE::BLE(){}

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      delay(500); 
      pServer->startAdvertising();
    }
};

void BLE::begin(){
    BLEDevice::init("TAG01"); // Nama perangkat BLE yang muncul saat di-scan di Android
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID,
                    BLECharacteristic::PROPERTY_READ   |
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                  
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // Membantu masalah koneksi pada iOS/Android
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
  }

void BLE::send(String data){
  if (deviceConnected) {
    pCharacteristic->setValue(data.c_str());
    pCharacteristic->notify(); // Kirim push notification data terbaru ke Android
  } 
}

bool BLE::connectionStatus(){
  return deviceConnected;
}
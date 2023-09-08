#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

BLEClient* pClient;
bool deviceConnected = false;
BLERemoteCharacteristic* pRemoteCharacteristicX;
BLERemoteCharacteristic* pRemoteCharacteristicY;
BLERemoteCharacteristic* pRemoteCharacteristicZ;

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* client) {
    deviceConnected = true;
  }
  void onDisconnect(BLEClient* client) {
    deviceConnected = false;
  }
};

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // No action needed for now
  }
};

bool connectToServer(BLEAddress pAddress) {
  //Serial.print("Connecting to ");
  //Serial.println(pAddress.toString().c_str());

  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  if (pClient->connect(pAddress)) {
    BLERemoteService* pRemoteService = pClient->getService("180D");
    if (pRemoteService != nullptr) {
      pRemoteCharacteristicX = pRemoteService->getCharacteristic("2A37");
      pRemoteCharacteristicY = pRemoteService->getCharacteristic("2A38");
      pRemoteCharacteristicZ = pRemoteService->getCharacteristic("2A39");
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  BLEDevice::init("");

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  BLEScanResults foundDevices = pBLEScan->start(5, false);

  //Serial.println("Scan done!");

  for (int i = 0; i < foundDevices.getCount(); i++) {
    BLEAdvertisedDevice device = foundDevices.getDevice(i);
    if(device.haveServiceUUID() && device.isAdvertisingService(BLEUUID("180D"))) {
      if(connectToServer(device.getAddress())) {
        //Serial.println("Connected to server");
        break; // Exit the loop once connected
      } else {
        //Serial.println("Failed to connect to server");
      }
    }
  }
}

void loop() {
  if (deviceConnected) {
    float accelX = pRemoteCharacteristicX->readFloat();
    float accelY = pRemoteCharacteristicY->readFloat();
    float accelZ = pRemoteCharacteristicZ->readFloat();
    //Serial.print("Accel X: "); Serial.println(accelX);
    //Serial.print("Accel Y: "); Serial.println(accelY);
    //Serial.print("Accel Z: "); Serial.println(accelZ);
    if (abs(accelX) > 2 || abs(accelY) > 2 || abs(accelZ) > 2) {
      Serial.write(1);  // Send '1' over Serial if any value exceeds 2
      //Serial.println("Impact detected");
    } else {
      Serial.write(0);  // Send '0' over Serial if no value exceeds 2
    }
  }
  else{
    
    BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  BLEScanResults foundDevices = pBLEScan->start(5, false);

  //Serial.println("Scan done!");

  for (int i = 0; i < foundDevices.getCount(); i++) {
    BLEAdvertisedDevice device = foundDevices.getDevice(i);
    if(device.haveServiceUUID() && device.isAdvertisingService(BLEUUID("180D"))) {
      if(connectToServer(device.getAddress())) {
        //Serial.println("Connected to server");
        break; // Exit the loop once connected
      } else {
        //Serial.println("Failed to connect to server");
      }
    }
  }
  }
  
  delay(10);
}

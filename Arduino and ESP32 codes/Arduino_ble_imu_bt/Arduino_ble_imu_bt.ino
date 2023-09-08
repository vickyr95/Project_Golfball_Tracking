#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>

BLEService imuService("180D");
BLEFloatCharacteristic accelXChar("2A37", BLERead | BLENotify);
BLEFloatCharacteristic accelYChar("2A38", BLERead | BLENotify);
BLEFloatCharacteristic accelZChar("2A39", BLERead | BLENotify);

void setup() {
  Serial.begin(9600);
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  BLE.setLocalName("ArduinoIMU");
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(accelXChar);
  imuService.addCharacteristic(accelYChar);
  imuService.addCharacteristic(accelZChar);
  BLE.addService(imuService);
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    while (central.connected()) {
      float x, y, z;
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);
        accelXChar.writeValue(x);
        accelYChar.writeValue(y);
        accelZChar.writeValue(z);
      }
      Serial.println(x);
      delay(100);
    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

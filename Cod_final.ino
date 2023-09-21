#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <math.h>
#include <SD.h>
#include <SPI.h>

BLEService imuService("917649A0-D98E-11E5-9EEC-0002A5D5C51B"); // Custom UUID

BLEByteCharacteristic commandCharacteristic("917649A2-D98E-11E5-9EEC-0002A5D5C51B", BLEWrite); // Custom UUID for the command characteristic

bool dataCollectionEnabled = false;
float ax, ay, az; // Aceleración
float gx, gy, gz; // Giroscopio
float mx, my, mz; // Magnetometro
float vx, vy, vz; // Velocidad
float roll, pitch, heading; // Angulos de orientación
float sumAx = 0.0, sumAy = 0.0, sumAz = 0.0;
int numSamples = 0;
float prevVx = 0.0, prevVy = 0.0, prevVz = 0.0;
float prevAx = 0.0, prevAy = 0.0, prevAz = 0.0;
float prevRoll = 0.0, prevPitch = 0.0, prevHeading = 0.0;
float meanAx = 0.0, meanAy = 0.0, meanAz = 0.0;
float magnitude_mean = 0.0;
float magnitude_vel = 0.0;
unsigned long startTime = 0;
float totalElapsedTime = 0.0;
int file_counter = 0;
const int chipSelect = 10;
unsigned long prevTime = 0; // Declaración de prevTime

File file;
File samplesFile;

void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("Hello world");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("ArduinoIMU");
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(commandCharacteristic);
  BLE.addService(imuService);

  commandCharacteristic.setEventHandler(BLEWritten, onCommandWritten);

  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");

  Serial.println("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
}

float normalizeAngle(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle <= -180.0) angle += 360.0;
  return angle;
}

void readAndSendSensorData() {

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    Serial.print("Aceleración: ");
    Serial.print("X = ");
    Serial.print(ax);
    Serial.print(" g, Y = ");
    Serial.print(ay);
    Serial.print(" g, Z = ");
    Serial.print(az);
    Serial.println(" g");

    Serial.print("Giroscopio: ");
    Serial.print("X = ");
    Serial.print(gx);
    Serial.print(" dps, Y = ");
    Serial.print(gy);
    Serial.print(" dps, Z = ");
    Serial.print(gz);
    Serial.println(" dps");

    Serial.print("Magnetómetro: ");
    Serial.print("X = ");
    Serial.print(mx);
    Serial.print(" uT, Y = ");
    Serial.print(my);
    Serial.print(" uT, Z = ");
    Serial.print(mz);
    Serial.println(" uT");

    roll = atan2(ay, az) * RAD_TO_DEG;
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    heading = atan2(-my, mx) * RAD_TO_DEG;

    float deltaTime = (millis() - prevTime) / 1000.0;

    roll = 0.98 * (prevRoll + gx * deltaTime) + 0.02 * roll;
    pitch = 0.98 * (prevPitch + gy * deltaTime) + 0.02 * pitch;
    heading = 0.98 * (prevHeading + gz * deltaTime) + 0.02 * heading;

    prevRoll = roll;
    prevPitch = pitch;
    prevHeading = heading;
    prevTime = millis();

    Serial.print("Roll: ");
    Serial.print(prevRoll);
    Serial.print(" degrees, Pitch: ");
    Serial.print(prevPitch);
    Serial.print(" degrees, Yaw: ");
    Serial.print(prevHeading);
    Serial.println(" degrees");

    if (dataCollectionEnabled) {
      sumAx += abs(ax);
      sumAy += abs(ay);
      sumAz += abs(az);
      numSamples++;
    }
  }
}

void turnOnRGBLED() {
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, LOW);
}

void turnOffRGBLED() {
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
}

void onCommandWritten(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t command = *characteristic.value();

  switch (command) {
    case 1:
      if (!dataCollectionEnabled) {
        Serial.println("Start data collection");
        dataCollectionEnabled = true;
        turnOnRGBLED();

        if (numSamples == 0) {
          startTime = millis();
          totalElapsedTime = 0.0;
        }

        create_new_file();
      }
      break;
    case 2:
      if (dataCollectionEnabled) {
        Serial.println("Stop data collection");
        dataCollectionEnabled = false;
        turnOffRGBLED();

        if (numSamples > 0) {
          meanAx = sumAx / numSamples;
          meanAy = sumAy / numSamples;
          meanAz = sumAz / numSamples;
          Serial.print("Aceleración: ");
          Serial.print("X = ");
          Serial.print(meanAx);
          Serial.print(" g, Y = ");
          Serial.print(meanAy);
          Serial.print(" g, Z = ");
          Serial.print(meanAz);
          Serial.println(" g");

          magnitude_mean = sqrt(meanAx * meanAx + meanAy * meanAy + meanAz * meanAz);
          Serial.print("Magnitud de la media de aceleración: ");
          Serial.print(magnitude_mean);
          Serial.print(" g,   ");
          Serial.print(magnitude_mean * 9.81);
          Serial.println(" m/s^2");

          unsigned long currentTime = millis();
          totalElapsedTime = (currentTime - startTime) / 1000.0;
          Serial.print("Tiempo: ");
          Serial.print(totalElapsedTime);
          Serial.println("Seg");

          vx = prevVx + 0.5 * ((meanAx + prevAx) * 9.81) * totalElapsedTime;
          vy = prevVy + 0.5 * ((meanAy + prevAy) * 9.81) * totalElapsedTime;
          vz = prevVz + 0.5 * ((meanAz + prevAz) * 9.81) * totalElapsedTime;

          Serial.print("Velocidad en X: ");
          Serial.print(vx);
          Serial.println(" m/s");

          Serial.print("Velocidad en Y: ");
          Serial.print(vy);
          Serial.println(" m/s");

          Serial.print("Velocidad en Z: ");
          Serial.print(vz);
          Serial.println(" m/s");

          magnitude_vel = sqrt(vx * vx + vy * vy + vz * vz);
          Serial.print("Magnitud de la media de la velocidad: ");
          Serial.print(magnitude_vel);
          Serial.println(" m/s");


          sumAx = 0.0;
          sumAy = 0.0;
          sumAz = 0.0;

          numSamples = 0;

          write_to_file();
          file.close();
        }
      }
      break;
    default:
      turnOffRGBLED();
      break;
  }
}

void create_new_file() {
  File root = SD.open("/");
  file_counter = 1;
  char name_string[13] = "1.csv";

  while (SD.exists(name_string)) {
    file_counter++;
    sprintf(name_string, "%d.csv", file_counter);
  }
  root.close();
  file = SD.open(name_string, FILE_WRITE);
  file.println("Ma X, Ma Y, Ma Z, Mag Acel, Tiempo, Vel x, Vel y, Vel z, Mag Vel");
}

void write_to_file() {
  file.print(meanAx);
  file.print(",");
  file.print(meanAy);
  file.print(",");
  file.print(meanAz);
  file.print(",");
  file.print(magnitude_mean);
  file.print(",");
  file.print(totalElapsedTime);
  file.print(",");
  file.print(vx);
  file.print(",");
  file.print(vy);
  file.print(",");
  file.print(vz);
  file.print(",");
  file.print(magnitude_vel);
  file.println("");
}


void write_to_samples_file() {
  samplesFile = SD.open("samples.csv", FILE_WRITE);
  if (samplesFile) {
    if (samplesFile && numSamples == 0) {
      samplesFile.print("ax"); samplesFile.print(";");
      samplesFile.print("ay"); samplesFile.print(";");
      samplesFile.print("az"); samplesFile.print(";");
      samplesFile.print("gx"); samplesFile.print(";");
      samplesFile.print("gy"); samplesFile.print(";");
      samplesFile.print("gz"); samplesFile.print(";");
      samplesFile.print("mx"); samplesFile.print(";");
      samplesFile.print("my"); samplesFile.print(";");
      samplesFile.print("mz"); samplesFile.print(";");
      samplesFile.print("roll"); samplesFile.print(";");
      samplesFile.print("pitch"); samplesFile.print(";");
      samplesFile.print("yaw"); samplesFile.print(";");
      samplesFile.println("time_ms");
    }
    samplesFile.print(ax); samplesFile.print(";");
    samplesFile.print(ay); samplesFile.print(";");
    samplesFile.print(az); samplesFile.print(";");
    samplesFile.print(gx); samplesFile.print(";");
    samplesFile.print(gy); samplesFile.print(";");
    samplesFile.print(gz); samplesFile.print(";");
    samplesFile.print(mx); samplesFile.print(";");
    samplesFile.print(my); samplesFile.print(";");
    samplesFile.print(mz); samplesFile.print(";");
    samplesFile.print(prevRoll); samplesFile.print(";");
    samplesFile.print(prevPitch); samplesFile.print(";");
    samplesFile.print(prevHeading); samplesFile.print(";");
    samplesFile.print(millis() - startTime);
    samplesFile.println("");
    samplesFile.close();
  } else {
    Serial.println("Error opening samples.csv");
  }
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
      if (dataCollectionEnabled) {
        readAndSendSensorData();
        if (dataCollectionEnabled) {
          write_to_samples_file();
        }
      }
    }

    digitalWrite(LED_BUILTIN, LOW);
    turnOffRGBLED();
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

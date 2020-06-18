#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>

#define MPU9250_ADDRESS 0x68
#define BMP280_ADDRESS 0x68

#define GYRO_FULL_SCALE_250_DPS 0x00
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2_G 0x00
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

Servo myservo;  // create servo object to control a servo

const int chipSelect = 10;

int pos = 0;  // variable to store the servo position
int direction = 0;

float acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;

float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float elapsedTime, currentTime, previousTime;
float roll, pitch, yaw;

int c = 0;
char counter_char[5];
int counter;
char counter_arr[10];
char filename[10];
Sd2Card card;
SdVolume volume;
SdFile root;
File myFile;

void stop(){
    while(1){

    }
}
void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the
  // accelerometer and gyro data error. From here we will get the error values
  // used in the above equations printed on the Serial Monitor. Note that we
  // should place the IMU flat in order to get the proper values, so that we
  // then can the correct values Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDRESS, 6, true);
    acc_x = (Wire.read() << 8 | Wire.read()) / 16384.0;
    acc_y = (Wire.read() << 8 | Wire.read()) / 16384.0;
    acc_z = (Wire.read() << 8 | Wire.read()) / 16384.0;
    // Sum all readings
    AccErrorX =
        AccErrorX +
        ((atan((acc_y) / sqrt(pow((acc_x), 2) + pow((acc_z), 2))) * 180 / PI));
    AccErrorY = AccErrorY +
                ((atan(-1 * (acc_x) / sqrt(pow((acc_y), 2) + pow((acc_z), 2))) *
                  180 / PI));
    c++;
  }
  // Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDRESS, 6, true);
    gyro_x = Wire.read() << 8 | Wire.read();
    gyro_y = Wire.read() << 8 | Wire.read();
    gyro_z = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (gyro_x / 131.0);
    GyroErrorY = GyroErrorY + (gyro_y / 131.0);
    GyroErrorZ = GyroErrorZ + (gyro_z / 131.0);
    c++;
  }
  // Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}

void setup() {
  myservo.attach(8);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);

  //   Serial.print("\nInitializing SD card...");

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  //   if (!card.init(SPI_HALF_SPEED, chipSelect)) {
  //     Serial.println("initialization failed. Things to check:");
  //     Serial.println("* is a card inserted?");
  //     Serial.println("* is your wiring correct?");
  //     Serial.println(
  //         "* did you change the chipSelect pin to match your shield or
  //         module?");
  //     while (1)
  //       ;
  //   } else {
  //     Serial.println("Wiring is correct and a card is present.");
  //   }

  //   // print the type of card
  // //   Serial.println();
  // //   Serial.print("Card type:         ");
  // //   switch (card.type()) {
  // //     case SD_CARD_TYPE_SD1:
  // //       Serial.println("SD1");
  // //       break;
  // //     case SD_CARD_TYPE_SD2:
  // //       Serial.println("SD2");
  // //       break;
  // //     case SD_CARD_TYPE_SDHC:
  // //       Serial.println("SDHC");
  // //       break;
  // //     default:
  // //       Serial.println("Unknown");
  // //   }

  //   // Now we will try to open the 'volume'/'partition' - it should be FAT16
  //   or
  //   // FAT32
  //   if (!volume.init(card)) {
  //     Serial.println(
  //         "Could not find FAT16/FAT32 partition.\nMake sure you've formatted
  //         the " "card");
  //     while (1)
  //       ;
  //   }

  Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.
  pinMode(10, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    stop();
  }
  Serial.println("initialization done.");

  // re-open the file for reading:
  myFile = SD.open("COUNTER.TXT");
  if (myFile) {
    Serial.print("Counter: ");

    // read from the file until there's nothing else in it:
    uint8_t i = 0;  // a counter

    while (myFile.available()) {
      counter_arr[i] = myFile.read();
      i++;
    }
    counter_arr[i] = '\0';
    counter = atoi(counter_arr);  // convert result string to numeric value
    sprintf(filename, "%03d.TXT", counter);
    Serial.println(counter);

    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening COUNTER.TXT");
    stop();
  }
  myFile = SD.open("COUNTER.TXT", FILE_WRITE | O_TRUNC);



  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.

      if (myFile) {
    Serial.print("Adding counter... ");
    myFile.println(counter+1);
    // close the file:
    myFile.close();
    Serial.println("Done!");
  } else {
    // if the file didn't open, print an error:
        Serial.println("error opening COUNTER.TXT");

  }

  myFile = SD.open(filename, FILE_WRITE | O_TRUNC | O_CREAT);
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing header... ");
    myFile.println("looptime,ax,ay,az,gx,gy,gz");
    // close the file:
    myFile.close();
    Serial.println("Done!");
  } else {
    // if the file didn't open, print an error:
    Serial.print("error opening ");
    Serial.println(filename);
  }

  //   Serial.print("Clusters:          ");
  //   Serial.println(volume.clusterCount());
  //   Serial.print("Blocks x Cluster:  ");
  //   Serial.println(volume.blocksPerCluster());

  //   Serial.print("Total Blocks:      ");
  //   Serial.println(volume.blocksPerCluster() * volume.clusterCount());
  //   Serial.println();

  //   // print the type and size of the first FAT-type volume
  //   uint32_t volumesize;
  //   Serial.print("Volume type is:    FAT");
  //   Serial.println(volume.fatType(), DEC);

  //   volumesize = volume.blocksPerCluster();  // clusters are collections of
  //   blocks volumesize *= volume.clusterCount();     // we'll have a lot of
  //   clusters volumesize /= 2;  // SD card blocks are always 512 bytes (2
  //   blocks are 1KB) Serial.print("Volume size (Kb):  ");
  //   Serial.println(volumesize);
  //   Serial.print("Volume size (Mb):  ");
  //   volumesize /= 1024;
  //   Serial.println(volumesize);
  //   Serial.print("Volume size (Gb):  ");
  //   Serial.println((float)volumesize / 1024.0);

  //   Serial.println("\nFiles found on the card (name, date and size in bytes):
  //   "); root.openRoot(volume);

  // list all files in the card with date and size
  //   root.ls(LS_R | LS_DATE | LS_SIZE);
  //   root.close();

  Wire.begin();  // join i2c bus (address optional for master)

  Wire.beginTransmission(
      MPU9250_ADDRESS);        // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);            // Talk to the register 6B
  Wire.write(0x00);            // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);  // end the transmission

  /*
 // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
 Wire.beginTransmission(MPU);
 Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
 Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g
 full scale range) Wire.endTransmission(true);
 // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
 Wire.beginTransmission(MPU);
 Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B
 hex) Wire.write(0x10);                   // Set the register bits as 00010000
 (1000deg/s full scale) Wire.endTransmission(true); delay(20);
 */

  calculate_IMU_error();
  delay(20);
}

void loop() {
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 6, true);

  // For a range of +-2g, we need to divide the raw values by 16384, according
  // to the datasheet
  acc_x = (Wire.read() << 8 | Wire.read()) / 16384.0;  // X-axis value
  acc_y = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Y-axis value
  acc_z = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Z-axis value

  accAngleX = (atan(acc_y / sqrt(pow(acc_x, 2) + pow(acc_z, 2))) * 180 / PI) -
              0.88;  // AccErrorX ~(0.58) See the calculate_IMU_error()custom
                     // function for more details
  accAngleY =
      (atan(-1 * acc_x / sqrt(pow(acc_y, 2) + pow(acc_z, 2))) * 180 / PI) +
      6.07;  // AccErrorY ~(-1.58)

  // === Read gyroscope data === //
  previousTime =
      currentTime;  // Previous time is stored before the actual time read
  currentTime = millis();  // Current time actual time read
  elapsedTime =
      (currentTime - previousTime) / 1000;  // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x43);  // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 6,
                   true);  // Read 4 registers total, each axis value is stored
                           // in 2 registers
  gyro_x = (Wire.read() << 8 | Wire.read()) /
           131.0;  // For a 250deg/s range we have to divide first the raw value
                   // by 131.0, according to the datasheet
  gyro_y = (Wire.read() << 8 | Wire.read()) / 131.0;
  gyro_z = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  gyro_x = gyro_x + 1.92;  // GyroErrorX ~(-0.56)
  gyro_y = gyro_y + 2.30;  // GyroErrorY ~(2)
  gyro_z = gyro_z - 1.06;  // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to
  // multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + gyro_x * elapsedTime;  // deg/s * s = deg
  gyroAngleY = gyroAngleY + gyro_y * elapsedTime;
  yaw = yaw + gyro_z * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  // Print the values on the serial monitor
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);

  if (pos >= 180) {
    direction = 1;
  }
  if (pos <= 0) {
    direction = 0;
  }

  if (direction == 0) {
    pos++;
  } else {
    pos--;
  }
  myservo.write(pitch);

  myFile = SD.open(filename, FILE_WRITE);

  myFile.print(currentTime);
  myFile.print(",");
  myFile.print(acc_x);
  myFile.print(",");
  myFile.print(acc_y);
  myFile.print(",");
  myFile.print(acc_z);
  myFile.print(",");
  myFile.print(gyro_x);
  myFile.print(",");
  myFile.print(gyro_y);
  myFile.print(",");
  myFile.println(gyro_z);
  myFile.close();

  // Serial.println(pos, DEC);
}
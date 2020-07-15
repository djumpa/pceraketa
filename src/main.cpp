#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>

#define MPU9250_ADDRESS 0x68
#define BMP280_ADDRESS 0x76

#define GYRO_FULL_SCALE_250_DPS 0x00
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2_G 0x00
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

bool debug_imu = false;
bool debug_baro = false;
bool debug_deploy = false;

Adafruit_BMP280 bmp;

Servo myservo;  // create servo object to control a servo

const PROGMEM uint8_t led_hb = 4;
uint8_t led_state_hb = LOW;
const PROGMEM uint8_t led_arm = 3;
const PROGMEM uint8_t butt_arm = 2;
const PROGMEM uint8_t servo_1 = 9;
const PROGMEM uint8_t servo_latch = 0; //degrees
const PROGMEM uint8_t servo_release = 90; //degrees
uint8_t print_counter;

const PROGMEM uint8_t chipSelect = 10;

uint8_t pos = 0;  // variable to store the servo position
uint8_t direction = 0;

float acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;

float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float elapsedTime, currentTime, previousTime;
float roll, pitch, yaw;

uint8_t c = 0;

int counter;
char counter_arr[10];
char filename[10];

uint8_t armed;
bool deploy_chute;
File myFile;

float baro_pressure_avg;
float baro_pressure_avg_previous;
uint8_t baro_samples = 25;
int frame_num;

unsigned long previousMillis = 0;  // will store last time LED was updated

const long interval = 1000;

// This function halts the execution
void stop() {
  while (1) {
    if (led_state_hb == LOW) {
      led_state_hb = HIGH;
    } else {
      led_state_hb = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(led_hb, led_state_hb);
    delay(100);
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
    Wire.requestFrom(MPU9250_ADDRESS, 6);
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
    Wire.requestFrom(MPU9250_ADDRESS, 6);
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
  Serial.print(F("AccErrorX: "));
  Serial.println(AccErrorX);
  Serial.print(F("AccErrorY: "));
  Serial.println(AccErrorY);
  Serial.print(F("GyroErrorX: "));
  Serial.println(GyroErrorX);
  Serial.print(F("GyroErrorY: "));
  Serial.println(GyroErrorY);
  Serial.print(F("GyroErrorZ: "));
  Serial.println(GyroErrorZ);
}

void setup() {
  pinMode(led_hb, OUTPUT);
  pinMode(led_arm, OUTPUT);
  pinMode(butt_arm, INPUT_PULLUP);
  digitalWrite(led_arm, LOW);
  myservo.attach(servo_1);
  myservo.write(servo_latch);
  Serial.begin(115200);

  // Init SD Card
  Serial.print(F("Initializing SD card..."));
  pinMode(10, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Initialization failed!"));
    stop();
  }
  Serial.println(F("Initialization done."));

  // Read current counter
  myFile = SD.open("COUNTER.TXT");
  if (myFile) {
    Serial.print(F("Counter: "));
    uint8_t i = 0;
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
    Serial.println(F("error opening COUNTER.TXT"));
    stop();
  }

  // Add +1 to counter and save it for the future
  myFile = SD.open("COUNTER.TXT", FILE_WRITE | O_TRUNC);
  if (myFile) {
    Serial.print(F("Adding counter... "));
    myFile.println(counter + 1);
    myFile.close();
    Serial.println(F("Done!"));
  } else {
    // if the file didn't open, print an error:
    Serial.println(F("error adding COUNTER.TXT"));
     stop();
  }
  // Write header to the actual log
  myFile = SD.open(filename, FILE_WRITE | O_TRUNC | O_CREAT);
  if (myFile) {
    Serial.print(F("Writing header... "));
    myFile.println(F("looptime,ax,ay,az,gx,gy,gz,baro_temp,baro_pressure"));
    // close the file:
    myFile.close();
    Serial.println(F("Done!"));
  } else {
    // if the file didn't open, print an error:
    Serial.print(F("error opening "));
    Serial.println(filename);
    stop();
  }

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

  if (!bmp.begin(BMP280_ADDRESS)) {
    Serial.println(F("Pressure sensor not found!"));
    stop();
  }

  calculate_IMU_error();
  delay(20);
}

void loop() {
  previousTime =
      currentTime;  // Previous time is stored before the actual time read
  currentTime = millis();  // Current time actual time read
  elapsedTime =
      (currentTime - previousTime) / 1000;  // Divide by 1000 to get seconds

  if (currentTime - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentTime;

    // if the LED is off turn it on and vice-versa:
    if (led_state_hb == LOW) {
      led_state_hb = HIGH;
    } else {
      led_state_hb = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(led_hb, led_state_hb);
  }

  armed = !digitalRead(butt_arm);  // read the input pin

  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 6);

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

  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x43);  // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 6);  // Read 4 registers total, each axis
                                         // value is stored in 2 registers
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

  if (debug_imu) {
    // Print the values on the serial monitor
    Serial.print(roll);
    Serial.print("/");
    Serial.print(pitch);
    Serial.print("/");
    Serial.println(yaw);
  }
  float baro_temp = bmp.readTemperature();
  // načtení naměřeného tlaku ze senzoru
  float baro_pressure = (bmp.readPressure());
  // výpis všech dostupných informací ze senzoru BMP
  // výpis teploty
  if (debug_baro) {
    Serial.print(F("Temperature: "));
    Serial.print(baro_temp);
    Serial.println(" °C.");
    // výpis barometrického tlaku v hekto Pascalech
    Serial.print(F("Pressure: "));
    Serial.print(baro_pressure);
    Serial.println(F(" Pa"));
    // vytištění prázdného řádku a pauza po dobu 2 vteřin
    Serial.println();
  }

  baro_pressure_avg += baro_pressure;

  if ((frame_num + 1) % baro_samples == 0) {
    baro_pressure_avg /= baro_samples;

    if (frame_num==baro_samples-1)
      baro_pressure_avg_previous = baro_pressure_avg; // init first pass delta
    if (debug_deploy) {
      Serial.print(F("Frame: "));
      Serial.println(frame_num);
      Serial.print(F("Previous window: "));
      Serial.println(baro_pressure_avg_previous);
      Serial.print(F("Current window: "));
      Serial.println(baro_pressure_avg);
      Serial.print(F("Delta: "));
      Serial.println(baro_pressure_avg - baro_pressure_avg_previous);
    }
    if (((baro_pressure_avg - baro_pressure_avg_previous) > 10) && armed) {
      deploy_chute = true;
    }
    baro_pressure_avg_previous = baro_pressure_avg;
    baro_pressure_avg = 0;
  }



  if (armed) {
    digitalWrite(led_arm, HIGH);
    
    if (deploy_chute) {;
      myservo.write(servo_release);
    } else {
      myservo.write(servo_latch);
    }

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
    myFile.print(gyro_z);
    myFile.print(",");
    myFile.print(baro_temp);
    myFile.print(",");
    myFile.println(baro_pressure);
    myFile.close();
  }
  // Serial.println(pos, DEC);
  frame_num++;

if (print_counter%100==0){
  Serial.print(F("Arm: "));
  Serial.println(armed);
  Serial.print(F("Deploy: "));
  Serial.println(deploy_chute);
  print_counter=0;
}
print_counter++;
}
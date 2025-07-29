#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

void __read_flex_data();
void __gen_data_string();
void __apply_mean_filter();
void __auto_calibration();
void __apply_median_filter();
void __auto_calibration_median();


#define FLEX_1 12
#define FLEX_2 27
#define FLEX_3 25
#define FLEX_4 32
#define FLEX_5 34

int FLEX_PINS[] = {12, 27, 25, 32, 34};
float flex_data[] = {0, 0, 0, 0, 0};
float calibrated_values[] = {0, 0, 0, 0, 0};
int16_t ax, ay, az, gx, gy, gz;

String data;
char buffer[1000];

int count = 0;
float ___init_yaw__ = 0;

MPU6050 mpu;

#define INTERRUPT_PIN 23
bool blinkState = false;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 gyr;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

int current_time, future_time, interval = 150;

volatile bool mpuInterrupt = false;

void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.begin(115200);
  while (!Serial);

  
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read());
  //while (!Serial.available());
  while (Serial.available() && Serial.read());
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(124);
  mpu.setYGyroOffset(35);
  mpu.setZGyroOffset(31);
  mpu.setXAccelOffset(-3562);
  mpu.setYAccelOffset(1821);
  mpu.setZAccelOffset(1781);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);


}

void loop() {
  // current_time = millis();
  // future_time = current_time + interval;
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGyro(&gyr, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  }

  __gen_data_string();
  Serial.println(data);


  //while(future_time > millis());
  mpu.resetFIFO();
}

void __gen_data_string() {
  sprintf(buffer, "%d,%d,%d,%d,%d,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%d,%d,%d,%d,%d,%d", analogRead(FLEX_1), analogRead(FLEX_2), analogRead(FLEX_3), analogRead(FLEX_4), analogRead(FLEX_5), q.w, q.x, q.y, q.z, gyr.x, gyr.y, gyr.z, aa.x, aa.y, aa.z, aaReal.x, aaReal.y, aaReal.z, aaWorld.x, aaWorld.y, aaWorld.z, gravity.x, gravity.y, gravity.z, ax, ay, az, gx, gy, gz);
  //sprintf(buffer, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", analogRead(FLEX_1), analogRead(FLEX_2), analogRead(FLEX_3), analogRead(FLEX_4), analogRead(FLEX_5), gyr.x, gyr.y, gyr.z, aa.x, aa.y, aa.z, aaReal.x, aaReal.y, aaReal.z, aaWorld.x, aaWorld.y, aaWorld.z, ax, ay, az, gx, gy, gz);
 
  //sprintf(buffer, "%d,%d,%d,%d,%d,%d,%d,%d", analogRead(FLEX_1), analogRead(FLEX_2), analogRead(FLEX_3), analogRead(FLEX_4), analogRead(FLEX_5), aaWorld.x / 100, aaWorld.y / 100, aaWorld.z / 100);
  //sprintf(buffer, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", analogRead(FLEX_1), analogRead(FLEX_2), analogRead(FLEX_3), analogRead(FLEX_4), analogRead(FLEX_5), ax, ay, az, gx, gy, gz);
  data = buffer;
}




















































































































































































































// #include <Arduino.h>
// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"

// ///////////////////////////////////// FLEX RAW DATA  ////////////////////////////////////////////
// #define FLEX_1 12
// #define FLEX_2 27
// #define FLEX_3 25
// #define FLEX_4 32
// #define FLEX_5 34
// #define INTERRUPT_PIN 23

// String data;
// char buffer[100];
// //bool blinkState = false;

// int FLEX_PINS[] = {12, 27, 25, 32, 34};
// float flex_data[] = {0, 0, 0, 0, 0};
// float calibrated_values[] = {0, 0, 0, 0, 0};

// void __apply_mean_filter() {
//   float sum[] = {0, 0, 0, 0, 0};
//   for (int i = 0; i < 10; i++) {
//     for(int i = 0; i < 5; i++) {
//       sum[i] = sum[i] + analogRead(FLEX_PINS[i]);
//     }
//     delay(10);
//   }
//   for(int i = 0; i < 5; i++) {
//     flex_data[i] = (sum[i] / 10 - calibrated_values[i]) * 2;
//   }
// }

// void __auto_calibration() {
//   float sum[] = {0, 0, 0, 0, 0};
//   for (int i = 0; i < 700; i++) {
//     for(int i = 0; i < 5; i++) {
//       sum[i] = sum[i] + analogRead(FLEX_PINS[i]);
//     }
//     delay(10);
//   }
//   for(int i = 0; i < 5; i++) {
//     calibrated_values[i] = sum[i] / 700;
//   }
// }

// void __apply_median_filter() {
//   int flex[5][7];
//   for(int i = 0; i < 5; i++) {
//     for(int j = 0; j < 7; j++) {
//       flex[i][j] = 0;
//     }
//   }
//   for(int i = 0; i < 7; i++) {
//     for(int j = 0; j < 5; j++) {
//       flex[j][i] = analogRead(FLEX_PINS[j]);
//       for(int k = 0; k < i; k++) {
//         for(int l = 1; l < 7; l++) {
//           if(flex[j][l] > flex[j][l-1]) {
//             int temp = flex[j][l];
//             flex[j][l] = flex[j][l-1];
//             flex[j][l-1] = temp;
//           }
//         }
//       }
//     }
//     delay(10);
//   }
//   for(int i = 0; i < 5; i++) {
//     flex_data[i] = (flex[i][4] - calibrated_values[i]) * 2;
//   }
// }

// void __auto_calibration_median() {
//   int flex[5][100];
//   for(int i = 0; i < 5; i++) {
//     for(int j = 0; j < 100; j++) {
//       flex[i][j] = 0;
//     }
//   }
//   for(int i = 0; i < 100; i++) {
//     for(int j = 0; j < 5; j++) {
//       flex[j][i] = analogRead(FLEX_PINS[j]);
//       for(int k = 0; k < i; k++) {
//         for(int l = 1; l < 100; l++) {
//           if(flex[j][l] > flex[j][l-1]) {
//             int temp = flex[j][l];
//             flex[j][l] = flex[j][l-1];
//             flex[j][l-1] = temp;
//           }
//         }
//       }
//     }
//     delay(70);
//   }
//   for(int i = 0; i < 5; i++) {
//     calibrated_values[i] = flex[i][50];
//   }
// }

// void __gen_data_string() {
//   sprintf(buffer, "%d\t,%d\t,%d\t,%d\t,%d\t", (int)flex_data[0], (int)flex_data[1], (int)flex_data[2], (int)flex_data[3], (int)flex_data[4]);
//   data = buffer;
// }


// void setup()
// {
//   pinMode(LED_BUILTIN, OUTPUT);
//   pinMode(INTERRUPT_PIN, INPUT);

//   Serial.begin(115200);
//   while (!Serial);

//   // digitalWrite(LED_BUILTIN, HIGH);
//   // //__auto_calibration();
//   // __auto_calibration_median();
//   // digitalWrite(LED_BUILTIN, LOW);
//   // delay(1400);
//   digitalWrite(LED_BUILTIN, HIGH);
// }

// void loop()
// {
//   // __apply_median_filter();
//   // __gen_data_string();
//   // Serial.println(data);
//   // Serial.print(); Serial.print("\t");
//   // Serial.print(analogRead(FLEX_2)); Serial.print("\t");
//   // Serial.print(analogRead(FLEX_3)); Serial.print("\t");
//   // Serial.print(analogRead(FLEX_4)); Serial.print("\t");
//   // Serial.println(analogRead(FLEX_5)); 
//   sprintf(buffer, "%d,%d,%d,%d,%d", analogRead(FLEX_1), analogRead(FLEX_2), analogRead(FLEX_3), analogRead(FLEX_4), analogRead(FLEX_5));
//   Serial.println(buffer);
//   delay(100);
// }





// // /////////////////////////////////////// MPU RAW DATA  ////////////////////////////////////////////
// // #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
// //     #include "Wire.h"
// // #endif

// // MPU6050 accelgyro;

// // int16_t ax, ay, az;
// // int16_t gx, gy, gz;
// // double mag;

// // #define OUTPUT_READABLE_ACCELGYRO

// // #define LED_PIN 13
// // bool blinkState = false;


// // void setup() {
// //     // join I2C bus (I2Cdev library doesn't do this automatically)
// //     #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
// //         Wire.begin();
// //     #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
// //         Fastwire::setup(400, true);
// //     #endif

// //     // initialize serial communication
// //     // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
// //     // it's really up to you depending on your project)
// //     Serial.begin(115200);

// //     // initialize device
// //     Serial.println("Initializing I2C devices...");
// //     accelgyro.initialize();

// //     // verify connection
// //     Serial.println("Testing device connections...");
// //     Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

// //     // use the code below to change accel/gyro offset values
    
// //     Serial.println("Updating internal sensor offsets...");
// //     // template: -76	-2359	1688	0	0	0
// //     // this device: -3970   1839    1748    220     76      -85
// //     Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
// //     Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
// //     Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
// //     Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
// //     Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
// //     Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
// //     Serial.print("\n");

// //     accelgyro.setXGyroOffset(220);
// //     accelgyro.setYGyroOffset(76);
// //     accelgyro.setZGyroOffset(-85);
// //     accelgyro.setXAccelOffset(-3970);
// //     accelgyro.setYAccelOffset(1839);
// //     accelgyro.setZAccelOffset(1748);

// //     Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
// //     Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
// //     Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
// //     Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
// //     Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
// //     Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
// //     Serial.print("\n");
    
// //     delay(5000);
// //     // configure Arduino LED pin for output
// //     pinMode(LED_PIN, OUTPUT);
// // }

// // void loop() {
// //     // read raw accel/gyro measurements from device
// //     accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

// //     // these methods (and a few others) are also available
// //     //accelgyro.getAcceleration(&ax, &ay, &az);
// //     //accelgyro.getRotation(&gx, &gy, &gz);

// //     mag = sqrt(((ax/100)^2) + ((ay/100)^2) + ((az/100)^2));

// //     #ifdef OUTPUT_READABLE_ACCELGYRO
// //         // display tab-separated accel/gyro x/y/z values
// //         Serial.print(mag); Serial.print("\t");
// //         Serial.print("a/g:\t");
// //         Serial.print(ax); Serial.print("\t");
// //         Serial.print(ay); Serial.print("\t");
// //         Serial.print(az); Serial.print("\t");
// //         Serial.print(gx); Serial.print("\t");
// //         Serial.print(gy); Serial.print("\t");
// //         Serial.println(gz);
// //         delay(100);
// //     #endif

// //     #ifdef OUTPUT_BINARY_ACCELGYRO
// //         Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
// //         Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
// //         Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
// //         Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
// //         Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
// //         Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
// //     #endif

// //     // blink LED to indicate activity
// //     blinkState = !blinkState;
// //     digitalWrite(LED_PIN, blinkState);
// // }

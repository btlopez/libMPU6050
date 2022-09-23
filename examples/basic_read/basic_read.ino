#include <MPU6050.h>

MPU6050lib mpu;

IntervalTimer mainTimer;

int counter = 0;
int initial = 1;

int led = LED_BUILTIN;
int ledState = HIGH;

int16_t accel_tmp[3];
int16_t gyro_tmp[3];

float accel_data[3];
float gyro_data[3];

float a_res;
float g_res; 
float rate = 10;

void setup() {  
  Wire.begin();
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  mainTimer.begin(mainThread,1/rate*1000000);
}

void loop() {
  // NO-OP
}

void mainThread(){      
  if (initial){
    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t c = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);

    Serial.print("I AM ");
    Serial.print(c, HEX);  
    Serial.print(" I Should Be ");
    Serial.println(0x68, HEX);

    if (c == MPU6050_ADDRESS){
      mpu.initMPU6050();
      initial = 0;
    }
  }

  else{      
    // Read from MPU 6050
    mpu.readAccelData(accel_tmp);
    a_res = mpu.getAres();
    accel_data[0] = (float) 9.80665 * accel_tmp[0] * a_res;
    accel_data[1] = (float) 9.80665 * accel_tmp[1] * a_res;
    accel_data[2] = (float) 9.80665 * accel_tmp[2] * a_res;
    
    mpu.readGyroData(gyro_tmp);
    g_res = mpu.getGres();
    gyro_data[0] = (float) 0.01745329251 * gyro_tmp[0] * g_res;
    gyro_data[1] = (float) 0.01745329251 * gyro_tmp[1] * g_res;
    gyro_data[2] = (float) 0.01745329251 * gyro_tmp[2] * g_res;

    Serial.print("Acceleration X: ");
    Serial.print(accel_data[0]);
    Serial.print(", Y: ");
    Serial.print(accel_data[1]);
    Serial.print(", Z: ");
    Serial.print(accel_data[2]);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(gyro_data[0]);
    Serial.print(", Y: ");
    Serial.print(gyro_data[1]);
    Serial.print(", Z: ");
    Serial.print(gyro_data[2]);
    Serial.println(" rad/s");

    counter++;
  }

  if (counter%(int)rate == 0){
    ledState = !ledState;
    digitalWrite(led,ledState);
  }
}
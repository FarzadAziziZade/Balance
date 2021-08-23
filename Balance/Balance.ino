#include <Wire.h>
//#include <PID_v1.h>
char report[80];

uint32_t counter = 0;
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data
char tmp_str[7]; // temporary variable used in convert function
char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

//int gzzz=0;
int axxx;
//int degree;

int p5 = 5;
int p6 = 6;
int desire=0;

//double Setpoint, Input, Output;
//double Kp=2, Ki=5, Kd=1;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//PID myPIDD(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // put your setup code here, to run once:
  pinMode(p5, OUTPUT);
  pinMode(p6, OUTPUT);

  //Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);


  //initialize the variables we're linked to
//  Input = 0;
//  Setpoint = -65;
//
//  //turn the PID on
//  myPID.SetMode(AUTOMATIC);
//  myPIDD.SetMode(AUTOMATIC);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7 * 2, true); // request a total of 7*2=14 registers

  accelerometer_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read() << 8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
//
//   if (counter % 25 == 0)
//    {
//     Serial.println("\tn\tACCELEROMETER\t\tGYROSCOPE\t\tTEMPERATURE");
//     Serial.println("\tax\tay\taz\tgx\tgy\tgz\tT");
//    }
//  
//  
//    Serial.print(counter);
//    Serial.print('\t');
//    Serial.print(convert_int16_to_str(accelerometer_x));
//    Serial.print('\t');
//    Serial.print(convert_int16_to_str(accelerometer_y));
//    Serial.print('\t');
//    Serial.print(convert_int16_to_str(accelerometer_z));
//    Serial.print('\t');
//    Serial.print(convert_int16_to_str(gyro_x));
//    Serial.print('\t');
//    Serial.print(convert_int16_to_str(gyro_y));
//    Serial.print('\t');
//    Serial.print(convert_int16_to_str(gyro_z));
//    Serial.print('\t');
//    Serial.print(temperature / 340.00 + 36.53);
//    Serial.println();
//    counter++;
axxx=int(accelerometer_y)/4000;
//gzzz=int(accelerometer_x)/100;
//degree=sqrt(axxx*axxx+gzzz*gzzz);
//delay(100);
//    Serial.print(axxx);
//    Serial.println();
//analogWrite(p6, 10);
//analogWrite(p5, 10);
    //left
//if (gzzz==0){
//  desire=axxx;
//  gzzz=1; 
//}
delay(20);
    if (axxx > desire) {
        analogWrite(p6, 100);
        delay(50);
        analogWrite(p6, 0);
    }



    //right

    if (axxx < desire) {
        analogWrite(p5, 100);
        delay(50);
        analogWrite(p5, 0);
    }

//  Input = float(accelerometer_y);
//  myPID.Compute();
//  analogWrite(p5, Output);
//
//  Input = float(accelerometer_y);
//  myPIDD.Compute();
//  analogWrite(p6, Output);
}

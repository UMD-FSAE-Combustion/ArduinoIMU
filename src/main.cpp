#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#include <FlexCAN_T4.h>
#include <TeensyThreads.h>


// PIN, BAUDRATE and CAN ID definitions
#define BAUD        1000000     // 1000000 for ECU, 250000 for Arduino
#define CANRXID     0x640
#define CANTXID1    0x5B0
#define CANTXID2    0x5B1
// BNO055 AND CTL REGISTER ADDRESSES
#define BNO_ADDR    0x28
#define BNO_OPR     0x3D
#define BNO_PGSEL   0x07
#define SET_UNITS   0x3B
#define SET_ACC     0x08
#define SET_GYRO    0x0A
// BNO055 ACCELERATION DATA REGISTER ADDRESSES
/*#define ACCX_LO     0x08
#define ACCX_HI     0x09
#define ACCY_LO     0x0A
#define ACCY_HI     0x0B
#define ACCZ_LO     0x0C
#define ACCZ_HI     0x0D
// BNO055 GYROSCOPE DATA REGISTER ADDRESSES
#define GYRX_LO     0x14
#define GYRX_HI     0x15
#define GYRY_LO     0x16
#define GYRY_HI     0x17
#define GYRZ_LO     0x18
#define GYRZ_HI     0x19*/
// CONSTANTS FOR FLOAT->UINT CONVERSION
#define ACC_CONST   104.223     // Convert m/s^2 to milli-g + 2.25% error correction
#define GYR_CONST   100.0        // Apply a gain of 100 for better resolution


// CAN 16-bit struct, gyro struct and accel struct
typedef struct CANint16_t { unsigned int top:8, bot:8; };
typedef struct CANDBtx_imu { CANint16_t x, y, z; };


//Declare everything global;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO_ADDR, &Wire);
//i2c_dev = new Adafruit_I2CDevice(BNO_ADDR, theWire);

CANDBtx_imu CANDBtx_accel;
CANDBtx_imu CANDBtx_gyro;

CAN_message_t msg[2];


// put function declarations here:
void taskCANTransmit();
bool bno_write(uint8_t i2c_addr, uint8_t reg, uint8_t data);
uint8_t bno_read(uint8_t reg);
void convert_to_int(float source, CANint16_t &dest, bool isAccel);

void setup() {

  Serial.begin(115200);

  // put your setup code here, to run once:

  if(!bno.begin(OPERATION_MODE_ACCGYRO)) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Oops, no BNO055 detected ... Check your wiring or I2C ADDR.");
    while(1);
  }

  //----------Configure BNO055----------//
  /*if(!bno_write(BNO_ADDR, BNO_PGSEL, 0x01)) {
    Serial.print("Configuring sensitivity ranges ...");
    while(1);
  }
  if(!bno_write(BNO_ADDR, SET_ACC, 0x0C)) {
    Serial.print("/nSetting accel range.");
    while(1);
  }
  if(!bno_write(BNO_ADDR, SET_GYRO, 0x32)) {
    Serial.print("/nSetting gyro range.");
    while(1);
  }
  if(!bno_write(BNO_ADDR, BNO_PGSEL, 0x00)) {
    Serial.print("/nWrapping up ...");
    while(1);
  }
  if(!bno_write(BNO_ADDR, SET_UNITS, 0x91)) {
    while(1);
  }*/

  Serial.print("Operation mode: ");
  Serial.print(bno.getMode());

  // Starts CAN comms at a select Baudrate of the ECU
  Can0.begin();
  Can0.setBaudRate(BAUD);
  threads.addThread(taskCANTransmit, 0, 512, 0);
}


void loop()
{
  sensors_event_t event;

  // Get linear acceleration data from IMU, convert to two 8-bit integers
  bno.getEvent(&event, bno.VECTOR_ACCELEROMETER);
  Serial.print("\nX lin: ");
  Serial.print(event.acceleration.x);
  convert_to_int(event.acceleration.x, CANDBtx_accel.x, true);
  convert_to_int(event.acceleration.y, CANDBtx_accel.y, true);
  convert_to_int(event.acceleration.z, CANDBtx_accel.z, true);
  Serial.print("\t");
  Serial.print(event.acceleration.x*ACC_CONST);

  // Get gyroscopic data from IMU, convert to two 8-bit integers
  bno.getEvent(&event, bno.VECTOR_GYROSCOPE);

  convert_to_int(event.gyro.x, CANDBtx_gyro.x, false);
  convert_to_int(event.gyro.y, CANDBtx_gyro.y, false);
  convert_to_int(event.gyro.z, CANDBtx_gyro.z, false);
}


void taskCANTransmit()
{
  while(1) {
    CAN_message_t msg[2];
    // Assign Data to message 1
    msg[0].id = CANTXID1;
    msg[0].len = 8;
    msg[0].buf[0] = CANDBtx_accel.x.top;
    msg[0].buf[1] = CANDBtx_accel.x.bot;
    msg[0].buf[2] = CANDBtx_accel.y.top;
    msg[0].buf[3] = CANDBtx_accel.y.bot;
    msg[0].buf[4] = CANDBtx_accel.z.top;
    msg[0].buf[5] = CANDBtx_accel.z.bot;
    msg[0].buf[6] = 0;
    msg[0].buf[7] = 0;

    // Assign Data to message 2
    msg[1].len = 8;
    msg[1].id = CANTXID2;
    msg[1].buf[0] = CANDBtx_gyro.x.top;
    msg[1].buf[1] = CANDBtx_gyro.x.bot;
    msg[1].buf[2] = CANDBtx_gyro.y.top;
    msg[1].buf[3] = CANDBtx_gyro.y.bot;
    msg[1].buf[4] = CANDBtx_gyro.z.top;
    msg[1].buf[5] = CANDBtx_gyro.z.bot;
    msg[1].buf[6] = 0;
    msg[1].buf[7] = 0;

    // Send messages
    Can0.write(msg[0]);
    delay(20);
    Can0.write(msg[1]);
    delay(20);
  }
}


///////////////////////////////////////////////////////////////////
/*----------WRITE VALUES TO BNO SENSOR CONFIG REGISTERS----------*/
///////////////////////////////////////////////////////////////////
bool bno_write(uint8_t i2c_addr, uint8_t reg, uint8_t data)  // write one BNO register
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);  // send stop

  return true;
}


//////////////////////////////////////////////////////////////////
/*----------READ VALUES FROM BNO SENSOR DATA REGISTERS----------*/
//////////////////////////////////////////////////////////////////
uint8_t read8(uint8_t reg) {
  uint8_t buffer[1] = {reg};
  //i2c_dev->Adafruit_I2CDevice::write_then_read(buffer, 1, buffer, 1);
  return (byte)buffer[0];
}


///////////////////////////////////////////////////////////////////////
/*----------CONVERT DATA TO 8-BIT INTEGERS FOR CAN TRANSMIT----------*/
///////////////////////////////////////////////////////////////////////
void convert_to_int(float bno_data, CANint16_t &dest, bool isAccel)
{
  short int_data;

  if (isAccel) int_data = static_cast<int16_t>(bno_data*ACC_CONST);
  else int_data = static_cast<int16_t>(bno_data*GYR_CONST);

  dest.bot = int_data;
  dest.top = int_data >> 8;
}

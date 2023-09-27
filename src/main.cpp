#include <string.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>



/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);



int intConvert(float x) {

  int num = ((static_cast<int>(x + 19.62)) * (1672));
  return num;
}

String decToBinary(int n) {
  // array to store binary number
  int binaryNum[32];

  // counter for binary array
  int i = 0;
  while (n > 0) {

    // storing remainder in binary array
    binaryNum[i] = n % 2;
    n = n / 2;
    i++;
  }

  // printing binary array in reverse order
  String BinNum = "";
  for (int j = i - 1; j >= 0; j--)
    BinNum += binaryNum[j];

  return BinNum;
}


String binToHex(String bin) {

    String hex = "0x";
    if (bin.length() % 4 == 0) {
        for (int i = 0; i < bin.length(); i += 4) {

            String subStr = bin.substring(i, 4);
            int SubNum = subStr.toInt();

            switch (SubNum) {
            case (0):            //0000
                hex += "0"; break;
            case (1):            //0001
                hex += "1"; break;
            case (10):            //0010
                hex += "2";    break;
            case (11):            //0011
                hex += "3"; break;
            case (100):            //0100
                hex += "4"; break;
            case (101):            //0101
                hex += "5"; break;
            case (110):            //0110
                hex += "6"; break;
            case (111):            //0111
                hex += "7"; break;
            case (1000):
                hex += "8"; break;
            case (1001):
                hex += "9"; break;
            case (1010):
                hex += "A"; break;
            case (1011):
                hex += "B"; break;
            case (1100):
                hex += "C"; break;
            case (1101):
                hex += "D"; break;
            case (1110):
                hex += "E"; break;
            case (1111):
                hex += "F"; break;
            default:
                break;
            }
        }
    }
else {
        int padding = bin.length() % 4;
        for (int i = 0; i < (4 - padding); i++)
            bin = "0" + bin;


        for (int i = 0; i < bin.length(); i += 4) {

            String subStr = bin.substring(i, 4);
            int SubNum = subStr.toInt();

            switch (SubNum) {
            case (0):            //0000
                hex += "0"; break;
            case (1):            //0001
                hex += "1"; break;
            case (10):            //0010
                hex += "2";    break;
            case (11):            //0011
                hex += "3"; break;
            case (100):            //0100
                hex += "4"; break;
            case (101):            //0101
                hex += "5"; break;
            case (110):            //0110
                hex += "6"; break;
            case (111):            //0111
                hex += "7"; break;
            case (1000):
                hex += "8"; break;
            case (1001):
                hex += "9"; break;
            case (1010):
                hex += "A"; break;
            case (1011):
                hex += "B"; break;
            case (1100):
                hex += "C"; break;
            case (1101):
                hex += "D"; break;
            case (1110):
                hex += "E"; break;
            case (1111):
                hex += "F"; break;
            default:
                break;
            }
        }
    }

    return hex;
}



/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) {
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Raw Data Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) {
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

  //Sensor Outputs
  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  int xVal = intConvert(accelerometer.x());
  Serial.println(xVal);
  String xVal_Bin = decToBinary(xVal);
  String xVal_Hex = binToHex(xVal_Bin);


  // Display the floating point data
  Serial.print("X: ");
  Serial.print(xVal_Hex);
  Serial.print(" Y: ");
  Serial.print(accelerometer.y());
  Serial.print(" Z: ");
  Serial.print(accelerometer.z());
  Serial.print("\t\t");

  Serial.print("X: ");
  Serial.print(gyroscope.x());
  Serial.print(" Y: ");
  Serial.print(gyroscope.y());
  Serial.print(" Z: ");
  Serial.print(gyroscope.z());
  Serial.print("\t\t");

  Serial.print("X: ");
  Serial.print(linAccel.x());
  Serial.print(" Y: ");
  Serial.print(linAccel.y());
  Serial.print(" Z: ");
  Serial.print(linAccel.z());
  Serial.print("\t\t");





  /*
   // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  */



  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
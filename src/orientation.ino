/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.
 Modified by Brent Wilkins July 19, 2016

 Demonstrate basic MPU-9250 functionality including parameterizing the register
 addresses, initializing the sensor, getting properly scaled accelerometer,
 gyroscope, and magnetometer data out. Added display functions to allow display
 to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
 Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
 and the Teensy 3.1.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 */

#include "quaternionFilters.h"
#include "MPU9250.h"

#ifdef LCD
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 9 - Serial clock out (SCLK)
// pin 8 - Serial data out (DIN)
// pin 7 - Data/Command select (D/C)
// pin 5 - LCD chip select (CS)
// pin 6 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 7, 5, 6);
#endif // LCD

#define AHRS true         // Set to false for basic data read
#define SerialDebug false  // Set to true to get Serial output for debugging
#define TCPcomms false


// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
// PS. I think on photon pin 12 is A0?

//int myLed  = 13;  // Set up pin 13 led for toggling
char msg[50];

#define FRONT_LEFT_PIN RX
#define FRONT_RIGHT_PIN WKP
#define REAR_LEFT_PIN A5
#define REAR_RIGHT_PIN D3

Servo front_left;
Servo front_right;
Servo rear_left;
Servo rear_right;

bool attitude_control_enabled = false;
float P_pitch = 0.4f;
float P_roll = 0.2f;
float pitch_error = 0.0f;
float roll_error = 0.0f;
int pitch_control = 0;
int roll_control = 0;
int u_0 = 1100;
int pitch_in_tenths = 0;

MPU9250 myIMU;
TCPClient client;
byte server[] = { 192, 168, 0, 106 };
//#define HOST_NAME "posttestserver.com" // would this work instead of the line above?

void setup()
{

  // Motor servos attachments
  front_left.attach(FRONT_LEFT_PIN, 1000, 2000);
  front_right.attach(FRONT_RIGHT_PIN, 1000, 2000);
  rear_left.attach(REAR_LEFT_PIN, 1000, 2000);
  rear_right.attach(REAR_RIGHT_PIN, 1000, 2000);

  Particle.function("motor_commands", motor_commands);
  Particle.function("pitch gain", pitch_gain);
  Particle.function("roll gain", roll_gain);
  Particle.variable("pitch_in_tenths", pitch_in_tenths);
  Particle.variable("pitch_control", pitch_control);
  
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(38400);

  // Set up the interrupt pin, its set as active high, push-pull
  // Jan note: not sure about this intPin -- does it do anything?
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  //pinMode(myLed, OUTPUT);
  //digitalWrite(myLed, HIGH);

  if(TCPcomms){

    while(!client.connect(server, 8000))
      {
        Particle.publish("Trying to connect...");
        Serial.println("Trying to coonnect...");
        delay(1000);
        // client.println("GET /search?q=unicorn HTTP/1.0");
      }
  }

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);


  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    delay(3000);
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[2], 2);
    }

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1){delay(1000);} ; // Loop forever if communication doesn't happen
  }
  
}

void loop()
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();

    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
               
    
    //Spark.publish("gpsloc", szInfo); //Publish Data
    delay(5); //can thottle refresh rate here
    myIMU.data_read_counter ++;

  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(myIMU.ax, myIMU.az, myIMU.ay, -myIMU.gx*DEG_TO_RAD, -myIMU.gz*DEG_TO_RAD, -myIMU.gy*DEG_TO_RAD, 
                           -myIMU.my,  myIMU.mz, -myIMU.mx, myIMU.deltat); 

  // swapping middle coordinates changes roll sign, but picth keeps being reverse of what i want (negative nose up)
  // MahonyQuaternionUpdate(myIMU.ax, myIMU.az, myIMU.ay, myIMU.gx*DEG_TO_RAD,
  //                       myIMU.gz*DEG_TO_RAD, myIMU.gy*DEG_TO_RAD, myIMU.mz,
  //                       myIMU.mx, myIMU.my, myIMU.deltat);

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        // Print acceleration values in milligs!
        Serial.print("X-acceleration: "); Serial.print(1000*myIMU.ax);
        Serial.print(" mg ");
        Serial.print("Y-acceleration: "); Serial.print(1000*myIMU.ay);
        Serial.print(" mg ");
        Serial.print("Z-acceleration: "); Serial.print(1000*myIMU.az);
        Serial.println(" mg ");

        // Print gyro values in degree/sec
        Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
        Serial.println(" degrees/sec");

        // Print mag values in degree/sec
        Serial.print("X-mag field: "); Serial.print(myIMU.mx);
        Serial.print(" mG ");
        Serial.print("Y-mag field: "); Serial.print(myIMU.my);
        Serial.print(" mG ");
        Serial.print("Z-mag field: "); Serial.print(myIMU.mz);
        Serial.println(" mG");

        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
        Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
        Serial.println(" degrees C");
      }

      myIMU.count = millis();
      
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
    myIMU.delt_t = millis() - myIMU.count; // time since we displayed

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                    *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                    *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                    *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));

      myIMU.pitch *= -RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw   -= 6.1;
      myIMU.roll  *= RAD_TO_DEG;

      // I don't know why did I have to put this here but seems to be working fine now:
      // 0 roll is level, right wing down is negative (around 0.2deg dead zone should be fine)
    if (myIMU.roll > 0)
      myIMU.roll -= 180.0f;
    else
      myIMU.roll += 180.0f;

    pitch_in_tenths = (int) myIMU.pitch * 10;

    if(attitude_control_enabled){
             
      pitch_error = 0.0f - (myIMU.pitch + 0.2f); // weird offset
      roll_error = 0.0f - myIMU.roll;

      // Safety precaution
      if(pitch_error > 20.0f || roll_error > 20.0f){ 
        attitude_control_enabled = false;

        front_left.writeMicroseconds(1000);
        front_right.writeMicroseconds(1000);
        rear_left.writeMicroseconds(1000);
        rear_right.writeMicroseconds(1000);
      }
      else {
        // Check if errors exceed dead zone and apply control
        if (abs(pitch_error) > 0.15f){
          // Scaling error into [-1, 1] range assuming [-45, 45] deg attitude range
          // Result = ((Input - InputLow) / (InputHigh - InputLow))
          //    * (OutputHigh - OutputLow) + OutputLow;
          pitch_error = ((pitch_error + 45.0f) / (45.0f + 45.0f)) * (1.0f + 1.0f) - 1.0f;

          // Apply P term and        
          // Scale the pitch_control signal into [-1000, 1000]
          pitch_control = (int) (((P_pitch * pitch_error + 1.f) / (1.f + 1.f)) * (1000.f + 1000.f) - 1000.f);

        }
        else
          { pitch_control = 0; } 

        if (abs(roll_error) > 0.15f){

          roll_error = ((roll_error + 45.0f) / (45.0f + 45.0f)) * (1.0f + 1.0f) - 1.0f;
          roll_control = (int) (((P_roll * roll_error + 1.f) / (1.f + 1.f)) * (1000.f + 1000.f) - 1000.f);

        }
        else{ roll_control = 0; }

        // Add the pitch_control and roll_control values to steady state (u_0)
        // and clip the final signal into [1000, 2000] (roughly)
        front_left.writeMicroseconds(max(1080, min(u_0 + pitch_control - roll_control, 1500)));
        front_right.writeMicroseconds(max(1080, min(u_0 + pitch_control + roll_control, 1500)));
        rear_left.writeMicroseconds(max(1080, min(u_0 - pitch_control - roll_control, 1500)));
        rear_right.writeMicroseconds(max(1080, min(u_0 - pitch_control + roll_control, 1500)));

      }
    }


    // Serial print and/or display at 0.5 s rate independent of data rates
    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > 200)
    {
      if (TCPcomms){
        if (client.status())
        {
          //int bytes = client.write(buf, len);  
          //int err = client.getWriteError();
          //if (err != 0) {
            // Log.trace("TCPClient::write() failed (error = %d), number of bytes written: %d", err, bytes);
          //}
          
          sprintf(msg, "%f, %f, %f", myIMU.roll, myIMU.pitch, myIMU.yaw);
          Serial.print("bytes_written: ");
          Serial.println(client.print(msg));
          
        }
      }

      if(SerialDebug)
      {
        Serial.print("ax = "); Serial.print((int)1000*myIMU.ax);
        Serial.print(" ay = "); Serial.print((int)1000*myIMU.ay);
        Serial.print(" az = "); Serial.print((int)1000*myIMU.az);
        Serial.println(" mg");

        Serial.print("gx = "); Serial.print( myIMU.gx, 2);
        Serial.print(" gy = "); Serial.print( myIMU.gy, 2);
        Serial.print(" gz = "); Serial.print( myIMU.gz, 2);
        Serial.println(" deg/s");

        Serial.print("mx = "); Serial.print( (int)myIMU.mx );
        Serial.print(" my = "); Serial.print( (int)myIMU.my );
        Serial.print(" mz = "); Serial.print( (int)myIMU.mz );
        Serial.println(" mG");

        Serial.print("q0 = "); Serial.print(*getQ());
        Serial.print(" qx = "); Serial.print(*(getQ() + 1));
        Serial.print(" qy = "); Serial.print(*(getQ() + 2));
        Serial.print(" qz = "); Serial.println(*(getQ() + 3));

        Serial.print("deltat = "); Serial.println(myIMU.deltat);
        Serial.print("delt_t = "); Serial.println(myIMU.delt_t);
      
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(myIMU.yaw, 2);
        Serial.print(", ");
        Serial.print(myIMU.pitch, 2);
        Serial.print(", ");
        Serial.println(myIMU.roll, 2);

        Serial.print("rate = ");
        Serial.print((float)myIMU.sumCount/myIMU.sum, 2);
        Serial.println(" Hz");
        
        Serial.print("new_data_rate = ");
        Serial.print((float)myIMU.data_read_counter/myIMU.sum, 2);
        Serial.println(" Hz");
      }
      
     
      // Check if still conected to server, try to reconnect
      if (TCPcomms && !client.connected())
      {
        Serial.println("Lost TCP connection.");

        while(!client.connect(server, 8000))
        {
          Particle.publish("Trying to reconnect...");
          Serial.println("Trying to reconnect...");
          delay(1000);
          // client.println("GET /search?q=unicorn HTTP/1.0");
        }
        //client.stop();
      }
      


    // With these settings the filter is updating at a ~145 Hz rate using the
    // Madgwick scheme and >200 Hz using the Mahony scheme even though the
    // display refreshes at only 2 Hz. The filter update rate is determined
    // mostly by the mathematical steps in the respective algorithms, the
    // processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum
    // magnetometer ODR of 100 Hz produces filter update rates of 36 - 145 and
    // ~38 Hz for the Madgwick and Mahony schemes, respectively. This is
    // presumably because the magnetometer read takes longer than the gyro or
    // accelerometer reads. This filter update rate should be fast enough to
    // maintain accurate platform orientation for stabilization control of a
    // fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050
    // 6 DoF and MPU9150 9DoF sensors. The 3.3 V 8 MHz Pro Mini is doing pretty
    // well!
      // display.setCursor(0, 40); display.print("rt: ");
      // display.print((float) myIMU.sumCount / myIMU.sum, 2);
      // display.print(" Hz");
      // display.display();

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
      myIMU.data_read_counter = 0;
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)
}


int pitch_gain(String command) {

    if(command.toFloat() <= 0.f || command.toFloat() > 1.f)
      return -1;

    P_pitch = command.toFloat();
    return 1;
}

int roll_gain(String command) {

    if(command.toFloat() <= 0.f || command.toFloat() > 1.f)
      return -1;

    P_roll = command.toFloat();
    return 1;
}

int motor_commands(String command) {
    /* Particle.functions always take a string as an argument and return an integer.
    Since we can pass a string, it means that we can give the program commands on how the function should be used.
    In this case, telling the function "on" will turn the LED on and telling it "off" will turn the LED off.
    Then, the function returns a value to us to let us know what happened.
    In this case, it will return 1 for the LEDs turning on, 0 for the LEDs turning off,
    and -1 if we received a totally bogus command that didn't do anything to the LEDs.
    */

    if (command=="off") {
        
        attitude_control_enabled = false;

        front_left.writeMicroseconds(1000);
        front_right.writeMicroseconds(1000);
        rear_left.writeMicroseconds(1000);
        rear_right.writeMicroseconds(1000);
        
        return 1;
    }
    else if (command=="idle"){

      // Set to (theoretical) 8% of power
      front_left.writeMicroseconds(1080);
      front_right.writeMicroseconds(1080);
      rear_left.writeMicroseconds(1080);
      rear_right.writeMicroseconds(1080);

      return 1;
    }
    else if (command=="spin up"){
      
      for (int i = 1000; i <= u_0; i += 5) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        
        front_left.writeMicroseconds(i);
        front_right.writeMicroseconds(i);
        rear_left.writeMicroseconds(i);
        rear_right.writeMicroseconds(i);
        
        delay(200);
      }
      
      return 1;
    }
    else if (command=="enable"){
        attitude_control_enabled = true;
        return 1;
    }
    else if (command=="disable"){
        attitude_control_enabled = false;
        return 1;
    }
    else
      return -1;   
}
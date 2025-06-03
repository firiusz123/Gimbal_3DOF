#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>
#include <Arduino_LSM6DS3.h>
#include <Servo.h>

// Servo control
Servo servoRoll;
Servo servoPitch;
Servo servoYaw;

const int SERVO_ROLL_PIN = 9;
const int SERVO_PITCH_PIN = 10;
const int SERVO_YAW_PIN = 2;

// PID parameters (tune individually per axis)
float Kp[3] = {1.5, 1.5, 1};
float Ki[3] = {0.0, 0.0, 0};
float Kd[3] = {0.1, 0.1, 0};

// PID state
float errorSum[3] = {0, 0, 0};
float lastError[3] = {0, 0, 0};

// Angles: 0 = Roll, 1 = Pitch, 2 = Yaw
float measuredAngles[3] = {0, 0, 0};
float desiredAngles[3]  = {0, 0, 0};  // Set externally (e.g. via Serial)
float fusedAngle[3] = {0, 0, 0};
// value for complementary filter
static float acc_measure[3];
static float mag_measure[3];
static float pidOutput[3], error[3], dError[3];
static float  acc_x, acc_y, acc_z;
static float  mag_x, mag_y, mag_z;
static float dt;

// Static variables to keep previous magnetometer values for filtering
static float prev_mag_x = 0.0;
static float prev_mag_y = 0.0;
static float prev_mag_z = 0.0;


const float MAG_THRESHOLD = 1;
const float alpha = 1;

unsigned long lastTime = 0;

/// defines 
#define PI  3.14159 
#define START_COMMAND_CHAR ':'
#define SEPARATE_VALUE_CHAR "|"

/////////////////// variables setting 

SemaphoreHandle_t dataSemaphore;  // Semaphore for data synchronization
SemaphoreHandle_t pass_angles_Semaphore;

bool SpecialCharacterCapturing = false;
unsigned long lastTransfer = 0;
#define MAX_VALUES 4 
#define TIMEOUT_MS 2000
char command[50];
int CommandIndex = 0;

///////////////////////////////////////
//sensor data config 

// Task to read serial data
void readSerialTask(void *pvParameters) 
{
    
    while (1) {
      
        while (Serial.available()) 
        {

            char c = Serial.read();

            if(millis() - lastTransfer > TIMEOUT_MS)
            {
                SpecialCharacterCapturing = false;    
            }

            if(c == START_COMMAND_CHAR)
            {
              if(SpecialCharacterCapturing)
              {
                //if already triggered it means it finished
                
                SpecialCharacterCapturing = false;
                //Serial.println(receivedData);
                xSemaphoreGive(dataSemaphore);
              }

              else
              {     
                //clearing data buffer and setting the character saving to true                           
                memset(command, 0, sizeof(command));
                CommandIndex = 0 ;
                SpecialCharacterCapturing = true;  
                lastTransfer = millis();            
              }

            }
            else if(SpecialCharacterCapturing)
            {
              command[CommandIndex++] = c;  
              
              
              
            }
            //Serial.println(receivedData);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);  // Small delay to avoid CPU overload
    }
}

// Task to write received serial data
void commandChecker(void *pvParameters) 
{
    while (1) {
        if (xSemaphoreTake(dataSemaphore, portMAX_DELAY) == pdTRUE) {
            //Serial.print("Received: ");
            //Serial.println(receivedData);

           // if (comm.length() > 2) {
                // Trim leading and trailing delimiters
                //String trimmed = receivedData.substring(0, receivedData.length() - 1);
                //Serial.println(receivedData);
                //receivedData.trim();
                //char command[50];
                //receivedData.toCharArray(command, sizeof(command));
                

                char *token = strtok(command, SEPARATE_VALUE_CHAR);
                char *commandType = token;
                
                Serial.print("Command: ");
                Serial.println(commandType);
                int valueIndex = 0;
                while ((token = strtok(NULL, SEPARATE_VALUE_CHAR)) != NULL && valueIndex < MAX_VALUES) {
                    Serial.println(token);
                    desiredAngles[valueIndex++] = atof(token);
                }

                
                if(strcmp(commandType, "GETINFO") == 0)
                {
                  
                  xSemaphoreGive(pass_angles_Semaphore);
                }
                else if(commandType == "ROT")
                {
                  for (int i = 0; i < valueIndex; i++) 
                  {
                      Serial.print("Value ");
                      Serial.print(i + 1);
                      Serial.print(": ");
                      Serial.println(desiredAngles[i]);
                  }
                }
            //} else {
            //    Serial.println("Received data too short.");
            }

            
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    
}

void control_Loop(void *pvParameters)
{
    static float acc_measure[3];
    static float mag_measure[3];
    static float pidOutput[3], error[3], dError[3];
    static float  acc_x, acc_y, acc_z;
    static float  mag_x, mag_y, mag_z;
    static float dt;

    // Static variables to keep previous magnetometer values for filtering
    static float prev_mag_x = 0.0;
    static float prev_mag_y = 0.0;
    static float prev_mag_z = 0.0;

    

    while (1)
    {
        // Timing
        unsigned long now = millis();
        dt = (now - lastTime) / 1000.0; // in seconds
        lastTime = now;
      //////

        if (IMU.accelerationAvailable()) 
        {
          IMU.readAcceleration(acc_x, acc_y, acc_z);
        }
        acc_measure[0] = acc_x;
        acc_measure[1] = acc_y;
        acc_measure[2] = acc_z;
        
      if (IMU.gyroscopeAvailable()) 
      {
        IMU.readGyroscope(mag_x, mag_y, mag_z);

        // Apply threshold filter to magnetometer values
        if (fabs(mag_x - prev_mag_x) < MAG_THRESHOLD) {
            mag_x = prev_mag_x;
        } else {
            prev_mag_x = mag_x;
            fusedAngle[0] += mag_x * dt;
        }

        if (fabs(mag_y - prev_mag_y) < MAG_THRESHOLD) {
            mag_y = prev_mag_y;
        } else {
            prev_mag_y = mag_y;
            fusedAngle[1] += mag_y * dt;
        }

        if (fabs(mag_z - prev_mag_z) < MAG_THRESHOLD) {
            mag_z = prev_mag_z;
        } else {
            prev_mag_z = mag_z;
            fusedAngle[2] += mag_z * dt;
        }
      }

          // Roll
          // Pitch
          // Yaw (will drift unless magnetometer used)

        // Calculate angle from accelerometer (in degrees)
        static float accRoll  = atan2(acc_y, acc_z) * 180.0 / PI;
        static float accPitch = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 180.0 / PI;

        // Complementary filter
        fusedAngle[0] = alpha * fusedAngle[0] + ((1 - alpha) * accRoll);
        fusedAngle[1] = alpha * fusedAngle[1] + ((1 - alpha) * accPitch);
        
        //Serial.print(":ANGLE|");
        //Serial.print(fusedAngle[0], 3);  // 3 decimal places
        //Serial.print("|");
        //Serial.print(fusedAngle[1], 3);
        //Serial.print("|");
        //Serial.print(fusedAngle[2], 3);
        //Serial.println("|:");

        // PID control for Roll, Pitch, Yaw
        for (int i = 0; i < 3; i++) {
            error[i] = desiredAngles[i] - fusedAngle[i];
            errorSum[i] += error[i] * dt;
            dError[i] = (error[i] - lastError[i]) / dt;

            pidOutput[i] = Kp[i] * error[i] + Ki[i] * errorSum[i] + Kd[i] * dError[i];
            lastError[i] = error[i];
        }

        // Convert to servo angles
        int rollAngle  = constrain(90 + pidOutput[0], 0, 180);
        int pitchAngle = constrain(90 + pidOutput[1], 0, 180);
        int yawAngle   = constrain(90 + pidOutput[2], 0, 180);

        // Output to servos
        servoRoll.write(rollAngle);
        servoPitch.write(pitchAngle);
        servoYaw.write(yawAngle);

        vTaskDelay(10 / portTICK_PERIOD_MS); // ~100Hz
    }
}

void displayOrientationTask(void *pvParameters) {
  while(1) {
    if(xSemaphoreTake(pass_angles_Semaphore, 0) == pdTRUE)
    {
    Serial.print(":INFO|");
    Serial.print(fusedAngle[0], 3);  // 3 decimal places
    Serial.print("|");
    Serial.print(fusedAngle[1], 3);
    Serial.print("|");
    Serial.print(fusedAngle[2], 3);
    Serial.println("|:");

    vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
}

void setup() {
    Serial.begin(9600); 
    pinMode(2, OUTPUT);
    servoRoll.attach(SERVO_ROLL_PIN);
    servoPitch.attach(SERVO_PITCH_PIN);
    servoYaw.attach(SERVO_YAW_PIN);
    lastTime = millis();

    if (!IMU.begin()) 
    {
      while (1);
    }

    dataSemaphore = xSemaphoreCreateBinary();
    pass_angles_Semaphore = xSemaphoreCreateBinary();

    
    xTaskCreate(readSerialTask, "ReadSerialTask", 128, NULL, 1, NULL);
    xTaskCreate(commandChecker, "commandChecker", 128, NULL, 1, NULL);
    xTaskCreate(control_Loop , "control Loop" , 256 ,NULL , 1 , NULL);
    xTaskCreate(displayOrientationTask, "Display", 256, NULL, 1, NULL);
}

void loop() 
{
    // FreeRTOS handles tasks, so no need for code in loop()
}


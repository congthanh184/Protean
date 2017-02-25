#include <core.h>
#include <Wire.h>

#include "config.h"

#include <sys/types.h>
#include <unistd.h>

#include <sys/resource.h>
#include <signal.h>

//=========================
#include <time.h>
#include <sys/stat.h>
#include <stdlib.h>

//#define DEBUG_MODE

time_t rawtime;
struct tm * timeinfo;
FILE *fp;


#ifdef DEBUG_MODE
	uint32_t NumberOfLine = 100;	// for testing
#else
	uint32_t NumberOfLine = 3000;	// 15mins
#endif

uint32_t countLine = 0;
//=========================


TinyGPS gps;
MPU6050 mpu;
SFE_BMP180 bmp;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
int16_t accel[3];
int16_t magX, magY, magZ;

// packet structure for InvenSense teapot demo
uint8_t countSample = 0;
uint32_t previousMillis;

// GPS data
float flat, flon;

// Ambient temprature
double ambientTemp;

// flow sensor
volatile uint16_t _pulse;
float flowPulse;

// ADC reading
uint8_t ADC_PIN2 = A2;
uint8_t ADC_PIN3 = A3;
uint8_t ADC_PIN4 = A4;

uint16_t adcVal2, adcVal3, adcVal4;

char csvHeader[] = "q0,q1,q2,q3,accelX,accelY,accelZ,magX,magY,magZ,flat,flon,ambientTemp,flowPulse,IR Temperature(C),ADC2,ADC3,ADC4";

Thread inr200msThread, inr5minThread, inr2secThread;
ThreadController threadManager = ThreadController();
//===========================================================================================
volatile bool mpuInterrupt = false;
void dmpDataReady() {
	mpuInterrupt = true;
}


//===========================================================================================
void imuInit() {
	// Wire.begin();
	// delay(50);    // wait a bit before start to talk with IMU

	printf("Initialize I2C devices ... \n");
	mpu.initialize();
	printf(mpu.testConnection() ? ("MPU6050 connection successful\n") : ("MPU6050 connection failed\n"));

	uint8_t devStatus = mpu.dmpInitialize();

	if (devStatus == 0) {
		printf("\nEnable DMP ...\n");
		mpu.setDMPEnabled(true);    

		// printf("Enable interrupt detection\n");
		// attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();
		dmpReady = true;
		packetSize = mpu.dmpGetFIFOPacketSize();
		printf("Fifo Count = %d && Packet Size = %d\n", fifoCount, packetSize);
	}
	else {
		printf("Failed\n");
	} 
}

void imuGetValue() {
	if (!dmpReady) return;
	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(("FIFO overflow!"));

	// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} 
	else if (mpuIntStatus & 0x01) {
			countSample++;
			// wait for correct available data length, should be a VERY short wait
			while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

			// read a packet from FIFO
			mpu.getFIFOBytes(fifoBuffer, packetSize);

			// track FIFO count here in case there is > 1 packet available
			// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= packetSize;
		
			// display quaternion values in easy matrix form: w x y z
			mpu.dmpGetQuaternion(&q, fifoBuffer);

			mpu.dmpGetAccel( accel, fifoBuffer);
			// int16_t x, y, z;
			mpu.getMag(&magX, &magY, &magZ);

			// printf("[%f, %f, %f, %f, %d, %d, %d]\n", q.w, q.x, q.y, q.z, x, y, z);
	}
//	logData();
}
//===========================================================================================
void gpsInit() {
	Serial.begin(9600);
	delay(1000);
}

void gpsGetValue() {
	bool newData = false;

	uint32_t last = millis();

	while (Serial.available())
	{
		char c = Serial.read();
		// printf("%c", c); // uncomment this line if you want to see the GPS data flowing
		if (gps.encode(c)) // Did a new valid sentence come in?
			newData = true;
	}

	// printf("time = %d\n", millis() - last);
	if (newData)
	{
		unsigned long age;
		gps.f_get_position(&flat, &flon, &age);
		// printf("[Lat: %f, LON: %f]\n", flat, flon);
		inr5minThread.setInterval(60000);
	}
}
//===========================================================================================
void bmpInit() {
	if (bmp.begin())	{
		printf("BMP180 init success\n");
	}
	else {
		printf("BMP180 init fail\n");
	}
}

void bmpGetValue() {
	char status;
	double T;
	status = bmp.startTemperature();
	if (status != 0)
	{
		// printf("yes delay!!\n");
		delay(status);
		status = bmp.getTemperature(T);
		if (status!=0)
		{
			// printf("temperature: %f\n", T);
			ambientTemp = T;
		}
	}
	
}
//===========================================================================================
uint32_t flowStartMillis;

// output pulse/ms => pulse/s
void flowGetValue() {
	uint32_t interval = millis() - flowStartMillis;
	flowPulse = (float)_pulse/interval; // pulse/ms 
	flowStartMillis = millis();
	_pulse = 0;
}

void pulseCount() {
	_pulse++;
}

void flowSensorInit() {
	attachInterrupt(0, pulseCount, FALLING);
	flowStartMillis = millis();
}

//============================================================================================
static const byte PIN_DATA    = 4; // Choose any pins you like for these
static const byte PIN_CLOCK   = 3;
static const byte PIN_ACQUIRE = 5;


// int led = 13;
// int led_state = 0;

void irTempInit() {
  pinMode(PIN_ACQUIRE, HIGH);

  pinMode(PIN_CLOCK, INPUT);
  pinMode(PIN_DATA, INPUT);

  digitalWrite(PIN_CLOCK, HIGH);
  digitalWrite(PIN_DATA, HIGH);

  attachInterrupt(1, readIrTempratureISR, RISING);
  printf("Finish IR temp init\n"); 
  // pinMode(led, OUTPUT);
}

volatile uint8_t _data_byte, _data_bit; 
volatile uint8_t _data[5];
const int IRTEMP_DATA_SIZE = 5;
volatile uint8_t ENABLE_IR = 0;
volatile uint8_t _countUnValid = 0;
volatile uint16_t irMsb = 0, irLsb = 0;
volatile uint8_t irNewData = 0;
float irTemperature;



void enableIrReading() {

  attachInterrupt(1, readIrTempratureISR, RISING);

  uint32_t delay15ms = millis() + 15;
  while (millis() < delay15ms) {};

  ENABLE_IR = 1;
  _data_byte = 0;
  _data_bit = 7;
  digitalWrite(PIN_ACQUIRE, LOW);
  // printf("enable IR reading\n");
}

bool validData(volatile byte data[]) {

  uint8_t checksum = (data[0] + data[1] + data[2]) & 0xff;

  return(data[3] == checksum  &&  data[4] == '\r');
}

void readIrTempratureISR() {
  
  if (ENABLE_IR)
  {
    if (digitalRead(PIN_DATA))
    {
      _data[_data_byte] |= 1<<_data_bit;
    }

    if (_data_bit > 0)
    {
      _data_bit--;
    }
    else 
    {
    	_data_bit = 7;
      	_data_byte++;
    	// led_state = !led_state;
        // digitalWrite(led, led_state);

      if (_data_byte >= IRTEMP_DATA_SIZE)
      {
        if (_data[0] == 0x4c && validData(_data)) {
          ENABLE_IR = 0;
          digitalWrite(PIN_ACQUIRE, HIGH);
          irMsb = _data[1] << 8;
          irLsb = _data[2];
          irNewData = 1;
          detachInterrupt(1);
        }
        else {
          _countUnValid++;
          if (_countUnValid > 3)
          {
            ENABLE_IR = 0;
            _countUnValid = 0;
            digitalWrite(PIN_ACQUIRE, HIGH);
          	detachInterrupt(1);
          }
        }
        _data_byte = 0;
      }
      _data[_data_byte] = 0;
    }
  }
}

void irTempGetValue() {
	if (irNewData)
	{
		irTemperature = (irMsb + irLsb)/16.0 - 273.15;
		// printf("%.3f\n", irTemperature);
		irNewData = 0;
	}
}

//============================================================================================
char file_name[50];
char folderr[50];

void logData() {

	if (fp==NULL)
	{
		// create new folder => open file => reset count
	    time( &rawtime );
	    timeinfo = localtime( &rawtime );
	    //printf("Current local time and date: %s", asctime(timeinfo));
	    sprintf(folderr, "%02d-%02d-%d", timeinfo->tm_mday, timeinfo->tm_mon+1, timeinfo->tm_year+1900);

	    struct stat sb;
	    if (stat(folderr, &sb) == 0 && S_ISDIR(sb.st_mode))
	    {
	        // printf("The directory is existed.\n");
	    }
	    else {
	        printf("Creating new directory.\n");
	        mkdir(folderr, 0777);
	    }
	    char path_file[50];	    
        sprintf(path_file, "%s/%02d-%02d-%02d.csv", folderr, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
        sprintf(file_name, "%02d-%02d-%02d.csv", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
        fp = fopen(path_file, "a");
        fprintf(fp, "%s\n", csvHeader);
        countLine = 0;
	}
	else {
		countLine++;
		fprintf(fp, "%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%d,%d,%d\n", q.w, q.x, q.y, q.z, accel[0], accel[1], accel[2], magX, magY, magZ, flat, flon, ambientTemp, flowPulse, irTemperature, adcVal2, adcVal3, adcVal4);
#ifdef DEBUG_MODE		
		printf(		"%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%d,%d,%d\n", q.w, q.x, q.y, q.z, accel[0], accel[1], accel[2], magX, magY, magZ, flat, flon, ambientTemp, flowPulse, irTemperature, adcVal2, adcVal3, adcVal4);		
#endif
		// printf("%d\n", countLine);
		if (countLine >= NumberOfLine) {
			countLine = 0;			
			char fcReturn = fclose(fp);		// if close successfully it returns 0, not clear file pointer
			fp = NULL;
			if (fcReturn!=0)
			{
				printf("Failed to cLose file!\n");
			}

			// Start a new thread to upload the data
//			pid_t pID = fork();
//			if (pID == 0)
//			{
				// char py_command[100] = "python pydrive_upload.py";
				// sprintf(py_command, "%s %s %s", py_command, folderr, file_name);
				// system(py_command);
				// printf("Exit child thread!\n");
				// 
				
				// because the parent thread got nice value at -18 so add 28 more to have 10 niceness
				// execl("/usr/bin/nice", "/usr/bin/nice", "-n", "28", "./pydrive_upload.py", folderr, file_name, (char *)0);
				// printf("Should never print it\n");
				// exit(0);
//			}
		}
	}
}



void inr200msSensorPoll() {
	// uint32_t millisThread = millis();
	imuGetValue();
	// flowGetValue();
	irTempGetValue();

	adcVal2 = analogRead(ADC_PIN2);
	adcVal3 = analogRead(ADC_PIN3);
	adcVal4 = analogRead(ADC_PIN4);

	logData();
	// printf("execute time 200ms=%d\n", millis()-millisThread);

}

void inr5minSensorPoll() {
	// uint32_t millisThread = millis();
	gpsGetValue();
	// printf("pass gps\n");
	bmpGetValue();
	// printf("pass bmp\n");
	// printf("execute time 5 min=%d\n", millis()-millisThread);
}

uint8_t state = HIGH;
uint8_t statusPin = 10; 

void inr2secSensorPoll() {
	flowGetValue();
	enableIrReading();

	digitalWrite(statusPin, state);
	if (state==HIGH)
  	{
  		state = LOW;
	}  
	else {
		state = HIGH;
	}
}
//===========================================================================================
void setup() {
	// disable signal from child process so it will automatically 
	// exit after finish its work, and not turn to zombie process
	// signal(SIGCHLD, SIG_IGN);

	// turn off all interrupt before setup
	noInterrupts();

	Wire.begin();	// have to init first for the other functions
	delay(50);    	// wait a bit before start to talk with IMU

	imuInit();
	gpsInit();
	bmpInit();
	flowSensorInit();

	irTempInit();

	pinMode(statusPin, OUTPUT);
        
	inr200msThread = Thread(inr200msSensorPoll, 200);
	inr200msThread.run();

	inr5minThread = Thread(inr5minSensorPoll, 5000);
	// delay(1000);		// delay 1sec for GPS reading.
	inr5minThread.run();

	inr2secThread = Thread(inr2secSensorPoll, 2000);
	inr2secThread.run();

	threadManager.add(&inr200msThread);
	threadManager.add(&inr5minThread);
	threadManager.add(&inr2secThread);
}


void loop() {

	threadManager.run();

}

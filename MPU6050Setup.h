#ifndef __DMP_STATUS_H__
#define __DMP_STATUS_H__

#include "Adafruit_MPU6050.h"
#include "Logging.h"

struct SensorState
{
	bool hasData(Adafruit_MPU6050 &mpu)
	{
		return mpu.getMotionInterruptStatus();
	}

	void readData(Adafruit_MPU6050 &mpu)
	{
		mpu.getEvent(&a, &g, &t);
	}

	sensors_event_t a, g, t;
};

/// initialize the MPU and its digital motion processing
void initMPU(Adafruit_MPU6050 &mpu)
{
	log("Initializing MPU6050 ... ");
	logln("");
	if (!mpu.begin())
	{
		logln("connection failed!");
		while (true) delay(10);
	}
	logln("connection successful!");

	// setup motion detection
	//mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
	//mpu.setMotionDetectionThreshold(1);
	//mpu.setMotionDetectionDuration(20);
	//mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
	//mpu.setInterruptPinPolarity(true);
	//mpu.setMotionInterrupt(true);
	mpu.setTemperatureStandby(true);
	mpu.setGyroStandby(true, true, true);
	mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

	Serial.println("");
	delay(100);
}

#endif // __DMP_STATUS_H__

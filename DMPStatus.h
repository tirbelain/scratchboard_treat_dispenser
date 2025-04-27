#ifndef __DMP_STATUS_H__
#define __DMP_STATUS_H__

#include <math.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "Logging.h"

/*---MPU6050 Control/Status Variables---*/
struct DMPStatus
{
	bool DMPReady = false;  // Set true if DMP init was successful
	uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
	uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
	uint8_t FIFOBuffer[64]; // FIFO storage buffer
};

struct MotionSensorState
{
	/*---Orientation/Motion Variables---*/ 
	Quaternion q;			// [w, x, y, z]			Quaternion container
	VectorInt16 aa;			// [x, y, z]			Accel sensor measurements
	VectorInt16 gy;			// [x, y, z]			Gyro sensor measurements
	VectorInt16 aaReal;		// [x, y, z]			Gravity-free accel sensor measurements
	//VectorInt16 aaWorld;	// [x, y, z]			World-frame accel sensor measurements
	VectorFloat gravity;	// [x, y, z]			Gravity vector
	float ypr[3];			// [yaw, pitch, roll]	Yaw/Pitch/Roll container and gravity vector

	void read(MPU6050 &mpu, DMPStatus &dmpStatus)
	{
		mpu.dmpGetQuaternion(&q, dmpStatus.FIFOBuffer);
		mpu.dmpGetAccel(&aa, dmpStatus.FIFOBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		mpu.dmpGetYawPitchRoll(&ypr[0], &q, &gravity);
	}

	void getYPRInDegrees(float (&yprDeg)[3])
	{
		const float toDeg = 180.0 / M_PI;

		yprDeg[0] = ypr[0] * toDeg;
		yprDeg[1] = ypr[1] * toDeg;
		yprDeg[2] = ypr[2] * toDeg;
		//float yprDeg[3] = {0, 0, 0};
		//return yprDeg;
	}
};


////////////////////////////////////////////////////////////
// interrupt callback function
volatile bool hasMPUInterrupt = false;
void DMPDataReady()
{
	hasMPUInterrupt = true;
}

/// initialize the MPU and its digital motion processing
void initMPUAndDMP(int interruptPin, MPU6050 &mpu, DMPStatus &dmpStatus)
{
	debugLog("Initializing MPU ... ");
	mpu.initialize();
	pinMode(interruptPin, INPUT);

	if (mpu.testConnection())
	{
		debugLogln("connection successful!");
	}
	else
	{
		debugLogln("connection failed!");
		while (true);
	}

	debugLog("Initialize DMP ... ");
	dmpStatus.devStatus = mpu.dmpInitialize();

#ifdef MPU_X_ACCEL_OFFSET
	mpu.setXAccelOffset(MPU_X_ACCEL_OFFSET);
#endif
#ifdef MPU_Y_ACCEL_OFFSET
	mpu.setYAccelOffset(MPU_Y_ACCEL_OFFSET);
#endif
#ifdef MPU_Z_ACCEL_OFFSET
	mpu.setZAccelOffset(MPU_Z_ACCEL_OFFSET);
#endif
#ifdef MPU_X_GYRO_OFFSET
	mpu.setXGyroOffset(MPU_X_GYRO_OFFSET);
#endif
#ifdef MPU_Y_GYRO_OFFSET
	mpu.setYGyroOffset(MPU_Y_GYRO_OFFSET);
#endif
#ifdef MPU_Z_GYRO_OFFSET
	mpu.setZGyroOffset(MPU_Z_GYRO_OFFSET);
#endif

	if (dmpStatus.devStatus == 0) // success
	{
		debugLogln("successful");
		mpu.CalibrateAccel(6);
		mpu.CalibrateGyro(6);
		//debugLogln("These are the Active offsets: ");
		//mpu.PrintActiveOffsets();
		//debugLogln("Enabling DMP...");   //Turning ON DMP
		mpu.setDMPEnabled(true);

		// enable interrupt
		attachInterrupt(digitalPinToInterrupt(interruptPin), DMPDataReady, RISING);
		dmpStatus.MPUIntStatus = mpu.getIntStatus();
		dmpStatus.packetSize = mpu.dmpGetFIFOPacketSize();
		dmpStatus.DMPReady = true;

		debugLogln("");
	}
	else
	{
		debugLog("DMP initialization failed (code ");
		debugLog(dmpStatus.devStatus);
		switch (dmpStatus.devStatus)
		{
		case 1:
			debugLog(" - initial memory load failed)");
			break;
		case 2:
			debugLog(" - DMP configuration updates failed)");
			break;
		}
	}
}

#endif // __DMP_STATUS_H__

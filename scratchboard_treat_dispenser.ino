#include "MPU6050_6Axis_MotionApps20.h"
#include "Servo.h"

#include "RingBuffer.h"
//#define DEBUG_OUTPUT
#include "Logging.h"

/* Supply your gyro offsets here, scaled for min sensitivity */
//#define MPU_X_ACCEL_OFFSET -5212
//#define MPU_Y_ACCEL_OFFSET 1323
//#define MPU_Z_ACCEL_OFFSET 1656
//#define MPU_X_GYRO_OFFSET 101
//#define MPU_Y_GYRO_OFFSET -52
//#define MPU_Z_GYRO_OFFSET 0
#include "DMPStatus.h"



enum RuntimeStates
{
	Prewarm,			// just waits a moment for vibrations to settle before starting the calibration
	PrimeRingBuffer,	// state where the mpu's noise level is measured to identify a save range where the servo will not be triggered
	Dispense
};

const int16_t	CALIBRATION_PREWARM_DURATION	= 5000;		// 5 seconds - delay before noise detection calibration starts

const int8_t	PIN_SERVO						= 9;
const int8_t	PIN_MPU_INTERRUPT				= 2;		// Define the interruption #0 pin

const uint32_t	SECONDS							= 1000u;
const uint32_t	MINUTES							= 60u * SECONDS;
const uint32_t	HOURS							= 60u * MINUTES;


const int SERVO_POSITION_STACK_0				= 0;
const int SERVO_POSITION_STACK_1				= 180;
const int SERVO_POSITION_CENTER					= 90;

////////////////////////////////////////////////////////////
// dispensing

// all these times are in milliseconds
const uint32_t	DISPENSE_COOLDOWN				= 5000u;			// When dispensing was triggered, the next trigger will not happen until this cooldown is complete.
const uint8_t	DISPENSE_AMOUNT					= 1u;				// The number of treats that will be dispensed when triggerd.
const uint32_t	DISPENSE_PERIOD_LENGTH			= 60u * MINUTES;	// The period is a means of limiting the number of treats that are dispensed over
const uint8_t	DISPENSE_AMOUNT_PER_PERIOD		= 3u;				// time. During a period only the set amount per period will be dispensed.
const uint32_t	DISPENSE_INTERACTION_TIME		= 3u * SECONDS;		// This is a means of controlling the sensitivity for triggering. Interactions with the MPU need to be recorded for at least this amount of time before the treat is dispensed. Higher values require longer interactions.
const uint32_t	DISPENSE_INTERACTION_TIMEOUT	= 6u * SECONDS;		// An interaction can consist of multiple small interactions, not only a continuous long one. This is the time before a interaction will time out. If another interaction is recorded within this period, this is considered the same interaction.


enum HopperSlot : uint8_t
{
	slot0,
	slot1
} nextHopperSlot = HopperSlot::slot0;

uint32_t dispenseCooldownEndTime = 0;

uint32_t recentDispenseTimes[DISPENSE_AMOUNT_PER_PERIOD];
uint16_t recentDispenseCount = 0;
uint32_t lastInteractionStartTime = -1;


////////////////////////////////////////////////////////////
// periphery interfaces
MPU6050 mpu;
Servo servo;

////////////////////////////////////////////////////////////
// periphery state
DMPStatus dmpStatus;
MotionSensorState sensorState;

////////////////////////////////////////////////////////////
// state machine
RuntimeStates calibrationState = RuntimeStates::Prewarm;
unsigned long calibrationStateChangedTime = 0;

////////////////////////////////////////////////////////////
// MPU sample data
const uint8_t SAMPLE_RING_BUFFER_SIZE = 50;
RingBuffer accelSamplesX(SAMPLE_RING_BUFFER_SIZE), accelSamplesY(SAMPLE_RING_BUFFER_SIZE), accelSamplesZ(SAMPLE_RING_BUFFER_SIZE);
int deadzoneX, deadzoneY, deadzoneZ;


void setup()
{
	Serial.begin(115200);
	while (!Serial);

	initI2C();

	initMPUAndDMP(PIN_MPU_INTERRUPT, mpu, dmpStatus);

	pinMode(LED_BUILTIN, OUTPUT);
	analogWrite(LED_BUILTIN, 0);

	servo.attach(PIN_SERVO);
	servo.write(SERVO_POSITION_CENTER); // center servo

	setState(RuntimeStates::Prewarm);
}

/// initialize the I2C interface
void initI2C()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
    Wire.setWireTimeout(3000, true);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
}

void loop()
{
	if (mpu.dmpGetCurrentFIFOPacket(dmpStatus.FIFOBuffer))
	{
		sensorState.read(mpu, dmpStatus);

		switch (calibrationState)
		{
		case RuntimeStates::Prewarm:
			prewarm();
			break;
		case RuntimeStates::PrimeRingBuffer:
			primeRingBuffer();
			break;
		case RuntimeStates::Dispense:
			monitorAccelerometerForInteraction();
			break;
		}

	}
	delay(100);
}

////////////////////////////////////////////////////////////
// advances the state machine to the next state
void advanceToNextState()
{
	switch (calibrationState)
	{
	case RuntimeStates::Prewarm:
		setState(RuntimeStates::PrimeRingBuffer);
		break;
	case RuntimeStates::PrimeRingBuffer:
		setState(RuntimeStates::Dispense);
		break;
	case RuntimeStates::Dispense:
		break;
	}
}
////////////////////////////////////////////////////////////
// code executed when the state changes
void onEnterState()
{
	switch (calibrationState)
	{
	case RuntimeStates::Prewarm:
		logln("New State: Prewarm");
		break;
	case RuntimeStates::PrimeRingBuffer:
		logln("New State: PrimeRingBuffer");
		break;
	case RuntimeStates::Dispense:
		logln("New State: Dispense");
		// wiggle the servo once to inform about the servo review is ready
		servo.write(SERVO_POSITION_CENTER - 10);
		delay(250);
		servo.write(SERVO_POSITION_CENTER);
		break;
	}
}


////////////////////////////////////////////////////////////
// sets the state machine to a new state
void setState(RuntimeStates newState)
{
	calibrationState = newState;
	calibrationStateChangedTime = millis();
	onEnterState();
}


////////////////////////////////////////////////////////////
// the first state. nothing relevant happens here. the code runs a countdown until it is completed then switches to the next state
void prewarm()
{
	if (doStateCountDown(CALIBRATION_PREWARM_DURATION))
	{
		advanceToNextState();
		return;
	}
}


////////////////////////////////////////////////////////////
// fills the ring buffer until it is completely filled with sensor data for a couple of seconds.
// this is done to get the average noise level for the sensors.
// when done, the gathered min/max vales are used to calculate the deadzones where the servo
// should not be triggered.
void primeRingBuffer()
{
	if (accelSamplesX.getUsedSampleCount() == accelSamplesX.getTotalSampleCount())
	{
		deadzoneX = (accelSamplesX.getMax() - accelSamplesX.getMin());
		deadzoneY = (accelSamplesY.getMax() - accelSamplesY.getMin());
		deadzoneZ = (accelSamplesZ.getMax() - accelSamplesZ.getMin());
		logln("Deadzones:");
		log("    x: "); logln(deadzoneX);
		log("    y: "); logln(deadzoneY);
		log("    z: "); logln(deadzoneZ);

		advanceToNextState();
		return;
	}

	accelSamplesX.add(sensorState.aaReal.x);
	accelSamplesY.add(sensorState.aaReal.y);
	accelSamplesZ.add(sensorState.aaReal.z);

	//plotSample(accelSamplesX, "accX", true, true, true, false);
	//plotSample(accelSamplesY, "accY", true, false, false, false);
	//plotSample(accelSamplesZ, "accZ", true, false, false, false);
}


////////////////////////////////////////////////////////////
// the main state for operation. this moitors accelerometer values and triggers the servo if
// an interaction is detected.
void monitorAccelerometerForInteraction()
{
	// update the ring buffer for being able to react to sensor value drift
	// TODO: this should probably ignore large values that triggered an interaction as this will
	// have an impact on the average value, that the detection is based upon. due to the large number
	// of samples this shouldn't be too big of a problem though
	accelSamplesX.add(sensorState.aaReal.x);
	accelSamplesY.add(sensorState.aaReal.y);
	accelSamplesZ.add(sensorState.aaReal.z);

	//plotSample(accelSamplesX, "accX", false, false, false, false);
	//plotSample(accelSamplesY, "accY", false, false, false, false);
	//plotSample(accelSamplesZ, "accZ", false, false, false, false);

	int16_t avgX = accelSamplesX.getAvg();
	//log("\txMin:");
	//log(avgX - deadzoneX);
	//log("\txMax:");
	//log(avgX + deadzoneX);

	int16_t avgY = accelSamplesY.getAvg();
	//log("\tyMin:");
	//log(avgY - deadzoneY);
	//log("\tyMax:");
	//log(avgY + deadzoneY);

	int16_t avgZ = accelSamplesZ.getAvg();
	//log("\tzMin:");
	//log(avgZ - deadzoneZ);
	//log("\tzMax:");
	//log(avgZ + deadzoneZ);
	//logln("");

	uint32_t now = millis();
	// check the current acceleration values against the deadzone to detect an interaction
	if (abs(accelSamplesX.getLast() - avgX) > deadzoneX ||
		abs(accelSamplesY.getLast() - avgY) > deadzoneY ||
		abs(accelSamplesZ.getLast() - avgZ) > deadzoneZ)
	{
		for (int i = 0; i < DISPENSE_AMOUNT; ++i)
		{
			dispense(now);
		}
	}

	// check recent dispense times for values that are older than the dispense-period and remove them
	// to unlock the next dispense
	if (canRemoveOldestDispenseTime(now))
	{
		removeOldestDispenseTime();
	}
}


////////////////////////////////////////////////////////////
// performs the dispensing. after a few checks that determin whether dispensing is allowed, select
// the hopper slot to take the treat from and trigger the servo
void dispense(uint32_t now)
{
	if (hasDispenseCooldown(now))
	{
		logln("Not dispensing! Dispense cooldown active.");
		return;
	}
	if (!hasPeriodDispensesLeft(now))
	{
		logln("Not dispensing. No dispenses left for current period.");
		return;
	}

	if (lastInteractionStartTime == 0 || lastInteractionStartTime + DISPENSE_INTERACTION_TIMEOUT < now)
	{
		// restart interaction. there wasn't one before or the current one timed-out
		lastInteractionStartTime = now;
		logln("Not dispensing. Just started the interaction.");
		return;
	}

	if (lastInteractionStartTime + DISPENSE_INTERACTION_TIME > now)
	{
		// interaction was not long enough. do not dispense
		logln("Not dispensing. Interaction was not long enough.");
		return;
	}

	logln("Dispensing");
	lastInteractionStartTime = 0;
	dispenseCooldownEndTime = now + DISPENSE_COOLDOWN;
	addDispenseTime(now);
	int servoTarget = SERVO_POSITION_CENTER;
	switch (nextHopperSlot)
	{
	case HopperSlot::slot0:
		servoTarget = SERVO_POSITION_STACK_0;
		nextHopperSlot = HopperSlot::slot1;
		break;
	case HopperSlot::slot1:
		servoTarget = SERVO_POSITION_STACK_1;
		nextHopperSlot = HopperSlot::slot0;
		break;
	default:
		break;
	}

	servo.write(servoTarget);
	delay(250);
	servo.write(SERVO_POSITION_CENTER);
	delay(250);
}


////////////////////////////////////////////////////////////
// checks whether there is a cooldown active at the moment that prevents dispensing
bool hasDispenseCooldown(uint32_t now)
{
	return now < dispenseCooldownEndTime;
}


////////////////////////////////////////////////////////////
// record the time for a dispense for the dispenses-per-period check
bool addDispenseTime(uint32_t now)
{
	if (!hasPeriodDispensesLeft(now)) return false;
	recentDispenseTimes[recentDispenseCount] = now;
	++recentDispenseCount;
	DEBUG_printRecentDispenseTimes();
	return true;
}


////////////////////////////////////////////////////////////
// checks for recent dispenses that are older than the dispense-period
bool canRemoveOldestDispenseTime(uint32_t now)
{
	return recentDispenseCount > 0
		&& (recentDispenseTimes[0] + DISPENSE_PERIOD_LENGTH) < now;
}


////////////////////////////////////////////////////////////
// remove old dispense times from the record to allow for more dispensing
void removeOldestDispenseTime()
{
	--recentDispenseCount;
	for (int i = 0; i < recentDispenseCount; ++i)
	{
		recentDispenseTimes[i] = recentDispenseTimes[i + 1];
	}
	DEBUG_printRecentDispenseTimes();
}


////////////////////////////////////////////////////////////
// checks the number of valid recent dispenses in order to determin whether dispensing is allowed at the moment
bool hasPeriodDispensesLeft(uint32_t now)
{
	return recentDispenseCount < DISPENSE_AMOUNT_PER_PERIOD;
}


////////////////////////////////////////////////////////////
// a non-blocking countdown that displays remaining seconds
// totalDuration is in milliseconds
bool doStateCountDown(uint16_t totalDuration)
{
	static int prevCountdownValue = 0;

	long timeSinceStateStart = millis() - calibrationStateChangedTime;
	long remaining = max(0, totalDuration - timeSinceStateStart);
	int countdownValue = ceil(remaining * 0.001f);
	if (countdownValue != prevCountdownValue)
	{
		log("Time Remaining: ");
		logln(countdownValue);
		prevCountdownValue = countdownValue;
	}

	return countdownValue == 0;
}


////////////////////////////////////////////////////////////
// debug output that logs the recent dispense times
void DEBUG_printRecentDispenseTimes()
{
	log("recent dispenses: ");
	for (int i = 0; i < recentDispenseCount; ++i)
	{
		log(recentDispenseTimes[i]);
		log(" ");
	}
	logln("");
}


////////////////////////////////////////////////////////////
//  Sends the state of the RingBuffer to the Serial Monitor/Plotter
void DEBUG_plotSample(const RingBuffer &sample, const char *label, bool avg, bool min, bool max, bool offsetByAvg)
{
	int16_t offset = 0;
	if (offsetByAvg)
	{
		offset = sample.getAvg();
	}

	log("\t");
	log(label);
	log("val:");
	log(sample.getLast() - offset);
	if (avg)
	{
		log("\t");
		log(label);
		log("avg:");
		log(sample.getAvg());
	}
	if (min)
	{
		log("\t");
		log(label);
		log("min:");
		log(sample.getMin() - offset);
	}
	if (max)
	{
		log("\t");
		log(label);
		log("max:");
		log(sample.getMax() - offset);
	}
}

//#define LOG_SENSOR_READINGS_X
//#define LOG_SENSOR_READINGS_Y
//#define LOG_SENSOR_READINGS_Z


#include "Adafruit_MPU6050.h"
#include "Servo.h"
#include "Wire.h"

#include "RingBuffer.h"
//#define DEBUG_OUTPUT
#include "Logging.h"

#include "MPU6050Setup.h"



enum RuntimeStates
{
	Prewarm,			// just waits a moment for vibrations to settle before starting the calibration
	PrimeRingBuffer,	// state where the mpu's noise level is measured to identify a save range where the servo will not be triggered
	Dispense
};

const int16_t	CALIBRATION_PREWARM_DURATION	= 5000;		// 5 seconds - delay before noise detection calibration starts

const int8_t	PIN_SERVO						= 9;

const uint32_t	SECONDS							= 1000u;
const uint32_t	MINUTES							= 60u * SECONDS;
const uint32_t	HOURS							= 60u * MINUTES;

const int SERVO_POSITION_STACK_0				= 0;
const int SERVO_POSITION_STACK_1				= 180;
const int SERVO_POSITION_CENTER					= 90;

////////////////////////////////////////////////////////////
// dispensing

// all these times are in milliseconds
const uint32_t	DISPENSE_COOLDOWN				= 5000u;			// When dispensing was triggered, the next trigger will not happen until this cooldown is over.
const uint32_t	DISPENSE_PERIOD_LENGTH			= 60u * MINUTES;	// The period is a means of limiting the number of treats that are dispensed over
const uint32_t	DISPENSE_INTERACTION_DURATION	= 3u * SECONDS;		// This is a means of controlling the sensitivity for triggering. Interactions with the MPU need to be recorded for at least this amount of time before the treat is dispensed. Higher values require longer interactions.
const uint32_t	DISPENSE_INTERACTION_TIMEOUT	= 2u * SECONDS;		// An interaction usually consist of multiple small interactions, not a continuous long one. This constant defines the time before a interaction will time out. If another interaction is recorded within this period, this is considered the same interaction.

const uint8_t	DISPENSE_AMOUNT					= 1u;				// The number of treats that will be dispensed when triggerd.
const uint8_t	DISPENSE_AMOUNT_PER_PERIOD		= 3u;				// During one period only the set amount of treats will be dispensed.
const float		DISPENSE_TRIGGER_SENSITIVITY	= 1.5f;				// Factor to scale the deadzone for sensor input. Smaller values increase sensitivity, ie. ie the trigger threshold is exceeded more easily. Larger values decrease sensitivity

enum HopperSlot : uint8_t
{
	slot0,
	slot1
} nextHopperSlot = HopperSlot::slot0;

uint32_t dispenseCooldownEndTime = 0;

uint32_t recentDispenseTimes[DISPENSE_AMOUNT_PER_PERIOD];
uint16_t recentDispenseCount = 0;
uint32_t interactionStartTime = 0;
uint32_t interactionEndTime = 0;


////////////////////////////////////////////////////////////
// periphery interfaces and state
Adafruit_MPU6050 mpu;
Servo servo;

SensorState sensorState;

////////////////////////////////////////////////////////////
// state machine
RuntimeStates runtimeState = RuntimeStates::Prewarm;
unsigned long runtimeStateChangedTime = 0;

////////////////////////////////////////////////////////////
// MPU sample data
const uint8_t SAMPLE_RING_BUFFER_SIZE = 50;
RingBuffer<float> accelSamplesX(SAMPLE_RING_BUFFER_SIZE), accelSamplesY(SAMPLE_RING_BUFFER_SIZE), accelSamplesZ(SAMPLE_RING_BUFFER_SIZE);
float deadzoneX, deadzoneY, deadzoneZ;


void setup()
{
	Serial.begin(115200);
	while (!Serial) delay(10);

	//initMPU(mpu);
	if (!mpu.begin()) delay(10);


	//pinMode(LED_BUILTIN, OUTPUT);
	//analogWrite(LED_BUILTIN, 0);

	servo.attach(PIN_SERVO);
	servo.write(SERVO_POSITION_CENTER); // center servo

	setState(RuntimeStates::Prewarm);
}

void loop()
{
	//if (sensorState.hasData(mpu))
	{
		sensorState.readData(mpu);

		switch (runtimeState)
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
	switch (runtimeState)
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
	switch (runtimeState)
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
	runtimeState = newState;
	runtimeStateChangedTime = millis();
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
		logln("Deadzones:");
		log("    x: "); logln(deadzoneX);
		log("    y: "); logln(deadzoneY);
		log("    z: "); logln(deadzoneZ);

		advanceToNextState();
		return;
	}

	accelSamplesX.add(sensorState.a.acceleration.x);
	accelSamplesY.add(sensorState.a.acceleration.y);
	accelSamplesZ.add(sensorState.a.acceleration.z);

	deadzoneX = DISPENSE_TRIGGER_SENSITIVITY * (accelSamplesX.getMax() - accelSamplesX.getMin());
	deadzoneY = DISPENSE_TRIGGER_SENSITIVITY * (accelSamplesY.getMax() - accelSamplesY.getMin());
	deadzoneZ = DISPENSE_TRIGGER_SENSITIVITY * (accelSamplesZ.getMax() - accelSamplesZ.getMin());
#ifdef LOG_SENSOR_READINGS_X
	float avgX = accelSamplesX.getAvg();
	DEBUG_plotSample(accelSamplesX, "accX", false, false, false, false);
	log("\txMin:");
	log(avgX - deadzoneX);
	log("\txMax:");
	log(avgX + deadzoneX);
#endif
#ifdef LOG_SENSOR_READINGS_Y
	float avgY = accelSamplesY.getAvg();
	DEBUG_plotSample(accelSamplesY, "accY", false, false, false, false);
	log("\tyMin:");
	log(avgY - deadzoneY);
	log("\tyMax:");
	log(avgY + deadzoneY);
#endif
#ifdef LOG_SENSOR_READINGS_Z
	float avgZ = accelSamplesZ.getAvg();
	DEBUG_plotSample(accelSamplesZ, "accZ", false, false, false, false);
	log("\tzMin:");
	log(avgZ - deadzoneZ);
	log("\tzMax:");
	log(avgZ + deadzoneZ);
#endif
#if defined(LOG_SENSOR_READINGS_X) || defined(LOG_SENSOR_READINGS_Y) || defined(LOG_SENSOR_READINGS_Z)
	logln("");
#endif
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
	accelSamplesX.add(sensorState.a.acceleration.x);
	accelSamplesY.add(sensorState.a.acceleration.y);
	accelSamplesZ.add(sensorState.a.acceleration.z);

	const bool logAvg = false;
	const bool logMin = false;
	const bool logMax = false;
	float avgX = accelSamplesX.getAvg();
	float avgY = accelSamplesY.getAvg();
	float avgZ = accelSamplesZ.getAvg();

#ifdef LOG_SENSOR_READINGS_X
	DEBUG_plotSample(accelSamplesX, "accX", logAvg, logMin, logMax, false);
	log("\txMin:");
	log(avgX - deadzoneX);
	log("\txMax:");
	log(avgX + deadzoneX);
#endif

#ifdef LOG_SENSOR_READINGS_Y
	DEBUG_plotSample(accelSamplesY, "accY", logAvg, logMin, logMax, false);
	log("\tyMin:");
	log(avgY - deadzoneY);
	log("\tyMax:");
	log(avgY + deadzoneY);
#endif

#ifdef LOG_SENSOR_READINGS_Z
	DEBUG_plotSample(accelSamplesZ, "accZ", logAvg, logMin, logMax, false);
	log("\tzMin:");
	log(avgZ - deadzoneZ);
	log("\tzMax:");
	log(avgZ + deadzoneZ);
#endif

#if defined(LOG_SENSOR_READINGS_X) || defined(LOG_SENSOR_READINGS_Y) || defined(LOG_SENSOR_READINGS_Z)
	logln("");
#endif

	uint32_t now = millis();
	// check the current acceleration values against the deadzone to detect an interaction
	if (abs(accelSamplesX.getLast() - avgX) > deadzoneX ||
		abs(accelSamplesY.getLast() - avgY) > deadzoneY ||
		abs(accelSamplesZ.getLast() - avgZ) > deadzoneZ)
	{
		// is there an active interaction? if not, start a new one (if not in cooldown)
		if (!hasDispenseCooldown(now) && !isInteracting(now))
		{
			// (re-)start interaction. there isn't an active interaction yet or the latest one has timed-out
			interactionStartTime = now;
			logln("Detected new interaction.");
		}
		interactionEndTime = now + DISPENSE_INTERACTION_TIMEOUT;

		if (isDispensingAllowed(now))
		{
			for (int i = 0; i < DISPENSE_AMOUNT; ++i)
			{
				dispense(now);
			}
		}
	}

	// check recent dispense times for values that are older than the dispense-period and remove them
	// to unlock the next dispense
	if (canRemoveOldestDispenseTime(now))
	{
		removeOldestDispenseTime();
	}
}


bool isInteracting(uint32_t now)
{
	return interactionStartTime != 0 && interactionStartTime <= now && now <= interactionEndTime; 
}


////////////////////////////////////////////////////////////
// performs a few checks whether dispensing is allowed at the moment
// 1. is there an active cooldown from the previous dispensing?
// 2. has the current interaction been long enough to trigger? (when this is called, there is always an active interaction)
bool isDispensingAllowed(uint32_t now)
{
	// the cooldown needs to be checked ouside the dispense function as the first dispense does
	// activate the cooldown which would prevent the following dispensations from happening
	if (hasDispenseCooldown(now))
	{
		logln("Not dispensing! Dispense cooldown active.");
		return false;
	}

	if (now < interactionStartTime + DISPENSE_INTERACTION_DURATION)
	{
		// interaction was not long enough. do not dispense
		logln("Not dispensing. Interaction was not long enough.");
		return false;
	}

	return true;
}


////////////////////////////////////////////////////////////
// performs the dispensing. after a few checks that determin whether dispensing is allowed, select
// the hopper slot to take the treat from and trigger the servo
void dispense(uint32_t now)
{
	if (!hasPeriodDispensationsLeft(now))
	{
		log("Not dispensing. No dispensations left for current period. Next available in ");
		log((DISPENSE_PERIOD_LENGTH - (now - recentDispenseTimes[0])) / SECONDS);
		logln(" seconds");
		return;
	}

	logln("Dispensing");
	interactionStartTime = 0; // clear the current interaction
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
// record the time for a dispense for the dispensations-per-period check
bool addDispenseTime(uint32_t now)
{
	if (!hasPeriodDispensationsLeft(now)) return false;
	recentDispenseTimes[recentDispenseCount] = now;
	++recentDispenseCount;
	DEBUG_printRecentDispenseTimes();
	return true;
}


////////////////////////////////////////////////////////////
// checks for recent dispensations that are older than the dispense-period
bool canRemoveOldestDispenseTime(uint32_t now)
{
	return recentDispenseCount > 0
		&& (recentDispenseTimes[0] + DISPENSE_PERIOD_LENGTH) < now;
}


////////////////////////////////////////////////////////////
// remove old dispense times from the record to allow for more dispensations
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
// checks the number of valid recent dispensations in order to determin whether dispensing is allowed at the moment
bool hasPeriodDispensationsLeft(uint32_t now)
{
	return recentDispenseCount < DISPENSE_AMOUNT_PER_PERIOD;
}


////////////////////////////////////////////////////////////
// a non-blocking countdown that displays remaining seconds
// totalDuration is in milliseconds
bool doStateCountDown(uint16_t totalDuration)
{
	static int prevCountdownValue = 0;

	long timeSinceStateStart = millis() - runtimeStateChangedTime;
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
	log("recent dispensations: ");
	for (int i = 0; i < recentDispenseCount; ++i)
	{
		log(recentDispenseTimes[i]);
		log(" ");
	}
	logln("");
}


////////////////////////////////////////////////////////////
//  Sends the state of the RingBuffer to the Serial Monitor/Plotter
void DEBUG_plotSample(const RingBuffer<float> &sample, const char *label, bool avg, bool min, bool max, bool offsetByAvg)
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

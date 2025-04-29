#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__

template<typename T>
class RingBuffer
{
public:
	RingBuffer(uint8_t count) :
		totalSampleCount(count),
		usedSampleCount(0),
		currentSample(-1),
		samples(new T[count])
	{
		for (int i = 0; i < totalSampleCount; ++i)
		{
			samples[i] = T();
		}
	}

	void resize(int8_t newCount)
	{
		totalSampleCount = newCount;
		usedSampleCount = 0;
		currentSample = -1;
		samples = new T[newCount];
		for (int i = 0; i < totalSampleCount; ++i)
		{
			samples[i] = T();
		}
	}

	void add(T v)
	{
		currentSample = currentSample + 1;
		currentSample = currentSample % totalSampleCount;

		usedSampleCount = min(usedSampleCount + 1, totalSampleCount);
		samples[currentSample] = v;
	}

	T getMin()
	{
		T val = samples[0]; 
		for (int i = 1; i < usedSampleCount; ++i)
		{
			val = min(samples[i], val);
		}
		return val;
	}

	T getMax()
	{
		T val = samples[0];
		for (int i = 1; i < usedSampleCount; ++i)
		{
			val = max(samples[i], val);
		}
		return val;
	}

	T getSum()
	{
		T val = 0;
		for (int i = 0; i < usedSampleCount; ++i)
		{
			val += samples[i];
		}
		return val;
	}

	T getAvg()
	{
		return usedSampleCount == 0 ? 0 : getSum() / usedSampleCount;
	}

	T getLast()
	{
		return currentSample < 0 ? 0 : samples[currentSample];
	}

	uint8_t getCurrentSample()
	{
		return currentSample;
	}

	uint8_t getTotalSampleCount()
	{
		return totalSampleCount;
	}

	uint8_t getUsedSampleCount()
	{
		return usedSampleCount;
	}

	void printSamples()
	{
		Serial.print("samples: ");
		if (usedSampleCount > 0)
		{
			Serial.print(samples[0]);
			if (0 == currentSample) Serial.print("*");
			for (int i = 1; i < usedSampleCount; ++i)
			{
				Serial.print(", ");
				Serial.print(samples[i]);
				if (i == currentSample) Serial.print("*");
			}
		}
		Serial.print(" (");
		Serial.print(currentSample);
		Serial.print(")");
		Serial.println("");
	}

	void printDerived()
	{
		Serial.print("avg: ");
		Serial.print(getAvg());
		Serial.print(" sum: ");
		Serial.print(getSum());
		Serial.print(" min: ");
		Serial.print(getMin());
		Serial.print(" max: ");
		Serial.print(getMax());
		Serial.println("");
	}

private:
	uint8_t totalSampleCount;
	uint8_t usedSampleCount;
	uint8_t currentSample;
	T *samples;
};

#endif // __RINGBUFFER_H__

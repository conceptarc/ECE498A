#include "buffer_wrapper.h"
#include <iostream>
#include <vector>

#define PI 3.14159
#define phaseOffset 0.23

using namespace std;

int main() {
	float SAMPLE_RATE = 30.3; // Hz
	int testSecondsLimit = 10; // seconds
	int noiseFrequencyLimit = 30; // hertz

	for (int noiseFreq = 0; noiseFreq < noiseFrequencyLimit + 1; noiseFreq++) {
		Buffer_Wrapper buffer = Buffer_Wrapper();
		float avgX = 0;
		float avgXb = 0;

		// only test X component - sufficient
		for (int i = 0; i < testSecondsLimit * SAMPLE_RATE; i++) {
			float t = i / ((float)SAMPLE_RATE);

			// phaseOffset is to avoid having sampling time be some multiple of frequency
			float xInput = (float)sin(2*PI*noiseFreq*t + phaseOffset);

			int id = 111;
			float xBuffer = 0;
			float yBuffer = 0;
			float dxBuffer = 0;
			float dyBuffer = 0;

			buffer.Update_Object(id, xInput, 0, t);
			buffer.Get_Object_Properties(id, xBuffer, yBuffer, dxBuffer, dyBuffer);
			avgX += abs(xInput);
			avgXb += abs(xBuffer);
		}
		avgX = avgX / (float)(testSecondsLimit * SAMPLE_RATE);
		avgXb = avgXb / (float)(testSecondsLimit * SAMPLE_RATE);
		float db = 20 * log10(avgXb / avgX);

		// notice the aliasing effect after 15 Hz due to 30 Hz sampling rate
		printf("noise_f = %2d, input_avg = %2.4f, output_avg = %2.4f, Db = %6.2f \r\n", noiseFreq, avgX, avgXb, db);
		//printf("\r\n");
	}

	char pause;
	cin >> pause;


	return 0;
}
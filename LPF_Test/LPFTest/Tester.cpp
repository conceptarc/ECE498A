#include "buffer_wrapper.h"
#include <iostream>
#include <vector>

#define PI 3.14159
#define phaseOffset 0.23

using namespace std;

int main() {
	float SAMPLE_RATE = 30.3f; // Hz
	int testSecondsLimit = 10; // seconds
	int noiseFrequencyLimit = 30; // hertz

	printf("noise_f, input_avg_power, output_avg_power, Db\r\n");
	for (float noiseFreq = 0; noiseFreq <= noiseFrequencyLimit + 0.1; noiseFreq += 0.1f) {
		Buffer_Wrapper buffer = Buffer_Wrapper();
		float avgPowerX = 0;
		float avgPowerXb = 0;

		// only test X component - sufficient
		for (int i = 0; i < testSecondsLimit * SAMPLE_RATE; i++) {
			float t = i / ((float)SAMPLE_RATE);

			// phaseOffset is to avoid having sampling time be some multiple of frequency
			float xInput = (float)sin(2 * PI*noiseFreq*t + phaseOffset);

			int id = 111;
			float xBuffer = 0;
			float yBuffer = 0;
			float dxBuffer = 0;
			float dyBuffer = 0;

			buffer.Update_Object(id, xInput, 0, t);
			buffer.Get_Object_Properties(id, xBuffer, yBuffer, dxBuffer, dyBuffer);
			avgPowerX += xInput*xInput/2;
			avgPowerXb += xBuffer*xBuffer/2;
		}
		avgPowerX = avgPowerX / (float)(testSecondsLimit * SAMPLE_RATE);
		avgPowerXb = avgPowerXb / (float)(testSecondsLimit * SAMPLE_RATE);
		float db = 20 * log10(avgPowerXb / avgPowerX);

		// notice the aliasing effect after 15 Hz due to 30 Hz sampling rate
		//printf("noise_f = %5.2f, input_avg = %2.4f, output_avg = %2.4f, Db = %6.2f \r\n", noiseFreq, avgX, avgXb, db);
		printf("%.1f,%.4f,%.4f,%.4f\r\n", noiseFreq, avgPowerX, avgPowerXb, db);
		//printf("\r\n");
	}

	char pause;
	cin >> pause;


	return 0;
}
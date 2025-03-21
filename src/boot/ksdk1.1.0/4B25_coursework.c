#include <stdlib.h>

#include "config.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devMMA8451Q.h"
#include "devSSD1331.h"

#include "4B25_coursework.h"
#include "math.h"

#define NUM_ACTIVITIES 4

// Activity stats (mean and variance)
static float activity_stats[NUM_ACTIVITIES][3][2] = {
    {{-0.28230, 1.14809}, {-0.33306, 1.72370}, {0.53284, 0.57563}},  // 'jabs'
    {{-0.74457, 0.0217}, {0.46312, 0.0364}, {-0.04632, 0.0243}},  // 'rest'
    {{-0.73373, 1.56340}, {0.89990, 0.96200}, {0.19348, 1.81288}},  // 'rotating_punch'
    {{-0.39428, 1.11125}, {-0.02906, 0.07340}, {0.61833, 0.58095}}   // 'shoulder_rotation'
};

// Function to normalise accelerometer data
void normalise(int16_t X[3], float X_processed[3]) {
    float mean = 642.886643666831;
	float std_dev = 4818.475265257539;

    for (int i = 0; i < 3; i++) {
        X_processed[i] = ((float)X[i] - mean) / std_dev;
    }
}

void print_float(const char* message, float x) {
	int intPart = (int)x;  // Extract integer part
    int fracPart = (int)(fabs(x - intPart) * 1e10);  // Extract fractional part
    warpPrint("%s: %d.%010d\n", message, intPart, fracPart);  // Print with 10 decimal places
}

float gaussian_pdf(float x, float mean, float var) {
    float diff = (x - mean) * (x - mean);  // (x - mean)^2
    float exponent = -diff / (2 * var);    // -((x - mean)^2) / (2 * var)
    float coeff = 1 / sqrt(2 * 3.141596 * var);  // Precompute coefficient
	float gauss = coeff * exp(exponent);

    print_float("Gauss: ", gauss);
    return gauss;  // Multiply with exp(exponent)
}

void classify_activity(float data[3], const char** predicted_activity, float* probability) {
	float posteriors[NUM_ACTIVITIES] = {0};
	float total_prob = 0;

	// Calculate posterior probabilities
	for (int activity=0; activity<NUM_ACTIVITIES; activity++) {
		float likelihood = 1;

		for (int i=0; i<3; i++) {
			float gaussian = gaussian_pdf(data[i], activity_stats[activity][i][0], activity_stats[activity][i][1]);
			likelihood *= gaussian;
		}	

		posteriors[activity] = likelihood / (float)NUM_ACTIVITIES;
		total_prob += posteriors[activity];
	}

	// Normalise posterior probabilities
	for (int activity=0; activity<NUM_ACTIVITIES; activity++) {
		posteriors[activity] /= total_prob;	
		// print_float("Posterior: ", posteriors[activity]*100);
	}

	// Find activity with highest posterior probability
	int max_activity = 0;
	for (int activity=1; activity<NUM_ACTIVITIES; activity++) {
		if (posteriors[activity] > posteriors[max_activity]) {
			max_activity = activity;
		}
	}

	// Map activity index to activity name
	// const char *activity_names[NUM_ACTIVITIES] = {"hook", "jabs", "rest", "rotating_punch", "shoulder_rotation", "starjump"};
	const char *activity_names[NUM_ACTIVITIES] = {"jabs", "rest", "rotating_punch", "shuolder_rotation"};
	*predicted_activity = activity_names[max_activity];
	*probability = posteriors[max_activity];
}

void run_4B25_coursework() {
	init_4B25_coursework();
	get_data_and_classify();	
}

void read_data() {
	while(1) {
		uint8_t buffer[6] = {0};
		appendSensorDataMMA8451Q(buffer);

		int16_t x = ((buffer[0] << 8) | buffer[1]);
		int16_t y = ((buffer[2] << 8) | buffer[3]);
		int16_t z = ((buffer[4] << 8) | buffer[5]);

		warpPrint("%d, %d, %d\n", x, y, z);
	}
}

void get_data_and_classify() {
	float processed_data[3] = {0};

	while (1) {
		uint8_t buffer[6] = {0};
		appendSensorDataMMA8451Q(buffer);

		int16_t x = ((buffer[0] << 8) | buffer[1]);
		int16_t y = ((buffer[2] << 8) | buffer[3]);
		int16_t z = ((buffer[4] << 8) | buffer[5]);

		normalise((int16_t[3]){x, y, z}, processed_data);

		const char *predicted_activity = NULL;
		float probability;
		classify_activity(processed_data, &predicted_activity, &probability);

		warpPrint("Predicted activity: %s", predicted_activity);
		print_float(", probability:", probability*100);
		warpPrint("\n\n");

		OSA_TimeDelay(1000);
	}
}

void init_4B25_coursework() {
	warpPrint("Initialising MMA8451Q...");
	initMMA8451Q( 0x1D, kWarpDefaultSupplyVoltageMillivoltsMMA8451Q);
	configureSensorMMA8451Q(0x00, /* Payload: Disable FIFO */ 0x01  /* Normal read 8bit, 800Hz, normal, active mode */);
	OSA_TimeDelay(100);
}
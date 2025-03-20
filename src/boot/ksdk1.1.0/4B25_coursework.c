#include <stdlib.h>

#include "config.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devMMA8451Q.h"
#include "devSSD1331.h"

#include "4B25_coursework.h"

#define NUM_ACTIVITIES 6

// Activity stats (mean and variance)
static float activity_stats[NUM_ACTIVITIES][3][2] = {
    {{-1.17354348, 0.03832778}, {-0.16887904, 0.14545671}, {0.52166356, 0.14491121}},  // 'hook'
    {{0.17056889, 0.1874709}, {0.0162215, 0.06428413}, {0.87960422, 0.03886931}},    // 'jabs'
    {{-1.01745237, 0.00118505}, {1.24314211, 0.00229343}, {0.51450573, 0.02665251}},  // 'rest'
    {{-1.8523882, 0.06258422}, {1.08685806, 0.05106414}, {0.86016743, 0.06803735}},   // 'rotating_punch'
    {{-0.80411911, 0.10849167}, {0.2523631, 0.00915533}, {1.39250002, 0.06654452}},   // 'shoulder_rotation'
    {{-0.93954108, 0.25710409}, {1.07482124, 0.14788009}, {-0.29212692, 0.36697733}}  // 'starjump'
};

float exp(float x) {
	return 1.0f + x + (x * x) / 2 + (x * x * x) / 6;
}

float sqrt(float x) {
	if (x <= 0) return 0;  // Avoid sqrt of negative numbers
    float guess = x / 2.0f; // Initial estimate
    for (int i = 0; i < 5; i++) {  // More iterations = more accuracy
        guess = (guess + x / guess) * 0.5f;
    }
    return guess;
}

void low_pass_filter(int8_t data[3], float filtered_data[3]) {
    filtered_data[0] = data[0];  // Initialize first value
    for (int i = 1; i < 3; i++) {
        filtered_data[i] = 0.01 * data[i] + (1 - 0.01) * filtered_data[i - 1];
    }
}

// Function to apply Z-score normalization
void z_score_normalization(int8_t data[3], float normalized_data[3]) {
    int8_t sum = 0.0f, sum_sq = 0.0f;
    
    // Compute mean
    for (int i = 0; i < 3; i++) {
        sum += data[i];
        sum_sq += data[i] * data[i];
    }
	warpPrint("Sum: %d", sum);

    float mean = sum / 3;
    float variance = (sum_sq / 3) - (mean * mean);
    float std_dev = sqrt(variance);

    // Normalize data
    for (int i = 0; i < 3; i++) {
        normalized_data[i] = (data[i] - mean) / std_dev;
    }
}

// Function to preprocess accelerometer data
void preprocess_data(int8_t X[3], float X_processed[3]) {
    // float filtered[3];

    // low_pass_filter(X, filtered);
    z_score_normalization(X, X_processed);
}

float gaussian_pdf(float x, float mean, float var) {
    return (1.0f / sqrt(2 * 3.141596 * var)) * exp(-((x - mean) * (x - mean)) / (2 * var));
}

void classify_activity(float x, float y, float z, const char** predicted_activity, float* probability) {
	float data[3] = {x, y, z};
	float posteriors[NUM_ACTIVITIES] = {0.0f};
	float total_prob = 0.0f;

	// Calculate posterior probabilities
	for (int activity=0; activity<NUM_ACTIVITIES; activity++) {
		float likelihood = 1.0f;

		for (int i=0; i<3; i++) {
			float mean = (float)activity_stats[activity][i][0];
			float var = (float)activity_stats[activity][i][1];
			likelihood *= gaussian_pdf(data[i], mean, var);
		}		

		float prior = 1.0f / NUM_ACTIVITIES;
		posteriors[activity] = likelihood * prior;
		total_prob += posteriors[activity];
	}

	// Normalise posterior probabilities
	for (int activity=0; activity<NUM_ACTIVITIES; activity++) {
		posteriors[activity] /= total_prob;
	}

	// Find activity with highest posterior probability
	int max_activity = 0;
	for (int activity=1; activity<NUM_ACTIVITIES; activity++) {
		if (posteriors[activity] > posteriors[max_activity]) {
			max_activity = activity;
		}
	}

	// Map activity index to activity name
	const char *activity_names[NUM_ACTIVITIES] = {"hook", "jabs", "rest", "rotating_punch", "shoulder_rotation", "starjump"};
	*predicted_activity = activity_names[max_activity];
	*probability = posteriors[max_activity];
}

void run_4B25_coursework() {
	init_4B25_coursework();
	get_data_and_classify();	
}

void get_data_and_classify() {
	float processed_data[3] = {0};

	while (1) {
		uint8_t buffer[6] = {0};
		appendSensorDataMMA8451Q(buffer);

		int8_t x = ((buffer[0] << 8) | buffer[1]);
		int8_t y = ((buffer[2] << 8) | buffer[3]);
		int8_t z = ((buffer[4] << 8) | buffer[5]);


		preprocess_data((int8_t[3]){x, y, z}, processed_data);

		const char *predicted_activity = NULL;
		float probability;
		classify_activity(processed_data[0], processed_data[1], processed_data[2], &predicted_activity, &probability);

		warpPrint("Predicted Activity: %s (Prbability: %d)\n", predicted_activity, probability*100);

		OSA_TimeDelay(1000);
	}
}

void init_4B25_coursework() {
	warpPrint("Initialising MMA8451Q...");
	initMMA8451Q( 0x1D, kWarpDefaultSupplyVoltageMillivoltsMMA8451Q);
	configureSensorMMA8451Q(0x00, /* Payload: Disable FIFO */ 0x01  /* Normal read 8bit, 800Hz, normal, active mode */);
	OSA_TimeDelay(1000);
}
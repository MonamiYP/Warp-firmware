#include <stdlib.h>

#include "config.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "math.h"

#include "devMMA8451Q.h"
#include "devSSD1331.h"
#include "4B25_coursework.h"

float calculate_mean(int16_t data[WINDOW_SIZE]) {
    int sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum += data[i];
    }
    return (float)sum / WINDOW_SIZE;
}

float calculate_rms(int16_t data[WINDOW_SIZE]) {
    float sum_of_squares = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum_of_squares += data[i] * data[i];
    }
    return sqrt(sum_of_squares / WINDOW_SIZE);
}

float calculate_std_dev(int16_t data[WINDOW_SIZE], float mean) {
    float sum_squared_diff = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum_squared_diff += (data[i] - mean) * (data[i] - mean);
    }
    return sqrt(sum_squared_diff / (WINDOW_SIZE - 1));
}

float calculate_peak_to_peak(int16_t data[WINDOW_SIZE]) {
    int16_t max_val = data[0];
    int16_t min_val = data[0];
    
    for (int i = 1; i < WINDOW_SIZE; i++) {
        if (data[i] > max_val) {
            max_val = data[i];
        }
        if (data[i] < min_val) {
            min_val = data[i];
        }
    }
    return (float)(max_val - min_val);
}

// Function to print floating point values in warp
void print_float(const char* message, float x) {
	int intPart = (int)x;
    int fracPart = (int)(fabs(x - intPart) * 1e3);
    warpPrint("%s: %d.%03d\n", message, intPart, fracPart);
}

// Function to classify a single instance
void classify(float features[NUM_FEATURES], const char** predicted_activity, float* probability) {
    float probabilities[NUM_ACTIVITIES] = {0};

    // Calculate the weighted sum for each class and apply the softmax function
    float sum_exp = 0;
    for (int i = 0; i < NUM_ACTIVITIES; i++) {
        float weighted_sum = bias[i];
        for (int j = 0; j < NUM_FEATURES; j++) {
            weighted_sum += features[j] * weights[i][j];
        }
        probabilities[i] = exp(weighted_sum);  // Exponentiate
        sum_exp += probabilities[i];  // Sum for normalization
    }

    // Normalize to get probabilities summing to 1
    for (int i = 0; i < NUM_ACTIVITIES; i++) {
        probabilities[i] /= sum_exp;
    }

    // Find the class with the highest probability
    int predicted_class = 0;
    for (int i = 1; i < NUM_ACTIVITIES; i++) {
        if (probabilities[i] > probabilities[predicted_class]) {
            predicted_class = i;
        }
    }

	*predicted_activity = activity_names[predicted_class];
	*probability = probabilities[predicted_class];
}

// Function to update the sliding window and compute mean and variance
void extract_features(SlidingWindow* window, int16_t x, int16_t y, int16_t z, float* features) {
    // Shift the window and add new data point
    window->x[window->index] = x;
    window->y[window->index] = y;
    window->z[window->index] = z;
    window->index = (window->index + 1) % WINDOW_SIZE;

    // Calculate features
	features[0] = calculate_mean(window->x);
    features[1] = calculate_mean(window->y);
    features[2] = calculate_mean(window->z);

    features[3] = calculate_std_dev(window->x, features[0]);
    features[4] = calculate_std_dev(window->y, features[1]);
    features[5] = calculate_std_dev(window->z, features[2]);

    features[6] = calculate_rms(window->x);
    features[7] = calculate_rms(window->y);
    features[8] = calculate_rms(window->z);

    features[9] = calculate_peak_to_peak(window->x);
    features[10] = calculate_peak_to_peak(window->y);
    features[11] = calculate_peak_to_peak(window->z);
}

void run_4B25_coursework() {
    OSA_TimeDelay(1000);
	init_4B25_coursework();
	read_data();
}

// Function to collect accelerometer data for training the classifier
void read_data() { 
	while(1) {
		uint8_t buffer[6] = {0};
		appendSensorDataMMA8451Q(buffer);

		int16_t x = ((buffer[0] << 8) | buffer[1]);
		int16_t y = ((buffer[2] << 8) | buffer[3]);
		int16_t z = ((buffer[4] << 8) | buffer[5]);

		warpPrint("%d, %d, %d\n", x, y, z);
        OSA_TimeDelay(10);
	}
}

// Functoion to read data from accelerometer and predict the activity the user is performing and probability
void get_data_and_classify() {
	SlidingWindow window = {0};
	float features[NUM_FEATURES];

	while (1) {
		uint8_t buffer[6] = {0};
		appendSensorDataMMA8451Q(buffer);
		int16_t x = ((buffer[0] << 8) | buffer[1]);
		int16_t y = ((buffer[2] << 8) | buffer[3]);
		int16_t z = ((buffer[4] << 8) | buffer[5]);

		extract_features(&window, x, y, z, features); // Extract features used for the classification algorithm

		const char *predicted_activity = NULL;
		float probability;
    	classify(features, &predicted_activity, &probability);

		warpPrint("Predicted activity: %s", predicted_activity);
		print_float(", probability", probability*100);
		warpPrint("\n\n");

		OSA_TimeDelay(10);
	}
}

void init_4B25_coursework() {
	warpPrint("\n\nInitialising MMA8451Q...\n\n");
	initMMA8451Q( 0x1D, kWarpDefaultSupplyVoltageMillivoltsMMA8451Q);
	configureSensorMMA8451Q(0x00, /* Payload: Disable FIFO */ 0x01  /* Normal read 8bit, 800Hz, normal, active mode */);
    OSA_TimeDelay(1000);
}
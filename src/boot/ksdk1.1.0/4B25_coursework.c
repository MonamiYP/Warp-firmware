#include <stdlib.h>

#include "config.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devMMA8451Q.h"
#include "devSSD1331.h"

#include "4B25_coursework.h"
#include "math.h"

#define NUM_ACTIVITIES 7
#define WINDOW_SIZE 20
#define NUM_FEATURES 12

const char *activity_names[NUM_ACTIVITIES] = {"hook", "jab", "rotating punch", "shoulder rotation", "front hold", "curl", "rest"};

static float weights[7][12] = {
    {-0.000632823426, -0.0000857277951, 0.000136433582, 0.000152228493, 0.000167050971, -0.000150593007, 0.000605740177, 0.000149978161, -0.000180863214, 0.000354459969, 0.000535042379, -0.000372347485},
    {0.000510156933, -0.000600671276, -0.0000268376900, 0.0000934580524, 0.000426962616, -0.000105807701, -0.000246464142, 0.0000541940265, -0.000456150178, 0.000453900576, 0.00112158448, -0.0000150925543},
    {0.0000196913187, 0.000235674147, -0.000403753911, 0.000223946116, 0.0000161815650, 0.000422750539, 0.0000304498796, -0.000318996815, -0.000315109369, 0.000444967954, 0.000161067349, 0.000544630828},
    {0.0000539618359, -0.000247042447, 0.000401280270, 0.000218376366, -0.000226364625, 0.000143023670, 0.000156277951, -0.000744107407, 0.000144387082, 0.000632343654, -0.000709382417, 0.000422313424},
    {0.000140283058, -0.000579629803, 0.00100699805, -0.000248470177, -0.000213875857, -0.000259654706, -0.000332729657, -0.000229006925, 0.000729867857, -0.000745738724, -0.000665811089, -0.000764388231},
    {0.0000908897793, -0.000151990133, -0.000997153154, -0.000183264064, 0.0000473271670, 0.000221476476, -0.000197409957, -0.00000885060525, 0.000477845165, -0.000366208874, 0.000235327732, 0.000994287432},
    {-0.000182159498, 0.00142938731, -0.000116967147, -0.000256274786, -0.000217281837, -0.000271195271, -0.0000158642517, 0.00109678956, -0.000399977344, -0.000773724555, -0.000677828428, -0.000809403414}
};

static float bias[7] = {-0.0000000702527161, -0.000000152361913, -0.000000236053228, -0.0000000190387558, 0.000000152602824, 0.0000000795899255, 0.000000245513864};

typedef struct {
    int16_t x[WINDOW_SIZE];
    int16_t y[WINDOW_SIZE];
    int16_t z[WINDOW_SIZE];
    int index;
} SlidingWindow;

// Function to calculate mean
float calculate_mean(int16_t data[WINDOW_SIZE]) {
    int sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum += data[i];
    }
    return (float)sum / WINDOW_SIZE;
}

// Function to calculate the Root Mean Square (RMS)
float calculate_rms(int16_t data[WINDOW_SIZE]) {
    float sum_of_squares = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum_of_squares += data[i] * data[i];
    }
    return sqrt(sum_of_squares / WINDOW_SIZE);
}

// Function to calculate standard deviation
float calculate_std_dev(int16_t data[WINDOW_SIZE], float mean) {
    float sum_squared_diff = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum_squared_diff += (data[i] - mean) * (data[i] - mean);
    }
    return sqrt(sum_squared_diff / (WINDOW_SIZE - 1));
}

// Function to calculate peak-to-peak value
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

void print_float(const char* message, float x) {
	int intPart = (int)x;  // Extract integer part
    int fracPart = (int)(fabs(x - intPart) * 1e3);  // Extract fractional part
    warpPrint("%s: %d.%03d\n", message, intPart, fracPart);  // Print with 10 decimal places
}

// Function to classify a single instance
void classify_logistic(float features[NUM_FEATURES], const char** predicted_activity, float* probability) {
    float probabilities[NUM_ACTIVITIES] = {0};

    // Calculate the weighted sum for each class and apply the sigmoid function
    for (int i = 0; i < NUM_ACTIVITIES; i++) {
        float weighted_sum = bias[i];  // Start with the bias for this class
        for (int j = 0; j < NUM_FEATURES; j++) {
            weighted_sum += features[j] * weights[i][j];  // Dot product
        }
        probabilities[i] = 1.0 / (1.0 + exp(-weighted_sum));  // Apply sigmoid
        // print_float("Proability:", probabilities[i]);
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

	// Calculate peak-to-peak
    features[9] = calculate_peak_to_peak(window->x);
    features[10] = calculate_peak_to_peak(window->y);
    features[11] = calculate_peak_to_peak(window->z);
}

void run_4B25_coursework() {
	init_4B25_coursework();
	get_data_and_classify();
    // read_data();
}

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

void get_data_and_classify() {
	SlidingWindow window = {0};
	float features[NUM_FEATURES];

	while (1) {
		uint8_t buffer[6] = {0};
		appendSensorDataMMA8451Q(buffer);

		int16_t x = ((buffer[0] << 8) | buffer[1]);
		int16_t y = ((buffer[2] << 8) | buffer[3]);
		int16_t z = ((buffer[4] << 8) | buffer[5]);

		extract_features(&window, x, y, z, features);

		const char *predicted_activity = NULL;
		float probability;
    	classify_logistic(features, &predicted_activity, &probability);

		warpPrint("Predicted activity: %s", predicted_activity);
		print_float(", probability", probability*100);
		warpPrint("\n\n");

		OSA_TimeDelay(10);
	}
}

void init_4B25_coursework() {
	warpPrint("Initialising MMA8451Q...");
	initMMA8451Q( 0x1D, kWarpDefaultSupplyVoltageMillivoltsMMA8451Q);
	configureSensorMMA8451Q(0x00, /* Payload: Disable FIFO */ 0x01  /* Normal read 8bit, 800Hz, normal, active mode */);
	OSA_TimeDelay(100);
}
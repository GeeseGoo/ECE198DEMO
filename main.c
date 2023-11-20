@@ -0,0 +1,130 @@
// Include the STM32F1xx Hardware Abstraction Layer (HAL) library
#include "stm32f1xx_hal.h"

// Declare a handle for I2C communication
I2C_HandleTypeDef hi2c1;

// Function prototypes for system clock configuration and GPIO and I2C initialization
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

// Function to calculate mean value of the measurements over past x days
int mean(int measurements[], int days, int length)
{
    // Want to access the past x days from measurement
    int loop = length - days;
    int sum{0};
    // If x is greater than number of days measured, use all data available
    if (loop < 0)
    {
        loop = 0;
    }

    // Sum all of the past x days in measurements
    for(int i = length - 1; i >= loop; i--)
    {
        sum += measurements[i];
    }

    // Return mean as sum of the measurements / number of days counted
    return sum / (length - loop);
}

// Function to calculate minimum value of the measurements over past x days
int minimum(int measurements[], int days, int length)
{
    // Want to access the past x days from measurement
    int loop = length - days;
    // If x is greater than the number of days measured, use all data available
    if (loop < 0)
    {
        loop = 0;
    }

    // Initialize min_value to the first measurement in the range
    int min_value = measurements[length - 1];

    // Find the minimum value in the past x days in measurements
    for(int i = length - 2; i >= loop; i--)
    {
        if (measurements[i] < min_value)
        {
            min_value = measurements[i];
        }
    }

    // Return the minimum value found
    return min_value;
}

// Function to calculate maximum value of the measurements over past x days
int maximum(int measurements[], int days, int length)
{
    // Want to access the past x days from measurement
    int loop = length - days;
    // If x is greater than the number of days measured, use all data available
    if (loop < 0)
    {
        loop = 0;
    }

    // Initialize max_value to the first measurement in the range
    int max_value = measurements[length - 1];

    // Find the maximum value in the past x days in measurements
    for(int i = length - 2; i >= loop; i--)
    {
        if (measurements[i] > max_value)
        {
            max_value = measurements[i];
        }
    }

    // Return the maximum value found
    return max_value;
}

// Main function
int main(void)
{
  // Initialize the HAL library
  HAL_Init();

  // Configure the system clock
  SystemClock_Config();

  // Initialize GPIO
  MX_GPIO_Init();

  // Initialize I2C
  MX_I2C1_Init();

  // Buffer to hold data received from the sensor
  uint8_t buf[18];

  // Command to read measurement from the sensor
  uint8_t read_measurement_cmd[] = {0x03, 0x00};

  // Command to trigger continuous data from the sensor
  uint8_t trigger_continuous_data[] = {0x00, 0x10, 0x00, 0x00};

  // Transmit the command to trigger continuous data to the sensor
  HAL_I2C_Master_Transmit(&hi2c1, TMP102_ADDR, trigger_continuous_data, 4, HAL_MAX_DELAY);

  // Main loop
  while (1)
  {
    // Transmit the command to read measurement to the sensor
    HAL_I2C_Master_Transmit(&hi2c1, TMP102_ADDR, read_measurement_cmd, 2, HAL_MAX_DELAY);

    // Receive data from the sensor
    HAL_I2C_Master_Receive(&hi2c1, TMP102_ADDR, buf, 18, HAL_MAX_DELAY);

    // Process the received data from the CO2 sensor
    // ...

    // Delay for 500ms
    HAL_Delay(500);
  }
}
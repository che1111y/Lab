#include "board.h"

#define VOLTAGE_STEP 0.1
#define MAX_VOLTAGE  18.0
#define MIN_VOLTAGE  0.0


void set_converter_voltage(float voltage);

void mppt_p_o(void) {
    static float last_power = 0.0;
    static float last_voltage = 12.0;
    float current_voltage = last_voltage;
    float current, voltage, power;

    voltage = Measure_Voltage(1);
    current = Measure_Current(2,100);
    power = voltage * current;

    if (power > last_power) {
        if (voltage > last_voltage) {
            current_voltage += VOLTAGE_STEP;
        } else {
            current_voltage -= VOLTAGE_STEP;
        }
    } else {
        if (voltage > last_voltage) {
            current_voltage -= VOLTAGE_STEP;
        } else {
            current_voltage += VOLTAGE_STEP;
        }
    }

    if (current_voltage > MAX_VOLTAGE) current_voltage = MAX_VOLTAGE;
    if (current_voltage < MIN_VOLTAGE) current_voltage = MIN_VOLTAGE;

    //set_converter_voltage(current_voltage);

    last_voltage = voltage;
    last_power = power;
}


void set_converter_voltage(float voltage) 
{
    
}


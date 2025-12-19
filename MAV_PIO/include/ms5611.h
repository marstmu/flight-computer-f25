#pragma once

#include <cstdlib>

extern volatile float ms5611_temp_c;
extern volatile float ms5611_pressure_mbar;
extern volatile float altitude_asl_m;
extern volatile float height_agl_m;
extern bool refSet;

void ms5611_reset();
void ms5611_read_prom();
void ms5611_start_conversion(uint8_t cmd);
uint32_t ms5611_read_adc();

void ms5611_begin();
void ms5611_task_update();
void ms5611_compute(float &temperature_c, float &pressure_mbar, uint32_t D1, uint32_t D2);

void calibrateAltitudeReference(float pressure_mbar, float temp_c, float altitude_asl_m_in);
float calcDeltaH_m(float pressure_mbar);
float filterAltitude(float a);
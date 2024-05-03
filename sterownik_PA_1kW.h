/*
 * sterownik_PA_1kW.h
 *
 *  Created on: 2 wrz 2020
 *      Author: witek
 */

#ifndef STEROWNIK_PA_1KW_H_
#define STEROWNIK_PA_1KW_H_

#define ACS713	// pomiar prądu czujnikiem Halla
//#define SP2HYO
//#define ALTER	// inne przyporządkowanie wejść dla kodu DCBA i wyjść dla tłumików
//#define DEBUG
//#define DEBUGt
#define CZAS_PETLI

// tabela dla KTY81/110
#define SENS_TABLE_COUNT 24
static const int16_t KTY81_120_sensTable[SENS_TABLE_COUNT][2] =
	{{-55, 490}, {-50, 515}, {-40, 567}, {-30, 624}, {-20, 684}, {-10, 747},  {0, 815},    {10, 886},   {20, 961},   {25, 1000},  {30, 1040},  {40, 1122},
	{50, 1209}, {60, 1299}, {70, 1392}, {80, 1490}, {90, 1591}, {100, 1696}, {110, 1805}, {120, 1915}, {125, 1970}, {130, 2023}, {140, 2124}, {150, 2211}};

void getTemperatura1(uint8_t pin, int Rf);
void getTemperatura2(uint8_t pin, int Rf);
void getTemperatura3(uint8_t pin);
void read_inputs();
float calc_SWR(int forward, int ref);
bool UpdatePowerAndVSWR();
void startCounting (unsigned int ms);
void switch_bands();

#endif /* STEROWNIK_PA_1KW_H_ */

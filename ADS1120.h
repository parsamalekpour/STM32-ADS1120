/*
 * ADS1120.h
 *
 *  Created on: Nov 16, 2023
 *      Author: Parsa
 */

#ifndef INC_ADS1120_H_
#define INC_ADS1120_H_

#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"


typedef enum ADS1120_MUX {
    ADS1120_MUX_0_1     = 0x00,   //default
    ADS1120_MUX_0_2     = 0x10,
    ADS1120_MUX_0_3     = 0x20,
    ADS1120_MUX_1_2     = 0x30,
    ADS1120_MUX_1_3     = 0x40,
    ADS1120_MUX_2_3     = 0x50,
    ADS1120_MUX_1_0     = 0x60,
    ADS1120_MUX_3_2     = 0x70,
    ADS1120_MUX_0_AVSS  = 0x80,
    ADS1120_MUX_1_AVSS  = 0x90,
    ADS1120_MUX_2_AVSS  = 0xA0,
    ADS1120_MUX_3_AVSS  = 0xB0,
    ADS1120_MUX_REFPX_REFNX_4 = 0xC0,
    ADS1120_MUX_AVDD_M_AVSS_4 = 0xD0,
    ADS1120_MUX_AVDD_P_AVSS_2 = 0xE0
} ads1120Mux;

typedef enum ADS1120_GAIN {
    ADS1120_GAIN_1   = 0x00,   //default
    ADS1120_GAIN_2   = 0x02,
    ADS1120_GAIN_4   = 0x04,
    ADS1120_GAIN_8   = 0x06,
    ADS1120_GAIN_16  = 0x08,
    ADS1120_GAIN_32  = 0x0A,
    ADS1120_GAIN_64  = 0x0C,
    ADS1120_GAIN_128 = 0x0E
} ads1120Gain;

typedef enum ADS1120_DATA_RATE {
    ADS1120_DR_LVL_0 = 0x00,   // default
    ADS1120_DR_LVL_1 = 0x20,
    ADS1120_DR_LVL_2 = 0x40,
    ADS1120_DR_LVL_3 = 0x60,
    ADS1120_DR_LVL_4 = 0x80,
    ADS1120_DR_LVL_5 = 0xA0,
    ADS1120_DR_LVL_6 = 0xC0
} ads1120DataRate;

typedef enum ADS1120_OP_MODE {
    ADS1120_NORMAL_MODE     = 0x00,  // default
    ADS1120_DUTY_CYCLE_MODE = 0x08,
    ADS1120_TURBO_MODE      = 0x10
} ads1120OpMode;

typedef enum ADS1120_CONV_MODE {
    ADS1120_SINGLE_SHOT     = 0x00,  // default
    ADS1120_CONTINUOUS      = 0x04
} ads1120ConvMode;

typedef enum ADS1120_VREF{
    ADS1120_VREF_INT            = 0x00,  // default
    ADS1120_VREF_REFP0_REFN0    = 0x40,
    ADS1120_VREF_REFP1_REFN1    = 0x80,
    ADS1120_VREF_AVDD_AVSS      = 0xC0
} ads1120VRef;

typedef enum ADS1120_FIR{
    ADS1120_NONE        = 0x00,   // default
    ADS1120_50HZ_60HZ   = 0x10,
    ADS1120_50HZ        = 0x20,
    ADS1120_60HZ        = 0x30
} ads1120FIR;

typedef enum ADS1120_PSW {
    ADS1120_ALWAYS_OPEN = 0x00,  // default
    ADS1120_SWITCH      = 0x08
} ads1120PSW;

typedef enum ADS1120_IDAC_CURRENT {
    ADS1120_IDAC_OFF        = 0x00,  // defaul
    ADS1120_IDAC_10_MU_A    = 0x01,
    ADS1120_IDAC_50_MU_A    = 0x02,
    ADS1120_IDAC_100_MU_A   = 0x03,
    ADS1120_IDAC_250_MU_A   = 0x04,
    ADS1120_IDAC_500_MU_A   = 0x05,
    ADS1120_IDAC_1000_MU_A  = 0x06,
    ADS1120_IDAC_1500_MU_A  = 0x07
} ads1120IdacCurrent;

typedef enum ADS1120_IDAC_ROUTING {
    ADS1120_IDAC_NONE       = 0x00,  // default
    ADS1120_IDAC_AIN0_REFP1 = 0x01,
    ADS1120_IDAC_AIN1       = 0x02,
    ADS1120_IDAC_AIN2       = 0x03,
    ADS1120_IDAC_AIN3_REFN1 = 0x04,
    ADS1120_IDAC_REFP0      = 0x05,
    ADS1120_IDAC_REFN0      = 0x06,
} ads1120IdacRouting;

typedef enum ADS1120_DRDY_MODE {
    ADS1120_DRDY      = 0x00,   // default
    ADS1120_DOUT_DRDY = 0x02
} ads1120DrdyMode;


typedef struct ADS1120_params{
	SPI_HandleTypeDef *adsSpi;
	GPIO_TypeDef *csPort;
	uint16_t  csPin;
	GPIO_TypeDef *drdyPort;
	uint16_t drdyPin;
	uint8_t regValue;
	float vRef;
	uint8_t gain;
	bool refMeasurement;
	bool doNotBypassPgaIfPossible;
	ads1120ConvMode convMode;
}ADS1120_params;

//ADS1120 SPI commands
static const uint8_t ADS1120_RESET = 0x06;
static const uint8_t ADS1120_START = 0x08;    //Send the START/SYNC command (08h) to start converting in continuous conversion mode
static const uint8_t ADS1120_PWRDOWN = 0x02;
static const uint8_t ADS1120_RDATA = 0x10;
static const uint8_t ADS1120_WREG = 0x40;    // write register
static const uint8_t ADS1120_RREG = 0x20;    // read register

/* registers */
static const uint8_t ADS1120_CONF_REG_0 = 0x00;
static const uint8_t ADS1120_CONF_REG_1 = 0x01;
static const uint8_t ADS1120_CONF_REG_2 = 0x02;
static const uint8_t ADS1120_CONF_REG_3 = 0x03;

/* other */
static const float ADS1120_RANGE = 32767.0; // = 2^15 - 1 as float

/* Commands */
uint8_t ADS1120_init(ADS1120_params *adsParam);
void ADS1120_start(ADS1120_params *adsParam);
void ADS1120_reset(ADS1120_params *adsParam);
void ADS1120_powerDown(ADS1120_params *adsParam);

/* Configuration Register 0 settings */
void ADS1120_setCompareChannels(ADS1120_params *adsParam, ads1120Mux mux);
void ADS1120_setGain(ADS1120_params *adsParam, ads1120Gain enumGain);
void ADS1120_bypassPGA(ADS1120_params *adsParam, bool bypass);
bool ADS1120_isPGABypassed(ADS1120_params *adsParam);

/* Configuration Register 1 settings */
void ADS1120_setDataRate(ADS1120_params *adsParam, ads1120DataRate rate);
void ADS1120_setOperatingMode(ADS1120_params *adsParam, ads1120OpMode mode);
void ADS1120_setConversionMode(ADS1120_params *adsParam, ads1120ConvMode mode);
void ADS1120_enableTemperatureSensor(ADS1120_params *adsParam, bool enable);
void ADS1120_enableBurnOutCurrentSources(ADS1120_params *adsParam, bool enable);

/* Configuration Register 2 settings */
void ADS1120_setVRefSource(ADS1120_params *adsParam, ads1120VRef vRefSource);
void ADS1120_setFIRFilter(ADS1120_params *adsParam, ads1120FIR fir);
void ADS1120_setLowSidePowerSwitch(ADS1120_params *adsParam, ads1120PSW psw);
void ADS1120_setIdacCurrent(ADS1120_params *adsParam, ads1120IdacCurrent current);

/* Configuration Register 3 settings */
void ADS1120_setIdac1Routing(ADS1120_params *adsParam, ads1120IdacRouting route);
void ADS1120_setIdac2Routing(ADS1120_params *adsParam, ads1120IdacRouting route);
void ADS1120_setDrdyMode(ADS1120_params *adsParam, ads1120DrdyMode mode);

/* Other settings */
void ADS1120_setAvddAvssAsVrefAndCalibrate(ADS1120_params *adsParam);
void ADS1120_setRefp0Refn0AsVefAndCalibrate(ADS1120_params *adsParam);
void ADS1120_setRefp1Refn1AsVefAndCalibrate(ADS1120_params *adsParam);
void ADS1120_setIntVRef(ADS1120_params *adsParam);

/* Results */
float ADS1120_getVoltage_mV(ADS1120_params *adsParam);
float ADS1120_getVoltage_muV(ADS1120_params *adsParam);
int16_t ADS1120_getRawData(ADS1120_params *adsParam);
float ADS1120_getTemperature(ADS1120_params *adsParam);

/************************************************
    private functions
*************************************************/

void ADS1120_forcedBypassPGA(ADS1120_params *adsParam);
int16_t ADS1120_getData(ADS1120_params *adsParam);
uint16_t ADS1120_readResult(ADS1120_params *adsParam);
uint8_t ADS1120_readRegister(ADS1120_params *adsParam, uint8_t reg);
void ADS1120_writeRegister(ADS1120_params *adsParam, uint8_t reg, uint8_t val);
void ADS1120_command(ADS1120_params *adsParam, uint8_t cmd);


#endif /* INC_ADS1120_H_ */

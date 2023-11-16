/*
 * ADS1120.c
 *
 *  Created on: Nov 16, 2023
 *      Author: Parsa
 */

#include "ADS1120.h"

uint8_t ADS1120_init(ADS1120_params *adsParam){
	adsParam->vRef = 2.048;
    adsParam->gain = 1;
    adsParam->refMeasurement = false;
    adsParam->convMode = ADS1120_SINGLE_SHOT;

//    pinMode(csPin, OUTPUT);
//    pinMode(drdyPin, INPUT);
    HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_SET);


    ADS1120_reset(adsParam);
    ADS1120_start(adsParam);
    uint8_t ctrlVal = 0;
    ADS1120_bypassPGA(adsParam, true); // just a test if the ADS1220 is connected
    ctrlVal = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_0);
    ctrlVal = ctrlVal & 0x01;
    ADS1120_bypassPGA(adsParam, false);
    return ctrlVal;
}

void ADS1120_start(ADS1120_params *adsParam){
	ADS1120_command(adsParam, ADS1120_START);
}

void ADS1120_reset(ADS1120_params *adsParam){
	ADS1120_command(adsParam, ADS1120_RESET);
    HAL_Delay(1);
}

void ADS1120_powerDown(ADS1120_params *adsParam){
	ADS1120_command(adsParam, ADS1120_PWRDOWN);
}

/* Configuration Register 0 settings */

void ADS1120_setCompareChannels(ADS1120_params *adsParam, ads1120Mux mux){
    if((mux == ADS1120_MUX_REFPX_REFNX_4) || (mux == ADS1120_MUX_AVDD_M_AVSS_4)){
    	adsParam->gain = 1;    // under these conditions gain is one by definition
    	adsParam->refMeasurement = true;
    }
    else{            // otherwise read gain from register
    	adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_0);
    	adsParam->regValue = adsParam->regValue & 0x0E;
    	adsParam->regValue = adsParam->regValue>>1;
    	adsParam->gain = 1 << adsParam->regValue;
    	adsParam->refMeasurement = false;
    }
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_0);
    adsParam->regValue &= ~0xF1;
    adsParam->regValue |= mux;
    adsParam->regValue |= !(adsParam->doNotBypassPgaIfPossible & 0x01);
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_0, adsParam->regValue);
    if((mux >= 0x80) && (mux <=0xD0)){
        if(adsParam->gain > 4){
        	adsParam->gain = 4;           // max gain is 4 if single-ended input is chosen or PGA is bypassed
        }
        ADS1120_forcedBypassPGA(adsParam);
    }
}


void ADS1120_setGain(ADS1120_params *adsParam, ads1120Gain enumGain){
	adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_0);
    ads1120Mux mux = (ads1120Mux)(adsParam->regValue & 0xF0);
    adsParam->regValue &= ~0x0E;
    adsParam->regValue |= enumGain;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_0, adsParam->regValue);

    adsParam->gain = 1<<(enumGain>>1);
    if((mux >= 0x80) && (mux <=0xD0)){
        if(adsParam->gain > 4){
        	adsParam->gain = 4;   // max gain is 4 if single-ended input is chosen or PGA is bypassed
        }
        ADS1120_forcedBypassPGA(adsParam);
    }
}


void ADS1120_bypassPGA(ADS1120_params *adsParam, bool bypass){
	adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_0);
	adsParam->regValue &= ~0x01;
	adsParam->regValue |= bypass;
	adsParam->doNotBypassPgaIfPossible = !(bypass & 0x01);
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_0, adsParam->regValue);
}

bool ADS1120_isPGABypassed(ADS1120_params *adsParam){
	adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_0);
    return adsParam->regValue & 0x01;
}


/* Configuration Register 1 settings */

void ADS1120_setDataRate(ADS1120_params *adsParam, ads1120DataRate rate){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_1);
    adsParam->regValue &= ~0xE0;
    adsParam->regValue |= rate;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_1, adsParam->regValue);
}

void ADS1120_setOperatingMode(ADS1120_params *adsParam, ads1120OpMode mode){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_1);
    adsParam->regValue &= ~0x18;
    adsParam->regValue |= mode;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_1, adsParam->regValue);
}

void ADS1120_setConversionMode(ADS1120_params *adsParam, ads1120ConvMode mode){
	adsParam->convMode = mode;
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_1);
    adsParam->regValue &= ~0x04;
    adsParam->regValue |= mode;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_1, adsParam->regValue);
}

void ADS1120_enableTemperatureSensor(ADS1120_params *adsParam, bool enable){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_1);
    if(enable){
        adsParam->regValue |= 0x02;
    }
    else{
        adsParam->regValue &= ~0x02;
    }
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_1, adsParam->regValue);
}

void ADS1120_enableBurnOutCurrentSources(ADS1120_params *adsParam, bool enable){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_1);
    if(enable){
        adsParam->regValue |= 0x01;
    }
    else{
        adsParam->regValue &= ~0x01;
    }
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_1, adsParam->regValue);
}

/* Configuration Register 2 settings */

void ADS1120_setVRefSource(ADS1120_params *adsParam, ads1120VRef vRefSource){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_2);
    adsParam->regValue &= ~0xC0;
    adsParam->regValue |= vRefSource;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_2, adsParam->regValue);
}

void ADS1120_setFIRFilter(ADS1120_params *adsParam, ads1120FIR fir){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_2);
    adsParam->regValue &= ~0x30;
    adsParam->regValue |= fir;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_2, adsParam->regValue);
}

void ADS1120_setLowSidePowerSwitch(ADS1120_params *adsParam, ads1120PSW psw){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_2);
    adsParam->regValue &= ~0x08;
    adsParam->regValue |= psw;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_2, adsParam->regValue);
}

void ADS1120_setIdacCurrent(ADS1120_params *adsParam, ads1120IdacCurrent current){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_2);
    adsParam->regValue &= ~0x07;
    adsParam->regValue |= current;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_2, adsParam->regValue);
    HAL_Delay(1);
}


/* Configuration Register 3 settings */

void ADS1120_setIdac1Routing(ADS1120_params *adsParam, ads1120IdacRouting route){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_3);
    adsParam->regValue &= ~0xE0;
    adsParam->regValue |= (route<<5);
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_3, adsParam->regValue);
}

void ADS1120_setIdac2Routing(ADS1120_params *adsParam, ads1120IdacRouting route){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_3);
    adsParam->regValue &= ~0x1C;
    adsParam->regValue |= (route<<2);
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_3, adsParam->regValue);
}

void ADS1120_setDrdyMode(ADS1120_params *adsParam, ads1120DrdyMode mode){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_3);
    adsParam->regValue &= ~0x02;
    adsParam->regValue |= mode;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_3, adsParam->regValue);
}

/* Other settings */

void ADS1120_setAvddAvssAsVrefAndCalibrate(ADS1120_params *adsParam){
    float avssVoltage = 0.0;
    ADS1120_setVRefSource(adsParam, ADS1120_VREF_AVDD_AVSS);
    ADS1120_setCompareChannels(adsParam, ADS1120_MUX_AVDD_M_AVSS_4);
    for(int i = 0; i<10; i++){
        avssVoltage += ADS1120_getVoltage_mV(adsParam);
    }
    adsParam->vRef = avssVoltage * 4.0 / 10000.0;
}

void ADS1120_setRefp0Refn0AsVefAndCalibrate(ADS1120_params *adsParam){
    float ref0Voltage = 0.0;
    ADS1120_setVRefSource(adsParam, ADS1120_VREF_REFP0_REFN0);
    ADS1120_setCompareChannels(adsParam, ADS1120_MUX_REFPX_REFNX_4);
    for(int i = 0; i<10; i++){
        ref0Voltage += ADS1120_getVoltage_mV(adsParam);
    }
    adsParam->vRef = ref0Voltage * 4.0 / 10000.0;
}

void ADS1120_setRefp1Refn1AsVefAndCalibrate(ADS1120_params *adsParam){
    float ref1Voltage = 0.0;
    ADS1120_setVRefSource(adsParam, ADS1120_VREF_REFP1_REFN1);
    ADS1120_setCompareChannels(adsParam, ADS1120_MUX_REFPX_REFNX_4);
    for(int i = 0; i<10; i++){
        ref1Voltage += ADS1120_getVoltage_mV(adsParam);
    }
    adsParam->vRef = ref1Voltage * 4.0 / 10000.0;
}

void ADS1120_setIntVRef(ADS1120_params *adsParam){
	ADS1120_setVRefSource(adsParam, ADS1120_VREF_INT);
    adsParam->vRef = 2.048;
}

/* Results */
float ADS1120_getVoltage_mV(ADS1120_params *adsParam){
    int32_t rawData = ADS1120_getData(adsParam);
    float resultInMV = 0.0;
    if(adsParam->refMeasurement){
        resultInMV = (rawData / ADS1120_RANGE) * 2.048 * 1000.0 / (adsParam->gain * 1.0);
    }
    else{
        resultInMV = (rawData / ADS1120_RANGE) * adsParam->vRef * 1000.0 / (adsParam->gain * 1.0);
    }
    return resultInMV;
}

float ADS1120_getVoltage_muV(ADS1120_params *adsParam){
    return ADS1120_getVoltage_mV(adsParam) * 1000.0;
}

int16_t ADS1120_getRawData(ADS1120_params *adsParam){
    return ADS1120_getData(adsParam);
}

float ADS1120_getTemperature(ADS1120_params *adsParam){
	ADS1120_enableTemperatureSensor(adsParam, true);
    int16_t rawResult = ADS1120_readResult(adsParam);
    ADS1120_enableTemperatureSensor(adsParam, false);

    uint16_t result = (rawResult >> 2);
    if(result>>13){
        result = ~(result-1) & 0x3777;
        return result * (-0.03125);
    }

    return result * 0.03125;
}


/************************************************
    private functions
*************************************************/

void ADS1120_forcedBypassPGA(ADS1120_params *adsParam){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_0);
    adsParam->regValue |= 0x01;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_0, adsParam->regValue);
}

int16_t ADS1120_getData(ADS1120_params *adsParam){
	union Data{
		uint16_t rawResult;
		int16_t result;
	};

	union Data data;
	data.rawResult = ADS1120_readResult(adsParam);

    return data.result;
}

uint16_t ADS1120_readResult(ADS1120_params *adsParam){
    uint8_t buf[3];
    uint32_t rawResult = 0;

    if(adsParam->convMode == ADS1120_SINGLE_SHOT){
    	ADS1120_start(adsParam);
    }
    while(HAL_GPIO_ReadPin(adsParam->drdyPort, adsParam->drdyPin) == GPIO_PIN_SET) {}

    HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_RESET);
    HAL_SPI_Receive(adsParam->adsSpi, buf, 2, 500);
    HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_SET);

    rawResult = buf[0];
    rawResult = (rawResult << 8) | buf[1];
//    rawResult = (rawResult << 8) | buf[2];
    //rawResult = (rawResult << 8);

    return rawResult;
}

uint8_t ADS1120_readRegister(ADS1120_params *adsParam, uint8_t reg){
	adsParam->regValue = 0;

    uint8_t buf[1] = {ADS1120_RREG | (reg<<2)};

    HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(adsParam->adsSpi, buf, 1, 500);
    HAL_SPI_Receive(adsParam->adsSpi, &adsParam->regValue, 1, 500);

    HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_SET);


    return adsParam->regValue;
}

void ADS1120_writeRegister(ADS1120_params *adsParam, uint8_t reg, uint8_t val){

	HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_RESET);

	uint8_t buf[1] = {(ADS1120_WREG | (reg<<2))};
    HAL_SPI_Transmit(adsParam->adsSpi, buf, 1, 500);

    HAL_SPI_Transmit(adsParam->adsSpi, &val, 1, 500);

    HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_SET);

}

void ADS1120_command(ADS1120_params *adsParam, uint8_t cmd){

	HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(adsParam->adsSpi, &cmd, 1, 500);

	HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_SET);

}


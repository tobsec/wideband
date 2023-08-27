#pragma once

#include <stdint.h>

#include <rusefi/fragments.h>

#include "wideband_config.h"

#include "efi_scaled_channel.h"

/* +0 offset, size 32 bytes */
struct livedata_common_s {
	scaled_voltage vbatt;
	uint8_t pad0[30];
};
static_assert(sizeof(livedata_common_s) == 32);

/* +32 offset, size 32 bytes */
struct livedata_afr_s {
	// lambda also displayed by TS as AFR, same data with different scale factor
	scaled_lambda lambda;
	uint8_t pad0[2];
	scaled_high_temperature temperature;
	scaled_voltage heaterSupplyVoltage;
	scaled_voltage nernstDc;
	scaled_voltage nernstAc;
	scaled_current_signed pumpCurrentTarget;
	uint8_t pad1[2];
	scaled_current_signed pumpCurrentMeasured;
	uint8_t pad2[2];
	scaled_percent heaterDuty;
	scaled_voltage heaterEffectiveVoltage;
	uint16_t esr;
	scaled_voltage_signed nernstV;
	uint8_t fault; // See wbo::Fault
	uint8_t heaterState;
	uint8_t pad3[2];
};
static_assert(sizeof(livedata_afr_s) == 32);

/* update functions */
void SamplingUpdateLiveData();

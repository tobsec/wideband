#include "wideband_config.h"
#include "livedata.h"

#include "lambda_conversion.h"
#include "sampling.h"
#include "pump_dac.h"
#include "heater_control.h"
#include "max3185x.h"
#include "fault.h"

#include <rusefi/arrays.h>
#include <rusefi/fragments.h>

static livedata_common_s livedata_common;
static livedata_afr_s livedata_afr[AFR_CHANNELS];

void SamplingUpdateLiveData()
{
    float vbat = 0;
    for (int ch = 0; ch < AFR_CHANNELS; ch++)
    {
        struct livedata_afr_s *data = &livedata_afr[ch];

        const auto& sampler = GetSampler(ch);
        const auto& heater = GetHeaterController(ch);

        float voltage = sampler.GetInternalHeaterVoltage();

        data->lambda = GetLambda(ch);
        data->temperature = sampler.GetSensorTemperature();
        data->heaterSupplyVoltage = voltage;
        data->nernstDc = sampler.GetNernstDc();
        data->nernstV = sampler.GetNernstV();
        data->nernstAc = sampler.GetNernstAc();
        /* in mA, not A */
        data->pumpCurrentTarget = GetPumpCurrent(ch);
        /* in mA, not A */
        data->pumpCurrentMeasured = sampler.GetPumpNominalCurrent();
        data->heaterDuty = GetHeaterDuty(ch) * 100.0;
        data->heaterEffectiveVoltage = heater.GetHeaterEffectiveVoltage();
        data->esr = sampler.GetSensorInternalResistance();
        data->fault = (uint8_t)GetCurrentFault(ch);
        data->heaterState = (uint8_t)GetHeaterState(ch);
        /* TODO: add GetPumpOutputDuty() */
        if (voltage > vbat)
            vbat = voltage;
    }

    livedata_common.vbatt = vbat;
}

template<>
const livedata_common_s* getLiveData(size_t)
{
    return &livedata_common;
}

template<>
const struct livedata_afr_s * getLiveData(size_t ch)
{
    if (ch < AFR_CHANNELS)
    {
        return &livedata_afr[ch];
    }

    return nullptr;
}

static const FragmentEntry fragments[] = {
    decl_frag<livedata_common_s>{},
    decl_frag<livedata_afr_s, 0>{},
    decl_frag<livedata_afr_s, 1>{},
    decl_frag<livedata_egt_s, 0>{},
    decl_frag<livedata_egt_s, 1>{},
};

FragmentList getFragments() {
	return { fragments, efi::size(fragments) };
}

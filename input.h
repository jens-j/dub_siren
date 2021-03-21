#include "fixedpoint.h"

#ifndef __INPUT_H
#define __INPUT_H


#define N_POTS 10

enum pots_t {POT_OSC_FREQ,
             POT_LFO_DEPTH, POT_LFO_FREQ, POT_RELEASE,
             POT_FILTER_FREQ, POT_FILTER_RES, POT_FILTER_SWEEP,
             POT_DELAY_WET, POT_DELAY_TIME, POT_DELAY_FB};

typedef struct pot_data_s { // this must be exactly N_POTS
    uint16_t osc_frequency;
    uint16_t lfo_depth;
    uint16_t lfo_frequency;
    uint16_t decay_time;
    uint16_t filter_cutoff;
    uint16_t filter_resonance;
    uint16_t filter_sweep;
    uint16_t delay_wet;
    uint16_t delay_time;
    uint16_t delay_feedback;
} pot_data_t;

typedef struct sreg_data_s {
    uint8_t encoder_data;
    uint8_t button_data;
} sreg_data_t;

typedef struct pot_route_s {
    uint8_t adc_channel;
    uint8_t mux_setting;
} pot_route_t;

class Input {
private:
    void _setupAdc ();
    uint16_t _readAdc (int channel);
    sreg_data_t _readShiftRegister ();
public:
    waveform_t osc_waveform;
    pot_data_t pot_data;
    uint8_t button_state;

    Input ();
    void update ();
    void printPots ();
};

#endif

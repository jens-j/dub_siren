#include "fixedpoint.h"

#ifndef __INPUT_H
#define __INPUT_H

class Input {
private:
    void _setupAdc ();
    uint16_t _readAdc (int channel);
public:
    int osc_waveform;
    int osc_frequency;
    int lfo_waveform;
    int lfo_frequency;
    int lfo_depth;
    int decay_time;
    int filter_cutoff;
    int filter_resonance;

    Input ();
    void update ();
};

#endif

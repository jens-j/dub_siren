//#include <I2S.h>
#include <sam.h>
#include <wiring_private.h>
#include "dubsiren.h"
#include "input.h"
#include "fixedpoint.h"
#include "spi_dma.h"
#include "i2s.h"
#include "waveforms.h"
#include "biquad.h"
#include "dsvf.h"
#include "timer.h"


Input *input;
SpiDma *spiDma;
uint32_t led_t0                  = 0;
bool led_state                   = true;
volatile uint32_t loop_t0        = 0;
volatile uint32_t loop_t1        = 0;
volatile uint32_t isr_t0         = 0;
volatile uint32_t isr_dt         = 0;

// button variables
uint8_t last_btn_state           = 0;
uint32_t input_t0                = 0;
uint32_t btn_t0                  = 0;
bool trigger_state               = false;
volatile bool trigger_flag       = false;

// oscillator variables
qu16_t osc_setpoint              = 0; // base frequency setpoint set by knob
qu16_t osc_setpoint_sweep        = 0; // setpoint + sweep offset
qs15_t osc_sweep_offset          = 0.0;
qs15_t osc_sweep_step            = 0.0;
volatile qu32_t osc_phase        = 0;                // always positive [0 - 1]
volatile qu32_t sub_phase        = 0;
volatile qu32_t osc_third_phase  = 0;
volatile qu32_t osc_fifth_phase  = 0;
volatile qu16_t osc_frequency    = osc_setpoint;     // base frequency current value
volatile uint16_t mod_frequency  = 0;                // additive frequency shift from lfo
volatile qu32_t osc_step         = osc_setpoint * (QU32_ONE / SAMPLE_RATE); // 1000 Hz phase change per sample


// lfo variables
qs15_t lfo_value                 = 0;
waveform_t lfo_shape             = SQUARE;
qu8_t lfo_frequency              = 0;
qu32_t lfo_step                  = 0; // phase change per sample
qs12_t lfo_depth                 = 0;
volatile qu32_t lfo_phase        = 0;

// envelope variables
qu8_t release_time               = 0;
qu32_t release_step              = 0;
volatile qu32_t release_value    = QU32_ONE;
uint16_t release_sample          = 0;

// filter variables
float cutoff                     = FILTER_MAX;
float cutoff_sweep_offset        = 0.0;
float cutoff_sweep_rate          = 0.0;
float resonance                  = RESONANCE_MIN;
uint32_t sweep_t0                = 0;
qs15_t dsvf_f                    = 0;
qs15_t dsvf_q                    = 0;
uint16_t dsvf_x                  = 0;
uint16_t dsvf_y                  = 0;
uint16_t dsvf_r0                 = 0;
uint16_t dsvf_r1                 = 0;

// lfsr variables
uint16_t lfsr_value              = 0xFFFF;
qs15_t lfsr_output               = 0;
uint16_t lfsr_counter            = 0;
qu32_t lfsr_phase                = 0; // used to detect new period

// delay variables
qs15_t delay_feedback            = float_to_qs15(0.75);
qs15_t delay_wet                 = float_to_qs15(0.3);
qs15_t delay_dry                 = float_to_qs15(0.7);
uint16_t delay_time              = 0;
uint16_t delay_mix               = 0;               // mixed orginal and delayed sample
uint32_t buffer_index            = 0;               // current index in the send and resv buffers
volatile qs15_t output_sample    = 0;
volatile uint32_t read_address   = 0;               // RAM read address
volatile uint32_t write_address  = 0xC000;          // RAM write address ~1s
volatile int active_buffer       = 0;               // indexes which of the two sets of buffers the ISR should use
volatile dma_state_t dma_state   = DMA_IDLE;        // flag for the ISR to signal the main loop to update the buffers

void setup () {
    pinMode(PIN_ADC_CH0, INPUT);
    pinMode(PIN_ADC_CH1, INPUT);
    pinMode(PIN_SREG_DATA, INPUT);
    pinMode(PIN_SW_SUB, INPUT);
    pinMode(PIN_SW_RANGE, INPUT);
    pinMode(PIN_SW_SWEEP, INPUT);
    pinMode(PIN_TRIGGER, INPUT);
    pinMode(PIN_BOARD_LED, OUTPUT);
    pinMode(PIN_LFO_LED, OUTPUT);
    pinMode(PIN_I2S_BCLK, OUTPUT);
    pinMode(PIN_I2S_WCLK, OUTPUT);
    pinMode(PIN_I2S_DATA, OUTPUT);
    pinMode(PIN_SPI_CLK, OUTPUT);
    pinMode(PIN_SPI_MOSI, OUTPUT);
    pinMode(PIN_SPI_SS, OUTPUT);
    pinMode(PIN_SREG_CLK, OUTPUT);
    pinMode(PIN_SREG_LATCH, OUTPUT);
    pinMode(PIN_MUX_S0, OUTPUT);
    pinMode(PIN_MUX_S1, OUTPUT);
    pinMode(PIN_MUX_S2, OUTPUT);

    digitalWrite(PIN_SREG_LATCH, HIGH);
    digitalWrite(PIN_SREG_CLK, LOW);
    //digitalWrite(PIN_LFO_LED, LOW);

    Serial.begin(115200);
    // while (!Serial); // wait for a serial connection (terminal)

    input = new Input();
    spiDma = new SpiDma();

    setupI2S();
    setupTimer();
    TC4->COUNT16.CC[0].reg = 0x8000;

    for (uint16_t i = 0; i < SPI_BLOCK_SIZE; i++) {
        spiDma->write_buffer[0].data[i] = i + 0x100;
        spiDma->write_buffer[1].data[i] = i + 0x100;
        spiDma->read_buffer[0].data[i] = 0;
        spiDma->read_buffer[1].data[i] = 0;
    }

    spiDma->printWriteBuffer(0);

    Serial.println("Initialization completed");
}


// IRQ wrapper must be in this file
void DMAC_Handler () {
    spiDma->irqHandler();

    if (dma_state == DMA_WRITE_B) {
        // Serial.println('B');
        dma_state = DMA_READ_A;
    } else if (dma_state == DMA_READ_B) {
        // Serial.println('D');
        write_address = (write_address + SPI_BLOCK_BYTES) & (RAM_BYTES - 1);
        read_address = (write_address + (RAM_BYTES - delay_time * SPI_BLOCK_BYTES)) & (RAM_BYTES - 1);
        // read_address = (read_address + SPI_BLOCK_BYTES) & (RAM_BYTES - 1);
        dma_state = DMA_IDLE;
    }
}


inline uint16_t update_lfsr (uint16_t input) {
    uint16_t b10 = input >> 10;
    uint16_t b12 = input >> 12;
    uint16_t b13 = input >> 13;
    uint16_t b15 = input >> 15;
    return (input << 1) + ((b10 ^ b12 ^ b13 ^ b15) & 1);
}


qs15_t inline getSineAmplitude (qu32_t phase) {

    uint16_t index = mul_qu32_uint16(phase, SINE_SAMPLES * 4) & (SINE_SAMPLES - 1);

    if (phase < QU32_ONE / 4) {
        return SINE_TABLE[index];
    } else if (phase < QU32_ONE / 2) {
        return SINE_TABLE[SINE_SAMPLES-index-1];
    } else if (phase < QU32_ONE / 4 * 3) {
        return qs_invert(SINE_TABLE[index]);
    } else {
        return qs_invert(SINE_TABLE[SINE_SAMPLES-index-1]);
    }
}


// return the amplitude in [-1 - 1]
qs15_t inline getAmplitude(waveform_t waveform, qu32_t phase) {

    uint16_t index;
    qs15_t local_phase;
    qu32_t third;
    qu32_t fifth;

    switch (waveform) {
        case SQUARE:
            return (phase < QU32_ONE / 2) ? QS15_ONE : QS15_MINUS_ONE;

        case PULSE:
            return (phase < QU32_ONE / 5) ? QS15_ONE : QS15_MINUS_ONE;

        // the phase to amplitude ratio has a gain of two. Because of the fixed point limits of
        // [-1, 1) a multiplication could overflow. Therefore the phase is just subtracted twice.
        case SAW_DOWN:
           return QS15_ONE - qu32_to_qs15(phase) - qu32_to_qs15(phase);

        case SAW_UP:
           return QS15_MINUS_ONE + qu32_to_qs15(phase) + qu32_to_qs15(phase);

        case SAW_WHOOP:
            if (phase < QU32_ONE / 3) {
                local_phase = qu32_to_qs15(phase);
            } else { // amp = 1.375 * phase - 0.375 (with phase normalized to 0.5..1)
                local_phase = qu32_to_qs15(phase) - QS15_ONE / 3;
            }
            return QS15_MINUS_ONE + local_phase + local_phase + local_phase;

        case CAPACITOR:
            index = mul_qu32_uint16(phase, CAPACITOR_SAMPLES * 2) & (CAPACITOR_SAMPLES - 1);

            if (phase < QU32_ONE / 2) {
                return CAPACITOR_TABLE[index];
            } else {
                return qs_invert(CAPACITOR_TABLE[index]);
            }

        case SINE:
        case CHORD:
            return getSineAmplitude(phase);

        case SINE_H3:
            // sine + 3rd overtone
            return rshift1_qs15(getSineAmplitude(phase))
                   + rshift1_qs15(getSineAmplitude(phase + phase + phase));

        case LASER_SQUARE:
            if (phase >= QU32_ONE / 2) {
                return QS15_MINUS_ONE;
            } else {
                return getAmplitude(SQUARE, phase << 3);
            }

        case LASER_SAW:
            if (phase >= QU32_ONE / 2) {
                return QS15_MINUS_ONE;
            } else {
                return getAmplitude(SAW_DOWN, phase << 3);
            }

        case RANDOM:
            if (phase < lfsr_phase) { // new period, update value
                lfsr_output = (qs15_t) lfsr_value;
                lfsr_counter = 0;
            } else if (++lfsr_counter <= 16) {
                lfsr_value = update_lfsr(lfsr_value);
            }
            lfsr_phase = phase;
            return lfsr_output;
    }
}


void I2S_Handler() {

    // isr_t0 = micros();

    // write sample in compact stereo mode, interrupt flag is cleared automatically
    // uint16_t sample = spiDma->read_buffer[active_buffer].data[buffer_index];
    // I2S->DATA[0].reg = (uint32_t) sample << 16 | sample;
    I2S->DATA[0].reg = output_sample;

    // update oscillator and lfo phase. these overflow naturally
    osc_phase += osc_step;
    sub_phase += osc_step >> 1;
    lfo_phase += lfo_step;

    // decrease decay coefficient
    if (!trigger_state) {
        if (release_step < release_value) {
            release_value -= release_step;
        } else {
            release_value = 0;
        }
    }

    // update oscillator sweep
    if (!digitalRead(PIN_SW_SWEEP)) {
        osc_sweep_offset += osc_sweep_step;
        if (osc_sweep_offset > float_to_qs15(OSC_FREQ_RANGE)) {
            osc_sweep_offset = float_to_qs15(OSC_FREQ_RANGE);
        } else if (qs_invert(osc_sweep_offset) > float_to_qs15(OSC_FREQ_RANGE)) {
            osc_sweep_offset = qs_invert(float_to_qs15(OSC_FREQ_RANGE));
        }
    } else {
        osc_sweep_offset = 0;
    }

    qs15_t sweep_sum = qu16_to_qs15(osc_setpoint) + osc_sweep_offset;
    if (sweep_sum < float_to_qs15(OSC_FREQ_MIN)) {
        osc_setpoint_sweep = float_to_qu16(OSC_FREQ_MIN);
    } else if (sweep_sum > float_to_qs15(OSC_FREQ_MAX)) {
        osc_setpoint_sweep = float_to_qu16(OSC_FREQ_MAX);
    } else {
        osc_setpoint_sweep = qs15_to_qu16(sweep_sum);
    }

    // update oscillator frequency (glide)
    qs15_t osc_frequency_diff = qu16_to_qs15(osc_setpoint_sweep) - qu16_to_qs15(osc_frequency);
    if (osc_frequency_diff & 0x80000000) {
        osc_frequency -= min(qs_invert(osc_frequency_diff), GLIDE_RATE);
    } else {
        osc_frequency += min(osc_frequency_diff, GLIDE_RATE);
    }

    // get lfo value
    lfo_value = getAmplitude(lfo_shape, lfo_phase);
    lfo_value = rshift1_qs15(lfo_value) + QS15_ONE / 2 + 1; // normalize waveforms to [0, 1] for lfo

    // calculate oscillator velocity
    mod_frequency = mul_qs15_int16(mul_qs12_qs15(lfo_depth, lfo_value),
                                   qu16_to_uint16(osc_frequency));
    osc_step = (uint16_t) (qu16_to_uint16(osc_frequency) + mod_frequency) * (QU32_ONE / SAMPLE_RATE);

    // calculate oscillator value
    qs15_t amplitude = getAmplitude(input->osc_waveform, osc_phase);

    if (!digitalRead(PIN_SW_SUB)) {
        amplitude += getAmplitude(input->osc_waveform, sub_phase);
    }

    // update the filter
    dsvf_x = mul_qs15_int16(amplitude, OSC_AMP);
    dsvf_y = mul_qs15_int16(dsvf_f, dsvf_r0) + dsvf_r1;
    dsvf_r1 = dsvf_y;
    dsvf_r0 = mul_qs15_int16(dsvf_f, (dsvf_x - dsvf_y - mul_qs15_int16(dsvf_q, dsvf_r0))) + dsvf_r0;

    // multiply with envelope
    qs15_t release_coeff = qu32_to_qs15(release_value);
    release_coeff = mul_qs15(release_coeff, release_coeff); // make quadratic
    release_sample = mul_qs15_int16(release_coeff, dsvf_y);

    // update delay
    uint16_t feedback_sample = spiDma->read_buffer[active_buffer].data[buffer_index];
    feedback_sample = mul_qs15_int16(delay_feedback, feedback_sample);
    spiDma->write_buffer[active_buffer].data[buffer_index] =
        add_uint16_clip(release_sample, feedback_sample);
    delay_mix = add_uint16_clip(
        mul_qs15_int16(delay_dry, release_sample), mul_qs15_int16(delay_wet, feedback_sample));

    // store sample for next loop
    output_sample = (uint32_t) delay_mix << 16 | delay_mix;

    // check delay buffer update
    if (++buffer_index == SPI_BLOCK_SIZE) {
        buffer_index = 0;
        active_buffer = 1 - active_buffer;
        if (dma_state != DMA_IDLE) {
            Serial.println("E2");
        } else {
            dma_state = DMA_WRITE_A;
        }
    }

    // update LFO LED
    TCC0->CC[0].bit.CC = (uint16_t) qs15_to_qu16(lfo_value);

    // flag external trigger
    trigger_flag = trigger_flag || digitalRead(PIN_TRIGGER) == 0;

    // isr_dt = micros() - isr_t0;
}


void startLfo (waveform_t waveform) {
    lfo_shape = waveform;
    lfo_phase = 0;
}


void loop () {

    int i;
    char stringBuffer[100];

    loop_t0 = millis();

    // input_t0 = micros();
    input->update();
    // Serial.println(micros() - input_t0);

    // read buttons and trigger
    if (loop_t0 - btn_t0 >= BTN_TIME) {
        btn_t0 = loop_t0;

        if (input->button_state == 0 && !trigger_flag) {
            trigger_state = false;
        } else {
            trigger_state = true;
            release_value = QU32_ONE;
            cutoff_sweep_offset = 0.0;
            osc_sweep_offset = 0;
            sweep_t0 = micros();
        }

        if ((input->button_state & 0x40) & (last_btn_state ^ 0x40)) {
            startLfo(SINE);
        } else if ((input->button_state & 0x80) & (last_btn_state ^ 0x80)) {
            startLfo(SQUARE);
        } else if ((input->button_state & 0x01) & (last_btn_state ^ 0x01)) {
            startLfo(SAW_DOWN);
        } else if ((input->button_state & 0x04) & (last_btn_state ^ 0x04)) {
            startLfo(SAW_UP);
        } else if ((input->button_state & 0x20) & (last_btn_state ^ 0x20)) {
            startLfo(SINE_H3);
        } else if ((input->button_state & 0x10) & (last_btn_state ^ 0x10)) {
            startLfo(LASER_SQUARE);
        } else if ((input->button_state & 0x08) & (last_btn_state ^ 0x08)) {
            startLfo(LASER_SAW);
        } else if ((input->button_state & 0x02) & (last_btn_state ^ 0x02)) {
            startLfo(RANDOM);
        } else if (trigger_flag) {
            trigger_flag = false;
            startLfo(lfo_shape);
        }

        last_btn_state = input->button_state;
    }

    // calculate frequency
    qu16_t norm_osc_reading = uint16_to_qu16(input->pot_data.osc_frequency) >> ADC_RES_LOG2; // normalize reading to [0 - 1)
    norm_osc_reading = mul_qu16(norm_osc_reading, norm_osc_reading); // create quadratic curve
    osc_setpoint = norm_osc_reading * OSC_FREQ_RANGE + uint16_to_qu16(OSC_FREQ_MIN); // calculate frequency setpoint

    // calculate lfo frequency
    qu16_t norm_lfo_reading = uint16_to_qu16(input->pot_data.lfo_frequency) >> ADC_RES_LOG2;
    norm_lfo_reading = mul_qu16(norm_lfo_reading, norm_lfo_reading);

    float range = digitalRead(PIN_SW_RANGE) ? LFO_FREQ_RANGE_LOW : LFO_FREQ_RANGE_HIGH;
    lfo_frequency = mul_qu8(qu16_to_qu8(norm_lfo_reading), float_to_qu8(range));
    lfo_frequency += float_to_qu8(LFO_FREQ_MIN);
    lfo_step = mul_qu8_uint32(lfo_frequency, QU32_ONE / SAMPLE_RATE);

    // calculate lfo mod depth coefficient
    qs15_t norm_depth_reading = uint16_to_qs15(input->pot_data.lfo_depth) >> ADC_RES_LOG2;
    lfo_depth = mul_qs15_qs12(norm_depth_reading, float_to_qs12(LFO_DEPTH_RANGE))
        + float_to_qs12(LFO_DEPTH_MIN);

    // calculate decay coefficient
    qu16_t norm_release_reading = uint16_to_qu16(input->pot_data.release_time) >> ADC_RES_LOG2;
    release_time = mul_qu8(qu16_to_qu8(norm_release_reading), float_to_qu8(DECAY_MAX));
    if (release_time == 0) {
        release_step = QU32_ONE;
    } else {
        release_step = QU32_ONE / mul_qu8_uint32(release_time, SAMPLE_RATE);
    }

    // calculate delay coefficients
    uint16_t delay_wet_reading = input->pot_data.delay_wet; // [0 - 1023]
    qu16_t norm_delay_wet_reading = uint16_to_qu16(delay_wet_reading) >> (ADC_RES_LOG2 - 1); // [0 - 2]
    if (norm_delay_wet_reading < QU16_ONE) {
        delay_dry = QU16_ONE;
        delay_wet = mul_qu16(norm_delay_wet_reading, QU16_ONE);
    } else {
        delay_dry = QU16_ONE - mul_qu16(norm_delay_wet_reading - QU16_ONE, QU16_ONE);
        delay_wet = QU16_ONE;
    }

    uint16_t norm_delay_feedback_reading = uint16_to_qu16(input->pot_data.delay_feedback) >> ADC_RES_LOG2;
    delay_feedback = mul_qu16(norm_delay_feedback_reading, float_to_qu16(DELAY_FEEDBACK_MAX));
    delay_time = ((input->pot_data.delay_time * DELAY_TIME_RANGE) >> 10) + DELAY_TIME_MIN; // updated in the dma isr

    // calculate filter coefficients
    cutoff = input->pot_data.filter_cutoff / 1024.0 * FILTER_RANGE + FILTER_MIN;
    resonance = input->pot_data.filter_resonance / 1024.0 * RESONANCE_RANGE + RESONANCE_MIN;

    // update sweep offsets
    cutoff_sweep_rate = (input->pot_data.filter_sweep / 512.0 - 1.0) * FILTER_SWEEP_MAX;
    float dt;
    if (!trigger_state) {
        uint32_t sweep_t1 = micros();
        dt = (sweep_t1 - sweep_t0) * 1E-6;
        cutoff_sweep_offset += dt * cutoff_sweep_rate;
        sweep_t0 = sweep_t1;
    }

    // update frequency sweep parameters
    float norm_sweep_reading = input->pot_data.filter_sweep / 512.0 - 1.0;
    norm_sweep_reading = norm_sweep_reading * norm_sweep_reading * (norm_sweep_reading / abs(norm_sweep_reading));
    float osc_sweep_rate = norm_sweep_reading * OSC_SWEEP_MAX;
    osc_sweep_step = float_to_qs15(osc_sweep_rate / SAMPLE_RATE);

    // update filter coefficients
    float sweep_cutoff = constrain(cutoff + cutoff_sweep_offset, FILTER_MIN, FILTER_MAX);
    get_dsvf_coeffs(sweep_cutoff, resonance, &dsvf_f, &dsvf_q);

    if (dma_state == DMA_WRITE_A) {
        dma_state = DMA_WRITE_B;
        spiDma->write(1 - active_buffer, write_address);
    } else if (dma_state == DMA_READ_A) {
        dma_state = DMA_READ_B;
        spiDma->read(1 - active_buffer, read_address);
    }

    loop_t1 = millis() - loop_t0;

    // blink led at 2 Hz
    int t = millis();

    // // Serial.println(dma_state);
    // Serial.print(read_address, HEX);
    // Serial.print(" ");
    // Serial.println(write_address, HEX);

    if (t - led_t0 > PRINT_MS) {

        digitalWrite(PIN_LED, led_state);
        led_state = !led_state;
        led_t0 += PRINT_MS;

        Serial.println("");
        Serial.println(osc_sweep_rate);
        Serial.println(qs15_to_float(osc_sweep_step));
        Serial.println(qs15_to_float(osc_sweep_offset));
        Serial.println(osc_setpoint_sweep, HEX);
        Serial.println(qu16_to_float(osc_setpoint_sweep));
    }
}

//#include <I2S.h>
#include <sam.h>
#include <wiring_private.h>
#include "dubsiren.h"
#include "input.h"
#include "fixedpoint.h"
#include "spi_dma.h"
#include "i2s.h"
#include "sine.h"
#include "biquad.h"

Input *input;
SpiDma *spiDma;
uint32_t led_t0                  = 0;
bool led_state                   = true;
volatile uint32_t loop_t0        = 0;
volatile uint32_t loop_t1        = 0;

// button variables
uint8_t last_btn_state           = 0;
uint32_t input_t0                = 0;
uint32_t btn_t0                  = 0;
bool btn_state                   = false;

// oscillator variables
waveform_t osc_waveform          = SQUARE;
qu16_t osc_setpoint              = uint16_to_qu16(1000); // base frequency setpoint, set by PIN_OSC_POT
volatile qu32_t osc_phase        = 0;                // always positive [0 - 1]
volatile uint16_t osc_amplitude  = 5000;
volatile qu16_t osc_frequency    = osc_setpoint;     // base frequency current value
volatile uint16_t mod_frequency  = 0;                // additive frequency shift from lfo
volatile qu32_t osc_step         = osc_setpoint * (QU32_ONE / SAMPLE_RATE); // 1000 Hz phase change per sample

// lfo variables
waveform_t lfo_shape             = SQUARE;
qu8_t lfo_frequency              = float_to_qu8(2.0);
qu32_t lfo_step                  = mul_qu8_uint32(lfo_frequency, QU32_ONE / SAMPLE_RATE); // phase change per sample
qs15_t lfo_depth                 = QS15_ONE / 10;
volatile qu32_t lfo_phase        = 0;

// envelope variables
qu8_t decay_time                 = 0;
qu32_t decay_step                = 0;
volatile qu32_t decay_value      = QU32_ONE;

// filter variables
uint16_t cutoff                  = FILTER_MAX;

qs12_t filter_a[2]               = {0, 0};
qs12_t filter_b[3]               = {0, 0, 0};
uint16_t filter_in[3]            = {0, 0, 0};        // unfiltered
uint16_t filter_out[3]           = {0, 0, 0};        // filtered

// delay variables
qu8_t delay_time                 = float_to_qu8(0.5);
uint32_t delay_blocks            =
    qu8_to_uint32(mul_qu8(delay_time, float_to_qu8((float) SAMPLE_RATE / SPI_BLOCK_SIZE)));
qu16_t delay_mix_feedback        = float_to_qu16(0.3);
qu16_t delay_mix_original        = float_to_qu16(0.7);
uint16_t delay_mix               = 0;               // mixed orginal and delayed sample
uint32_t buffer_index            = 0;               // current index in the send and resv buffers
volatile uint32_t read_address   = 0;               // RAM read address
volatile int32_t write_address   = RAM_BYTES - (delay_blocks * SPI_BLOCK_BYTES); // RAM write address
volatile int active_buffer       = 0;               // indexes which of the two sets of buffers the ISR should use
volatile dma_state_t dma_state   = DMA_IDLE;        // flag for the ISR to signal the main loop to update the buffers

void setup () {
    pinMode(PIN_ADC_CH0, INPUT);
    pinMode(PIN_ADC_CH1, INPUT);
    pinMode(PIN_SREG_DATA, INPUT);
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

    Serial.begin(115200);
    // while (!Serial); // wait for a serial connection (terminal)

    input = new Input();
    spiDma = new SpiDma();

    setupI2S();
    get_lpf_coeffs(cutoff, 4, filter_a, filter_b);

    for (uint16_t i = 0; i < SPI_BLOCK_SIZE; i++) {
        spiDma->write_buffer[0].data[i] = i;
        spiDma->write_buffer[1].data[i] = i;
        spiDma->read_buffer[0].data[i] = 0;
        spiDma->read_buffer[1].data[i] = 0;
    }

    Serial.println("Initialization completed");
}


// IRQ wrapper must be in this file
void DMAC_Handler () {
    spiDma->irqHandler();

    if (dma_state == DMA_WRITE_B) {
        // Serial.println('a');
        dma_state = DMA_READ_A;
    } else if (dma_state == DMA_READ_B) {
        // Serial.println('b');
        read_address = (read_address + SPI_BLOCK_BYTES) & (RAM_BYTES - 1);
        write_address = (write_address + SPI_BLOCK_BYTES) & (RAM_BYTES - 1);
        dma_state = DMA_IDLE;
    } else {
        Serial.println("E1");
    }
}


// return the amplitude in [-1 - 1]
qs15_t inline getAmplitude(waveform_t waveform, qu32_t phase) {

    switch (waveform) {
        case SQUARE:
            //return (phase < QU16_ONE / 4) ? QS15_ONE : QS15_MINUS_ONE;
            if (phase < QU32_ONE / 2) {
                return QS15_ONE;
            } else {
                return QS15_MINUS_ONE;
            }

        // the phase to amplitude ratio has a gain of two. Because of the fixed point limits of
        // [-1, 1) a multiplication could overflow. Therefore the phase is just subtracted twice.
        case SAW:
           return QS15_ONE - qu32_to_qs15(phase) - qu32_to_qs15(phase);

        case TRIANGLE:
            qs15_t local_phase;
            if (phase < QU32_ONE / 2) {
                local_phase = qu32_to_qs15(phase);
                return QS15_MINUS_ONE + (local_phase << 2);
            } else {
                local_phase = (qu32_to_qs15(phase) - (QS15_ONE >> 1)); // [0.5 - 1] -> [0 - 1]
                return QS15_ONE - (local_phase << 2);
            }

        case SINE:
            // TODO: interpolation
            uint16_t index = mul_qu32_uint16(phase, SINE_SAMPLES * 4) & (SINE_SAMPLES - 1);
            if (phase < QU32_ONE / 4) {
                return SINE_TABLE[index];
            } else if (phase < QU32_ONE / 2) {
                return SINE_TABLE[SINE_SAMPLES - index - 1];
            } else if (phase < QU32_ONE / 4 * 3) {
                return qs_invert(SINE_TABLE[index]);
            } else {
                return qs_invert(SINE_TABLE[SINE_SAMPLES - index - 1]);
            }
    }
}


void I2S_Handler() {
    // write sample in compact stereo mode, interrupt flag is cleared automatically
    I2S->DATA[0].reg = (uint32_t) filter_out[0] << 16 | filter_out[0];
    // I2S->DATA[0].reg = (uint32_t) delay_mix << 16 | delay_mix;

    // uint16_t sample = spiDma->read_buffer[active_buffer].data[buffer_index];
    // I2S->DATA[0].reg = (uint32_t) sample << 16 | sample;

    // shift sample buffers
    filter_in[2] = filter_in[1];
    filter_in[1] = filter_in[0];
    filter_out[2] = filter_out[1];
    filter_out[1] = filter_out[0];

    // update oscillator and lfo phase. these overflow naturally
    osc_phase += osc_step;
    lfo_phase += lfo_step;

    // decrease decay coefficient
    if (!btn_state) {
        if (decay_step < decay_value) {
            decay_value -= decay_step;
        } else {
            decay_value = 0;
        }
    }

    // update oscillator frequency (glide)
    qs15_t osc_frequency_diff = qu16_to_qs15(osc_setpoint) - qu16_to_qs15(osc_frequency);
    if (osc_frequency_diff & 0x80000000) {
        osc_frequency -= min(qs_invert(osc_frequency_diff), GLIDE_RATE);
    } else {
        osc_frequency += min(osc_frequency_diff, GLIDE_RATE);
    }

    // get lfo value
    qs15_t lfo_value = getAmplitude(lfo_shape, lfo_phase);

    mod_frequency = mul_qs15_int16(mul_qs15(lfo_value, lfo_depth), qu16_to_uint16(osc_frequency));
    osc_step = (uint16_t) (qu16_to_uint16(osc_frequency) + mod_frequency) * (QU32_ONE / SAMPLE_RATE);

    qs15_t amplitude = getAmplitude(osc_waveform, osc_phase);
    qs15_t decay_coeff = qu32_to_qs15(decay_value);
    decay_coeff = mul_qs15(decay_coeff, decay_coeff); // make quadratic

    // multiply the waveform with the decay coefficient and use that to scale the amplitude
    filter_in[0] = mul_qs15_int16(amplitude, osc_amplitude);
    filter_in[0] = mul_qs15_int16(decay_coeff, filter_in[0]);

    // apply biquad filter
    filter_out[0] = mul_qs12_int16(filter_b[0], filter_in[0])
                    + mul_qs12_int16(filter_b[1], filter_in[1])
                    + mul_qs12_int16(filter_b[2], filter_in[2])
                    + mul_qs12_int16(filter_a[0], filter_out[1])
                    + mul_qs12_int16(filter_a[1], filter_out[2]);

    // update delay
    delay_mix = add_uint16_clip(
        mul_qu16_uint16(delay_mix_original, filter_out[0]),
        mul_qu16_uint16(delay_mix_feedback, spiDma->read_buffer[active_buffer].data[buffer_index]));

    // spiDma->write_buffer[active_buffer].data[buffer_index] = delay_mix;
    // spiDma->write_buffer[active_buffer].data[buffer_index] = filter_out[0];

    // if (++buffer_index == SPI_BLOCK_SIZE) {
    //     buffer_index = 0;
    //     active_buffer = 1 - active_buffer;
    //     if (dma_state != DMA_IDLE) {
    //         Serial.println("E2");
    //     } else {
    //         dma_state = DMA_WRITE_A;
    //     }
    // }
}


void loop () {

    int i;
    char stringBuffer[100];

    loop_t0 = millis();

    // input_t0 = micros();
    input->update();
    // Serial.println(micros() - input_t0);

    // read buttons
    if (loop_t0 - btn_t0 >= BTN_TIME) {
        btn_t0 = loop_t0;

        if (input->button_state == 0) {
            btn_state = false;
        } else {
            btn_state = true;
            decay_value = QU32_ONE;
        }

        if ((input->button_state & 0x40) & (last_btn_state ^ 0x40)) {
            lfo_shape = SQUARE;
        } else if ((input->button_state & 0x80) & (last_btn_state ^ 0x80)) {
            lfo_shape = SAW;
        } else if ((input->button_state & 0x01) & (last_btn_state ^ 0x01)) {
            lfo_shape = TRIANGLE;
        } else if ((input->button_state & 0x04) & (last_btn_state ^ 0x04)) {
            lfo_shape = SINE;
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
    lfo_frequency = mul_qu8(qu16_to_qu8(norm_lfo_reading), float_to_qu8(LFO_FREQ_RANGE))
        + float_to_qu8(LFO_FREQ_MIN);
    lfo_step = mul_qu8_uint32(lfo_frequency, QU32_ONE / SAMPLE_RATE);

    // calculate lfo mod depth coefficient
    qs15_t norm_depth_reading = uint16_to_qs15(input->pot_data.lfo_depth) >> ADC_RES_LOG2;
    lfo_depth = mul_qs15_int16(norm_depth_reading, float_to_qs15(LFO_DEPTH_RANGE))
        + float_to_qs15(LFO_DEPTH_MIN);

    // calculate decay coefficient
    qu16_t norm_decay_reading = uint16_to_qu16(input->pot_data.decay_time) >> ADC_RES_LOG2;
    decay_time = mul_qu8(qu16_to_qu8(norm_decay_reading), float_to_qu8(DECAY_MAX));
    if (decay_time == 0) {
        decay_step = QU32_ONE;
    } else {
        decay_step = QU32_ONE / mul_qu8_uint32(decay_time, SAMPLE_RATE);
    }

    // calculate filter coefficients
    cutoff = (uint16_t) ((((uint32_t) input->pot_data.filter_cutoff * FILTER_RANGE)
        >> ADC_RES_LOG2) + FILTER_MIN);
    get_lpf_coeffs(cutoff, 4, filter_a, filter_b);

    // // update delay buffers
    // // the rest is handled in the dma ISR
    // if (dma_state == DMA_IDLE) {
    //     dma_state = DMA_WRITE_B;
    //     spiDma->write(0, write_address);
    // } else if (dma_state == DMA_READ_A) {
    //     dma_state = DMA_READ_B;
    //     spiDma->read(0, read_address);
    // }

    loop_t1 = millis() - loop_t0;

    // blink led at 2 Hz
    int t = millis();

    // // Serial.println(dma_state);
    // Serial.print(read_address, HEX);
    // Serial.print(" ");
    // Serial.println(write_address, HEX);

    if (t - led_t0 > MAIN_LOOP_MS) {

        digitalWrite(PIN_LED, led_state);
        led_state = !led_state;
        led_t0 += MAIN_LOOP_MS;

        // input->printPots();
        Serial.println("");
        Serial.println(input->button_state, HEX);
        Serial.println(input->osc_waveform);
        Serial.println(btn_state);
        Serial.println(lfo_shape);
        Serial.println(decay_time / 256.0);

        // Serial.println(dma_state);
        //
        // DMAC->CHID.reg = DMA_CH_READ;
        // sprintf(stringBuffer, "%04X %04X", SERCOM1->SPI.STATUS.reg, SERCOM1->SPI.INTFLAG.reg);
        // Serial.println(stringBuffer);
        //
        // spiDma->printDmaDescriptor(1, true);
        // spiDma->printWriteBuffer(0);
        // spiDma->printReadBuffer(0);
        // Serial.println("");

        // spiDma->printBtCount(0);

        // Serial.print("write ");
        // for (i = 0; i < 16; i++) {
        //     sprintf(stringBuffer, "%04X ", spiDma->write_buffer[0].data[i]);
        //     Serial.print(stringBuffer);
        // }
        // Serial.print(" ... ");
        // for (i = 1008; i < 1024; i++) {
        //     sprintf(stringBuffer, "%04X ", spiDma->write_buffer[0].data[i]);
        //     Serial.print(stringBuffer);
        // }
        // Serial.println("");
        // Serial.print("read  ");
        // for (i = 0; i < 16; i++) {
        //     sprintf(stringBuffer, "%04X ", spiDma->read_buffer[0].data[i]);
        //     Serial.print(stringBuffer);
        // }
        // Serial.print(" ... ");
        // for (i = 1008; i < 1024; i++) {
        //     sprintf(stringBuffer, "%04X ", spiDma->read_buffer[0].data[i]);
        //     Serial.print(stringBuffer);
        // }
        // Serial.println("\n");
    }
}

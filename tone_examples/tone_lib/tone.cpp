#include "tone.h"
#include <Arduino.h>

static volatile uint8_t  finish_flag;
static volatile uint32_t inter_count;

static uint8_t tone_pin;

void tone(uint8_t pin, uint16_t freq, uint32_t duration)
{
    uint8_t nrf_pin = digitalPinToPin(pin);

    // Store pin
    tone_pin = pin;

    log_info("TONE : Start a tone\r\n");
    log_info("TONE : frequency %d \r\n", freq);
    log_info("TONE : duration %d \r\n", duration);
   
    // Find appropriate values for PRESCALER and COMPARE registers
    uint8_t index;
    uint32_t compare;
    uint32_t prescaler;
    for (index=0; index<= 9; index++)
    {
        prescaler = index;
        compare = 16000000UL / freq;
        compare = compare >> (prescaler+1);
        compare = compare - 1;
        if ((compare >= 2) && (compare <= 65535))
            break;
    }
    log_info("TONE : The prescaler is %d \r\n", prescaler);
    log_info("TONE : The compare is %d \r\n", compare);
    // Check duration
    if(duration > 0) {
        finish_flag = 1;
        //inter_count = ((freq * duration) / 1000) * 2;
        inter_count = 2 * freq * duration / 1000;
        if(inter_count == 0)
            inter_count = 1;
    }
    else {
        finish_flag = 0;
        inter_count = 0xFFFFFFFF;
    }
    // Config GPIOTE task out.
    NRF_GPIOTE->CONFIG[TONE_USED_GPIOTE_NUM] &= ~( GPIOTE_CONFIG_MODE_Msk | GPIOTE_CONFIG_POLARITY_Msk);
    NRF_GPIOTE->CONFIG[TONE_USED_GPIOTE_NUM] = ( (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos) |
                                                 (nrf_pin << GPIOTE_CONFIG_PSEL_Pos) |
                                                 (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |  //Task toggle
                                                 (GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos) //Inital value LOW
                                               );
   
    NRF_PPI->CH[TONE_USED_PPI_CHANNAL].EEP = (uint32_t)(&TONE_USED_TIMER->EVENTS_COMPARE[0]);
    NRF_PPI->CH[TONE_USED_PPI_CHANNAL].TEP = (uint32_t)(&NRF_GPIOTE->TASKS_OUT[TONE_USED_GPIOTE_NUM]);
    NRF_PPI->CHEN |= (1 << TONE_USED_PPI_CHANNAL);

    log_info("TONE : Init TIMIERx \r\n");
    // Configure TIMERx
    TONE_USED_TIMER->TASKS_STOP = 1;
    TONE_USED_TIMER->TASKS_CLEAR = 1;

    TONE_USED_TIMER->MODE = TIMER_MODE_MODE_Timer;
    TONE_USED_TIMER->PRESCALER = prescaler;
    TONE_USED_TIMER->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
    TONE_USED_TIMER->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);

    TONE_USED_TIMER->CC[0] = (uint16_t)(compare);
    TONE_USED_TIMER->EVENTS_COMPARE[0] = 0;

    TONE_USED_TIMER->INTENCLR = 0xFFFFFFFF;
    TONE_USED_TIMER->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
    // Enable IRQn
    NVIC_SetPriority(TONE_USED_TIMER_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_ClearPendingIRQ(TONE_USED_TIMER_IRQn);
    NVIC_EnableIRQ(TONE_USED_TIMER_IRQn);
    // Start TIMER
    log_info("TONE : Start TIMIERx \r\n");
    TONE_USED_TIMER->TASKS_START = 1;
}

void noTone(uint8_t pin)
{
    if(pin != tone_pin)
        return;

    TONE_USED_TIMER->TASKS_STOP = 1;
    NVIC_DisableIRQ(TONE_USED_TIMER_IRQn);

    NRF_PPI->CHEN &= (1 << TONE_USED_PPI_CHANNAL);
    // Disable GPIOTE
    NRF_GPIOTE->CONFIG[TONE_USED_GPIOTE_NUM] &= ~( GPIOTE_CONFIG_MODE_Msk | GPIOTE_CONFIG_POLARITY_Msk);
    // Clear variables
    tone_pin    = 0xFF;
    finish_flag = 0;
    inter_count = 0;
}

#ifdef __cplusplus
extern "C"{
#endif
void TONE_USED_TIMER_IRQHandler(void)
{
    TONE_USED_TIMER->EVENTS_COMPARE[0] = 0;
    if(finish_flag == 1) {
        if(inter_count)
            inter_count--;
        else
            noTone(tone_pin);
    }
}

#ifdef __cplusplus
}
#endif

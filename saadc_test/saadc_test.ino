#define MY_DEBUG

#include "SEGGER_RTT.h"

__STATIC_INLINE void nrf_saadc_disable(void)
{
    NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos);
}

__STATIC_INLINE void nrf_saadc_int_enable(uint32_t saadc_int_mask)
{
    NRF_SAADC->INTENSET = saadc_int_mask;
}

__STATIC_INLINE void nrf_saadc_enable(void)
{
    NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);
}


#define LED_R_PIN  (11)
#define LED_G_PIN  (8)

#define BUTTON_PIN  (6)
#define ADC_PIN PIN_A1

#define SAMPLES_PER_SECOND  20000
#define PPI_CHANNEL         (7)
#define ADC_CHANNEL         (2)

volatile int16_t adcValue;
volatile bool adcFlag = false;

void setup()
{
  SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
  SEGGER_RTT_Init();

  pinMode(ADC_PIN, INPUT);
  digitalWrite(ADC_PIN,LOW);

  pinMode(BUTTON_PIN,INPUT);
  digitalWrite(BUTTON_PIN,HIGH);

  pinMode(LED_R_PIN,OUTPUT);
  digitalWrite(LED_R_PIN,HIGH);

  pinMode(LED_G_PIN,OUTPUT);
  digitalWrite(LED_G_PIN,HIGH);

  initADC();
  initTimer4();
  initPPI();

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onPulse, CHANGE);
}

void onPulse()
{
  const bool tripped = digitalRead(BUTTON_PIN) == HIGH;
  SEGGER_RTT_printf(0, "Button: %p.\r\n", tripped);
}

void loop()
{
  static uint32_t previousMillis = 0;
  if ( adcFlag )
  {
    adcFlag = false;

    digitalWrite(LED_R_PIN,LOW);
    delay(50);
    digitalWrite(LED_R_PIN,HIGH);
    delay(50);

    if (adcValue < 0) {
      adcValue = 0;
    }
    //adcValue = mapResolution(adcValue, resolution, saadcResolution);
    SEGGER_RTT_printf(0, " ms A0: %p.\r\n", adcValue);
  }
}

void initTimer4()
{
  NRF_TIMER4->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER4->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
  NRF_TIMER4->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
  NRF_TIMER4->PRESCALER = 0;
  NRF_TIMER4->CC[0] = 16000000 / SAMPLES_PER_SECOND; // Needs prescaler set to 0 (1:1) 16MHz clock
  NRF_TIMER4->TASKS_START = 1;
}

void initPPI()
{
  NRF_PPI->CH[PPI_CHANNEL].EEP = ( uint32_t )&NRF_TIMER4->EVENTS_COMPARE[0];
  NRF_PPI->CH[PPI_CHANNEL].TEP = ( uint32_t )&NRF_SAADC->TASKS_START;
  NRF_PPI->FORK[PPI_CHANNEL].TEP = ( uint32_t )&NRF_SAADC->TASKS_SAMPLE;
  NRF_PPI->CHENSET = ( 1UL << PPI_CHANNEL );
  NRF_PPI->CHEN = ( 1UL << PPI_CHANNEL );
}

extern "C" void SAADC_IRQHandler( void )
{
  if ( NRF_SAADC->EVENTS_END != 0 )
  {
    NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->EVENTS_CH[ADC_CHANNEL].LIMITH = 0;
    NRF_SAADC->EVENTS_CH[ADC_CHANNEL].LIMITL = 0;

    adcFlag = true;
  }
}

void initADC()
{
  static uint32_t saadcReference  = SAADC_CH_CONFIG_REFSEL_Internal;
  static uint32_t saadcGain       = SAADC_CH_CONFIG_GAIN_Gain1_5;

  static uint32_t pin             = SAADC_CH_PSELP_PSELP_AnalogInput1 << SAADC_CH_PSELP_PSELP_Pos; //P0.03/AIN1

  //saadcReference = SAADC_CH_CONFIG_REFSEL_VDD1_4;
  //saadcGain      = SAADC_CH_CONFIG_GAIN_Gain1_4;

  nrf_saadc_disable();

  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_14bit;

  for (int i = 0; i < 8; i++) {
    NRF_SAADC->CH[i].PSELN = SAADC_CH_PSELP_PSELP_NC;
    NRF_SAADC->CH[i].PSELP = SAADC_CH_PSELP_PSELP_NC;
    NRF_SAADC->CH[i].LIMIT = 0x00000000;
  }

  NRF_SAADC->CH[ADC_CHANNEL].CONFIG =
                              ((SAADC_CH_CONFIG_RESP_Bypass   << SAADC_CH_CONFIG_RESP_Pos)   & SAADC_CH_CONFIG_RESP_Msk)
                            | ((SAADC_CH_CONFIG_RESN_Bypass   << SAADC_CH_CONFIG_RESN_Pos)   & SAADC_CH_CONFIG_RESN_Msk)
                            | ((saadcGain                     << SAADC_CH_CONFIG_GAIN_Pos)   & SAADC_CH_CONFIG_GAIN_Msk)
                            | ((saadcReference                << SAADC_CH_CONFIG_REFSEL_Pos) & SAADC_CH_CONFIG_REFSEL_Msk)
                            | ((SAADC_CH_CONFIG_TACQ_3us      << SAADC_CH_CONFIG_TACQ_Pos)   & SAADC_CH_CONFIG_TACQ_Msk)
                            | ((SAADC_CH_CONFIG_MODE_SE       << SAADC_CH_CONFIG_MODE_Pos)   & SAADC_CH_CONFIG_MODE_Msk);

  NRF_SAADC->CH[ADC_CHANNEL].PSELP = pin;
  NRF_SAADC->CH[ADC_CHANNEL].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;

  NRF_SAADC->RESULT.PTR = ( uint32_t )&adcValue;
  NRF_SAADC->RESULT.MAXCNT = 1;  // One sample

  nrf_saadc_enable();

  NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
  while ( NRF_SAADC->EVENTS_CALIBRATEDONE == 0 );
  NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
  while ( NRF_SAADC->STATUS == ( SAADC_STATUS_STATUS_Busy << SAADC_STATUS_STATUS_Pos ) );
  
  NRF_SAADC->CH[ADC_CHANNEL].LIMIT = 2000 << 15 | 0;
  nrf_saadc_int_enable( SAADC_INTENSET_CH2LIMITH_Enabled << SAADC_INTENSET_CH2LIMITH_Pos );
  
  NRF_SAADC->EVENTS_END = 0;
  NVIC_ClearPendingIRQ( SAADC_IRQn );
  NVIC_SetPriority( SAADC_IRQn, 1UL );
  NVIC_EnableIRQ( SAADC_IRQn );
}

/*
********************************************************************************
* COPYRIGHT(c) ЗАО «ЧИП и ДИП», 2018
* 
* Программное обеспечение предоставляется на условиях «как есть» (as is).
* При распространении указание автора обязательно.
********************************************************************************
*/


#include "RDC2_0043_board.h"
#include "RDC2_0043_USB.h"
#include "SPI_EEPROM.h"
#include "Display.h"
#include "UserCtrl.h"
#include "EEPROM_MAP.h"
#include "LED.h"
#include "Output.h"
#include "Thermistor.h"
#include "ADC.h"


static volatile uint16_t State = STATE_DEVICE_ON | STATE_DATA_UPD;
static TempSetType TempSets; 
static SMAType SMAfilter;


int main()
{
  RDC2_0043_Init();
  
  for(;;)
  {    
    if ((*RDC2_0043_USB_GetStatus()) != RDC2_0043_USB_IDLE)
    {
      uint8_t *NewData = (uint8_t *)RDC2_0043_USB_GetPacket();
      uint8_t USBFlagToClear = 0;
      
      if (((*RDC2_0043_USB_GetStatus()) & RDC2_0043_USB_EEPROM_READ) == RDC2_0043_USB_EEPROM_READ)
      {      
        EEPROM_Read(CONFIG_SIZE, CONFIG_START_ADDRESS, &NewData[USB_DATA_POS]);        
        RDC2_0043_USB_WhileNotReadyToSend();
        RDC2_0043_USB_SendData(NewData);        
        USBFlagToClear = RDC2_0043_USB_EEPROM_READ;
      }
      
      else if (((*RDC2_0043_USB_GetStatus()) & RDC2_0043_USB_EEPROM_WRITE) == RDC2_0043_USB_EEPROM_WRITE)
      {           
        EEPROM_Write(CONFIG_SIZE, CONFIG_START_ADDRESS, &NewData[USB_DATA_POS]);
        ActivateConfiguration(&NewData[USB_DATA_POS]);        
        USBFlagToClear = RDC2_0043_USB_EEPROM_WRITE;
      }
      
      RDC2_0043_USB_ClearStatus(USBFlagToClear);
    }
    
    if (((State & STATE_DATA_UPD) == STATE_DATA_UPD) && ((State & STATE_MENU_ACTIVE) != STATE_MENU_ACTIVE))
    {
      int16_t Temperature = (int16_t)(Therm_GetTempFromADC(SMAfilter.Result) * TEMP_FLOAT_TO_INT_VALUE);
      
      Display_Update(Temperature);
      State &= ~STATE_DATA_UPD;
      DATA_UPDATE_TIMER->CR1 |= TIM_CR1_CEN;
      
      if ((State & STATE_OUT_ENABLED) == STATE_OUT_ENABLED)
      {
        if (Temperature <= (TempSets.Tnom - TempSets.DeltaLow))
          TempEventHandler(TEMP_LOWER);
        else if (Temperature >= (TempSets.Tnom + TempSets.DeltaHigh))
          TempEventHandler(TEMP_HIGHER);
      }      
    }
        
    if ((State & STATE_OUT_ENABLE_EVENT) == STATE_OUT_ENABLE_EVENT)
    {
      if (((State & STATE_OUT_ENABLED) == STATE_OUT_ENABLED))
        State |= STATE_DATA_UPD;
        
      else
      {
        SetOutNotActive();
      }
      
      State &= ~STATE_OUT_ENABLE_EVENT;
    }
    
    if ((State & STATE_KEY_PRESSED_EVENT) == STATE_KEY_PRESSED_EVENT)
    {
      KeyPressedEventHandler();
      State &= ~(STATE_KEY_PRESSED_EVENT | STATE_KEYS_VALUE);
    }
  }
}
//------------------------------------------------------------------------------
void RDC2_0043_Init()
{
  //HSI, PLL, 48 MHz
  FLASH->ACR = FLASH_ACR_PRFTBE | (uint32_t)FLASH_ACR_LATENCY;  
  // частота PLL: (HSI / 2) * 12 = (8 / 2) * 12 = 48 (МГц)
  RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI_DIV2 | RCC_CFGR_PLLMUL12);
  RCC->CR |= RCC_CR_PLLON;
  while((RCC->CR & RCC_CR_PLLRDY) == 0);
 
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
  RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL);
  
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN  | RCC_AHBENR_GPIOFEN;
  
  Output_Init();
  //LED
  LED_Init();  
  //EEPROM  
  if (EEPROM_Init() != EEPROM_PRESENT)
    ErrorHandler(ERROR_EEPROM);
  
  //Display
  Display_Init(TEMP_POINT_POSITION, POINT_ALWAYS_ON_DURATION);
  
  RCC->DATA_UPDATE_TIMER_ENR |= DATA_UPDATE_TIMER_CLK_EN;
  DATA_UPDATE_TIMER->PSC = DATA_UPDATE_TIMER_PSC;
  DATA_UPDATE_TIMER->ARR = DATA_UPDATE_TIMER_ARR;
  DATA_UPDATE_TIMER->DIER = TIM_DIER_UIE;
  NVIC_EnableIRQ(DATA_UPDATE_TIMER_IRQ);
  NVIC_SetPriority(DATA_UPDATE_TIMER_IRQ, DATA_UPDATE_PRIORITY);
  
  //USB
  RDC2_0043_USB_Init();
  //управление
  UserCtrl_Init(&KeyPressedEvent, &OutEnableChangedEvent);
  
  ADC_Init();
  
  uint8_t Config[CONFIG_SIZE];
  EEPROM_Read(CONFIG_SIZE, CONFIG_START_ADDRESS, Config);
  if (Config[MODE_OFFSET] > MODE_COOL)
  {
    Config[MODE_OFFSET] = MODE_DEFAULT;
    Config[ACTIVE_LEVEL_OFFSET] = ACTIVE_LEVEL_DEFAULT;
    
    for (uint8_t i = 0; i < THERMISTOR_R_NOMINAL_SIZE; i++)
      Config[THERMISTOR_R_NOMINAL_OFFSET + i] = THERMISTOR_R_NOMINAL_DEFAULT >> (i * 8);
    
    for (uint8_t i = 0; i < THERMISTOR_B_CONSTANT_SIZE; i++)
      Config[THERMISTOR_B_CONSTANT_OFFSET + i] = THERMISTOR_B_CONSTANT_DEFAULT >> (i * 8);
    
    for (uint8_t i = 0; i < TEMP_DELTA_LOW_SIZE; i++)
      Config[TEMP_DELTA_LOW_OFFSET + i] = TEMP_DELTA_LOW_DEFAULT >> (i * 8);
    
    for (uint8_t i = 0; i < TEMP_DELTA_HIGH_SIZE; i++)
      Config[TEMP_DELTA_HIGH_OFFSET + i] = TEMP_DELTA_HIGH_DEFAULT >> (i * 8);
    
    for (uint8_t i = 0; i < TEMP_NOMINAL_SIZE; i++)
      Config[TEMP_NOMINAL_OFFSET + i] = TEMP_NOMINAL_DEFAULT >> (i * 8);
  }
  else
    State |= STATE_OUT_ENABLE_EVENT;
          
  ActivateConfiguration(Config);
  
  if ((State & STATE_OUT_ENABLE_EVENT) == STATE_OUT_ENABLE_EVENT)
  {
    if (GetInputLevel(ENABLE_IN_GPIO, ENABLE_IN_PIN) == OUT_ENABLED)
      State |= STATE_OUT_ENABLED;
    
    State &= ~STATE_OUT_ENABLE_EVENT;
  }
  
  SMA_Init();
  
  Display_ON();
  
  DATA_UPDATE_TIMER->CR1 |= TIM_CR1_CEN;
  DATA_UPDATE_TIMER->EGR = TIM_EGR_UG;
}
//------------------------------------------------------------------------------
void TempEventHandler(uint8_t TempFlag)
{ 
  uint8_t Mode = (State & STATE_MODE_BIT) >> STATE_MODE_BIT_OFFSET;
    
  if (((Mode == MODE_HEAT) && (TempFlag == TEMP_LOWER)) 
   || ((Mode == MODE_COOL) && (TempFlag == TEMP_HIGHER)))
  {
    if ((State & STATE_OUT_ACTIVE) != STATE_OUT_ACTIVE)
    {
      Output_Activate();
      LED_ON();
      State |= STATE_OUT_ACTIVE;      
    }
  }
  
  else if (((Mode == MODE_HEAT) && (TempFlag == TEMP_HIGHER))
        || ((Mode == MODE_COOL) && (TempFlag == TEMP_LOWER)))
  {
    if ((State & STATE_OUT_ACTIVE) == STATE_OUT_ACTIVE)
    {
      SetOutNotActive(); 
    }
  }  
}
//------------------------------------------------------------------------------
void OutEnableChangedEvent(uint8_t EnableState)
{
  State |= STATE_OUT_ENABLE_EVENT;
  if (EnableState == OUT_DISABLED)
    State &= ~STATE_OUT_ENABLED;
  else
    State |= STATE_OUT_ENABLED;
}
//------------------------------------------------------------------------------
void KeyPressedEvent(uint8_t Key)
{
  State |= STATE_KEY_PRESSED_EVENT | (Key << STATE_KEYS_VALUE_OFFSET);  
}
//------------------------------------------------------------------------------
void KeyPressedEventHandler()
{
  switch((State & STATE_KEYS_VALUE) >> STATE_KEYS_VALUE_OFFSET)
  {
    case(MENU_KEY):
    {
      if ((State & STATE_MENU_ACTIVE) != STATE_MENU_ACTIVE)
      {
        State |= STATE_MENU_ACTIVE;        
        DATA_UPDATE_TIMER->CR1 &= ~TIM_CR1_CEN;        
        DATA_UPDATE_TIMER->SR = 0;
        SetOutNotActive();
        Display_PointSet(TEMP_POINT_POSITION, TEMP_SET_POINT_DURATION);
        Display_Update(TempSets.Tnom);
        State &= ~STATE_DATA_UPD;
      }
      else
      {
        State &= ~STATE_MENU_ACTIVE;        
        Display_PointSet(TEMP_POINT_POSITION, POINT_ALWAYS_ON_DURATION);
        
        uint8_t NewTempNom[] = { TempSets.Tnom, TempSets.Tnom >> 8 };        
        EEPROM_Write(TEMP_NOMINAL_SIZE, CONFIG_START_ADDRESS + TEMP_NOMINAL_OFFSET, NewTempNom);
        State |= STATE_DATA_UPD;
        DATA_UPDATE_TIMER->CR1 |= TIM_CR1_CEN;
      }
    }
    break;
    
    case(ON_OFF_KEY):
      if ((State & STATE_DEVICE_ON) == STATE_DEVICE_ON)
      {
        SetOutNotActive();
        Display_OFF();
        UserCtrlDisable();
        DATA_UPDATE_TIMER->CR1 &= ~TIM_CR1_CEN;
        
        State &= ~STATE_MASK_EXCEPT_MODE;
      }
      else
      {
        Display_ON();
        UserCtrlEnable();
        
        if (GetInputLevel(ENABLE_IN_GPIO, ENABLE_IN_PIN) == OUT_ENABLED)
          State |= STATE_OUT_ENABLED;
        
        SMA_Init();
        
        State |= STATE_DEVICE_ON | STATE_DATA_UPD;
        DATA_UPDATE_TIMER->CR1 |= TIM_CR1_CEN;
      }
    break;

    case(MINUS_KEY):
      if ((State & STATE_MENU_ACTIVE) == STATE_MENU_ACTIVE)
      {
        TempSets.Tnom--;
        Display_Update(TempSets.Tnom);
      }
    break;
    
    case(PLUS_KEY):
      if ((State & STATE_MENU_ACTIVE) == STATE_MENU_ACTIVE)
      {
        TempSets.Tnom++;      
        Display_Update(TempSets.Tnom);
      }
    break;
    
    default:      
    break;
  }
}
//------------------------------------------------------------------------------
void INFO_UPDATE_TIMER_ISR(void)
{
  DATA_UPDATE_TIMER->SR &= ~TIM_SR_UIF;
  
  SMAfilter.Data[SMA_SAMPLES] = (float)ADC_GetData() / SMA_SAMPLES;
  float SMANewResult = SMAfilter.Result - SMAfilter.Data[0] + SMAfilter.Data[SMA_SAMPLES];
  
  float Difference = SMANewResult - SMAfilter.Result;
  if (Difference < 0)
    Difference *= -1;
  
  if (Difference >= DATA_UPDATE_THRESHOLD)
  {
    SMAfilter.Result = SMANewResult;
    for(uint8_t i = 0; i < SMA_SAMPLES; i++)
      SMAfilter.Data[i] = SMAfilter.Data[i + 1];
    
    State |= STATE_DATA_UPD;
    DATA_UPDATE_TIMER->CR1 &= ~TIM_CR1_CEN;
  }
}
//------------------------------------------------------------------------------
void SetOutNotActive()
{
  Output_DeActivate();
  LED_OFF();
  State &= ~STATE_OUT_ACTIVE;
}
//------------------------------------------------------------------------------
void SMA_Init()
{
  SMAfilter.Result = 0;
  for (uint8_t i = 0; i < SMA_SAMPLES; i++)
  {
    uint16_t ADCData = ADC_GetData();
    SMAfilter.Data[i] = (float)ADCData / SMA_SAMPLES;
    SMAfilter.Result += SMAfilter.Data[i];    
  }
}
//------------------------------------------------------------------------------
void ActivateConfiguration(uint8_t *Configuration)
{
  State &= ~STATE_MODE_BIT;
  State |= Configuration[MODE_OFFSET] << STATE_MODE_BIT_OFFSET;  
  Output_SetActiveLevel(Configuration[ACTIVE_LEVEL_OFFSET]);
  
  uint32_t ThermRnom = 0;
  for (uint8_t i = 0; i < THERMISTOR_R_NOMINAL_SIZE; i++)
    ThermRnom |= Configuration[THERMISTOR_R_NOMINAL_OFFSET + i] << (i * 8);
  
  Therm_Init(ThermRnom, (Configuration[THERMISTOR_B_CONSTANT_OFFSET] | (Configuration[THERMISTOR_B_CONSTANT_OFFSET + 1] << 8)));
  
  TempSets.DeltaLow = (Configuration[TEMP_DELTA_LOW_OFFSET] | (Configuration[TEMP_DELTA_LOW_OFFSET + 1] << 8));
  TempSets.DeltaHigh = (Configuration[TEMP_DELTA_HIGH_OFFSET] | (Configuration[TEMP_DELTA_HIGH_OFFSET + 1] << 8));
  TempSets.Tnom = (Configuration[TEMP_NOMINAL_OFFSET] | (Configuration[TEMP_NOMINAL_OFFSET + 1] << 8));
}
//------------------------------------------------------------------------------
void ErrorHandler(uint8_t ErrorType)
{
  Output_DeActivate();
  
  uint8_t LEDfreq = 1;
      
  switch(ErrorType)
  {
    case ERROR_EEPROM:
      LEDfreq = EEPROM_ERROR_LED_FREQ;
    break;
    
    default:
    break;
  }
  
  LED_ONwithFreq(LEDfreq);
  for(;;);
}


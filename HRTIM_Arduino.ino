#include <Arduino.h>
#include <mbed.h>
#include "stm32h7xx.h"
#include <CommandHandler.h>

#define CAPTURE_BUFFER_SIZE 1024
volatile uint16_t capture_buffer[CAPTURE_BUFFER_SIZE];

#define HIST_BUFFER_SIZE 65535
volatile uint32_t accum_buffer[HIST_BUFFER_SIZE];

volatile bool CNT_START_ACQ = false;

volatile uint32_t HRTIM_PREESC = 0x4UL;
volatile uint32_t TIM_DELAY = 100;
volatile uint32_t TIM_CYCLES = 2;

int start_hrtim();
int stop_hrtim();
int get_hrtim();
int change_hrtim_preesc();
int change_cycles_number();

int start_counter();
int stop_counter();
int get_counts();
int change_cnt_delay();

CommandType Commands[] = {
    {"start_hrtim", &start_hrtim, "Start HRTIM"},
    {"stop_hrtim", &stop_hrtim, "Stop HRTIM"},
    {"get_hrtim", &get_hrtim, "Get histogram array"},
    {"hrtim_preesc", &change_hrtim_preesc, "Change HRTIM preescaler (0, 1, 2 or 3)"},
    {"trigger_cycle", &change_cycles_number, "Change the number of cycles"},
    
    {"start_cnt", &start_counter, "Start Counter"},
    {"stop_cnt", &stop_counter, "Stop Counter"},
    {"get_cnt", &get_counts, "Get counts value"},
    {"cnt_preesc", &change_cnt_delay, "Change count delay"}
 
};

CLI command_line(Commands, sizeof(Commands)/sizeof(Commands[0]));

void configure_GPIO_HRTIM(void) {
  //////////////// CHANNEL 1 (PG12, AF2) = HRTIM_EEV5 | Pin 24 Arduino Giga
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOGEN;  // Enable GPIOG clock

  // Set PG12 to Alternate Function mode
  GPIOG->MODER &= ~(3UL << (12 * 2));   // clear mode bits for pin PG12
  GPIOG->MODER |=  (2UL << (12 * 2));   // 2 = Alternate function mode

  // Select AF2 for PG12
  GPIOG->AFR[1] &= ~(0xFUL << ((12 - 8) * 4));  // clear previous AF (AFR[1] because PG12 >= 8)
  GPIOG->AFR[1] |=  (2UL << ((12 - 8) * 4));    // AF2 = HRTIM_EEV5

  // High speed, no pull-up/down
  //GPIOG->OSPEEDR |= (3UL << (12 * 2));          // optional: high speed
  GPIOG->PUPDR   &= ~(3UL << (12 * 2));         // no pull-up/down

  //////////////// CHANNEL 2 (PD5, AF2) = HRTIM_EEV3 | Pin 18 Arduino Giga
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;  // Enable GPIOD clock

  // Set PD5 to Alternate Function mode
  GPIOD->MODER &= ~(3UL << (5 * 2));   // clear mode bits for pin PD5
  GPIOD->MODER |=  (2UL << (5 * 2));   // 2 = Alternate function mode

  // Select AF2 for PD5
  GPIOD->AFR[0] &= ~(0xFUL << (5 * 4)); // clear previous AF for PD5
  GPIOD->AFR[0] |=  (2UL << (5 * 4));   // AF2 = HRTIM_EEV3

  // High speed, no pull-up/down
  //GPIOD->OSPEEDR |= (3UL << (5 * 2));
  GPIOD->PUPDR   &= ~(3UL << (5 * 2));

//  pin_function(PB_4, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF3_HRTIM1));
//  pin_function(PD_5, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF2_HRTIM1));
}

void configure_GPIO_CNT(void) {
  // --- Configure PB3 as AF1 (TIM2_CH2) ---
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
  // --- Set PB3 to Alternate Function mode ---
  GPIOB->MODER &= ~(3UL << (3 * 2));    // clear mode bits for pin 3
  GPIOB->MODER |=  (2UL << (3 * 2));    // 2 = Alternate function mode

  // --- Select AF1 for PB3 ---
  GPIOB->AFR[0] &= ~(0xFUL << (3 * 4)); // clear previous AF for PB3
  GPIOB->AFR[0] |=  (1UL << (3 * 4));   // AF1 = TIM2_CH2 on PB3

  //GPIOB->OSPEEDR |= (3UL << (3 * 2));
  GPIOB->PUPDR   &= ~(3UL << (3 * 2));
}

void configure_GPIO_TRIGGER(void) {
  // --- Configure PA7 as AF2 (TIM3_CH2) ---
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;

  // 2. Set PA7 to Alternate Function mode
  GPIOA->MODER &= ~(3UL << (7 * 2));   // clear mode bits
  GPIOA->MODER |=  (2UL << (7 * 2));   // set mode = 10 (AF)

  // 3. Choose AF2 for PA7 (TIM3_CH2)
  GPIOA->AFR[0] &= ~(0xFUL << (7 * 4));  // clear AF bits
  GPIOA->AFR[0] |=  (2UL   << (7 * 4));  // AF2 = TIM3_CH2 on PA7

  // 4. Configure speed and pull resistors
  //GPIOA->OSPEEDR |=  (3UL << (7 * 2));   // high speed
  GPIOA->PUPDR   &= ~(3UL << (7 * 2));   // no pull-up/down
}

void configure_HRTIM(void) {
  // enable HRTIM1 clock
  RCC -> APB2ENR |= RCC_APB2ENR_HRTIMEN;
  __DSB(); __ISB();
  
  HRTIM1->sTimerxRegs[0].TIMxCR &= ~(0x7UL << 0U); // Configure Timer A prescaler (bits CKPSC[2:0] in TIMxCR)
  HRTIM1->sTimerxRegs[0].TIMxCR |= (HRTIM_PREESC << 0U); // Configure Timer A prescaler (bits CKPSC[2:0] in TIMxCR)
  
  HRTIM1->sTimerxRegs[0].TIMxCR |= (0x1UL << HRTIM_TIMCR_CONT_Pos); // Configure Timer A in Continuous Mode

  /////////// CHANNEL 1 - CONFIGURE CAPTURE CONTROL FOR APD PULSES
  HRTIM1->sCommonRegs.EECR1 &= ~(HRTIM_EECR1_EE5SRC_Msk | 
                                  HRTIM_EECR1_EE5SNS_Msk);
  
  HRTIM1->sCommonRegs.EECR1 |= (0x0UL << HRTIM_EECR1_EE5SRC_Pos) |  // External pin
                               (0x1UL << HRTIM_EECR1_EE5SNS_Pos);   // Rising edge

  HRTIM1->sTimerxRegs[0].CPT1xCR = HRTIM_CPT1CR_EXEV5CPT; // Configure Capture Control on External Event 3 (PD5: pin 18 Arduino Giga)

  /////////// CHANNEL 2 - CONFIGURE TIMER RESET FOR REFERENCE TRIGGER - FROM GENERAL PURPOSE TIMER3
  HRTIM1->sCommonRegs.EECR1 &= ~(HRTIM_EECR1_EE3SRC_Msk | 
                                  HRTIM_EECR1_EE3SNS_Msk);
  
  HRTIM1->sCommonRegs.EECR1 |= (0x2UL << HRTIM_EECR1_EE3SRC_Pos) |  // TIM3_TRGO
                               (0x1UL << HRTIM_EECR1_EE3SNS_Pos);   // Rising edge

  HRTIM1->sTimerxRegs[0].CPT2xCR = HRTIM_CPT2CR_EXEV3CPT; // Configure Capture Control on External Event 3
  HRTIM1->sTimerxRegs[0].RSTxR = HRTIM_RSTR_CMP2; // Configure Reset Control on CAPTURE2 Event

  HRTIM1->sTimerxRegs[0].RSTxR = HRTIM_RSTR_EXTEVNT3; // Configure Reset Control on External Event 3
  
  //// DMA INTERRUPTION
  HRTIM1->sTimerxRegs[0].TIMxDIER |= HRTIM_TIMDIER_CPT1DE; // Configure DMA Interrupt control on Capture1 Control (each APD pulse)
  
}

void configure_DMA(){
  
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; // 1. Enable DMA1 clock

  DMA1->LISR = 0xFFFFFFFF; // Clear flag to initialize DMA
  DMA1->HISR = 0xFFFFFFFF; // Clear flag to initialize DMA

  DMA1_Stream6->CR = 0; // Clear DMA setup
  
  while (DMA1_Stream6->CR & DMA_SxCR_EN) {} // Wait until EN is cleared

  DMA1_Stream6->PAR  = (uint32_t)&HRTIM1->sTimerxRegs[0].CPT1xR;     // peripheral address = CPT1
  DMA1_Stream6->M0AR = (uint32_t)capture_buffer;        // memory buffer
  DMA1_Stream6->NDTR = CAPTURE_BUFFER_SIZE;

  // Configure DMA stream:
  DMA1_Stream6->CR |= DMA_SxCR_MINC;                     // memory increment
  DMA1_Stream6->CR |= (0x1U << DMA_SxCR_MSIZE_Pos);      // memory size = 16-bit
  DMA1_Stream6->CR |= (0x1U << DMA_SxCR_PSIZE_Pos);      // peripheral size = 16-bit
  DMA1_Stream6->CR |= DMA_SxCR_CIRC;                     // circular mode
  DMA1_Stream6->CR |= (0x2U << DMA_SxCR_PL_Pos);         // priority high (2)  
  DMA1_Stream6->CR |= DMA_SxCR_TCIE;                     // transfer complete interrupt enable

  NVIC_DisableIRQ(DMA1_Stream6_IRQn);
  NVIC_ClearPendingIRQ(DMA1_Stream6_IRQn);
  NVIC_SetPriority(DMA1_Stream6_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);

  // Configure DMAMUX1 channel (route request ID 96 = HRTIM1)  
  DMAMUX1_Channel6->CCR = 0; // clear first
  DMAMUX1_Channel6->CCR |= (96U << DMAMUX_CxCR_DMAREQ_ID_Pos); // Request ID 96 = HRTIM1

  /************ ENABLE DMA STREAM ************/
  DMA1_Stream6->CR |= DMA_SxCR_EN;
   
}

extern "C" {
  void DMA1_Stream6_IRQHandler(void){
    if (DMA1->HISR & DMA_HISR_TCIF6) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF6; // clear flag
      
      for (int i = 0; i < CAPTURE_BUFFER_SIZE; i++) {
        accum_buffer[capture_buffer[i]] ++;
      }
    }
  }
}


void configure_COUNTER(void) {
  //////////////// ENABLE TIM2 CLOCK ////////////////
  RCC->APB1LENR |= RCC_APB1LENR_TIM2EN;
  __DSB(); __ISB();
  
  // 1. Select proper TI2x source (internal/external)
  //    -> Internal connection: TI2 maps to its own input by default
  TIM2->TISEL &= ~(TIM_TISEL_TI2SEL_Msk); // TI2SEL = 0000 (default TI2 input pin)

  // 2. Configure channel 2 to detect rising edges on TI2
  TIM2->CCMR1 &= ~(TIM_CCMR1_CC2S_Msk);   // Clear CC2S bits
  TIM2->CCMR1 |=  (0b01 << TIM_CCMR1_CC2S_Pos); // CC2S = 01 (CC2 channel configured as input, mapped to TI2)
  
  // 3. No input filter
  TIM2->CCMR1 &= ~(TIM_CCMR1_IC2F_Msk);   // IC2F = 0000 (no filter)

  // 4. Select rising edge polarity (CC2P=0, CC2NP=0)
  TIM2->CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP); // Rising edge

  // 5. Configure external clock mode 1 (SMS = 111)
  TIM2->SMCR &= ~(TIM_SMCR_SMS_Msk);
  TIM2->SMCR |=  (0b111 << TIM_SMCR_SMS_Pos); // External clock mode 1

  // 6. Select TI2 as input source (TS = 00110)
  TIM2->SMCR &= ~(TIM_SMCR_TS_Msk);
  TIM2->SMCR |=  (0b00110 << TIM_SMCR_TS_Pos); // TS = 00110 (TI2FP2 as trigger)
  
}

void configure_TRIGGER(void) {
    // Enable TIM3 clock
    RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
    __DSB(); __ISB();

    // 1. Select TI2 source (PA7)
    TIM3->TISEL &= ~TIM_TISEL_TI2SEL_Msk;   // TI2SEL = 0000 → GPIO pin

    // 2. Configure channel 2 to detect rising edges on TI2
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC2S_Msk);   // Clear CC2S bits
    TIM3->CCMR1 |=  (0x1UL << TIM_CCMR1_CC2S_Pos); // CC2S = 01 (CC2 channel configured as input, mapped to TI2)

    // 3. No input filter (IC2F = 0000)
    TIM3->CCMR1 &= ~(TIM_CCMR1_IC2F_Msk);   // IC2F = 0000 (no filter)

    // 4. Both edges polarity (CC2P = 1, CC2NP = 1)
    TIM3->CCER &= ~(TIM_CCER_CC2P_Msk | TIM_CCER_CC2NP_Msk); // Both edges
    TIM3->CCER |= (0x1UL << TIM_CCER_CC2P_Pos |
                   0x1UL << TIM_CCER_CC2NP_Pos); // Both edges

    // 5. External Clock Mode 1 (SMS = 111)
    TIM3->SMCR &= ~(TIM_SMCR_SMS_Msk);
    TIM3->SMCR |=  (0b111 << TIM_SMCR_SMS_Pos); // External clock mode 1

    // 6. Select TI2FP2 as trigger source (TS = 110)
    TIM3->SMCR &= ~(TIM_SMCR_TS_Msk);
    TIM3->SMCR |=  (0b00110 << TIM_SMCR_TS_Pos); // TS = 00110 (TI2FP2 as trigger)

    // 7. Auto-reload
    TIM3->ARR = 2*TIM_CYCLES - 1;

    // 8. TRGO = update event (MMS = 010)
    TIM3->CR2 &= ~(TIM_CR2_MMS_Msk);
    TIM3->CR2 |=  (0x2UL << TIM_CR2_MMS_Pos);

    // 9. Enable capture on CH2 (CC2E = 1)
    TIM3->CCER |= (0x1UL << TIM_CCER_CC2E_Pos);
}

int start_hrtim(){

  for (int i = 0; i < HIST_BUFFER_SIZE; i++) { // Resets buffer
      accum_buffer[i] = 0;
  }
  
  for (int i = 0; i < CAPTURE_BUFFER_SIZE; i++) { // Resets buffer
      capture_buffer[i] = 0;
  }

  //Trigger Starting
  TIM3->SR = 0;
  TIM3->CR1 |= TIM_CR1_CEN; // start trigger timer
  
  HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_TACEN;   // Enable Timer A
}

int stop_hrtim(){ 
  
  HRTIM1->sMasterRegs.MCR &= ~HRTIM_MCR_TACEN;   // Disable Timer A

  //Trigger Stop
  TIM3->SR = 0;
  TIM3->CR1 &= ~TIM_CR1_CEN; // start free-running timer
}

int get_hrtim(){ 
  Serial.write((uint8_t*)&accum_buffer, sizeof(accum_buffer));
  Serial.flush();
}

int change_hrtim_preesc(){
  
  stop_hrtim();

  if (atof(command_line.args[1]) == 0) {  HRTIM_PREESC = 0x4UL; }
  if (atof(command_line.args[1]) == 1) {  HRTIM_PREESC = 0x5UL; }
  if (atof(command_line.args[1]) == 2) {  HRTIM_PREESC = 0x6UL; }
  if (atof(command_line.args[1]) == 3) {  HRTIM_PREESC = 0x7UL; }
 
  configure_HRTIM();
  configure_DMA();
}


int start_counter(){
  
  TIM2->SR = 0;
  TIM2->CR1 |= TIM_CR1_CEN; // start free-running timer
}

int stop_counter(){
  
  CNT_START_ACQ = false;
  TIM2->SR = 0;
  TIM2->CR1 &= ~TIM_CR1_CEN; // start free-running timer
}

int get_counts(){
  CNT_START_ACQ = true;
}

int change_cycles_number(){
  stop_hrtim();
  TIM_CYCLES = atof(command_line.args[1]);

  configure_TRIGGER();
  
}

int change_cnt_delay(){
  stop_counter();
  TIM_DELAY = atof(command_line.args[1]);
  start_counter();
}

void setup() {
  Serial.begin(115200);
  command_line.begin(&Serial);

  configure_GPIO_TRIGGER();
  configure_TRIGGER();
  
  configure_GPIO_HRTIM();
  configure_HRTIM();
  configure_DMA();

  configure_GPIO_CNT();
  configure_COUNTER();
}

void loop() {
  command_line.start_processing();

  if (CNT_START_ACQ == true){
    uint32_t counts_1 = TIM2->CNT;
    delay(TIM_DELAY);
    uint32_t counts = TIM2->CNT - counts_1;
  
    Serial.println(counts);
  }
}

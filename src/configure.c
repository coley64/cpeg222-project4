#include "configure.h"
#include "stm32f4xx.h"  // include MCU definitions

#define FREQUENCY 16000000UL
#define BAUDRATE 115200
#define BTN_PIN 13
#define SERVO3_PIN    (6) // Assuming servo motor 3 control pin is connected to GPIOC pin 6
#define ANALOG_PIN 1
#define ANALOG_PORT GPIOA
#define ADC_CHANNEL 1 // ADC Channel for PA1



void configure_clocks(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM5EN | RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_TIM8EN;  // ADD TIM8 HERE!
}

void configure_button(void){
    GPIOC->MODER &= ~(3 << (BTN_PIN*2)); // Input mode
    // Configure EXTI for Button (PC13)
    SYSCFG->EXTICR[3] &= ~(0xF << 4);    // Clear bits for EXTI13
    SYSCFG->EXTICR[3] |= (2 << 4);       // PC13 (0b0010 for GPIOC)
    EXTI->IMR |= (1 << BTN_PIN);         // Enable interrupt
    EXTI->FTSR |= (1 << BTN_PIN);        // Falling edge trigger (button press)
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI15_10_IRQn, 1);
}
void configure_tim2(void){
    // TIM2 for SSD multiplexing (0.5ms interrupt)
    TIM2->PSC = 16 - 1;      // Divide by 16
    TIM2->ARR = 500 - 1;     // 0.5ms at 1MHz
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->SR &= ~TIM_SR_UIF;
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 2);
    TIM2->CR1 |= TIM_CR1_CEN;
}
void configure_tim5(void){
    // TIM5 as free-running timer for pulse measurement (1MHz)
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // enable TIM5 clock
    TIM5->PSC = 16 - 1;                  // 16 MHz / 16 = 1 MHz → 1 µs tick
    TIM5->ARR = 0xFFFFFFFF;              // free-run to max
    TIM5->EGR = TIM_EGR_UG;              // force update
    TIM5->CR1 |= TIM_CR1_CEN;            // start counter
}
    
void PWM_Output_PC6_Init(void) {
    // 1. Enable GPIO Clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    
    // 2. Configure PC6 as Alternate Function
    GPIOC->MODER &= ~GPIO_MODER_MODER6;      // Clear mode bits
    GPIOC->MODER |= GPIO_MODER_MODER6_1;     // Alternate function mode
    
    // 3. Set AF3 for TIM8_CH1 (PC6 uses AF3 for TIM8, not AF2)
    GPIOC->AFR[0] &= ~(0xF << (6 * 4));      // Clear AF bits
    GPIOC->AFR[0] |= (3 << (6 * 4));         // AF3 for TIM8_CH1
    
    // 4. Enable TIM8 Clock (APB2 for advanced timers)
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    
    // 5. Configure TIM8 for PWM
    TIM8->PSC = 16 - 1;                      // 16MHz/16 = 1MHz timer (1µs resolution)
    TIM8->ARR = 20000 - 1;                   // 20ms period (50Hz servo frequency)
    
    // 6. Configure Channel 1 for PWM Mode 1
    TIM8->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;  // PWM mode 1 (110)
    TIM8->CCMR1 |= TIM_CCMR1_OC1PE;          // Output compare preload enable
    
    // 7. Enable Channel 1 output
    TIM8->CCER |= TIM_CCER_CC1E;             // Capture/Compare 1 output enable
    
    // 8. Advanced timer specific configurations
    TIM8->CR1 |= TIM_CR1_ARPE;               // Auto-reload preload enable
    TIM8->BDTR |= TIM_BDTR_MOE;              // MAIN OUTPUT ENABLE - CRITICAL FOR ADVANCED TIMERS!
    
    // 9. Generate update event and start timer
    TIM8->EGR |= TIM_EGR_UG;                 // Generate update
    TIM8->CR1 |= TIM_CR1_CEN;                // Counter enable
    
    // 10. Set initial pulse width (stop position)
    TIM8->CCR1 = 1500;                       // 1500µs = stop
}

void init_adc(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Set PA1 to analog mode
    ANALOG_PORT->MODER &= ~(0x3 << (ANALOG_PIN * 2));
    ANALOG_PORT->MODER |=  (0x3 << (ANALOG_PIN * 2));
    // Enable ADC1 clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    // Select channel in regular sequence (SQR3)
    ADC1->SQR3 = (ADC_CHANNEL & 0x1F);
    // Sample time for channel 1 (SMPR2 holds channels 0..9)
    ADC1->SMPR2 = ADC_SMPR2_SMP1_0 | ADC_SMPR2_SMP1_1;
    // Turn on ADC
    ADC1->CR2 = ADC_CR2_ADON;
}
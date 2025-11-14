// ****************************************************************
// * TEAM50: N. West and H. Collins
// * CPEG222 Project 3A Demo, 10/6/25
// 
/*
Change the PWR jumper on the Nucleo board from U5V (USB powered) to E5V (external power) ✅
so the microcontroller will be powered by the battery pack as well.✅
Use the PMOD LS1 connected to PMOD_C (J10) to read the IR sensors on PC0, PC1, PC2, and PC3.
Create a “bare metal” CMSIS Platform IO program to allow your robot to follow a line.  
The robot should start on a User Button (PC13) press. ✅
 It should stop when it reaches the end line.
Once started, the robot should display the sensor readings on the four SSDs where a “0” indicates no line and a “1” indicates a line.  Leading zeros should not be suppressed, i.e., 0110 should display all values. 
It is due 11/14, with the team code uploaded to Canvas and an in-person demo in Evans 127.
// ****************************************************************/

#include "stm32f4xx.h"
#include "SSD_Array.h"
#include "configure.h"
#include <stdbool.h>
#include <stdio.h>
#include "UART.h"
#include <math.h>
#define IR_SENSOR_PORT GPIOC


#define FREQUENCY 16000000UL
#define UART_PORT GPIOA
#define BTN_PIN 13
#define BTN_PORT GPIOC

#define ADC_CHANNEL 1 // ADC Channel for PA1
#define ADC_SAMPLES 16 // Number of samples for averaging
#define MIN_PULSE_WIDTH 1000   // microseconds
#define MAX_PULSE_WIDTH 2000   // microseconds  
#define SERVO3_PIN    (6) // Assuming servo motor 3 control pin is connected to GPIOC pin 6
#define SERVO3_PORT   (GPIOC)

uint8_t ssd_digits[4];
// global variables
volatile int digitSelect = 0;
volatile bool move = false;
volatile int line_detected = 0;
volatile int currentIR;


void servo_pwm_set(int pwm_width_left, int pwm_width_right);
// --------------------- Interrupt Handlers ---------------------
void SysTick_Handler(void) {
  if (USART2->CR1 & USART_CR1_UE) { // if USART2 enabled
    if (line_detected <= 2) {
        move = false; // stop moving
        servo_pwm_set(1500, 1500); // neutral position
    }
  }
}

// update SSD
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
    uint8_t b3 = (currentIR >> 3) & 1; // leftmost
    uint8_t b2 = (currentIR >> 2) & 1;
    uint8_t b1 = (currentIR >> 1) & 1;
    uint8_t b0 = (currentIR >> 0) & 1;

    int display_val = b3*1000 + b2*100 + b1*10 + b0;

    // then in TIM2_IRQHandler:
    SSD_update(digitSelect, display_val, 0);

        digitSelect = (digitSelect + 1) % 4;

        // Clear timer interrupt
        TIM2->SR &= ~TIM_SR_UIF;
    }
}


// button press handler
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & (1 << BTN_PIN)) {
        if (!move) {
            servo_pwm_set(1576, 1420); // left = CCW, right = CW
            move = true; // start moving
        } else {
            move = false; // stop moving
            servo_pwm_set(1500, 1500);
        }
        EXTI->PR |= (1 << BTN_PIN); // Clear pending bit
    }
    line_detected = 0; // reset line detected count
}

void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_CC2IF) {
        TIM3->SR &= ~TIM_SR_CC2IF; // Clear interrupt flag
    }
}
void TIM4_IRQHandler(void) {
    if (TIM4->SR & TIM_SR_UIF) {
        TIM4->SR &= ~TIM_SR_UIF; // Clear interrupt flag
    }
}
void configure_tim4(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 16000 - 1;     // 16MHz/16000 = 1kHz (1ms ticks)
    TIM4->ARR = 250 - 1;       // 250ms period
    TIM4->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM4_IRQn);
    TIM4->CR1 |= TIM_CR1_CEN;
}

void servo_pwm_set(int pwm_width_left, int pwm_width_right) {
    TIM8->CCR3 = pwm_width_left; // Set pulse width for servo motor 1 (CH1)
    TIM8->CCR1 = pwm_width_right; // Set pulse width for servo motor 3 (CH3)
}

// --------------------- Main Program ---------------------

int main(void) {
    configure_clocks();
    configure_button();
    SSD_init();
    configure_tim2();
    configure_tim5();  
    configure_tim4();
    SysTick_Config(SystemCoreClock/4); // 0.5 second SysTick
    PWM_Output_PC6_Init();
    // Enable GPIOC clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // Set PC0–PC3 as input
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // enable GPIOC clock
    IR_SENSOR_PORT->MODER &= ~((3<<0) | (3<<2) | (3<<4) | (3<<6));   // inputs


    // Enable pull-up for PC0–PC3
    // GPIOC->PUPDR &= ~((0x3<<0) | (0x3<<2) | (0x3<<4) | (0x3<<6)); // clear
    // GPIOC->PUPDR |=  ((0x1<<0) | (0x1<<2) | (0x1<<4) | (0x1<<6)); // set pull-up



    // ------------------ Main loop ------------------
// i wanna implement edge detection to increment on falling edge for IR sensor
static uint8_t prevIR = 0x0F; // previous reading, initialized to no line detected
while(1) {
    currentIR = IR_SENSOR_PORT->IDR & 0x0F;
    if (line_detected >= 2) {
        move = false; // stop moving
        servo_pwm_set(1500, 1500); // neutral position
    }
    if (prevIR != 0 && currentIR == 0) { // falling edge detected
        line_detected += 1;
    }
    prevIR = currentIR;
    }

  return 0;
}

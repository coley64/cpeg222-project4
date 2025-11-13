// ****************************************************************
// * TEAM50: N. West and H. Collins
// * CPEG222 Project 3A Demo, 10/6/25
// 
/*
Use the UD CPEG222 F25 shield to control the Parallax continuous rotation servo motor with feedback.

Connect the potentiometer wiper (center pin) to an unused GPIO pin configured as an analog input. ✅
As you turn the potentiometer, the wiper voltage will vary from 0 to 3.3 V. ✅

Create a “bare metal” CMSIS Platform IO program to display the speed in rotations per minute (RPM) in tenths on the SSDs. 

If the RPM is less than 100.0, the first SSD should be blank instead of 1.
User Button (PC13) with an interrupt is used to toggle between CW/stop/CCW/stop…. ✅
Adjust the PWM width (i.e., speed) with respect to the analog voltage input by varying the PWM signal. ✅
[CW 1480 to 1280 us, CCW 1520 to 1720 us]✅
Use the encoder signal to determine the speed of the motor in rotations per minute (RPM). 
Calculate the total rotation angle within a .25-second window and then convert that to RPM.
Use the hardware SysTick timer for 1-second interrupts to send the ADC value (0 – 4095),✅
the servo direction, PWM width, and the rpms to the serial monitor (USART2) at 115200 baud. 

Display the RPMs on the SSDs. 
A general-purpose timer (like TIM2) can generate a 0.5-ms interrupt to update the SSD array every 2 milliseconds.✅ 
Use another timer to measure pulse widths.
*/
// ****************************************************************

#include "stm32f4xx.h"
#include "SSD_Array.h"
#include "configure.h"
#include <stdbool.h>
#include <stdio.h>
#include "UART.h"
#include <math.h>

#define FREQUENCY 16000000UL
#define UART_PORT GPIOA
#define BTN_PIN 13
#define BTN_PORT GPIOC
volatile uint32_t last_rising = 0;
volatile uint32_t last_falling = 0;
volatile uint32_t pulse_width = 0;
volatile uint32_t pulse_period = 0;
volatile uint32_t last_period_capture = 0;
volatile int waiting_for_falling = 0;
volatile float rpm = 0.0f;
volatile uint32_t pulse_count = 0;
volatile uint32_t last_rpm_calc_time = 0;

#define ADC_CHANNEL 1 // ADC Channel for PA1
#define ADC_SAMPLES 16 // Number of samples for averaging
#define MIN_PULSE_WIDTH 1000   // microseconds
#define MAX_PULSE_WIDTH 2000   // microseconds  
#define ENCODER_PPR 12         // Pulses Per Revolution (adjust for your encoder)

#define SERVO3_PIN    (6) // Assuming servo motor 3 control pin is connected to GPIOC pin 6
#define SERVO3_PORT   (GPIOC)

float calculate_rpm(void);
// global variables
volatile int digitSelect = 0;
volatile int ADC_value = 0; // 0-4095
volatile int state = 0; // 0 = CW, 1 = stop, 2 = CCW
volatile int PWM_width = 1500; // initial pulse width in microseconds
volatile float rpms = 0.0f; // rotations per minute
volatile bool send_uart_flag = false;
volatile uint32_t encoder_pulse_count = 0;
volatile float current_rpm = 0.0f;
volatile uint32_t last_rpm_time = 0;
int min_pulse_width = 32; // minimum encoder pulse width in microseconds
int max_pulse_width = 1076; // maximum encoder pulse width in microseconds

uint16_t read_adc_average(void);

void PWM_Input_PC7_Init(void) {
	// Enable GPIOC and TIM3 clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	// Set PC7 to alternate function (AF2 for TIM3_CH2)
	GPIOC->MODER &= ~(0x3 << (7 * 2));
	GPIOC->MODER |=  (0x2 << (7 * 2)); // Alternate function
	GPIOC->AFR[0] &= ~(0xF << (7 * 4));
	GPIOC->AFR[0] |=  (0x2 << (7 * 4)); // AF2 = TIM3

	// Configure TIM3 for simple input capture on CH2 (PC7)
	TIM3->PSC = 15; // 16 MHz / 16 = 1 MHz timer clock (1us resolution)
	TIM3->ARR = 0xFFFF;
	TIM3->CCMR1 &= ~(0x3 << 8); // CC2S bits for CH2
	TIM3->CCMR1 |= (0x01 << 8); // CC2: IC2 mapped to TI2
	TIM3->CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP); // Rising edge
	TIM3->CCER |= TIM_CCER_CC2E; // Enable capture on CH2
	TIM3->DIER |= TIM_DIER_CC2IE; // Enable capture/compare 2 interrupt
	TIM3->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM3_IRQn); // Enable TIM3 interrupt in NVIC
}

// --------------------- Interrupt Handlers ---------------------
// updates uart
void SysTick_Handler(void) {
  if (USART2->CR1 & USART_CR1_UE) { // if USART2 enabled
    GPIOA->ODR ^= (1 << 0);
    send_uart_flag = true;   // tell main loop to send
  }
}

// update SSD
void TIM2_IRQHandler(void) {
    if(TIM2->SR & TIM_SR_UIF) {
        SSD_update(digitSelect, rpm, 0); // display rpms in tenths
        digitSelect = (digitSelect + 1) % 4; // cycle through digits
        TIM2->SR &= ~TIM_SR_UIF; // Clear interrupt flag
    }
}

// button press handler
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & (1 << BTN_PIN)) {
        state = (state + 1) % 4;  // Cycle through 0,1,2,3 (CW/stop/CCW/stop)
        EXTI->PR |= (1 << BTN_PIN); // Clear pending bit
    }
}
// TIM3 input capture interrupt handler for PC7 (CH2)
volatile float currentAngle = 0.0f;
volatile float lastAngle = 0.0f;
volatile float cumulativeAngle = 0.0f;
volatile float lastCumulativeAngle = 0.0f;

void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_CC2IF) {
        TIM3->SR &= ~TIM_SR_CC2IF;
        
        if (!waiting_for_falling) {
            last_rising = TIM3->CCR2;
            TIM3->CCER |= TIM_CCER_CC2P;
            waiting_for_falling = 1;
        } else {
            last_falling = TIM3->CCR2;
            if (last_falling >= last_rising) {
                pulse_width = last_falling - last_rising;
                // Convert pulse width to angle (32-1068µs → 0-360°)
                float raw_angle = (pulse_width - 32.0f) * 360.0f / (1068.0f - 32.0f);
                // Wrap angle to 0-360° range
                currentAngle = fmodf(raw_angle, 360.0f);
                if (currentAngle < 0) currentAngle += 360.0f;
            } else {
                // Handle timer overflow
                pulse_width = (0xFFFF - last_rising) + last_falling;
                if (pulse_width >= 32 && pulse_width <= 1068) {
                    float raw_angle = (pulse_width - 32.0f) * 360.0f / (1068.0f - 32.0f);
                    currentAngle = fmodf(raw_angle, 360.0f);
                    if (currentAngle < 0) currentAngle += 360.0f;
                }
            }
            TIM3->CCER &= ~TIM_CCER_CC2P;
            waiting_for_falling = 0;
        }
    }
}
void TIM4_IRQHandler(void) {
    if (TIM4->SR & TIM_SR_UIF) {
        float dt = 0.25f; // 250ms time window
        
        // Calculate angle change (handle wrap-around)
        float dTheta = currentAngle - lastAngle;
        if (dTheta > 180.0f) dTheta -= 360.0f;
        if (dTheta < -180.0f) dTheta += 360.0f;
        
        // Update cumulative angle (tracks multiple rotations)
        cumulativeAngle += dTheta;
        
        // Prevent overflow
        if (cumulativeAngle > 1000000.0f || cumulativeAngle < -1000000.0f) {
            lastCumulativeAngle -= cumulativeAngle;
            cumulativeAngle = 0.0f;
        }
        
        lastAngle = currentAngle;
        
        // Calculate angular velocity and RPM
        float dCumulative = cumulativeAngle - lastCumulativeAngle;
        lastCumulativeAngle = cumulativeAngle;
        float angularVelocity = dCumulative / dt; // degrees per second
        rpm = fabs((angularVelocity / 360.0f) * 60.0f); // Convert to RPM
    
        TIM4->SR &= ~TIM_SR_UIF;
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

void servo_pwm_set(int pwm_width) {
    // Ensure pulse width stays within valid range (avoid damage to servo)
    if (pwm_width < 1000) pwm_width = 1000;
    if (pwm_width > 2000) pwm_width = 2000;
    TIM8->CCR1 = pwm_width;
}


uint16_t read_adc_average(void) {
    uint32_t total = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        // Start conversion (software)
        ADC1->CR2 |= ADC_CR2_SWSTART;

        // Wait until conversion complete
        while (!(ADC1->SR & ADC_SR_EOC)) { /* spin */ }

        // Read data register (clears EOC)
        total += ADC1->DR;
    }
    uint16_t average = (uint16_t)(total / ADC_SAMPLES);
    return average;
}

// --------------------- Main Program ---------------------

int main(void) {
    configure_clocks();
    configure_uart();
    configure_button();
    SSD_init();
    configure_tim2();
    configure_tim5();  
    configure_tim4();
    SysTick_Config(SystemCoreClock/1); // 1 second interrupts
    init_adc();
    PWM_Output_PC6_Init();
    PWM_Input_PC7_Init();

    last_rpm_calc_time = TIM5->CNT;
    rpm = 0.0f;
    // ------------------ Startup message ------------------
    uart_send_string("CPEG222 Demo Program!\r\nRunning at 115200 baud...\r\n");
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Set PA5 as general-purpose output (MODER5 = 01)
    GPIOA->MODER &= ~(0x3 << (0 * 2));  // clear mode bits
    GPIOA->MODER |=  (0x1 << (0 * 2));  // set as output
    // ------------------ Main loop ------------------

    while(1) {
        uint16_t adc_val = read_adc_average();
        if (send_uart_flag) {
            send_uart_flag = false;  // clear flag
            ADC_value = adc_val; // update global for SSD display
        char msg[64];
        sprintf(msg, "ADC: %d\t", adc_val);
        uart_send_string(msg);
        sprintf(msg, "PWM: %d\t State: %d\tRPM: %.1f\n", PWM_width, state, rpm);
        uart_send_string(msg);
        }
        // Adjust PWM based on ADC value
        if (state == 0) { // CW
            PWM_width = 1480 - ((adc_val * 200) / 4095); // Map 0-4095 to 1480-1280
        } else if (state == 2) { // CCW
            PWM_width = 1520 + ((adc_val * 200) / 4095); // Map 0-4095 to 1520-1720
        } else { // stop
            PWM_width = 1500;
        }
        servo_pwm_set(PWM_width);
    }
  return 0;
}

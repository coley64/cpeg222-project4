// // ****************************************************************
// // * TEAM50: T. LUM and R. MARTIN
// // * CPEG222 cont_servo_main, 10/6/25
// // * NucleoF466RE CMSIS STM32F4xx example
// // * Move the continuous rotation servo using PWM on PC6
// // * Read the servo position using input capture on PC7
// // * Send angle and pulse width data over UART2
// // ****************************************************************

// #include "stm32f4xx.h"
// #include "UART2.h"
// #include <stdio.h>
// #include <stdlib.h>
// #include <SSD_Array.h>


// #define FREQUENCY   16000000UL // 16 MHz
// #define ENCODER_PIN   (7) // Assuming servo motor encoder is connected to GPIOC pin 7
// #define ENCODER_PORT  (GPIOC)
// #define SERVO3_PIN    (6) // Assuming servo motor 3 control pin is connected to GPIOC pin 6
// #define SERVO3_PORT   (GPIOC)	

// int offsetDeg = 235; // this will depend on your setup
// int min_pulse_width = 32; // minimum encoder pulse width in microseconds
// int max_pulse_width = 1076; // maximum encoder pulse width in microseconds
// int angle = 0;
// int cw_pulse_width = 1400; //1450 for slower movement;
// int ccw_pulse_width = 1600; //1550 for slower movement;
// volatile int current_angle = 0;
// volatile uint32_t last_rising = 0;
// volatile uint32_t last_falling = 0;
// volatile uint32_t pulse_width = 0;
// volatile uint8_t waiting_for_falling = 0;
// volatile uint8_t digitSelect = 0;

// void PWM_Output_PC6_Init(void) {
// 	// Enable GPIOC and TIM8 clocks
// 	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
// 	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

// 	// Set PC6 to alternate function (AF3 for TIM8_CH1)
// 	GPIOC->MODER &= ~(0x3 << (SERVO3_PIN * 2));
// 	GPIOC->MODER |=  (0x2 << (SERVO3_PIN * 2)); // Alternate function
// 	GPIOC->AFR[0] &= ~(0xF << (SERVO3_PIN * 4));
// 	GPIOC->AFR[0] |=  (0x3 << (SERVO3_PIN * 4)); // AF3 = TIM8

//     // Configure output type and speed for PC6: push-pull, high speed
//     GPIOC->OTYPER &= ~(1 << SERVO3_PIN); // push-pull
//     GPIOC->OSPEEDR &= ~(0x3 << (SERVO3_PIN * 2));
//     GPIOC->OSPEEDR |=  (0x2 << (SERVO3_PIN * 2)); // high speed

// 	// Configure TIM8 for PWM output on CH1 (PC6)
// 	TIM8->PSC = 15; // 16 MHz / 16 = 1 MHz timer clock (1us resolution)
// 	TIM8->ARR = 19999; // Period for 50 Hz
// 	TIM8->CCR1 = 1500; // Duty cycle (1.475 ms pulse width)
// 	TIM8->CCMR1 &= ~(TIM_CCMR1_OC1M);
// 	TIM8->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // PWM mode 1
// 	TIM8->CCMR1 |= TIM_CCMR1_OC1PE; // Preload enable
// 	TIM8->CCER |= TIM_CCER_CC1E; // Enable CH1 output (PC6)

// 	// For advanced-control timers (TIM8) the main output must be enabled
// 	// using the BDTR MOE bit or outputs will stay inactive even if CC1E is set.
// 	TIM8->BDTR |= TIM_BDTR_MOE;
// 	TIM8->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
// 	TIM8->EGR = TIM_EGR_UG; // Generate update event
// 	TIM8->CR1 |= TIM_CR1_CEN; // Enable timer
// }

// void PWM_Input_PC7_Init(void) {
// 	// Enable GPIOC and TIM3 clocks
// 	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
// 	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
// 	// Set PC7 to alternate function (AF2 for TIM3_CH2)
// 	GPIOC->MODER &= ~(0x3 << (7 * 2));
// 	GPIOC->MODER |=  (0x2 << (7 * 2)); // Alternate function
// 	GPIOC->AFR[0] &= ~(0xF << (7 * 4));
// 	GPIOC->AFR[0] |=  (0x2 << (7 * 4)); // AF2 = TIM3

// 	// Configure TIM3 for simple input capture on CH2 (PC7)
// 	TIM3->PSC = 15; // 16 MHz / 16 = 1 MHz timer clock (1us resolution)
// 	TIM3->ARR = 0xFFFF;
// 	TIM3->CCMR1 &= ~(0x3 << 8); // CC2S bits for CH2
// 	TIM3->CCMR1 |= (0x01 << 8); // CC2: IC2 mapped to TI2
// 	TIM3->CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP); // Rising edge
// 	TIM3->CCER |= TIM_CCER_CC2E; // Enable capture on CH2
// 	TIM3->DIER |= TIM_DIER_CC2IE; // Enable capture/compare 2 interrupt
// 	TIM3->CR1 |= TIM_CR1_CEN;
// 	NVIC_EnableIRQ(TIM3_IRQn); // Enable TIM3 interrupt in NVIC
// }

// // TIM3 input capture interrupt handler for PC7 (CH2)
// void TIM3_IRQHandler(void) {
// 	if (TIM3->SR & TIM_SR_CC2IF) { // Check if CC2IF is set
// 		TIM3->SR &= ~TIM_SR_CC2IF; // Clear interrupt flag
// 		if (!waiting_for_falling) {
// 			last_rising = TIM3->CCR2;
// 			// Switch to capture falling edge
// 			TIM3->CCER |= TIM_CCER_CC2P; // Set to falling edge
// 			waiting_for_falling = 1;
// 		} else {
// 			last_falling = TIM3->CCR2;
// 			if (last_falling >= last_rising){
// 				if (last_falling - last_rising < 1100) 
// 				pulse_width = last_falling - last_rising;
// 				current_angle = (pulse_width - min_pulse_width)*360/(max_pulse_width - min_pulse_width);
// 			} else
// 				pulse_width = (0xFFFF - last_rising) + last_falling + 1;
// 			// Switch back to capture rising edge
// 			TIM3->CCER &= ~TIM_CCER_CC2P; // Set to rising edge
// 			waiting_for_falling = 0;
// 		}
// 	}
// }

// void servo_angle_set(int angle) {
// 	while(abs(current_angle - angle) > 3) {  // Within ±2° is "close enough"
// 	//while(current_angle!=angle) {
// 		// uart2_send_string("set angle: ");
// 		// uart2_send_int32(angle-offsetDeg);
// 		// uart2_send_string("\tpulsewidth: ");
// 		// uart2_send_int32(pulse_width);
// 		// uart2_send_string("\tangle(deg): ");
// 		// uart2_send_int32(current_angle-offsetDeg);
// 		// uart2_send_string("\r\n");
// 		if(current_angle > angle){
// 			TIM8->CCR1 = ccw_pulse_width; // Duty cycle (1.475 ms pulse width)
// 		} 
// 		else {
// 			TIM8->CCR1 = cw_pulse_width; // Duty cycle (1.475 ms pulse width)
// 		}
// 	}
// 	TIM8->CCR1 = 1500;
// 	for (volatile int i = 0; i < 500000; ++i); // Simple delay 300000
// }

// void TIM2_IRQHandler(void){ // TIM2 interrupt handler for SSD refresh
//   if(TIM2->SR & TIM_SR_UIF){ // Check if the update interrupt flag is set
//     SSD_update(digitSelect, angle + 60, 0); // Update the SSD with the current distance showing hundredths
//     digitSelect = (digitSelect + 1) % 4; // Cycle through digitSelect values 0 to 3
//     TIM2->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag
//   }
// }

// int main(void) {
// 	UART2_Init();
// 	SSD_init(); // Initialize SSD
// 	    // Configure TIM2 for 0.5ms interrupt (assuming 16MHz HSI clock)
//     RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock to refresh SSD
//     TIM2->PSC = 15; // Prescaler: (16MHz/16 = 1MHz, 1usec period)
//     TIM2->ARR = 499; // Auto-reload: 500us (0.5ms period)
//     TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
//     TIM2->SR &= ~TIM_SR_UIF; // Clear any pending interrupt
//     NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt in NVIC
//     NVIC_SetPriority(TIM2_IRQn, 2); // Set priority for TIM2
//     TIM2->CR1 = TIM_CR1_CEN; // Enable TIM2

// 	PWM_Output_PC6_Init();
// 	PWM_Input_PC7_Init();
// 	uart2_send_string("CPEG222 Continuous Servo Demo Program!\r\n");
// 	uart2_send_string("Setting angle to 0 degrees.\r\n");
// 	servo_angle_set(offsetDeg);
// 	TIM8->CCR1 = 1500; // Duty cycle (1.5 ms pulse width to stop movement)
// 	for (volatile int i = 0; i < 1000000UL; ++i); // Simple delay
// 	while (1) {
// 		for(angle = -60; angle <= 60; angle += 10) {
// 			servo_angle_set(offsetDeg+angle);
// 			uart2_send_string("angle(deg): ");
// 			uart2_send_int32(angle);
// 			uart2_send_string("\t  servo pulsewidth(us): ");
// 			uart2_send_int32(pulse_width);
// 			uart2_send_string("\r\n");
// 		}
// 		angle = 60;
// 		TIM8->CCR1 = 1500; // Duty cycle (1.5 ms pulse width to stop movement)
// 		for (volatile int i = 0; i < 1000000; ++i); // Simple delay
// 		for(angle = 60; angle >= -60; angle -= 10) {
// 			servo_angle_set(offsetDeg+angle);
// 			uart2_send_string("angle(deg): ");
// 			uart2_send_int32(angle);
// 			uart2_send_string("\t  servo pulsewidth(us): ");
// 			uart2_send_int32(pulse_width);
// 			uart2_send_string("\r\n");
// 		}
// 		angle = -60;
// 		TIM8->CCR1 = 1500; // Duty cycle (1.5 ms pulse width to stop movement)
// 		for (volatile int i = 0; i < 1000000; ++i); // Simple delay	
// 	}
// }
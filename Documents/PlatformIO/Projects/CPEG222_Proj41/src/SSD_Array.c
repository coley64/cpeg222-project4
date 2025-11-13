// ****************************************************************
// * TEAM50: N. West and H. Collins
// * Helper file for SSD_Array.c
// * CPEG222, 9/19/25
// ****************************************************************
#include "SSD_Array.h"
#include "stm32f4xx.h"

#define SSDAA_PIN (10) // Assuming SSD segment AA is connected to GPIOB pin 10
#define SSDAA_PORT (GPIOB)
#define SSDAB_PIN (9) // Assuming SSD segment AB is connected to GPIOA pin 9
#define SSDAB_PORT (GPIOA)
#define SSDAC_PIN (13) // Assuming SSD segment AC is connected to GPIOB pin 13
#define SSDAC_PORT (GPIOB)
#define SSDAD_PIN (14) // Assuming SSD segment AD is connected to GPIOB pin 14
#define SSDAD_PORT (GPIOB)
#define SSDAE_PIN (4) // Assuming SSD segment AE is connected to GPIOB pin 4
#define SSDAE_PORT (GPIOB)
#define SSDAF_PIN (1) // Assuming SSD segment AF is connected to GPIOB pin 1
#define SSDAF_PORT (GPIOB)
#define SSDAG_PIN (10) // Assuming SSD segment AG is connected to GPIOA pin 10
#define SSDAG_PORT (GPIOA)
#define SSDDP_PIN (5) // Assuming SSD segment DP is connected to GPIOB pin 5
#define SSDDP_PORT (GPIOB)
#define DIGIT1_PIN                                                             \
  (15) // Assuming digit1 common pin is connected to GPIOB pin 15
#define DIGIT1_PORT (GPIOB)
#define DIGIT2_PIN (8) // Assuming digit2 common pin is connected to GPIOA pin 8
#define DIGIT2_PORT (GPIOA)
#define DIGIT3_PIN (2) // Assuming digit3 common pin is connected to GPIOB pin 2
#define DIGIT3_PORT (GPIOB)
#define DIGIT4_PIN (4) // Assuming digit4 common pin is connected to GPIOC pin 4
#define DIGIT4_PORT (GPIOC)

const unsigned char digitSegments[] = {
    0b0111111, // 0
    0b0000110, // 1
    0b1011011, // 2
    0b1001111, // 3
    0b1100110, // 4
    0b1101101, // 5
    0b1111101, // 6
    0b0000111, // 7
    0b1111111, // 8
    0b1101111, // 9
    0b0000000  // Blank
};

void SSD_init(void) { // Initialize GPIO pins for SSD
  // Enable GPIOA, GPIOB, and GPIOC clocks
  RCC->AHB1ENR |=
      RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
  // Configure segment pins as outputs
  SSDAA_PORT->MODER &= ~(0x3 << (SSDAA_PIN * 2));   // Clear mode bits
  SSDAB_PORT->MODER &= ~(0x3 << (SSDAB_PIN * 2));   // Clear mode bits
  SSDAC_PORT->MODER &= ~(0x3 << (SSDAC_PIN * 2));   // Clear mode bits
  SSDAD_PORT->MODER &= ~(0x3 << (SSDAD_PIN * 2));   // Clear mode bits
  SSDAE_PORT->MODER &= ~(0x3 << (SSDAE_PIN * 2));   // Clear mode bits
  SSDAF_PORT->MODER &= ~(0x3 << (SSDAF_PIN * 2));   // Clear mode bits
  SSDAG_PORT->MODER &= ~(0x3 << (SSDAG_PIN * 2));   // Clear mode bits
  SSDDP_PORT->MODER &= ~(0x3 << (SSDDP_PIN * 2));   // Clear mode bits
  DIGIT1_PORT->MODER &= ~(0x3 << (DIGIT1_PIN * 2)); // Clear mode bits
  DIGIT2_PORT->MODER &= ~(0x3 << (DIGIT2_PIN * 2)); // Clear mode bits
  DIGIT3_PORT->MODER &= ~(0x3 << (DIGIT3_PIN * 2)); // Clear mode bits
  DIGIT4_PORT->MODER &= ~(0x3 << (DIGIT4_PIN * 2)); // Clear mode bits
  SSDAA_PORT->MODER |=
      (0x1 << (SSDAA_PIN * 2)); // Set as General purpose output mode
  SSDAB_PORT->MODER |=
      (0x1 << (SSDAB_PIN * 2)); // Set as General purpose output mode
  SSDAC_PORT->MODER |=
      (0x1 << (SSDAC_PIN * 2)); // Set as General purpose output mode
  SSDAD_PORT->MODER |=
      (0x1 << (SSDAD_PIN * 2)); // Set as General purpose output mode
  SSDAE_PORT->MODER |=
      (0x1 << (SSDAE_PIN * 2)); // Set as General purpose output mode
  SSDAF_PORT->MODER |=
      (0x1 << (SSDAF_PIN * 2)); // Set as General purpose output mode
  SSDAG_PORT->MODER |=
      (0x1 << (SSDAG_PIN * 2)); // Set as General purpose output mode
  SSDDP_PORT->MODER |=
      (0x1 << (SSDDP_PIN * 2)); // Set as General purpose output mode
  // Configure digit common pins as outputs
  DIGIT1_PORT->MODER |=
      (0x1 << (DIGIT1_PIN * 2)); // Set as General purpose output mode
  DIGIT2_PORT->MODER |=
      (0x1 << (DIGIT2_PIN * 2)); // Set as General purpose output mode
  DIGIT3_PORT->MODER |=
      (0x1 << (DIGIT3_PIN * 2)); // Set as General purpose output mode
  DIGIT4_PORT->MODER |=
      (0x1 << (DIGIT4_PIN * 2)); // Set as General purpose output mode
}

void SSD_update(int digitSelect, int value, int decimalPoint) {
  int firstDigit = (value / 1000) % 10;
  int secondDigit = (value / 100) % 10;
  int thirdDigit = (value / 10) % 10;
  int fourthDigit = value % 10;

  DIGIT1_PORT->ODR |= (1 << DIGIT1_PIN); // Turn off common pin for first digit
  DIGIT2_PORT->ODR |= (1 << DIGIT2_PIN); // Turn off common pin for second digit
  DIGIT3_PORT->ODR |= (1 << DIGIT3_PIN); // Turn off common pin for third digit
  DIGIT4_PORT->ODR |= (1 << DIGIT4_PIN); // Turn off common pin for fourth digit
  switch (digitSelect) {
  case 0: { // Update first digit
    unsigned char segments = digitSegments[firstDigit];
    if (firstDigit == 0 && decimalPoint != 1) {
      segments = digitSegments[10]; // Blank leading zero
    }
    SSDAA_PORT->ODR = (SSDAA_PORT->ODR & ~(1 << SSDAA_PIN)) |
                      ((segments & 0x01) ? (1 << SSDAA_PIN) : 0);
    SSDAB_PORT->ODR = (SSDAB_PORT->ODR & ~(1 << SSDAB_PIN)) |
                      ((segments & 0x02) ? (1 << SSDAB_PIN) : 0);
    SSDAC_PORT->ODR = (SSDAC_PORT->ODR & ~(1 << SSDAC_PIN)) |
                      ((segments & 0x04) ? (1 << SSDAC_PIN) : 0);
    SSDAD_PORT->ODR = (SSDAD_PORT->ODR & ~(1 << SSDAD_PIN)) |
                      ((segments & 0x08) ? (1 << SSDAD_PIN) : 0);
    SSDAE_PORT->ODR = (SSDAE_PORT->ODR & ~(1 << SSDAE_PIN)) |
                      ((segments & 0x10) ? (1 << SSDAE_PIN) : 0);
    SSDAF_PORT->ODR = (SSDAF_PORT->ODR & ~(1 << SSDAF_PIN)) |
                      ((segments & 0x20) ? (1 << SSDAF_PIN) : 0);
    SSDAG_PORT->ODR = (SSDAG_PORT->ODR & ~(1 << SSDAG_PIN)) |
                      ((segments & 0x40) ? (1 << SSDAG_PIN) : 0);
    if (decimalPoint == 1) {
      SSDDP_PORT->ODR |= (1 << SSDDP_PIN); // Turn on decimal point
    } else {
      SSDDP_PORT->ODR &= ~(1 << SSDDP_PIN); // Turn off decimal point
    }
    DIGIT1_PORT->ODR &=
        ~(1 << DIGIT1_PIN); // Turn on common pin for first digit
    break;
  }
  case 1: { // Update second digit
    unsigned char segments1 = digitSegments[secondDigit];
    if (firstDigit == 0 && secondDigit == 0 && (decimalPoint != 2 || decimalPoint != 1)) {
      segments1 = digitSegments[10]; // Blank leading zero
    }
    SSDAA_PORT->ODR = (SSDAA_PORT->ODR & ~(1 << SSDAA_PIN)) |
                      ((segments1 & 0x01) ? (1 << SSDAA_PIN) : 0);
    SSDAB_PORT->ODR = (SSDAB_PORT->ODR & ~(1 << SSDAB_PIN)) |
                      ((segments1 & 0x02) ? (1 << SSDAB_PIN) : 0);
    SSDAC_PORT->ODR = (SSDAC_PORT->ODR & ~(1 << SSDAC_PIN)) |
                      ((segments1 & 0x04) ? (1 << SSDAC_PIN) : 0);
    SSDAD_PORT->ODR = (SSDAD_PORT->ODR & ~(1 << SSDAD_PIN)) |
                      ((segments1 & 0x08) ? (1 << SSDAD_PIN) : 0);
    SSDAE_PORT->ODR = (SSDAE_PORT->ODR & ~(1 << SSDAE_PIN)) |
                      ((segments1 & 0x10) ? (1 << SSDAE_PIN) : 0);
    SSDAF_PORT->ODR = (SSDAF_PORT->ODR & ~(1 << SSDAF_PIN)) |
                      ((segments1 & 0x20) ? (1 << SSDAF_PIN) : 0);
    SSDAG_PORT->ODR = (SSDAG_PORT->ODR & ~(1 << SSDAG_PIN)) |
                      ((segments1 & 0x40) ? (1 << SSDAG_PIN) : 0);
    if (decimalPoint == 2) {
      SSDDP_PORT->ODR |= (1 << SSDDP_PIN); // Turn on decimal point
    } else {
      SSDDP_PORT->ODR &= ~(1 << SSDDP_PIN); // Turn off decimal point
    }
    DIGIT2_PORT->ODR &=
        ~(1 << DIGIT2_PIN); // Turn on common pin for second digit
    break;
  }
  case 2: { // Update third digit
    unsigned char segments2 = digitSegments[thirdDigit];
    if (firstDigit == 0 && secondDigit == 0 && (thirdDigit == 0 && (decimalPoint == 0 || decimalPoint == 4))) {
      segments2 = digitSegments[10]; // Blank leading zero
    }
    SSDAA_PORT->ODR = (SSDAA_PORT->ODR & ~(1 << SSDAA_PIN)) |
                      ((segments2 & 0x01) ? (1 << SSDAA_PIN) : 0);
    SSDAB_PORT->ODR = (SSDAB_PORT->ODR & ~(1 << SSDAB_PIN)) |
                      ((segments2 & 0x02) ? (1 << SSDAB_PIN) : 0);
    SSDAC_PORT->ODR = (SSDAC_PORT->ODR & ~(1 << SSDAC_PIN)) |
                      ((segments2 & 0x04) ? (1 << SSDAC_PIN) : 0);
    SSDAD_PORT->ODR = (SSDAD_PORT->ODR & ~(1 << SSDAD_PIN)) |
                      ((segments2 & 0x08) ? (1 << SSDAD_PIN) : 0);
    SSDAE_PORT->ODR = (SSDAE_PORT->ODR & ~(1 << SSDAE_PIN)) |
                      ((segments2 & 0x10) ? (1 << SSDAE_PIN) : 0);
    SSDAF_PORT->ODR = (SSDAF_PORT->ODR & ~(1 << SSDAF_PIN)) |
                      ((segments2 & 0x20) ? (1 << SSDAF_PIN) : 0);
    SSDAG_PORT->ODR = (SSDAG_PORT->ODR & ~(1 << SSDAG_PIN)) |
                      ((segments2 & 0x40) ? (1 << SSDAG_PIN) : 0);
    if (decimalPoint == 3) {
      SSDDP_PORT->ODR |= (1 << SSDDP_PIN); // Turn on decimal point
    } else {
      SSDDP_PORT->ODR &= ~(1 << SSDDP_PIN); // Turn off decimal point
    }
    DIGIT3_PORT->ODR &=
        ~(1 << DIGIT3_PIN); // Turn on common pin for third digit
    break;
  }
  case 3: { // Update fourth digit
    unsigned char segments3 = digitSegments[fourthDigit];
    SSDAA_PORT->ODR = (SSDAA_PORT->ODR & ~(1 << SSDAA_PIN)) |
                      ((segments3 & 0x01) ? (1 << SSDAA_PIN) : 0);
    SSDAB_PORT->ODR = (SSDAB_PORT->ODR & ~(1 << SSDAB_PIN)) |
                      ((segments3 & 0x02) ? (1 << SSDAB_PIN) : 0);
    SSDAC_PORT->ODR = (SSDAC_PORT->ODR & ~(1 << SSDAC_PIN)) |
                      ((segments3 & 0x04) ? (1 << SSDAC_PIN) : 0);
    SSDAD_PORT->ODR = (SSDAD_PORT->ODR & ~(1 << SSDAD_PIN)) |
                      ((segments3 & 0x08) ? (1 << SSDAD_PIN) : 0);
    SSDAE_PORT->ODR = (SSDAE_PORT->ODR & ~(1 << SSDAE_PIN)) |
                      ((segments3 & 0x10) ? (1 << SSDAE_PIN) : 0);
    SSDAF_PORT->ODR = (SSDAF_PORT->ODR & ~(1 << SSDAF_PIN)) |
                      ((segments3 & 0x20) ? (1 << SSDAF_PIN) : 0);
    SSDAG_PORT->ODR = (SSDAG_PORT->ODR & ~(1 << SSDAG_PIN)) |
                      ((segments3 & 0x40) ? (1 << SSDAG_PIN) : 0);
    if (decimalPoint == 4) {
      SSDDP_PORT->ODR |= (1 << SSDDP_PIN); // Turn on decimal point
    } else {
      SSDDP_PORT->ODR &= ~(1 << SSDDP_PIN); // Turn off decimal point
    }
    DIGIT4_PORT->ODR &=
        ~(1 << DIGIT4_PIN); // Turn  on common pin for fourth digit
    break;
  }
  default:
    break;
  }
}

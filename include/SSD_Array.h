#ifndef SSD_ARRAY_H
#define SSD_ARRAY_H

void SSD_init(void); // intitalizes the 12 GPIO pins as outputs
void SSD_update(int digitSelect, int value, int decimalPoint);
/* updates the digitSelect digit (0-3) of the display with the
appropriate number from the value integer. It auto increments the digitSelect variable so the next digit will be
refreshed when called again. The decimal point corresponding to the decimalPoint (1-4) is turned on. If
decimalPoint = 0, no dps are illuminated.
*/

#endif // SSD_ARRAY_H
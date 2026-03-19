/*
 * tm1637.h
 *
 *  Created on: Mar 18, 2026
 *      Author: Karukosa
 */

#ifndef INC_TM1637_H_
#define INC_TM1637_H_

void tm1637Init(void);
void tm1637DisplayDecimal(int v, int displaySeparator);
void tm1637SetBrightness(char brightness);

#endif /* INC_TM1637_H_ */

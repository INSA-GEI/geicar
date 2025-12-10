/*
 * debug.h
 *
 *  Created on: Dec 20, 2024
 *      Author: dimercur
 */

#ifndef DEBUG_H_
#define DEBUG_H_

void DEBUG_Init(void);
int __io_putchar(int ch);
int __io_getchar(void);

void DEBUG_PrintITM(uint8_t port, char *str);

#endif /* DEBUG_H_ */

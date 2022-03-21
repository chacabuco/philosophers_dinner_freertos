/*
 * philosopher_interface.h
 *
 *  Created on: Mar 21, 2022
 *      Author: gonzalo
 */

#ifndef INC_PHILOSOPHER_INTERFACE_H_
#define INC_PHILOSOPHER_INTERFACE_H_
#include <stdbool.h>
//#include "cmsis_os.h"
#define N 5
#define THINKING 0
#define HUNGRY 1
#define EATING 2
#define LEFT (argument+N-1) % N
#define RIGHT (argument+1) % N
extern bool arr[5];
// oid take_forks(uint8_t argument, uint8_t* state);
void put_down_forks(uint8_t i);

#endif /* INC_PHILOSOPHER_INTERFACE_H_ */

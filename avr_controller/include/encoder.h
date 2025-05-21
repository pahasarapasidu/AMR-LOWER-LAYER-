/*
 * encoder.h
 *
 * Created: 5/6/2025 4:44:05 AM
 *  Author: Endeavor360
 */ 


#ifndef ENCODERS_H_
#define ENCODERS_H_


#include <stdint.h>

/* ---------- API ---------- */
void     encoder_init(void);

int32_t  encoder_get_left(void);
int32_t  encoder_get_right(void);

void     encoder_reset_left(void);
void     encoder_reset_right(void);


#endif /* ENCODERS_H_ */
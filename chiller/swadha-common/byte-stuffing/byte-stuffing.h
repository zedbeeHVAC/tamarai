/******************************************************************************
Copyright (c) 2018 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/

#pragma once

void
byte_stuff_init(void);

uint8_t
byte_stuff(uint8_t *src, uint8_t *dest, uint8_t size);

uint8_t
byte_unstuff(uint8_t data, uint8_t *dest);
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define TEAL_BALL_G_MIN 65
#define TEAL_BALL_B_MIN 65
#define TEAL_BALL_GR_MIN_DIFF 15
#define TEAL_BALL_BR_MIN_DIFF 15
#define TEAL_BALL_DOMINANCE_PERCENT 115

bool is_teal_ball_rgb(uint8_t r, uint8_t g, uint8_t b);

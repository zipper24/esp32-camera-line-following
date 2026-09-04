#include "teal_ball_color.h"

bool is_teal_ball_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    return g >= TEAL_BALL_G_MIN &&
           b >= TEAL_BALL_B_MIN &&
           (int)g - (int)r >= TEAL_BALL_GR_MIN_DIFF &&
           (int)b - (int)r >= TEAL_BALL_BR_MIN_DIFF &&
           (int)g * 100 >= (int)r * TEAL_BALL_DOMINANCE_PERCENT &&
           (int)b * 100 >= (int)r * TEAL_BALL_DOMINANCE_PERCENT;
}

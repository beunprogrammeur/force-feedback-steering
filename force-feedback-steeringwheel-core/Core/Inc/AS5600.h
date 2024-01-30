

#include "main.h"


/*
 * Sets up the peripherals so we can read any angle after this.
 * */
void AS5600_init(I2C_HandleTypeDef *hi2c);

/*
 * sets the current angle as the home angle.
 */
void AS5600_set_home();
/*
 * gets the current angle (possible beyond the range of the encoder)
 */
int32_t AS5600_get_angle();

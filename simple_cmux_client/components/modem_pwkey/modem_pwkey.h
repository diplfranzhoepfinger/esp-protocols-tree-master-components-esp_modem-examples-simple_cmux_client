/*
 * modem_pwkey.h
 *
 *  Created on: 30.04.2022
 *      Author: franz
 */

#ifndef COMPONENTS_MODEM_PWKEY_MODEM_PWKEY_H_
#define COMPONENTS_MODEM_PWKEY_MODEM_PWKEY_H_


#ifdef __cplusplus
extern "C" {
#endif

void init_modem_pwkey(void);
void power_up_modem_pwkey(void);
void power_down_modem_pwkey(void);
void power_reset_modem_pwkey(void);

#ifdef __cplusplus
}
#endif


#endif /* COMPONENTS_MODEM_PWKEY_MODEM_PWKEY_H_ */

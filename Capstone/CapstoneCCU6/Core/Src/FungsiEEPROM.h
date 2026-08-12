/*
 * FungsiEEPROOM.h
 *
 *  Created on: Jul 30, 2026
 *      Author: Tegar
 */

#ifndef SRC_FUNGSIEEPROM_H_
#define SRC_FUNGSIEEPROM_H_

void Load_Settings_From_Flash(void);
bool Save_Settings_To_Flash(void);

void Load_Default_Value(void);
void Salin_Data_Ke_Register(void);
void Ambil_Data_Dari_Holding_Register(void);

#endif /* SRC_FUNGSIEEPROM_H_ */

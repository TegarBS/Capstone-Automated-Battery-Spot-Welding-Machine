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

void loadDefault(void);
void salinDataKeRegister(void);
void ambilDataDariRegister(void);

#endif /* SRC_FUNGSIEEPROM_H_ */

/*
 * FungsiEEPROOM.h
 *
 *  Created on: Jul 30, 2026
 *      Author: Tegar
 */

#ifndef SRC_FUNGSIEEPROM_H_
#define SRC_FUNGSIEEPROM_H_

void Load_Settings_From_Flash();
bool Save_Settings_To_Flash();

void loadDefault();
void salinDataKeRegister();
void ambilDataDariRegister();

#endif /* SRC_FUNGSIEEPROM_H_ */

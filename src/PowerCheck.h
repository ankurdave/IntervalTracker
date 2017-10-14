#ifndef POWER_CHECK_H
#define POWER_CHECK_H

#include "Particle.h"

/**
 * Simple class to monitor for has power (USB or VIN), has a battery, and is charging
 *
 * Just instantiate one of these as a global variable and call setup() out of setup()
 * to initialize it. Then use the getHasPower(), getHasBattery() and getIsCharging()
 * methods as desired.
 *
 * From https://community.particle.io/t/when-is-electron-powered-by-battery/20702/18
 */
class PowerCheck {
public:

	PowerCheck();
	virtual ~PowerCheck();

	/**
	 * You must call this out of setup() to initialize the interrupt handler!
	 */
	void setup();

	/**
	 * Returns true if the Electron has power, either a USB host (computer), USB charger, or VIN power.
	 *
	 * Not interrupt or timer safe; call only from the main loop as it uses I2C to query the PMIC.
	 */
	bool getHasPower();

	/**
	 * Returns true if the Electron has a battery.
	 */
	bool getHasBattery();

	/**
	 * Returns true if the Electron is currently charging (red light on)
	 *
	 * Not interrupt or timer safe; call only from the main loop as it uses I2C to query the PMIC.
	 */
	bool getIsCharging();

private:
	void interruptHandler();

	PMIC pmic;
	volatile bool hasBattery = true;
	volatile unsigned long lastChange = 0;
};

#endif

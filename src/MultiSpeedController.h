/*
 * MultiSpeedController.h
 *
 *  Created on: Feb 18, 2017
 *      Author: Kids
 */

#ifndef SRC_MULTISPEEDCONTROLLER_H_
#define SRC_MULTISPEEDCONTROLLER_H_

#include <SpeedController.h>
#include <vector>

/**
 * A speed controller that controls an array of speed controllers with
 * the same value
 *
 * @author Thomas Clark
 */
class MultiSpeedController: public SpeedController {
	std::vector<SpeedController*> m_speedControllers;
	double m_speed;

public:
	/**
	 * Construct a MultiSpeedController
	 *
	 * @param speedControllers A vector of speed controllers to control with
	 *        the same value.  These are owned by this class.
	 */
	MultiSpeedController(std::vector<SpeedController*> const& speedControllers) :
			m_speedControllers(speedControllers), m_speed(0.0f) {
	}

	MultiSpeedController() :
			m_speedControllers(0), m_speed(0.0f) {
	}

	~MultiSpeedController() {
		for (unsigned int i = 0; i < m_speedControllers.size(); i++)
			delete m_speedControllers[i];
	}

	/**
	 * @return The speed of all of the motors
	 */
	double Get() {
		return m_speed;
	}

	/**
	 * @param speed The speed of all of the motors
	 */
	void Set(double speed) {
		for (unsigned int i = 0; i < m_speedControllers.size(); i++)
			m_speedControllers[i]->Set(speed);

		m_speed = speed;
	}

	void PIDWrite(double output) {
		this->Set(output);
	}

	/**
	 * Disable all of the motors
	 */
	void Disable() {
		for (unsigned int i = 0; i < m_speedControllers.size(); i++)
			m_speedControllers[i]->Disable();
	}
};

#endif /* SRC_MULTISPEEDCONTROLLER_H_ */

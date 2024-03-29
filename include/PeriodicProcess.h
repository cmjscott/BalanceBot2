#ifndef __PERIODICPROCESS_H__
#define __PERIODICPROCESS_H__

#include "Clock.h"


class PeriodicProcess
{
public:

	virtual ~PeriodicProcess(){}

	void enable();
	void disable();

	void setTimestep(float timestep){m_timestep = timestep;}

	bool update();

	void forceProcess();

	bool isEnabled() const {return m_enabled;}
	bool isUpdated();

	float getTimestep() const {return m_timestep;}

protected:

	virtual void process(float timestep) = 0;

	virtual void onProcessEnabling(){}
	virtual void onProcessDisabling(){}

private:

	bool  m_enabled;
	bool 	m_isUpdated;
	float m_timestep;

	Clock m_clock;
};

#endif // __PERIODICPROCESS_H__

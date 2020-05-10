#include "PeriodicProcess.h"


bool PeriodicProcess::update()
{
	if (m_enabled && m_clock.getElapsedTime() > m_timestep)
	{
		float timestep = m_clock.restart();
		process(timestep);
		m_isUpdated = true;

		return true;
	}
	return false;
}

void PeriodicProcess::enable()
{
	if (!m_enabled)
	{
		m_enabled = true;
		m_clock.restart();
		onProcessEnabling();
	}
}

void PeriodicProcess::disable()
{
	if (m_enabled)
	{
		m_enabled = false;
		onProcessDisabling();
	}
}

bool PeriodicProcess::isUpdated()
{
	if (m_isUpdated)
	{
		m_isUpdated = false;
		return true;
	}
	return false;
}

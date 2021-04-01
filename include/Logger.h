#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <Arduino.h>
#include <LinkedList.h>

////////////////////////////////////////////////////////////////////////////////////////////////////

class Logger
{
public:
	void config(Stream& ser, const String delim = ",", bool logTime = true);


	void appendFloat(float *var, const String& headerName);
	void appendDouble(double *var, const String& headerName);
	void appendInt(int *var, const String& headerName);
	void appendHeader(const String& headerName);

	void log();
	void logHeader();

private:


	Stream* m_ser;
	String m_header;
	String m_delim;

	LinkedList<float*> m_floatList;
	LinkedList<double*> m_doubleList;
	LinkedList<int*> m_intList;

	bool m_logTime;
};

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // __LOGGER_H__

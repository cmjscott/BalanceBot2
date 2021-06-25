#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <Arduino.h>
//#include <LinkedList.h>
//#include <ArduinoSTL.h>
#include <vector>

////////////////////////////////////////////////////////////////////////////////////////////////////

class Logger
{
public:

	class LoggerData
	{
	public:
		LoggerData();

		String m_header;
		float* m_floatData = nullptr;
		double* m_doubleData = nullptr;
		int* m_intData = nullptr;

		void appendFloat(float *var, const String& headerName);
		void appendDouble(double *var, const String& headerName);
		void appendInt(int *var, const String& headerName);

		void logData(Stream* serStream);
		void logHeader(Stream* serStream);

		~LoggerData();

	private:
		friend class Logger;
	};


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

	std::vector<LoggerData> m_data;

	bool m_logTime;
};

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // __LOGGER_H__

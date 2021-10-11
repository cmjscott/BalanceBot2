#include "Logger.h"

Logger::LoggerData::LoggerData(){}

Logger::LoggerData::~LoggerData(){}

void Logger::LoggerData::append(float *var, const String& headerName)
{
	m_floatData = var;
	m_header = headerName;
}

void Logger::LoggerData::append(double *var, const String& headerName)
{
	m_doubleData = var;
	m_header = headerName;
}

void Logger::LoggerData::append(int *var, const String& headerName)
{
	m_intData = var;
	m_header = headerName;
}

void Logger::LoggerData::logData(Stream* serStream)
{
	if(m_floatData)
	{
		serStream->print(*m_floatData);
	}else if(m_doubleData)
	{
		serStream->print(*m_doubleData);
	}else if(m_intData)
	{
		serStream->print(*m_intData);
	}
}

void Logger::LoggerData::logHeader(Stream* serStream)
{
	serStream->print(m_header);
}






void Logger::config(Stream& ser, const String delim, bool logTime)
{
	m_ser = &ser;
	m_delim = delim;
	m_logTime = logTime;

}

void Logger::append(float *var, const String& headerName)
{
	LoggerData tempData = LoggerData();
	tempData.append(var, headerName);
	m_data.push_back(tempData);
	//appendHeader(headerName);
}

void Logger::append(double *var, const String& headerName)
{
	LoggerData tempData = LoggerData();
	tempData.append(var, headerName);
	m_data.push_back(tempData);
	//appendHeader(headerName);
}

void Logger::append(int *var, const String& headerName)
{
	LoggerData tempData = LoggerData();
	tempData.append(var, headerName);
	m_data.push_back(tempData);
	//appendHeader(headerName);
}

/*
void Logger::appendHeader(const String& headerName)
{
	if(m_header.length()){
		m_header.append(m_delim);
		m_header.append(headerName);
	} else {
		m_header.append(headerName);
	}
}
*/

void Logger::log()
{
	
	if(m_logTime){
		m_ser->print(millis());

		for(auto &data : m_data){
			m_ser->print(m_delim);
			data.logData(m_ser);
		}
	} else {
		for(auto &data : m_data){
			data.logData(m_ser);
			m_ser->print(m_delim);
		}
	}
	m_ser->print("\n");
}

void Logger::logHeader()
{
	if(m_logTime){
		m_ser->print("t");

		for(auto &data : m_data){
			m_ser->print(m_delim);
			data.logHeader(m_ser);
		}
	} else {
		for(auto &data : m_data){
			data.logHeader(m_ser);
			m_ser->print(m_delim);
		}
	}

	m_ser->print("\n");
}


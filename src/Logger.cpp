#include "Logger.h"


void Logger::config(Stream& ser, const String delim, bool logTime)
{
	m_ser = &ser;
	m_delim = delim;
	m_logTime = logTime;

	if(logTime){
		appendHeader("t");
	}


}

void Logger::appendFloat(float *var, const String& headerName)
{
	m_floatList.add(var);
	appendHeader(headerName);
}

void Logger::appendDouble(double *var, const String& headerName)
{
	m_doubleList.add(var);
	appendHeader(headerName);
}

void Logger::appendInt(int *var, const String& headerName)
{
	m_intList.add(var);
	appendHeader(headerName);
}

void Logger::appendHeader(const String& headerName)
{
	if(m_header.length()){
		m_header.append(m_delim);
		m_header.append(headerName);
	} else {
		m_header.append(headerName);
	}
}

void Logger::log()
{
	
	if(m_logTime){
		m_ser->print(millis());
	}
	

	
	if(m_floatList.size()){
		for(int i=0; i<m_floatList.size(); i++){
			m_ser->print(m_delim);
			m_ser->print(*m_floatList.get(i));
		}
		
	}

	if(m_doubleList.size()){
		for(int i=0; i<m_doubleList.size(); i++){
			m_ser->print(m_delim);
			m_ser->print(*m_doubleList.get(i));
		}
		
	}

	if(m_intList.size()){
		for(int i=0; i<m_intList.size(); i++){
			m_ser->print(m_delim);
			m_ser->print(*m_intList.get(i));
		}
		
	}


	m_ser->print("\n");
	
}

void Logger::logHeader()
{
	if(m_header.length()){
		m_ser->println(m_header);
	}	
}


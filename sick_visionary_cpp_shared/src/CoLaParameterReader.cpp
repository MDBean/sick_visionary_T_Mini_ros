//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: October 2018
// 
// SICK AG, Waldkirch
// email: TechSupport0905@sick.de

#include "CoLaParameterReader.h"
#include "VisionaryEndian.h"

namespace visionary 
{

CoLaParameterReader::CoLaParameterReader(CoLaCommand command)
  : m_command(command)
{
  m_currentPosition = command.getParameterOffset();
}

CoLaParameterReader::~CoLaParameterReader()
{
}

void CoLaParameterReader::rewind()
{
  m_currentPosition = m_command.getParameterOffset();
}

const int8_t CoLaParameterReader::readSInt()
{
  const int8_t value = static_cast<const int8_t>(m_command.getBuffer()[m_currentPosition]);
  m_currentPosition += 1;
  return value;
}

const uint8_t CoLaParameterReader::readUSInt()
{
  const uint8_t value = m_command.getBuffer()[m_currentPosition];
  m_currentPosition += 1;
  return value;
}

const int16_t CoLaParameterReader::readInt()
{
  const int16_t value = readUnalignBigEndian<int16_t>(&m_command.getBuffer()[m_currentPosition]);
  m_currentPosition += 2;
  return value;
}

const uint16_t CoLaParameterReader::readUInt()
{
  const uint16_t value = readUnalignBigEndian<uint16_t>(&m_command.getBuffer()[m_currentPosition]);
  m_currentPosition += 2;
  return value;
}

const int32_t CoLaParameterReader::readDInt()
{
  const int32_t value = readUnalignBigEndian<int32_t>(&m_command.getBuffer()[m_currentPosition]);
  m_currentPosition += 4;
  return value;
}

const uint32_t CoLaParameterReader::readUDInt()
{
  const uint32_t value = readUnalignBigEndian<uint32_t>(&m_command.getBuffer()[m_currentPosition]);
  m_currentPosition += 4;
  return value;
}

const float CoLaParameterReader::readReal()
{
  const float value = readUnalignBigEndian<float>(&m_command.getBuffer()[m_currentPosition]);
  m_currentPosition += 4;
  return value;
}

const double CoLaParameterReader::readLReal()
{
  const double value = readUnalignBigEndian<double>(&m_command.getBuffer()[m_currentPosition]);
  m_currentPosition += 8;
  return value;
}

const bool CoLaParameterReader::readBool()
{
  return readUSInt() == 1;
}

std::string CoLaParameterReader::readFlexString()
{
  std::string str("");
  uint16_t len = readUInt();
  if (len)
  {
    str = std::string(reinterpret_cast<const char*>(&m_command.getBuffer()[m_currentPosition]), len);
  }
  m_currentPosition += str.length();
  return str;
}

}

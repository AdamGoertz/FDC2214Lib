#include "MyFDC2214.h"
#include <Wire.h>

MyFDC2214::MyFDC2214() :
  config_reg(INT_OSC_CONFIG),
  mux_config_reg(CONTINUOUS_CONFIG),
  i2c_addr(FDC2214_I2C_ADDR_0),
  channel_count(1),
  reference_count(0xFFFF),// Maximum Reference Count
  settle_count(0x0064),// Settle count 100 
  offset(0x0000),// No Offset
  i_drive(0xF800)// Maximum Drive Current
{
}

MyFDC2214& MyFDC2214::withI2cAddress(uint8_t addr_pin)
{
   i2c_addr = (addr_pin) ? FDC2214_I2C_ADDR_1 : FDC2214_I2C_ADDR_0; 
}

MyFDC2214& MyFDC2214::withContinuousConversion(uint8_t channel)
{
  config_reg &= 0x3FFF; // Clear channel bits [15:14]
  config_reg |= ((uint16_t)channel) << 14;
  mux_config_reg &= 0x7FFF; // Set MUX_CONFIG.AUTOSCAN_EN = 0 [15]
  return *this;
}

MyFDC2214& MyFDC2214::withAutoScan(uint8_t channel_count)
{
  if (channel_count >= 2 && channel_count <= 4)
  {
    mux_config_reg |= 0x8000;
    mux_config_reg |= ((uint16_t)(channel_count - 2)) << 13;
  }

  return *this;
}

MyFDC2214& MyFDC2214::withSettleCount(uint16_t _settle_count)
{
    settle_count = _settle_count;
    return *this;
}


MyFDC2214& MyFDC2214::withReferenceCount(uint16_t _reference_count)
{
  if (_reference_count >  0x0100) 
  {
    reference_count = _reference_count;
  }

  return *this;
}

MyFDC2214& MyFDC2214::withDriveCurrent(uint8_t drive_current)
{
  if (drive_current < 32)
  {
    i_drive = ((uint16_t)drive_current) << 11;
  }
}

MyFDC2214& MyFDC2214::withDeglitchValue(uint8_t deglitch)
{
  mux_config_reg &= 0xFFF8; // Clear deglitch bits [2:0]
  mux_config_reg |= (deglitch | 0x7);
  return *this;
}

MyFDC2214& MyFDC2214::withInternalOscillator()
{
  config_reg &= 0xFDFF;

  return *this;
}

MyFDC2214& MyFDC2214::withExternalOscillator()
{
  config_reg |= 0x0200;

  return *this;
}

MyFDC2214& MyFDC2214::withOffset(uint16_t _offset)
{
    offset = _offset;
    return *this;
}

void MyFDC2214::begin() 
{
  Wire.begin();
  
  _I2Cwrite16(FDC2214_RCOUNT_CH0, reference_count);
  _I2Cwrite16(FDC2214_RCOUNT_CH1, reference_count);
  _I2Cwrite16(FDC2214_RCOUNT_CH2, reference_count);
  _I2Cwrite16(FDC2214_RCOUNT_CH3, reference_count);

  _I2Cwrite16(FDC2214_SETTLECOUNT_CH0, settle_count);
  _I2Cwrite16(FDC2214_SETTLECOUNT_CH1, settle_count);
  _I2Cwrite16(FDC2214_SETTLECOUNT_CH2, settle_count);
  _I2Cwrite16(FDC2214_SETTLECOUNT_CH3, settle_count);

  _I2Cwrite16(FDC2214_OFFSET_CH0, offset);
  _I2Cwrite16(FDC2214_OFFSET_CH1, offset);
  _I2Cwrite16(FDC2214_OFFSET_CH2, offset);
  _I2Cwrite16(FDC2214_OFFSET_CH3, offset);

  _I2Cwrite16(FDC2214_DRIVE_CH0, i_drive);
  _I2Cwrite16(FDC2214_DRIVE_CH1, i_drive);
  _I2Cwrite16(FDC2214_DRIVE_CH2, i_drive);
  _I2Cwrite16(FDC2214_DRIVE_CH3, i_drive);

  _I2Cwrite16(FDC2214_CONFIG, config_reg);
  _I2Cwrite16(FDC2214_MUX_CONFIG, mux_config_reg);
}

uint16_t MyFDC2214::_I2Cread16(uint8_t address)
{
  uint16_t retVal = 0;

  Wire.beginTransmission(i2c_addr);
  Wire.write(address);
  Wire.endTransmission(false);

  Wire.requestFrom(i2c_addr, (uint8_t) 2);
  if (Wire.available() == 2) {
    retVal |= (uint16_t) Wire.read() << 8;
    retVal |= (uint16_t) Wire.read();
  }

  return retVal;
}

void MyFDC2214::_I2Cwrite16(uint8_t address, uint16_t data)
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(address);
  Wire.write(data >> 8);
  Wire.write(data);
  Wire.endTransmission();
}

uint32_t MyFDC2214::getSensorReading(uint8_t channel)
{
  uint8_t addressLSB;
  uint8_t addressMSB;
  uint32_t result = 0;

  switch(channel)
  {
    case 0:
      addressLSB = FDC2214_DATA_CH0_LSB;
      addressMSB = FDC2214_DATA_CH0_MSB;
      break;
    case 1:
      addressLSB = FDC2214_DATA_CH1_LSB;
      addressMSB = FDC2214_DATA_CH1_MSB;
      break;
    case 2:
      addressLSB = FDC2214_DATA_CH2_LSB;
      addressMSB = FDC2214_DATA_CH2_MSB;
      break;
    case 3:
      addressLSB = FDC2214_DATA_CH3_LSB;
      addressMSB = FDC2214_DATA_CH3_MSB;
      break;
    default:
      return 0;
  }
  
  result |= (_I2Cread16(addressMSB) & 0x0FFF) << 16; // Mask out status bits, keep only lower 12 bits.
  result |= _I2Cread16(addressLSB);

  return result;
  
}

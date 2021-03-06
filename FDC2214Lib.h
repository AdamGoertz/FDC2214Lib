#include <Wire.h>

#define INT_OSC_CONFIG       0x1C81
#define EXT_OSC_CONFIG       0x1E81
#define CONTINUOUS_CONFIG    0x020F
#define RESET_DEV_DEFAULT    0x0000
#define CLK_DIV_DEFAULT 0x0000

// Address is 0x2A (default) or 0x2B (if ADDR is high)
#define FDC2214_I2C_ADDR_0   0x2A
#define FDC2214_I2C_ADDR_1   0x2B

// Parameter Definitions
#define DEGLITCH_1MHZ 0x1
#define DEGLITCH_3MHZ 0x4
#define DEGLITCH_10MHZ 0x5
#define DEGLITCH_33MHZ 0x7

#define GAIN_1 0x0
#define GAIN_4 0x1
#define GAIN_8 0x2
#define GAIN_16 0x3

#define DIF_FREQ_LOW 0x0001
#define DIF_FREQ_HIGH 0x0002


//bitmasks
#define FDC2214_CH0_UNREADCONV 0x0008         //denotes unread CH0 reading in STATUS register
#define FDC2214_CH1_UNREADCONV 0x0004         //denotes unread CH1 reading in STATUS register
#define FDC2214_CH2_UNREADCONV 0x0002         //denotes unread CH2 reading in STATUS register
#define FDC2214_CH3_UNREADCONV 0x0001         //denotes unread CH3 reading in STATUS register

//registers
#define FDC2214_DEVICE_ID               0x7F
#define FDC2214_MUX_CONFIG              0x1B
#define FDC2214_CONFIG                  0x1A
#define FDC2214_RESET_DEV               0x1C
#define FDC2214_RCOUNT_CH0              0x08
#define FDC2214_RCOUNT_CH1              0x09
#define FDC2214_RCOUNT_CH2              0x0A
#define FDC2214_RCOUNT_CH3              0x0B
#define FDC2214_OFFSET_CH0              0x0C
#define FDC2214_OFFSET_CH1              0x0D
#define FDC2214_OFFSET_CH2              0x0E
#define FDC2214_OFFSET_CH3              0x0F
#define FDC2214_SETTLECOUNT_CH0         0x10
#define FDC2214_SETTLECOUNT_CH1         0x11
#define FDC2214_SETTLECOUNT_CH2         0x12
#define FDC2214_SETTLECOUNT_CH3         0x13
#define FDC2214_CLOCK_DIVIDERS_CH0      0x14
#define FDC2214_CLOCK_DIVIDERS_CH1      0x15
#define FDC2214_CLOCK_DIVIDERS_CH2      0x16
#define FDC2214_CLOCK_DIVIDERS_CH3      0x17
#define FDC2214_STATUS                  0x18
#define FDC2214_DATA_CH0_MSB            0x00
#define FDC2214_DATA_CH0_LSB            0x01
#define FDC2214_DATA_CH1_MSB            0x02
#define FDC2214_DATA_CH1_LSB            0x03
#define FDC2214_DATA_CH2_MSB            0x04
#define FDC2214_DATA_CH2_LSB            0x05
#define FDC2214_DATA_CH3_MSB            0x06
#define FDC2214_DATA_CH3_LSB            0x07
#define FDC2214_DRIVE_CH0               0x1E
#define FDC2214_DRIVE_CH1               0x1F
#define FDC2214_DRIVE_CH2               0x20
#define FDC2214_DRIVE_CH3               0x21

class FDC2214
{
  uint8_t channel_count;
  uint8_t i2c_addr;
  uint16_t i_drive;
  uint16_t reference_count;
  uint16_t settle_count;
  uint16_t config_reg;
  uint16_t mux_config_reg;
  uint16_t clock_dividers_reg;
  
public:
  uint16_t _I2Cread16(uint8_t address);
  void _I2Cwrite16(uint8_t address, uint16_t data);
  
  FDC2214();

  FDC2214& withI2cAddress(uint8_t addr_pin);
  FDC2214& withContinuousConversion(uint8_t channel);
  FDC2214& withAutoScan(uint8_t channel_count);

  FDC2214& withSettleCount(uint16_t _settle_count);
  FDC2214& withReferenceCount(uint16_t _reference_count);
  FDC2214& withDriveCurrent(uint8_t drive_current);

  FDC2214& withInternalOscillator();
  FDC2214& withExternalOscillator();

  FDC2214& withDeglitchValue(uint8_t deglitch);

  FDC2214& withSingleEndedMode();
  FDC2214& withDifferentialMode(uint16_t freq_range);

  void begin();

  uint32_t getSensorReading(uint8_t channel);
};


#ifndef _LTR390_H
#define _LTR390_H

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#define UV_SENSITIVITY  1400

#define WFAC            1
/*
*  For device under tinted window with coated-ink of flat transmission rate at 400-600nm wavelength,  
*  window factor  is to  compensate light  loss due to the  lower  transmission rate from the coated-ink. 
*     a. WFAC = 1 for NO window / clear window glass. 
*     b. WFAC >1 device under tinted window glass. Calibrate under white LED. 
*/

#define LTR390_ADDRESS 0x53         ///< I2C address
#define LTR390_MAIN_CTRL 0x00       ///< Main control register
#define LTR390_MEAS_RATE 0x04       ///< Resolution and data rate
#define LTR390_GAIN 0x05            ///< ALS and UVS gain range
#define LTR390_PART_ID 0x06         ///< Part id/revision register
#define LTR390_MAIN_STATUS 0x07     ///< Main status register
#define LTR390_ALSDATA_LSB 0x0D     ///< ALS data lowest byte
#define LTR390_ALSDATA_MSB 0x0E     ///< ALS data middle byte
#define LTR390_ALSDATA_HSB 0x0F     ///< ALS data highest byte
#define LTR390_UVSDATA_LSB 0x10     ///< UVS data lowest byte
#define LTR390_UVSDATA_MSB 0x11     ///< UVS data middle byte
#define LTR390_UVSDATA_HSB 0x12     ///< UVS data highest byte
#define LTR390_INT_CFG 0x19         ///< Interrupt configuration
#define LTR390_INT_PST 0x1A         ///< Interrupt persistance config
#define LTR390_THRESH_UP 0x21       ///< Upper threshold, low byte
#define LTR390_THRESH_LOW 0x24      ///< Lower threshold, low byte

/*!    @brief  Whether we are measuring ambient or UV light  */
typedef enum {
  LTR390_MODE_ALS,
  LTR390_MODE_UVS,
} ltr390_mode_t;

/*!    @brief  Sensor gain for UV or ALS  */
typedef enum {
  LTR390_GAIN_1 = 0,
  LTR390_GAIN_3,
  LTR390_GAIN_6,
  LTR390_GAIN_9,
  LTR390_GAIN_18,
} ltr390_gain_t;

/*!    @brief Measurement resolution (higher res means slower reads!)  */
typedef enum {
  LTR390_RESOLUTION_20BIT,
  LTR390_RESOLUTION_19BIT,
  LTR390_RESOLUTION_18BIT,
  LTR390_RESOLUTION_17BIT,
  LTR390_RESOLUTION_16BIT,
  LTR390_RESOLUTION_13BIT,
} ltr390_resolution_t;


class LTR390 {
public:
  LTR390(int addr);
  LTR390();
  LTR390(TwoWire *w, int addr);
  LTR390(TwoWire *w);

  bool init();

  bool reset(void);

  void enable(bool en);
  bool enabled(void);

  void setMode(ltr390_mode_t mode);
  ltr390_mode_t getMode(void);

  void setGain(ltr390_gain_t gain);
  ltr390_gain_t getGain(void);

  void setResolution(ltr390_resolution_t res);
  ltr390_resolution_t getResolution(void);

  void setThresholds(uint32_t lower, uint32_t higher);

  void configInterrupt(bool enable, ltr390_mode_t source,
                       uint8_t persistance = 0);

  bool newDataAvailable(void);
  uint32_t readUVS(void);
  uint32_t readALS(void);

  float getLux(void);

  float getUVI(void);

  uint8_t writeRegister(uint8_t reg, uint8_t val);
  uint8_t readRegister(uint8_t reg);

private:
  TwoWire *_wire;
  int i2cAddress;
  float gain_factor[5] = {1, 3, 6, 9, 18};
  float res_factor[6] = {4, 2, 1, 0.5, 0.25, 0.03125};
};

#endif

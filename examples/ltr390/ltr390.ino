#include <Wire.h>
#include <LTR390.h>

#define I2C_ADDRESS 0x53

/* There are several ways to create your LTR390 object:
 * LTR390 ltr390 = LTR390()                    -> uses Wire / I2C Address = 0x53
 * LTR390 ltr390 = LTR390(OTHER_ADDR)          -> uses Wire / I2C_ADDRESS
 * LTR390 ltr390 = LTR390(&wire2)              -> uses the TwoWire object wire2 / I2C_ADDRESS
 * LTR390 ltr390 = LTR390(&wire2, I2C_ADDRESS) -> all together
 * Successfully tested with two I2C busses on an ESP32
 */
LTR390 ltr390(I2C_ADDRESS);

void setup() {
   Serial.begin(115200);
  Wire.begin();
  if(!ltr390.init()){
    Serial.println("LTR390 not connected!");
  }

  ltr390.setMode(LTR390_MODE_ALS);

  ltr390.setGain(LTR390_GAIN_3);
  Serial.print("Gain : ");
  switch (ltr390.getGain()) {
    case LTR390_GAIN_1: Serial.println(1); break;
    case LTR390_GAIN_3: Serial.println(3); break;
    case LTR390_GAIN_6: Serial.println(6); break;
    case LTR390_GAIN_9: Serial.println(9); break;
    case LTR390_GAIN_18: Serial.println(18); break;
  }
  
  ltr390.setResolution(LTR390_RESOLUTION_18BIT);
  Serial.print("Resolution : ");
  switch (ltr390.getResolution()) {
    case LTR390_RESOLUTION_13BIT: Serial.println(13); break;
    case LTR390_RESOLUTION_16BIT: Serial.println(16); break;
    case LTR390_RESOLUTION_17BIT: Serial.println(17); break;
    case LTR390_RESOLUTION_18BIT: Serial.println(18); break;
    case LTR390_RESOLUTION_19BIT: Serial.println(19); break;
    case LTR390_RESOLUTION_20BIT: Serial.println(20); break;
  }

  //ltr390.setThresholds(100, 1000);
  //ltr390.configInterrupt(true, LTR390_MODE_UVS);

}

void loop() {
  if (ltr390.newDataAvailable()) {
      if (ltr390.getMode() == LTR390_MODE_ALS) {
         Serial.print("Ambient Light Lux: "); 
         Serial.println(ltr390.getLux());
         ltr390.setGain(LTR390_GAIN_18);                  //Recommended for UVI - x18
         ltr390.setResolution(LTR390_RESOLUTION_20BIT);   //Recommended for UVI - 20-bit
         ltr390.setMode(LTR390_MODE_UVS);             
      } else if (ltr390.getMode() == LTR390_MODE_UVS) {
         Serial.print("UV Index: "); 
         Serial.println(ltr390.getUVI());
         ltr390.setGain(LTR390_GAIN_3);                   //Recommended for Lux - x3
         ltr390.setResolution(LTR390_RESOLUTION_18BIT);   //Recommended for Lux - 18-bit
         ltr390.setMode(LTR390_MODE_ALS);
      }
  }
}

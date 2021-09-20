# FDC2214 Library for Arduino

### Dependencies
* Wire

### Example
FDC2214 objects can be constructed and configured using a sequence of function calls to modify the sensor's configurations registers. The `begin()` method transmits the configuration parameters to the FDC. Each object is initialized with a set of reasonable default parameters, but the default configuration may need to be tweaked depending on your application.

```c++
#include <FDC2214Lib.h>

FDC2214 sensor;

void setup() {
    sensor.withI2cAddress(0)
          .withInternalOscillator()
          .withContinuousConversion(0)
          .withReferenceCount(0xFFFF)
          .withSettleCount(100)
          .withDriveCurrent(31)
          .withDeglitchValue(DEGLITCH_10MHZ)
          .begin();

    Serial.begin(9600);
}

void loop() {
    Serial.println(sensor.getSensorReading(0));
}
```

### Thanks to

* zharijs <https://github.com/zharijs/FDC2214>
    - The inspiration for this library and many of defined constants are borrowed directly from the linked FDC2214 library. My implementation aims to rewrite and improve this          library to provide greater readability and flexibility. 

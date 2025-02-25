void		devINA219init();
WarpStatus	readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterINA219(uint8_t deviceRegister, uint8_t payloadBtye);
WarpStatus 	configureSensorINA219(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1);
void		printSensorDataINA219(bool hexModeFlag);
uint8_t		appendSensorDataINA219(uint8_t* buf);

const uint8_t bytesPerMeasurementINA219            = 6;
const uint8_t bytesPerReadingINA219                = 2;
const uint8_t numberOfReadingsPerMeasurementINA219 = 3;

static typedef enum {
    INA219_ConfigurationRegister = 0x0,
    INA219_ShuntVoltageRegister = 0x1,
    INA219_BusVoltageRegister = 0x2,
    INA219_PowerRegister = 0x3,
    INA219_CurrentRegister = 0x4,
    INA219_CalibrationRegister = 0x5,
}
#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devINA219.h"


extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t			gWarpI2cBaudRateKbps;
extern volatile uint32_t			gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t			gWarpSupplySettlingDelayMilliseconds;

typedef enum {
	INA219_ConfigurationRegister = 0x0,
	INA219_ShuntVoltageRegister = 0x1,
	INA219_BusVoltageRegister = 0x2,
	INA219_PowerRegister = 0x3,
	INA219_CurrentRegister = 0x4,
	INA219_CalibrationRegister = 0x5,
};

void
initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceINA219State.i2cAddress					= i2cAddress;
	deviceINA219State.operatingVoltageMillivolts	= operatingVoltageMillivolts;
	configureSensorINA219();

	WarpStatus regStatus;
	regStatus = readSensorRegisterINA219(INA219_ConfigurationRegister, 2);
	warpPrint("Configuration Buffer[0]: 0x%X, Buffer[1]: 0x%X\n", 
		deviceINA219State.i2cBuffer[0], deviceINA219State.i2cBuffer[1]);

	regStatus = readSensorRegisterINA219(INA219_CalibrationRegister, 2);
	warpPrint("Calibration Buffer[0]: 0x%X, Buffer[1]: 0x%X\n", 
		deviceINA219State.i2cBuffer[0], deviceINA219State.i2cBuffer[1]);

	regStatus = readSensorRegisterINA219(INA219_BusVoltageRegister, 2);
	warpPrint("Bus Buffer[0]: 0x%X, Buffer[1]: 0x%X\n", 
		deviceINA219State.i2cBuffer[0], deviceINA219State.i2cBuffer[1]);
	return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload)
{ // All INA219 16-bit registers are two 8-bit bytes via I2C 
	uint8_t		payloadByte[2], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x00: case 0x05: // Can only write to configuration and calibration registers
		{
			/* OK */
			break;
		}

		default:
		{
			warpPrint("kWarpStatusBadDeviceCommand\n");
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
		{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	payloadByte[0] = (uint8_t)(payload >> 8); // MSB
	payloadByte[1] = (uint8_t)(payload & 0xFF); // LSB
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1,
		payloadByte,
		2,
		gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorINA219()
{
	WarpStatus	i2cWriteStatus1, i2cWriteStatus2;
	warpPrint("\nConfiguring INA219...\n");

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	i2cWriteStatus1 = writeSensorRegisterINA219(INA219_ConfigurationRegister, (uint16_t)0b0000000110011111);

	i2cWriteStatus2 = writeSensorRegisterINA219(INA219_CalibrationRegister, (uint16_t)0x5000);  

	return (i2cWriteStatus1 | i2cWriteStatus2);
}

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03:
		case 0x04: case 0x05: // see INA219 pg. 24
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
		{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	cmdBuf[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
		0 /* I2C peripheral instance */,
		&slave,
		cmdBuf,
		1,
		(uint8_t *)deviceINA219State.i2cBuffer,
		numberOfBytes,
		gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		warpPrint("not successful reading");
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataINA219(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;


	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	i2cReadStatus = readSensorRegisterINA219(INA219_CurrentRegister, 2);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = (readSensorRegisterValueLSB | readSensorRegisterValueMSB << 8);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}
}

void
printCurrentINA219()
{
	uint16_t	shunt_voltage;
	uint16_t	current;
	uint16_t	calibration;
	WarpStatus	i2cReadStatus;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	i2cReadStatus = readSensorRegisterINA219(INA219_BusVoltageRegister, 2);
	shunt_voltage = (deviceINA219State.i2cBuffer[0] << 8) | deviceINA219State.i2cBuffer[1];
	float shunt_voltage_mV = (float)shunt_voltage * 0.01;
	warpPrint("Shunt voltage (mV) = %.2f,\n", shunt_voltage_mV);

	i2cReadStatus = readSensorRegisterINA219(INA219_CalibrationRegister, 2);
	calibration = (deviceINA219State.i2cBuffer[1] | deviceINA219State.i2cBuffer[0] << 8);

	i2cReadStatus = readSensorRegisterINA219(INA219_CurrentRegister, 2);
	current = (deviceINA219State.i2cBuffer[1] | deviceINA219State.i2cBuffer[0] << 8);
	float current_mA = shunt_voltage_mV * 0.01 * (float)calibration / 4096.f;

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		warpPrint("Calculated current (mA) = %.2f,\n", current_mA);
		warpPrint("Current (mA) = %.2f,\n\n", current);
	}
}

uint8_t
appendSensorDataINA219(uint8_t* buf)
{
	uint8_t index = 0;
	uint16_t readSensorRegisterValueLSB;
	uint16_t readSensorRegisterValueMSB;
	int16_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	i2cReadStatus                   = readSensorRegisterINA219(INA219_CurrentRegister, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = (readSensorRegisterValueLSB | readSensorRegisterValueMSB << 8);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}
}
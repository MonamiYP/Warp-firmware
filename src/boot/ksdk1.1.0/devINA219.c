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


extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t			gWarpI2cBaudRateKbps;
extern volatile uint32_t			gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t			gWarpSupplySettlingDelayMilliseconds;



void
initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceINA219State.i2cAddress					= i2cAddress;
	deviceINA219State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

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
	payloadByte[0] = payload;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1,
		payloadByte,
		1,
		gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorINA219(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1)
{
	WarpStatus	i2cWriteStatus1, i2cWriteStatus2;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	i2cWriteStatus1 = writeSensorRegisterINA219(INA219_ConfigurationRegister, (uint16_t)0b0011100001111101);

	i2cWriteStatus2 = writeSensorRegisterINA219(INA219_CalibrationRegister, (uint16_t)0xE4); // 228 

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

	/*
	 *	From the MMA8451Q datasheet:
	 *
	 *		"A random read access to the LSB registers is not possible.
	 *		Reading the MSB register and then the LSB register in sequence
	 *		ensures that both bytes (LSB and MSB) belong to the same data
	 *		sample, even if a new data sample arrives between reading the
	 *		MSB and the LSB byte."
	 *
	 *	We therefore do 2-byte read transactions, for each of the registers.
	 *	We could also improve things by doing a 6-byte read transaction.
	 */
	i2cReadStatus = readSensorRegisterINA219(INA219_CurrentRegister, 2 /* numberOfBytes */);
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
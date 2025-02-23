/*
	Authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
	2018-onwards. See git log.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

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



extern volatile WarpI2CDeviceState	deviceAMG8834State;
extern volatile uint32_t			gWarpI2cBaudRateKbps;
extern volatile uint32_t			gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t			gWarpSupplySettlingDelayMilliseconds;


/*
 *	AMG8834.
 */
void
initAMG8834(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceAMG8834State.i2cAddress					= i2cAddress;
	deviceAMG8834State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}

WarpStatus
writeSensorRegisterAMG8834(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t			payloadByte[1], commandByte[1];
	i2c_status_t	returnValue;


	warpScaleSupplyVoltage(deviceAMG8834State.operatingVoltageMillivolts);

	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03:
		case 0x05: case 0x07: case 0x08: case 0x09:
		case 0x0A: case 0x0B: case 0x0C: case 0x0D:
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
		.address = deviceAMG8834State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	warpEnableI2Cpins();
	returnValue = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1,
		payloadByte,
		1,
		gWarpI2cTimeoutMilliseconds);
	if (returnValue != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorAMG8834(uint8_t payloadConfigReg, uint8_t payloadFrameRateReg)
{
	WarpStatus	i2cWriteStatus1, i2cWriteStatus2;


	warpScaleSupplyVoltage(deviceAMG8834State.operatingVoltageMillivolts);

	i2cWriteStatus1 = writeSensorRegisterAMG8834(kWarpSensorConfigurationRegisterAMG8834RST /* register address configuration register */,
												 payloadConfigReg);

	i2cWriteStatus2 = writeSensorRegisterAMG8834(kWarpSensorConfigurationRegisterAMG8834FPSC /* register address frame rate register */,
												 payloadFrameRateReg);

	return (i2cWriteStatus1 | i2cWriteStatus2);
}

WarpStatus
readSensorRegisterAMG8834(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t cmdBuf[1] = {0xFF};
	i2c_status_t status;

	USED(numberOfBytes);

	warpScaleSupplyVoltage(deviceAMG8834State.operatingVoltageMillivolts);

	if (deviceRegister > 0xFF)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
		{
		.address       = deviceAMG8834State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	cmdBuf[0] = deviceRegister;

	warpEnableI2Cpins();
	status = I2C_DRV_MasterReceiveDataBlocking(
		0 /* I2C peripheral instance */,
		&slave,
		cmdBuf,
		1,
		(uint8_t*)deviceAMG8834State.i2cBuffer,
		numberOfBytes,
		gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataAMG8834(bool hexModeFlag)
{
	uint16_t readSensorRegisterValueLSB;
	uint16_t readSensorRegisterValueMSB;
	int16_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	warpScaleSupplyVoltage(deviceAMG8834State.operatingVoltageMillivolts);

	for (uint16_t bufAddress = kWarpSensorOutputRegisterAMG8834T01L; bufAddress <= kWarpSensorOutputRegisterAMG8834T64H; bufAddress = bufAddress + 2)
	{
		i2cReadStatus              = readSensorRegisterAMG8834(bufAddress, 2 /* numberOfBytes */);
		readSensorRegisterValueLSB = deviceAMG8834State.i2cBuffer[0];
		readSensorRegisterValueMSB = deviceAMG8834State.i2cBuffer[1];

		/*
		 *	Format is 12 bits with the highest-order bit being a sign (0 +ve, 1 -ve)
		 */
		readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0x07) << 8) | (readSensorRegisterValueLSB & 0xFF);
		readSensorRegisterValueCombined *= ((readSensorRegisterValueMSB & (1 << 3)) == 0 ? 1 : -1);

		/*
		 *	Specification, page 14/26, says LSB counts for 0.25 C (1/4 C)
		 */
		readSensorRegisterValueCombined >>= 2;

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

	i2cReadStatus              = readSensorRegisterAMG8834(kWarpSensorOutputRegisterAMG8834T01L, 2 /* numberOfBytes */);
	readSensorRegisterValueLSB = deviceAMG8834State.i2cBuffer[0];
	readSensorRegisterValueMSB = deviceAMG8834State.i2cBuffer[1];

	/*
	 *	Format is 12 bits with the highest-order bit being a sign (0 +ve, 1 -ve)
	 */
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0x07) << 8) | (readSensorRegisterValueLSB & 0xFF);
	readSensorRegisterValueCombined *= ((readSensorRegisterValueMSB & (1 << 3)) == 0 ? 1 : -1);

	/*
	 *	Specification, page 13/26, says LSB of the 12-bit thermistor value counts for 0.0625 C (1 / 16 C)
	 */
	readSensorRegisterValueCombined >>= 4;

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
appendSensorDataAMG8834(uint8_t* buf)
{
	uint8_t index = 0;

	uint16_t 	readSensorRegisterValueLSB;
	uint16_t 	readSensorRegisterValueMSB;
	int16_t 	readSensorRegisterValueCombined;
	WarpStatus 	i2cReadStatus;

	warpScaleSupplyVoltage(deviceAMG8834State.operatingVoltageMillivolts);

	for (uint16_t bufAddress = kWarpSensorOutputRegisterAMG8834T01L; bufAddress <= kWarpSensorOutputRegisterAMG8834T64H; bufAddress = bufAddress + 2)
	{
		i2cReadStatus              = readSensorRegisterAMG8834(bufAddress, 2 /* numberOfBytes */);
		readSensorRegisterValueLSB = deviceAMG8834State.i2cBuffer[0];
		readSensorRegisterValueMSB = deviceAMG8834State.i2cBuffer[1];

		/*
		 *	Format is 12 bits with the highest-order bit being a sign (0 +ve, 1 -ve)
		 */
		readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0x07) << 8) | (readSensorRegisterValueLSB & 0xFF);
		readSensorRegisterValueCombined *= ((readSensorRegisterValueMSB & (1 << 3)) == 0 ? 1 : -1);

		/*
		 *	Specification, page 14/26, says LSB counts for 0.25 C (1/4 C)
		 */
		readSensorRegisterValueCombined >>= 2;

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

	i2cReadStatus              = readSensorRegisterAMG8834(kWarpSensorOutputRegisterAMG8834T01L, 2 /* numberOfBytes */);
	readSensorRegisterValueLSB = deviceAMG8834State.i2cBuffer[0];
	readSensorRegisterValueMSB = deviceAMG8834State.i2cBuffer[1];

	/*
	 *	Format is 12 bits with the highest-order bit being a sign (0 +ve, 1 -ve)
	 */
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0x07) << 8) | (readSensorRegisterValueLSB & 0xFF);
	readSensorRegisterValueCombined *= ((readSensorRegisterValueMSB & (1 << 3)) == 0 ? 1 : -1);

	/*
	 *	Specification, page 13/26, says LSB of the 12-bit thermistor value counts for 0.0625 C (1 / 16 C)
	 */
	readSensorRegisterValueCombined >>= 4;

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

	return index;
}

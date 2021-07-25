//---------------------------------------------------------------------------------
// Copyright (c) July 2021, devMobile Software
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//---------------------------------------------------------------------------------
using System;
using System.Device.Gpio;
using System.Device.Spi;
using System.Diagnostics;
using System.Threading;

namespace devMobile.IoT.SX127x.RegisterScan
{
	public sealed class SX127XDevice
	{
		private readonly int SpiBusId;
		private readonly int ChipSelectLogicalPinNumber;
		private readonly SpiDevice SX127XTransceiver = null;
		private readonly GpioController gpioController = null;

		public SX127XDevice(int spiBusId = 0, int chipSelectLine = 0, int chipSelectLogicalPinNumber = 0)
		{
			this.SpiBusId = spiBusId;
			this.ChipSelectLogicalPinNumber = chipSelectLogicalPinNumber;

			var settings = new SpiConnectionSettings(spiBusId, chipSelectLine)
			{
				ClockFrequency = 500000,
				Mode = SpiMode.Mode0,
			};

			this.SX127XTransceiver = SpiDevice.Create(settings);

			Thread.Sleep(500);

			if (ChipSelectLogicalPinNumber != 0)
			{
				gpioController = new GpioController(PinNumberingScheme.Logical);
				gpioController.OpenPin(ChipSelectLogicalPinNumber, PinMode.Output);
				gpioController.Write(ChipSelectLogicalPinNumber, PinValue.High);
			}
		}

		public Byte RegisterReadByte(byte registerAddress)
		{
			Span<byte> writeBuffer = stackalloc byte[] { registerAddress, 0 };
			Span<byte> readBuffer = stackalloc byte[writeBuffer.Length];
			Debug.Assert(SX127XTransceiver != null);

			if (this.ChipSelectLogicalPinNumber != 0)
			{
				gpioController.Write(ChipSelectLogicalPinNumber, PinValue.Low);
			}

			this.SX127XTransceiver.TransferFullDuplex(writeBuffer, readBuffer);

			if (this.ChipSelectLogicalPinNumber != 0)
			{
				gpioController.Write(ChipSelectLogicalPinNumber, PinValue.High);
			}

			return readBuffer[writeBuffer.Length - 1];
		}
	}

	class Program
	{
		static void Main(string[] args)
		{
#if ADAFRUIT_RADIO_BONNET
			SX127XDevice sX127XDevice = new SX127XDevice(chipSelectLine:1);
#endif

#if DRAGINO_CS0 // Y
			SX127XDevice sX127XDevice = new SX127XDevice(chipSelectLine:0, chipSelectLogicalPinNumber:25);
#endif

#if DRAGINO_CS1 // Y
			SX127XDevice sX127XDevice = new SX127XDevice(chipSelectLine:0, chipSelectLogicalPinNumber:25);
#endif

#if ELECROW // Y
			SX127XDevice sX127XDevice = new SX127XDevice(chipSelectLine:1);
#endif

#if ELECTRONIC_TRICKS // Y
			SX127XDevice sX127XDevice = new SX127XDevice(chipSelectLine:0);
#endif

#if UPUTRONICS_RPIZERO_CS0 
			SX127XDevice sX127XDevice = new SX127XDevice(chipSelectLine: 0);
#endif

#if UPUTRONICS_RPIZERO_CS1 
			SX127XDevice sX127XDevice = new SX127XDevice(chipSelectLine: 1);
#endif

			while (true)
			{
				for (byte registerIndex = 0; registerIndex <= 0x42; registerIndex++)
				{
					byte registerValue = sX127XDevice.RegisterReadByte(registerIndex);

					Debug.WriteLine("Register 0x{0:x2} - Value 0x{1:x2} - Bits {2}", registerIndex, registerValue, Convert.ToString(registerValue, 2).PadLeft(8, '0'));
				}

				Thread.Sleep(10000);
			}
		}
	}
}

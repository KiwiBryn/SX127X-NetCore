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
#define ChipSelectNonStandard
using System;
#if ChipSelectNonStandard
using System.Device.Gpio;
#endif
using System.Device.Spi;
using System.Threading;

namespace devMobile.IoT.SX127x.ShieldSPIWriteRead
{
	class Program
	{
		private const int SpiBusId = 0;
		private const int ChipSelectLine = 1; // 0 or 1 for Uputronics depends on the switch, for the others choose CS pin not already in use
#if ChipSelectNonStandard
		private const int ChipSelectPinNumber = 25; // 25 for M2M, Dragino etc.
#endif
		private const byte RegisterAddress = 0x6; // RegFrfMsb 0x6c
		//private const byte RegisterAddress = 0x7; // RegFrfMid 0x80
		//private const byte RegisterAddress = 0x8; // RegFrfLsb 0x00
		//private const byte RegisterAddress = 0x42; // RegVersion 0x12

		static void Main(string[] args)
		{
#if ChipSelectNonStandard
			GpioController controller = null;

			controller = new GpioController(PinNumberingScheme.Logical);

			controller.OpenPin(ChipSelectPinNumber, PinMode.Output);
			controller.Write(ChipSelectPinNumber, PinValue.High);
#endif

			var settings = new SpiConnectionSettings(SpiBusId, ChipSelectLine)
			{
				ClockFrequency = 5000000,
				Mode = SpiMode.Mode0,   // From SemTech docs pg 80 CPOL=0, CPHA=0
			};

			SpiDevice spiDevice = SpiDevice.Create(settings);

			Thread.Sleep(500);

			while (true)
			{
				byte[] writeBuffer = new byte[] { RegisterAddress, 0 };
				byte[] readBuffer = new byte[writeBuffer.Length];

#if ChipSelectNonStandard
				controller.Write(ChipSelectPinNumber, PinValue.Low);
#endif

				spiDevice.Write(writeBuffer);
				spiDevice.Read(readBuffer);

#if ChipSelectNonStandard
				controller.Write(ChipSelectPinNumber, PinValue.High);
#endif

				byte registerValue = readBuffer[writeBuffer.Length - 1];

				Console.WriteLine($"Register 0x{RegisterAddress:x2} - Value 0X{registerValue:x2} - Bits {Convert.ToString(registerValue, 2).PadLeft(8, '0')}");

				Thread.Sleep(5000);
			}
		}
	}
}


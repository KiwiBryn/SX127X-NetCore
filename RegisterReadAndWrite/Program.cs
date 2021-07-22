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

namespace devMobile.IoT.SX127x.RegisterReadAndWrite
{
	public sealed class SX127XDevice
	{
		private const byte RegisterAddressMinimum = 0X0;
		private const byte RegisterAddressMaximum = 0x42;
		private const byte RegisterAddressReadMask = 0X7f;
		private const byte RegisterAddressWriteMask = 0x80;

		private readonly int SpiBusId;
		private readonly int ChipSelectLogicalPinNumber;
		private readonly SpiDevice SX127XTransceiver = null;
		private readonly GpioController gpioController = null;

		public SX127XDevice(int spiBusId = 0, int chipSelectLine = 0, int chipSelectLogicalPinNumber = 0, int resetPin = 0)
		{
			this.SpiBusId = spiBusId;
			this.ChipSelectLogicalPinNumber = chipSelectLogicalPinNumber;

			var settings = new SpiConnectionSettings(spiBusId, chipSelectLine)
			{
				ClockFrequency = 500000,
				Mode = SpiMode.Mode0,
			};

			this.SX127XTransceiver = SpiDevice.Create(settings);

			if ((chipSelectLogicalPinNumber != 0) || (resetPin != 0))
			{
				gpioController = new GpioController(PinNumberingScheme.Logical);
			}

			if (ChipSelectLogicalPinNumber != 0)
			{
				gpioController.OpenPin(ChipSelectLogicalPinNumber, PinMode.Output);
				gpioController.Write(ChipSelectLogicalPinNumber, PinValue.High);
			}

			if (resetPin != 0)
			{
				gpioController.OpenPin(resetPin, PinMode.Output);
				gpioController.Write(resetPin, PinValue.Low);

				Thread.Sleep(20);
				gpioController.Write(resetPin, PinValue.High);
				Thread.Sleep(20);
			}
		}

		public Byte ReadByte(byte registerAddress)
		{
			Span<byte> writeBuffer = stackalloc byte[2];
			Span<byte> readBuffer = stackalloc byte[writeBuffer.Length];

			if (SX127XTransceiver == null)
			{
				throw new ApplicationException("SX127XDevice is not initialised");
			}

			writeBuffer[0] = registerAddress &= RegisterAddressReadMask;

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

		public ushort ReadWord(byte registerAddress)
		{
			Span<byte> writeBuffer = stackalloc byte[3];
			Span<byte> readBuffer = stackalloc byte[writeBuffer.Length];

			if (SX127XTransceiver == null)
			{
				throw new ApplicationException("SX127XDevice is not initialised");
			}

			writeBuffer[0] = registerAddress &= RegisterAddressReadMask;

			if (this.ChipSelectLogicalPinNumber != 0)
			{
				gpioController.Write(ChipSelectLogicalPinNumber, PinValue.Low);
			}

			this.SX127XTransceiver.TransferFullDuplex(writeBuffer, readBuffer);

			if (this.ChipSelectLogicalPinNumber != 0)
			{
				gpioController.Write(ChipSelectLogicalPinNumber, PinValue.High);
			}

			return (ushort)(readBuffer[2] + (readBuffer[1] << 8));
		}

		public byte[] ReadBytes(byte registerAddress, byte length)
		{
			Span<byte> writeBuffer = stackalloc byte[length + 1];
			Span<byte> readBuffer = stackalloc byte[writeBuffer.Length];

			if (SX127XTransceiver == null)
			{
				throw new ApplicationException("SX127XDevice is not initialised");
			}

			writeBuffer[0] = registerAddress &= RegisterAddressReadMask;

			if (this.ChipSelectLogicalPinNumber != 0)
			{
				gpioController.Write(ChipSelectLogicalPinNumber, PinValue.Low);
			}

			this.SX127XTransceiver.TransferFullDuplex(writeBuffer, readBuffer);

			if (this.ChipSelectLogicalPinNumber != 0)
			{
				gpioController.Write(ChipSelectLogicalPinNumber, PinValue.High);
			}

			return readBuffer[1..readBuffer.Length].ToArray();
		}

		public void WriteByte(byte registerAddress, byte value)
		{
			Span<byte> writeBuffer = stackalloc byte[2];
			Span<byte> readBuffer = stackalloc byte[writeBuffer.Length];

			if (SX127XTransceiver == null)
			{
				throw new ApplicationException("SX127XDevice is not initialised");
			}

			writeBuffer[0] = registerAddress |= RegisterAddressWriteMask;
			writeBuffer[1] = value;

			if (this.ChipSelectLogicalPinNumber != 0)
			{
				gpioController.Write(ChipSelectLogicalPinNumber, PinValue.Low);
			}

			this.SX127XTransceiver.TransferFullDuplex(writeBuffer, readBuffer);

			if (this.ChipSelectLogicalPinNumber != 0)
			{
				gpioController.Write(ChipSelectLogicalPinNumber, PinValue.High);
			}
		}

		public void WriteWord(byte address, ushort value)
		{
			Span<byte> writeBuffer = stackalloc byte[3];
			Span<byte> readBuffer = stackalloc byte[writeBuffer.Length];

			if (SX127XTransceiver == null)
			{
				throw new ApplicationException("SX127XDevice is not initialised");
			}

			writeBuffer[0] = address |= RegisterAddressWriteMask;
			writeBuffer[1] = (byte)(value>>8);
			writeBuffer[2] = (byte)value;

			if (this.ChipSelectLogicalPinNumber != 0)
			{
				gpioController.Write(ChipSelectLogicalPinNumber, PinValue.Low);
			}

			this.SX127XTransceiver.TransferFullDuplex(writeBuffer, readBuffer);

			if (this.ChipSelectLogicalPinNumber != 0)
			{
				gpioController.Write(ChipSelectLogicalPinNumber, PinValue.High);
			}
		}

		public void WriteBytes(byte address, byte[] bytes)
		{
			Span<byte> writeBuffer = stackalloc byte[bytes.Length + 1];
			Span<byte> readBuffer = stackalloc byte[writeBuffer.Length];

			if (SX127XTransceiver == null)
			{
				throw new ApplicationException("SX127XDevice is not initialised");
			}

			writeBuffer[0] = address |= RegisterAddressWriteMask;
			for (byte index = 0; index < bytes.Length; index++)
			{
				writeBuffer[index + 1] = bytes[index];
			}

			if (this.ChipSelectLogicalPinNumber != 0)
			{
				gpioController.Write(ChipSelectLogicalPinNumber, PinValue.Low);
			}

			this.SX127XTransceiver.TransferFullDuplex(writeBuffer, readBuffer);

			if (this.ChipSelectLogicalPinNumber != 0)
			{
				gpioController.Write(ChipSelectLogicalPinNumber, PinValue.High);
			}
		}

		public void RegisterDump()
		{
			Debug.WriteLine("Register dump");

			if (SX127XTransceiver == null)
			{
				throw new ApplicationException("SX127XDevice is not initialised");
			}

			for (byte registerIndex = RegisterAddressMinimum; registerIndex <= RegisterAddressMaximum; registerIndex++)
			{
				byte registerValue = this.ReadByte(registerIndex);

				Debug.WriteLine($"Register 0x{registerIndex:x2} - Value 0X{registerValue:x2} - Bits {Convert.ToString(registerValue, 2).PadLeft(8, '0')}");
			}

			Debug.WriteLine("");
		}
	}

	class Program
	{
		static void Main(string[] args)
		{
			Byte regOpMode;
			ushort preamble;
			byte[] frequencyBytes;
			// Uptronics has no reset pin uses CS0 or CS1
			//SX127XDevice sX127XDevice = new SX127XDevice(chipSelectLine: 0); 
			//SX127XDevice sX127XDevice = new SX127XDevice(chipSelectLine: 1); 

			// M2M device has reset pin uses non standard chip select 
			//SX127XDevice sX127XDevice = new SX127XDevice(chipSelectLine: 0, chipSelectLogicalPinNumber: 25, resetPin: 17);
			SX127XDevice sX127XDevice = new SX127XDevice(chipSelectLine: 1, chipSelectLogicalPinNumber:25, resetPin: 17);

			Console.WriteLine("In FSK mode");
			sX127XDevice.RegisterDump();


			Console.WriteLine("Read RegOpMode (read byte)");
			regOpMode = sX127XDevice.ReadByte(0x1);
			Debug.WriteLine($"RegOpMode 0x{regOpMode:x2}");

			Console.WriteLine("Set LoRa mode and sleep mode (write byte)");
			sX127XDevice.WriteByte(0x01, 0b10000000);

			Console.WriteLine("Read RegOpMode (read byte)");
			regOpMode = sX127XDevice.ReadByte(0x1);
			Debug.WriteLine($"RegOpMode 0x{regOpMode:x2}");


			Console.WriteLine("In LoRa mode");
			sX127XDevice.RegisterDump();


			Console.WriteLine("Read the preamble (read word)"); // Should be 0x08
			preamble = sX127XDevice.ReadWord(0x20);
			Debug.WriteLine($"Preamble 0x{preamble:x2} - Bits {Convert.ToString(preamble, 2).PadLeft(16, '0')}");

			Console.WriteLine("Set the preamble to 0x8000 (write word)");
			sX127XDevice.WriteWord(0x20, 0x8000);

			Console.WriteLine("Read the preamble (read word)"); // Should be 0x08
			preamble = sX127XDevice.ReadWord(0x20);
			Debug.WriteLine($"Preamble 0x{preamble:x2} - Bits {Convert.ToString(preamble, 2).PadLeft(16, '0')}");


			Console.WriteLine("Read the centre frequency"); // RegFrfMsb 0x6c RegFrfMid 0x80 RegFrfLsb 0x00 which is 433MHz
			frequencyBytes = sX127XDevice.ReadBytes(0x06, 3);
			Console.WriteLine($"Frequency Msb 0x{frequencyBytes[0]:x2} Mid 0x{frequencyBytes[1]:x2} Lsb 0x{frequencyBytes[2]:x2}");

			Console.WriteLine("Set the centre frequency"); 
			byte[] frequencyWriteBytes = { 0xE4, 0xC0, 0x00 };
			sX127XDevice.WriteBytes(0x06, frequencyWriteBytes);

			Console.WriteLine("Read the centre frequency"); // RegFrfMsb 0xE4 RegFrfMid 0xC0 RegFrfLsb 0x00 which is 915MHz
			frequencyBytes = sX127XDevice.ReadBytes(0x06, 3);
			Console.WriteLine($"Frequency Msb 0x{frequencyBytes[0]:x2} Mid 0x{frequencyBytes[1]:x2} Lsb 0x{frequencyBytes[2]:x2}");


			sX127XDevice.RegisterDump();

			// Sleep forever
			Thread.Sleep(-1);
		}
	}
}

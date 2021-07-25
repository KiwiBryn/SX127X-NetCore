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
using System.Text;
using System.Threading;

namespace devMobile.IoT.SX127x.EnumsAndMasks
{
	public sealed class SX127XDevice
	{
		private const byte RegisterAddressReadMask = 0X7f;
		private const byte RegisterAddressWriteMask = 0x80;

		// Registers from SemTech SX127X Datasheet
		enum Registers : byte
		{
			MinValue = RegOpMode,

			RegFifo = 0x0,
			RegOpMode = 0x01,
			//Reserved 0x02-0x06 
			RegFrMsb = 0x06,
			RegFrMid = 0x7,
			RegFrLsb = 0x08,
			RegPAConfig = 0x09,
			//RegPARamp = 0x0A, // not inlcuded as FSK/OOK functionality
			RegOcp = 0x0B,
			RegLna = 0x0C,
			RegFifoAddrPtr = 0x0D,
			RegFifoTxBaseAddr = 0x0E,
			RegFifoRxCurrent = 0x10,
			RegIrqFlagsMask = 0x11,
			RegIrqFlags = 0x12,
			// RegRxNdBytes = 0x13
			// RegRxHeaderCnValueMsb=0x14
			// RegRxHeaderCnValueLsb=0x15
			// RegRxPacketCntValueMsb=0x16
			// RegRxPacketCntValueMsb=0x17
			// RegModemStat=0x18
			// RegPktSnrVale=0x19
			// RegPktRssiValue=0x1A
			// RegRssiValue=0x1B
			// RegHopChannel=0x1C
			RegModemConfig1 = 0x1D,
			RegModemConfig2 = 0x1E,
			RegSymbTimeout = 0x1F,
			RegPreambleMsb = 0x20,
			RegPreambleLsb = 0x21,
			RegPayloadLength = 0x22,
			RegMaxPayloadLength = 0x23,
			RegHopPeriod = 0x24,
			// RegFifiRxByteAddr = 0x25
			RegModemConfig3 = 0x26,
			RegPpmCorrection = 0x27,
			// RegFeiMsb = 0x28
			// RegFeiMid = 0x29
			// RegFeiLsb = 0x2A
			// Reserved 0x2B
			// RegRssiWideband = 0x2C
			// Reserved 0x2D-0x30
			RegDetectOptimize = 0x31,
			// Reserved 0x32
			RegInvertIQ = 0x33,
			// Reserved 0x34-0x36
			RegDetectionThreshold = 0x37,
			// Reserved 0x38
			RegSyncWord = 0x39,
			RegDioMapping1 = 0x40,
			RegVersion = 0x42,

			MaxValue = RegVersion,
		}

		// RegOpMode mode flags
		private const byte RegOpModeLongRangeModeLoRa = 0b10000000;
		private const byte RegOpModeLongRangeModeFskOok = 0b00000000;
		private const byte RegOpModeLongRangeModeDefault = RegOpModeLongRangeModeFskOok;

		private const byte RegOpModeAcessSharedRegLoRa = 0b00000000;
		private const byte RegOpModeAcessSharedRegFsk = 0b01000000;
		private const byte RegOpModeAcessSharedRegDefault = RegOpModeAcessSharedRegLoRa;

		private const byte RegOpModeLowFrequencyModeOnHighFrequency = 0b00000000;
		private const byte RegOpModeLowFrequencyModeOnLowFrequency = 0b00001000;
		private const byte RegOpModeLowFrequencyModeOnDefault = RegOpModeLowFrequencyModeOnLowFrequency;

		[Flags]
		public enum RegOpModeMode : byte
		{
			Sleep = 0b00000000,
			StandBy = 0b00000001,
			FrequencySynthesisTX = 0b00000010,
			Transmit = 0b00000011,
			FrequencySynthesisRX = 0b00000100,
			ReceiveContinuous = 0b00000101,
			ReceiveSingle = 0b00000110,
			ChannelActivityDetection = 0b00000111,
		};

		// Frequency configuration magic numbers from Semtech SX127X specs
		private const double RH_RF95_FXOSC = 32000000.0;
		private const double RH_RF95_FSTEP = RH_RF95_FXOSC / 524288.0;

		// RegFrMsb, RegFrMid, RegFrLsb
		private const double FrequencyDefault = 434000000.0;

		// RegPAConfig
		private const Byte RegPAConfigPASelectRFO = 0b00000000;
		private const Byte RegPAConfigPASelectPABoost = 0b10000000;
		private const Byte RegPAConfigPASelectDefault = RegPAConfigPASelectRFO;

		private const byte RegPAConfigMaxPowerMin = 0b00000000;
		private const byte RegPAConfigMaxPowerMax = 0b01110000;
		private const byte RegPAConfigMaxPowerDefault = 0b01000000;

		private const byte RegPAConfigOutputPowerMin = 0b00000000;
		private const byte RegPAConfigOutputPowerMax = 0b00001111;
		private const byte RegPAConfigOutputPowerDefault = 0b00001111;

		// RegPaRamp appears to be for FSK only ?

		// RegOcp
		private const byte RegOcpOn = 0b00100000;
		private const byte RegOcpOff = 0b00000000;
		private const byte RegOcpDefault = RegOcpOn;

		private const byte RegOcpOcpTrimMin = 0b00000000;
		private const byte RegOcpOcpTrimMax = 0b00011111;
		private const byte RegOcpOcpTrimDefault = 0b00001011;

		// RegLna
		[Flags]
		public enum RegLnaLnaGain : byte
		{
			G1 = 0b00000001,
			G2 = 0b00000010,
			G3 = 0b00000011,
			G4 = 0b00000100,
			G5 = 0b00000101,
			G6 = 0b00000110
		}
		// Note the gain is backwards less = more
		private const RegLnaLnaGain LnaGainDefault = RegLnaLnaGain.G1;

		private const byte RegLnaLnaBoostLfOn = 0b00011000;
		private const byte RegLnaLnaBoostLfOff = 0b00000000;
		private const byte RegLnaLnaBoostLfDefault = RegLnaLnaBoostLfOff;

		private const byte RegLnaLnaBoostHfOn = 0b00000011;
		private const byte RegLnaLnaBoostHfOff = 0b00000000;
		private const byte RegLnaLnaBoostHfDefault = RegLnaLnaBoostHfOff;

		[Flags]
		enum RegIrqFlagsMask : byte
		{
			RxTimeoutMask = 0b10000000,
			RxDoneMask = 0b01000000,
			PayLoadCrcErrorMask = 0b00100000,
			ValidHeadrerMask = 0b00010000,
			TxDoneMask = 0b00001000,
			CadDoneMask = 0b00000100,
			FhssChangeChannelMask = 0b00000010,
			CadDetectedMask = 0b00000001,
		}

		[Flags]
		enum RegIrqFlags : byte
		{
			RxTimeout = 0b10000000,
			RxDone = 0b01000000,
			PayLoadCrcError = 0b00100000,
			ValidHeadrer = 0b00010000,
			TxDone = 0b00001000,
			CadDone = 0b00000100,
			FhssChangeChannel = 0b00000010,
			CadDetected = 0b00000001,
			ClearAll = 0b11111111,
		}

		// RegModemConfig1
		public enum RegModemConfigBandwidth : byte
		{
			_7_8KHz = 0b00000000,
			_10_4KHz = 0b00010000,
			_15_6KHz = 0b00100000,
			_20_8KHz = 0b00110000,
			_31_25KHz = 0b01000000,
			_41_7KHz = 0b01010000,
			_62_5KHz = 0b01100000,
			_125KHz = 0b01110000,
			_250KHz = 0b10000000,
			_500KHz = 0b10010000
		}
		private const RegModemConfigBandwidth RegModemConfigBandwidthDefault = RegModemConfigBandwidth._125KHz;

		public enum RegModemConfigCodingRate
		{
			_4of5 = 0b00000010,
			_4of6 = 0b00000100,
			_4of7 = 0b00000110,
			_4of8 = 0b00001000,
		}
		private const RegModemConfigCodingRate RegModemConfigCodingRateDefault = RegModemConfigCodingRate._4of5;

		public enum RegModemConfigImplicitHeaderModeOn
		{
			ExplicitHeaderMode = 0b00000000,
			ImplicitHeaderMode = 0b00000001,
		}
		private const RegModemConfigImplicitHeaderModeOn RegModemConfigImplicitHeaderModeOnDefault = RegModemConfigImplicitHeaderModeOn.ExplicitHeaderMode;

		// RegModemConfig2
		public enum RegModemConfig2SpreadingFactor : byte
		{
			_64ChipsPerSymbol = 0b01100000,
			_128ChipsPerSymbol = 0b01110000,
			_256ChipsPerSymbol = 0b10000000,
			_512ChipsPerSymbol = 0b10010000,
			_1024ChipsPerSymbol = 0b10100000,
			_2048ChipsPerSymbol = 0b10110000,
			_4096ChipsPerSymbol = 0b11000000,
		}
		private const RegModemConfig2SpreadingFactor RegModemConfig2SpreadingFactorDefault = RegModemConfig2SpreadingFactor._128ChipsPerSymbol;

		private const byte RegModemConfig2TxContinuousModeOn = 0b00001000;
		private const byte RegModemConfig2TxContinuousModeOff = 0b00000000;
		private const byte RegModemConfig2TxContinuousModeDefault = RegModemConfig2TxContinuousModeOff;

		private const byte RegModemConfig2RxPayloadCrcOn = 0b00000100;
		private const byte RegModemConfig2RxPayloadCrcOff = 0b00000000;
		private const byte RegModemConfig2RxPayloadCrcDefault = RegModemConfig2RxPayloadCrcOff;

		// RegModemConfig2 for MSb RegSymbTimeoutLsb for LSB
		private const byte SymbolTimeoutDefault = 0x64;

		// RegReambleMsb & RegReambleLsb
		private const ushort PreambleLengthDefault = 0x08;

		// RegPayloadLength
		private const byte PayloadLengthDefault = 0x01;

		// RegMaxPayloadLength
		private const byte PayloadMaxLengthDefault = 0xff;

		// RegHopPeriod
		private const byte FreqHoppingPeriodDefault = 0x0;

		// RegModemConfig3
		private const byte RegModemConfig3LowDataRateOptimizeOn = 0b00001000;
		private const byte RegModemConfig3LowDataRateOptimizeOff = 0b00000000;
		private const byte RegModemConfig3LowDataRateOptimizeDefault = RegModemConfig3LowDataRateOptimizeOff;

		private const byte RegModemConfig3AgcAutoOn = 0b00000100;
		private const byte RegModemConfig3AgcAutoOff = 0b00000000;
		private const byte RegModemConfig3AgcAutoDefault = RegModemConfig3AgcAutoOff;

		// RegPpmCorrection
		private const byte ppmCorrectionDefault = 0x0;

		// RegDetectOptimize
		public enum RegDetectOptimizeDectionOptimize
		{
			_SF7toSF12 = 0x03,
			_SF6 = 0x05,
		};
		private const RegDetectOptimizeDectionOptimize RegDetectOptimizeDectionOptimizeDefault = RegDetectOptimizeDectionOptimize._SF7toSF12;

		// RegInvertId
		private const byte InvertIqOn = 0b01000000;
		private const byte InvertIqOff = 0b00000000;
		private const byte InvertIqDefault = InvertIqOn;

		// RegDetectionThreshold
		public enum RegisterDetectionThreshold
		{
			_SF7toSF12 = 0x0A,
			_SF6 = 0x0c,
		}
		private const RegisterDetectionThreshold RegisterDetectionThresholdDefault = RegisterDetectionThreshold._SF7toSF12;

		// RegSyncWord Syncword default for public networks
		public const byte RegSyncWordDefault = 0x12;

		// RegDioMapping1 
		[Flags]
		public enum RegDioMapping1
		{
			Dio0RxDone = 0b00000000,
			Dio0TxDone = 0b01000000,
			Dio0CadDone = 0b1000000,
		}

		// The Semtech ID Relating to the Silicon revision
		private const byte RegVersionValueExpected = 0x12;

		// Hardware configuration support
		private RegOpModeMode RegOpModeModeCurrent = RegOpModeMode.Sleep;


		private readonly int SpiBusId;
		private readonly int ChipSelectLogicalPinNumber;
		private readonly int ResetLogicalPinNumber;
		private readonly SpiDevice SX127XTransceiver = null;
		private readonly GpioController gpioController = null;
		private double Frequency = FrequencyDefault;

		public enum ChipSelectLine
		{
			CS0 = 0,
			CS1 = 1
		}

		public SX127XDevice(ChipSelectLine chipSelectLine, int interuptLogicalPinNumber, int spiBusId = 0, int chipSelectLogicalPinNumber = 0, int resetLogicalPinNumber = 0)
		{
			this.SpiBusId = spiBusId;
			this.ChipSelectLogicalPinNumber = chipSelectLogicalPinNumber;
			this.ResetLogicalPinNumber = resetLogicalPinNumber;

			var settings = new SpiConnectionSettings(spiBusId, (int)chipSelectLine)
			{
				ClockFrequency = 500000,
				Mode = SpiMode.Mode0,
			};

			this.SX127XTransceiver = SpiDevice.Create(settings);

			if ((interuptLogicalPinNumber != 0)||(chipSelectLogicalPinNumber != 0) || (resetLogicalPinNumber != 0))
			{
				gpioController = new GpioController(PinNumberingScheme.Logical);
			}

			if (ChipSelectLogicalPinNumber != 0)
			{
				gpioController.OpenPin(ChipSelectLogicalPinNumber, PinMode.Output);
				gpioController.Write(ChipSelectLogicalPinNumber, PinValue.High);
			}

			// As soon as ChipSelectLine/ChipSelectLogicalPinNumber check that SX127X chip is present
			Byte regVersionValue = this.ReadByte((byte)Registers.RegVersion);
			if (regVersionValue != RegVersionValueExpected)
			{
				throw new ApplicationException("Semtech SX127X not found");
			}

			if (resetLogicalPinNumber != 0)
			{
				gpioController.OpenPin(resetLogicalPinNumber, PinMode.Output);
				gpioController.Write(resetLogicalPinNumber, PinValue.Low);

				Thread.Sleep(20);
				gpioController.Write(resetLogicalPinNumber, PinValue.High);
				Thread.Sleep(20);
			}

			// Interrupt pin for RX message & TX done notification 
			gpioController.OpenPin(interuptLogicalPinNumber, PinMode.InputPullDown);

			gpioController.RegisterCallbackForPinValueChangedEvent(interuptLogicalPinNumber, PinEventTypes.Rising, PinChangeEventHandler);
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

		public ushort ReadWordMsbLsb(byte registerAddress)
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

			return (ushort)((readBuffer[1] << 8) + readBuffer[2]);
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

		public void WriteWordMsbLsb(byte address, ushort value)
		{
			Span<byte> writeBuffer = stackalloc byte[3];
			Span<byte> readBuffer = stackalloc byte[writeBuffer.Length];

			if (SX127XTransceiver == null)
			{
				throw new ApplicationException("SX127XDevice is not initialised");
			}

			writeBuffer[0] = address |= RegisterAddressWriteMask;
			writeBuffer[1] = (byte)(value >> 8);
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

			for (byte registerIndex = (byte)Registers.MinValue; registerIndex <= (byte)Registers.MaxValue; registerIndex++)
			{
				byte registerValue = this.ReadByte(registerIndex);

				Debug.WriteLine($"Register 0x{registerIndex:x2} - Value 0X{registerValue:x2} - Bits {Convert.ToString(registerValue, 2).PadLeft(8, '0')}");
			}

			Debug.WriteLine("");
		}

		public void SetMode(RegOpModeMode mode)
		{
			byte regOpModeValue;

			regOpModeValue = RegOpModeLongRangeModeLoRa;
			regOpModeValue |= RegOpModeAcessSharedRegLoRa;
			regOpModeValue |= RegOpModeLowFrequencyModeOnHighFrequency;
			regOpModeValue |= (byte)mode;
			this.WriteByte((byte)Registers.RegOpMode, regOpModeValue);
		}

		public void Initialise(RegOpModeMode modeAfterInitialise, // RegOpMode
			double frequency = FrequencyDefault, // RegFrMsb, RegFrMid, RegFrLsb
			bool paBoost = false, byte maxPower = RegPAConfigMaxPowerDefault, byte outputPower = RegPAConfigOutputPowerDefault, // RegPaConfig
			bool ocpOn = true, byte ocpTrim = RegOcpOcpTrimDefault, // RegOcp
			RegLnaLnaGain lnaGain = LnaGainDefault, bool lnaBoostLF = false, bool lnaBoostHf = false, // RegLna
			RegModemConfigBandwidth bandwidth = RegModemConfigBandwidthDefault, RegModemConfigCodingRate codingRate = RegModemConfigCodingRateDefault, RegModemConfigImplicitHeaderModeOn implicitHeaderModeOn = RegModemConfigImplicitHeaderModeOnDefault, //RegModemConfig1
			RegModemConfig2SpreadingFactor spreadingFactor = RegModemConfig2SpreadingFactorDefault, bool txContinuousMode = false, bool rxPayloadCrcOn = false,
			ushort symbolTimeout = SymbolTimeoutDefault,
			ushort preambleLength = PreambleLengthDefault,
			byte payloadLength = PayloadLengthDefault,
			byte payloadMaxLength = PayloadMaxLengthDefault,
			byte freqHoppingPeriod = FreqHoppingPeriodDefault,
			bool lowDataRateOptimize = false, bool agcAutoOn = false,
			byte ppmCorrection = ppmCorrectionDefault,
			RegDetectOptimizeDectionOptimize detectionOptimize = RegDetectOptimizeDectionOptimizeDefault,
			bool invertIQ = false,
			RegisterDetectionThreshold detectionThreshold = RegisterDetectionThresholdDefault,
			byte syncWord = RegSyncWordDefault)
		{
			Frequency = frequency; // Store this away for RSSI adjustments
			RegOpModeModeCurrent = modeAfterInitialise;

			// Strobe Reset pin briefly to factory reset SX127X chip
			if (ResetLogicalPinNumber != 0)
			{
				gpioController.Write(ResetLogicalPinNumber, PinValue.Low);
				Thread.Sleep(20);
				gpioController.Write(ResetLogicalPinNumber, PinValue.High);
				Thread.Sleep(20);
			}

			// Put the device into sleep mode so registers can be changed
			SetMode(RegOpModeMode.Sleep);

			// Configure RF Carrier frequency 
			if (frequency != FrequencyDefault)
			{
				byte[] bytes = BitConverter.GetBytes((long)(frequency / RH_RF95_FSTEP));
				this.WriteByte((byte)Registers.RegFrMsb, bytes[2]);
				this.WriteByte((byte)Registers.RegFrMid, bytes[1]);
				this.WriteByte((byte)Registers.RegFrLsb, bytes[0]);
			}

			// Set RegPaConfig if any of the associated settings are nto the default values
			if ((paBoost != false) || (maxPower != RegPAConfigMaxPowerDefault) || (outputPower != RegPAConfigOutputPowerDefault))
			{
				byte regPaConfigValue = maxPower;
				if (paBoost)
				{
					regPaConfigValue |= RegPAConfigPASelectPABoost;
				}
				regPaConfigValue |= outputPower;
				this.WriteByte((byte)Registers.RegPAConfig, regPaConfigValue);
			}

			// Set RegOcp if any of the settings not defaults
			if ((ocpOn != true) || (ocpTrim != RegOcpOcpTrimDefault))
			{
				byte regOcpValue = ocpTrim;
				if (ocpOn)
				{
					regOcpValue |= RegOcpOn;
				}
				this.WriteByte((byte)Registers.RegOcp, regOcpValue);
			}

			// Set RegLna if any of the settings not defaults
			if ((lnaGain != LnaGainDefault) || (lnaBoostLF != false) || (lnaBoostHf != false))
			{
				byte regLnaValue = (byte)lnaGain;
				if (lnaBoostLF)
				{
					regLnaValue |= RegLnaLnaBoostLfOn;
				}
				if (lnaBoostHf)
				{
					regLnaValue |= RegLnaLnaBoostHfOn;
				}
				this.WriteByte((byte)Registers.RegLna, regLnaValue);
			}

			// Set regModemConfig1 if any of the settings not defaults
			if ((bandwidth != RegModemConfigBandwidthDefault) || (codingRate != RegModemConfigCodingRateDefault) || (implicitHeaderModeOn != RegModemConfigImplicitHeaderModeOnDefault))
			{
				byte regModemConfig1Value = (byte)bandwidth;
				regModemConfig1Value |= (byte)codingRate;
				regModemConfig1Value |= (byte)implicitHeaderModeOn;
				this.WriteByte((byte)Registers.RegModemConfig1, regModemConfig1Value);
			}

			// Set regModemConfig2 if any of the settings not defaults
			if ((spreadingFactor != RegModemConfig2SpreadingFactorDefault) || (txContinuousMode != false) | (rxPayloadCrcOn != false) || (symbolTimeout != SymbolTimeoutDefault))
			{
				byte RegModemConfig2Value = (byte)spreadingFactor;
				if (txContinuousMode)
				{
					RegModemConfig2Value |= RegModemConfig2TxContinuousModeOn;
				}
				if (rxPayloadCrcOn)
				{
					RegModemConfig2Value |= RegModemConfig2RxPayloadCrcOn;
				}
				// Get the MSB of SymbolTimeout
				byte[] symbolTimeoutBytes = BitConverter.GetBytes(symbolTimeout);
				RegModemConfig2Value |= symbolTimeoutBytes[1];
				this.WriteByte((byte)Registers.RegModemConfig2, RegModemConfig2Value);
			}

			// RegModemConfig2.SymbTimout + RegSymbTimeoutLsb
			if (symbolTimeout != SymbolTimeoutDefault)
			{
				// Get the LSB of SymbolTimeout
				byte[] symbolTimeoutBytes = BitConverter.GetBytes(symbolTimeout);
				this.WriteByte((byte)Registers.RegSymbTimeout, symbolTimeoutBytes[0]);
			}

			// RegPreambleMsb + RegPreambleLsb
			if (preambleLength != PreambleLengthDefault)
			{
				byte[] premableBytes = BitConverter.GetBytes(preambleLength);
				this.WriteByte((byte)Registers.RegPreambleMsb, premableBytes[1]);
				this.WriteByte((byte)Registers.RegPreambleLsb, premableBytes[0]);
			}

			// RegPayloadLength
			if (payloadLength != PayloadLengthDefault)
			{
				this.WriteByte((byte)Registers.RegPayloadLength, payloadLength);
			}

			// RegMaxPayloadLength
			if (payloadMaxLength != PayloadMaxLengthDefault)
			{
				this.WriteByte((byte)Registers.RegMaxPayloadLength, payloadMaxLength);
			}

			// RegHopPeriod
			if (freqHoppingPeriod != FreqHoppingPeriodDefault)
			{
				this.WriteByte((byte)Registers.RegHopPeriod, freqHoppingPeriod);
			}

			// RegModemConfig3
			if ((lowDataRateOptimize != false) || (agcAutoOn != false))
			{
				byte regModemConfig3Value = 0;
				if (lowDataRateOptimize)
				{
					regModemConfig3Value |= RegModemConfig3LowDataRateOptimizeOn;
				}
				if (agcAutoOn)
				{
					regModemConfig3Value |= RegModemConfig3AgcAutoOn;
				}
				this.WriteByte((byte)Registers.RegModemConfig3, regModemConfig3Value);
			}

			// RegPpmCorrection
			if (ppmCorrection != ppmCorrectionDefault)
			{
				this.WriteByte((byte)Registers.RegPpmCorrection, ppmCorrection);
			}

			// RegDetectOptimize
			if (detectionOptimize != RegDetectOptimizeDectionOptimizeDefault)
			{
				this.WriteByte((byte)Registers.RegDetectOptimize, (byte)detectionOptimize);
			}

			// RegInvertIQ
			if (invertIQ != false)
			{
				this.WriteByte((byte)Registers.RegDetectionThreshold, (byte)detectionThreshold);
			}

			// RegSyncWordDefault 
			if (syncWord != RegSyncWordDefault)
			{
				this.WriteByte((byte)Registers.RegSyncWord, syncWord);
			}

			// TODO revist this split & move to onReceive function
			this.WriteByte(0x40, 0b00000000); // RegDioMapping1 0b00000000 DI0 RxReady & TxReady

			// Configure RegOpMode before returning
			SetMode(modeAfterInitialise);
		}


		private void PinChangeEventHandler(object sender, PinValueChangedEventArgs pinValueChangedEventArgs)
		{
			if (pinValueChangedEventArgs.ChangeType != PinEventTypes.Rising)
			{
				return;
			}

			RegIrqFlagsMask IrqFlags = (RegIrqFlagsMask)this.ReadByte((byte)Registers.RegIrqFlags);
			Debug.WriteLine(string.Format("RegIrqFlags {0}", Convert.ToString((byte)IrqFlags, 2).PadLeft(8, '0')));

			if ((IrqFlags & RegIrqFlagsMask.RxDoneMask) == RegIrqFlagsMask.RxDoneMask)  // RxDone 
			{
				Debug.WriteLine("RX-Done");
				byte currentFifoAddress = this.ReadByte((byte)Registers.RegFifoRxCurrent); // RegFifoRxCurrent
				this.WriteByte((byte)Registers.RegFifoAddrPtr, currentFifoAddress); // RegFifoAddrPtr

				byte numberOfBytes = this.ReadByte(0x13); // RegRxNbBytes

				// Allocate buffer for message
				byte[] messageBytes = new byte[numberOfBytes];

				for (int i = 0; i < numberOfBytes; i++)
				{
					messageBytes[i] = this.ReadByte((byte)Registers.RegFifo);
				}

				string messageText = UTF8Encoding.UTF8.GetString(messageBytes);
				Debug.WriteLine("Received {0} byte message {1}", messageBytes.Length, messageText);
			}

			if ((IrqFlags & RegIrqFlagsMask.TxDoneMask) == RegIrqFlagsMask.TxDoneMask)
			{
				Debug.WriteLine("TX-Done");
				SetMode(RegOpModeModeCurrent);
			}

			// TODO need to set this based on RX/TX mode handlers. Set based on OnTransmit, onReceive != null
			this.WriteByte((byte)Registers.RegDioMapping1, 0b00000000); // RegDioMapping1 0b00000000 DI0 RxReady & TxReady
			this.WriteByte((byte)Registers.RegIrqFlags, (byte)RegIrqFlags.ClearAll);
		}

		public void Send(byte[] messageBytes)
		{
			// Set the Register Fifo address pointer
			this.WriteByte((byte)Registers.RegFifoTxBaseAddr, 0x00); // RegFifoTxBaseAddress 

			// Set the Register Fifo address pointer
			this.WriteByte((byte)Registers.RegFifoAddrPtr, 0x0); // RegFifoAddrPtr 

			// load the message into the fifo
			this.WriteBytes((byte)Registers.RegFifo, messageBytes);

			// Set the length of the message in the fifo
			this.WriteByte((byte)Registers.RegPayloadLength, (byte)messageBytes.Length); // RegPayloadLength

			// TODO need to set this based on RX/TX mode handlers. Set based on OnTransmit, onReceive != null
			this.WriteByte((byte)Registers.RegDioMapping1, (byte)RegDioMapping1.Dio0TxDone); // RegDioMapping1 0b00000000 DI0 RxReady & TxReady

			this.SetMode(RegOpModeMode.Transmit);
		}
	}

	class Program
	{
		static void Main(string[] args)
		{
			int messageCount = 1;

#if ADAFRUIT_RADIO_BONNET
			SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, interuptLogicalPinNumber: 22, resetLogicalPinNumber: 25);
#endif

#if DRAGINO_CS0
			SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS0, interuptLogicalPinNumber: 4, chipSelectLogicalPinNumber: 25, resetLogicalPinNumber: 17);
#endif

#if DRAGINO_CS1
			SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, interuptLogicalPinNumber: 4, chipSelectLogicalPinNumber: 25, resetLogicalPinNumber: 17);
#endif

#if ELECROW
			SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, interuptLogicalPinNumber: 25, resetLogicalPinNumber: 22);
#endif

#if ELECTRONIC_TRICKS
			SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS0, interuptLogicalPinNumber: 25, resetLogicalPinNumber: 22);
#endif

#if M2M_CS0
			SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS0, interuptLogicalPinNumber: 4, chipSelectLogicalPinNumber: 25, resetLogicalPinNumber: 17);
#endif

#if M2M_CS1
			SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, interuptLogicalPinNumber: 4, chipSelectLogicalPinNumber: 25, resetLogicalPinNumber: 17);
#endif

			// Uptronics RPI Zero has no reset pin has switch selectable CS0 or CS1
#if UPUTRONICS_RPIZERO_CS0
			SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS0, 25);
#endif
#if UPUTRONICS_RPIZERO_CS1
			SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, 16);
#endif

			// Uptronics RPI Plus can have two devices one on CS0 the other on CS1
#if UPUTRONICS_RPIPLUS_CS0 && !UPUTRONICS_RPIPLUS_CS1
			SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS0, 25);
#endif
#if UPUTRONICS_RPIPLUS_CS1 && UPUTRONICS_RPIPLUS_CS0
			SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, 16);
#endif

			sX127XDevice.Initialise(SX127XDevice.RegOpModeMode.ReceiveContinuous, 915000000.0, paBoost:true);

			sX127XDevice.RegisterDump();

			while (true)
			{
				string messageText = "Hello LoRa from .NET Core! " + messageCount.ToString();
				messageCount += 1; 

				byte[] messageBytes = UTF8Encoding.UTF8.GetBytes(messageText);
				Debug.WriteLine("Sending {0} bytes message {1}", messageBytes.Length, messageText);
				sX127XDevice.Send(messageBytes);

				Thread.Sleep(50000);
			}
		}
	}
}

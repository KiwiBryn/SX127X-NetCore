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
using System.Text;
using System.Threading;

using devMobile.IoT.SX127xLoRaDevice;

namespace devMobile.IoT.SX127XLoRaDeviceClient
{
	class Program
	{
#if ADAFRUIT_RADIO_BONNET
		private readonly static SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, interuptLogicalPinNumber: 22, resetLogicalPinNumber: 25);
#endif

#if DRAGINO_CS0
		private readonly static SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS0, interuptLogicalPinNumber: 4, chipSelectLogicalPinNumber: 25, resetLogicalPinNumber: 17);
#endif

#if DRAGINO_CS1
		private readonly static SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, interuptLogicalPinNumber: 4, chipSelectLogicalPinNumber: 25, resetLogicalPinNumber: 17);
#endif

#if ELECROW
		private readonly static SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, interuptLogicalPinNumber: 25, resetLogicalPinNumber: 22);
#endif


#if M2M_CS0
		private readonly static SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS0, interuptLogicalPinNumber: 4, chipSelectLogicalPinNumber: 25, resetLogicalPinNumber: 17);
#endif

#if M2M_CS1
		private readonly static SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, interuptLogicalPinNumber: 4, chipSelectLogicalPinNumber: 25, resetLogicalPinNumber: 17);
#endif

		// Uptronics RPI Zero has no reset pin has switch selectable CS0 or CS1
#if UPUTRONICS_RPIZERO_CS0
		private readonly static SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS0, 25);
#endif
#if UPUTRONICS_RPIZERO_CS1
		private readonly static SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, 16);
#endif

		// Uptronics RPI Plus can have two devices one on CS0 the other on CS1
#if UPUTRONICS_RPIPLUS_CS0 && !UPUTRONICS_RPIPLUS_CS1
		private readonly static SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS0, 25);
#endif
#if UPUTRONICS_RPIPLUS_CS1 && UPUTRONICS_RPIPLUS_CS0
		private readonly static SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, 16);
#endif

		static void Main(string[] args)
		{
			int messageCount = 1;

			sX127XDevice.Initialise(
					SX127XDevice.RegOpModeMode.ReceiveContinuous,
					915000000.0,
					paBoost: true,
#if LORA_SENDER // From the Arduino point of view
					rxDoneignoreIfCrcMissing: false
#endif
#if LORA_RECEIVER // From the Arduino point of view
					invertIQTX: true
#endif

#if LORA_SET_SYNCWORD
					syncWord: 0xF3,
					invertIQTX: true,
					rxDoneignoreIfCrcMissing: false
#endif
#if LORA_SET_SPREAD
					spreadingFactor: SX127XDevice.RegModemConfig2SpreadingFactor._256ChipsPerSymbol,
					invertIQTX: true,
					rxDoneignoreIfCrcMissing: false
#endif
#if LORA_SIMPLE_NODE // From the Arduino point of view
					rxDoneignoreIfCrcMissing: false
#endif
#if LORA_SIMPLE_GATEWAY // From the Arduino point of view
					invertIQTX: true,
					invertIQRX: true,
					rxDoneignoreIfCrcMissing: false
#endif
#if LORA_DUPLEX
					invertIQTX: true,
					rxPayloadCrcOn: true
#endif
					);

#if DEBUG
			sX127XDevice.RegisterDump();
#endif

#if !LORA_RECEIVER
			sX127XDevice.OnReceive += SX127XDevice_OnReceive;
			sX127XDevice.Receive();
#endif
#if !LORA_SENDER
			sX127XDevice.OnTransmit += SX127XDevice_OnTransmit;
#endif

#if LORA_SENDER
			Thread.Sleep(-1);
#else
			Thread.Sleep(5000);
#endif

			while (true)
			{
				string messageText = "Hello LoRa from .NET Core! " + messageCount.ToString();

#if LORA_DUPLEX
				byte[] messageBytes = new byte[messageText.Length+4];

				messageBytes[0] = 0xaa;
				messageBytes[1] = 0x00;
				messageBytes[2] = (byte)messageCount;
				messageBytes[3] = (byte)messageText.Length;

				Array.Copy(UTF8Encoding.UTF8.GetBytes(messageText), 0, messageBytes, 4, messageBytes[3]);

				Console.WriteLine($"{DateTime.Now:HH:mm:ss}-TX to 0x{messageBytes[0]:X2} from 0x{messageBytes[1]:X2} count {messageBytes[2]} length {messageBytes[3]} \"{messageText}\"");
#else
				byte[] messageBytes = UTF8Encoding.UTF8.GetBytes(messageText);

				Console.WriteLine($"{DateTime.Now:HH:mm:ss}- Length {messageBytes.Length} \"{messageText}\""); 
#endif

				messageCount += 1;

				sX127XDevice.Send(messageBytes);

				Thread.Sleep(10000);
			}
		}

		private static void SX127XDevice_OnReceive(object sender, SX127XDevice.OnDataReceivedEventArgs e)
		{
			string messageText;

#if LORA_DUPLEX
			if ((e.Data[0] != 0x00) && (e.Data[0] != 0xFF))
			{
#if DEBUG
				Console.WriteLine($"{DateTime.UtcNow:hh:mm:ss}-RX to 0x{e.Data[0]:X2} from 0x{e.Data[1]:X2} invalid address");
#endif
				return;
			}

			// check payload not to long/short
			if  ((e.Data[3] + 4) != e.Data.Length)
			{
				Console.WriteLine($"{DateTime.UtcNow:hh:mm:ss}-RX Invalid payload");

				return;
			}

			try
			{
				messageText = UTF8Encoding.UTF8.GetString(e.Data, 4, e.Data[3]);

				Console.WriteLine($"{DateTime.Now:HH:mm:ss}-RX to 0x{e.Data[0]:X2} from 0x{e.Data[1]:X2} count {e.Data[2]} length {e.Data[3]} \"{messageText}\" snr {e.PacketSnr:0.0} packet rssi {e.PacketRssi}dBm rssi {e.Rssi}dBm ");
			}
			catch (Exception ex)
			{
				Console.WriteLine(ex.Message);
			}
#else
			try
			{
				messageText = UTF8Encoding.UTF8.GetString(e.Data);

				Console.WriteLine($"{DateTime.Now:HH:mm:ss}-RX length {e.Data[3]} \"{messageText}\" snr {e.PacketSnr:0.0} packet rssi {e.PacketRssi}dBm rssi {e.Rssi}dBm ");
			}
			catch (Exception ex)
			{
				Console.WriteLine(ex.Message);
			}
#endif
		}

		private static void SX127XDevice_OnTransmit(object sender, SX127XDevice.OnDataTransmitedEventArgs e)
		{
#if LORA_RECEIVER
			sX127XDevice.SetMode(SX127XDevice.RegOpModeMode.StandBy);
#else
			sX127XDevice.SetMode(SX127XDevice.RegOpModeMode.ReceiveContinuous);
#endif
			Console.WriteLine($"{DateTime.Now:HH:mm:ss}-TX Done");
		}
	}
}





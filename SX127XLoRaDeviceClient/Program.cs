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
	namespace devMobile.IoT.SX127XLoRaDeviceClient
	{
		class Program
		{
			private static SX127XDevice sX127XDevice;

			static void Main(string[] args)
			{
				const double Frequency = 915000000.0;
				int messageCount = 1;

#if ADAFRUIT_RADIO_BONNET
				sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, interuptLogicalPinNumber: 22, resetLogicalPinNumber: 25);
#endif

#if DRAGINO_CS0 // Y
				sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS0, interuptLogicalPinNumber: 4, chipSelectLogicalPinNumber: 25, resetLogicalPinNumber: 17);
#endif

#if DRAGINO_CS1 // Y
				sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, interuptLogicalPinNumber: 4, chipSelectLogicalPinNumber: 25, resetLogicalPinNumber: 17);
#endif

#if ELECROW
				sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, interuptLogicalPinNumber: 25, resetLogicalPinNumber: 22);
#endif

#if ELECTRONIC_TRICKS
				SX127XDevice sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS0, interuptLogicalPinNumber: 25, resetLogicalPinNumber: 22);
#endif

#if M2M_CS0
				sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS0, interuptLogicalPinNumber: 4, chipSelectLogicalPinNumber: 25, resetLogicalPinNumber: 17);
#endif

#if M2M_CS1
				sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, interuptLogicalPinNumber: 4, chipSelectLogicalPinNumber: 25, resetLogicalPinNumber: 17);
#endif

				// Uptronics RPI Zero has no reset pin has switch selectable CS0 or CS1
#if UPUTRONICS_RPIZERO_CS0
				sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS0, 25);
#endif
#if UPUTRONICS_RPIZERO_CS1
				sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, 16);
#endif

				// Uptronics RPI Plus can have two devices one on CS0 the other on CS1
#if UPUTRONICS_RPIPLUS_CS0 && !UPUTRONICS_RPIPLUS_CS1
				sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS0, 25);
#endif
#if UPUTRONICS_RPIPLUS_CS1 && UPUTRONICS_RPIPLUS_CS0
				sX127XDevice = new SX127XDevice(SX127XDevice.ChipSelectLine.CS1, 16);
#endif

				sX127XDevice.Initialise(SX127XDevice.RegOpModeMode.ReceiveContinuous, 
					Frequency,
					powerAmplifier: SX127XDevice.PowerAmplifier.PABoost, 
					rxPayloadCrcOn: true, 
					rxDoneignoreIfCrcMissing: false
					);

#if DEBUG
				sX127XDevice.RegisterDump();
#endif

				sX127XDevice.OnReceive += SX127XDevice_OnReceive;
				sX127XDevice.Receive();
				sX127XDevice.OnTransmit += SX127XDevice_OnTransmit;

				Thread.Sleep(10000);

				while (true)
				{
					string messageText = "Hello LoRa from .NET Core! " + messageCount.ToString();
					messageCount += 1;

					byte[] messageBytes = UTF8Encoding.UTF8.GetBytes(messageText);
					Console.WriteLine("Sending {0} bytes message {1}", messageBytes.Length, messageText);
					sX127XDevice.Send(messageBytes);

					Thread.Sleep(50000);
				}
			}

			private static void SX127XDevice_OnReceive(object sender, SX127XDevice.OnDataReceivedEventArgs e)
			{
				try
				{
					string messageText = UTF8Encoding.UTF8.GetString(e.Data);

					Console.WriteLine(@"{0:HH:mm:ss}-RX PacketSnr {1:0.0} Packet RSSI {2}dBm RSSI {3}dBm = {4} byte message ""{5}""", DateTime.Now, e.PacketSnr, e.PacketRssi, e.Rssi, e.Data.Length, messageText);
				}
				catch (Exception ex)
				{
					Console.WriteLine(ex.Message);
				}
			}

			private static void SX127XDevice_OnTransmit(object sender, SX127XDevice.OnDataTransmitedEventArgs e)
			{
				sX127XDevice.SetMode(SX127XDevice.RegOpModeMode.ReceiveContinuous);

				Console.WriteLine("{0:HH:mm:ss}-TX Done", DateTime.Now);
			}
		}
	}
}

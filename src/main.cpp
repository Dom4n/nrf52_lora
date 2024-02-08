// ADAPTED FROM
// https://github.dev/beegee-tokyo/SX126x-Arduino/tree/master/examples/PingPong

#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial
#include <Adafruit_SPIFlash.h>

#include <SX126x-Arduino.h>
#include <SPI.h>

hw_config hwConfig;

// nRF52832 - SX126x pin configuration
int PIN_LORA_RESET = D0;  // LORA RESET
int PIN_LORA_DIO_1 = D2;  // LORA DIO_1
int PIN_LORA_BUSY = D1;   // LORA SPI BUSY
int PIN_LORA_NSS = D5;    // LORA SPI CS
int PIN_LORA_SCLK = SCK;  // LORA SPI CLK
int PIN_LORA_MISO = MISO; // LORA SPI MISO
int PIN_LORA_MOSI = MOSI; // LORA SPI MOSI
int RADIO_TXEN = D3;      // LORA ANTENNA TX ENABLE
int RADIO_RXEN = D4;      // LORA ANTENNA RX ENABLE
// Replace PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_MOSI with your
SPIClass SPI_LORA(NRF_SPIM2, PIN_LORA_MISO, PIN_LORA_SCLK, PIN_LORA_MOSI);

#define RF_FREQUENCY 869525000  // Hz
#define TX_OUTPUT_POWER 10      // dBm
#define LORA_BANDWIDTH 0        // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1       // [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 3000
#define TX_TIMEOUT_VALUE 3000

#define BUFFER_SIZE 64 // Define the payload size here

static RadioEvents_t RadioEvents;
static uint16_t BufferSize = BUFFER_SIZE;
static uint8_t RcvBuffer[BUFFER_SIZE];
static uint8_t TxdBuffer[BUFFER_SIZE];
const uint8_t LoraMsg[] = "PING PONG";

time_t timeToSend;

time_t cadTime;

// Function declarations
void OnTxDone(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnTxTimeout(void);
void OnRxTimeout(void);
void OnRxError(void);
void OnCadDone(bool cadResult);

Adafruit_FlashTransport_QSPI flashTransport;

void setup()
{
  NRF_POWER->DCDCEN = 1;
  flashTransport.begin();
  flashTransport.runCommand(0xB9);  // enter deep power-down mode
  delayMicroseconds(5);             // tDP=3uS
  flashTransport.end();

  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, LOW);

  // pinMode(RADIO_RXEN, OUTPUT);
  // digitalWrite(RADIO_RXEN, HIGH);

  hwConfig.CHIP_TYPE = SX1262_CHIP;         // Example uses an eByte E22 module with an SX1262
  hwConfig.PIN_LORA_RESET = PIN_LORA_RESET; // LORA RESET
  hwConfig.PIN_LORA_NSS = PIN_LORA_NSS;     // LORA SPI CS
  hwConfig.PIN_LORA_SCLK = PIN_LORA_SCLK;   // LORA SPI CLK
  hwConfig.PIN_LORA_MISO = PIN_LORA_MISO;   // LORA SPI MISO
  hwConfig.PIN_LORA_DIO_1 = PIN_LORA_DIO_1; // LORA DIO_1
  hwConfig.PIN_LORA_BUSY = PIN_LORA_BUSY;   // LORA SPI BUSY
  hwConfig.PIN_LORA_MOSI = PIN_LORA_MOSI;   // LORA SPI MOSI
  hwConfig.RADIO_TXEN = RADIO_TXEN;         // LORA ANTENNA TX ENABLE
  hwConfig.RADIO_RXEN = RADIO_RXEN;         // LORA ANTENNA RX ENABLE
  hwConfig.USE_DIO2_ANT_SWITCH = true;     // Example uses an CircuitRocks Alora RFM1262 which uses DIO2 pins as antenna control
  hwConfig.USE_DIO3_TCXO = true;            // Example uses an CircuitRocks Alora RFM1262 which uses DIO3 to control oscillator voltage
  hwConfig.USE_DIO3_ANT_SWITCH = false;     // Only Insight ISP4520 module uses DIO3 as antenna control
  hwConfig.USE_RXEN_ANT_PWR = false;        // RXEN is used as power for antenna switch
  hwConfig.USE_LDO = false;                 // Set to 'true' if you want to use the LDO. Set to 'false' if you want to use the DC/DC converter

  Serial.begin(57600);
  int serial_retries = 10;
  while (!Serial)
  {
    delay(100);
    serial_retries--;
    if (serial_retries <= 0) {
      break;
    }
  };

  dbgPrintVersion();

  lora_hardware_init(hwConfig);

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
  RadioEvents.CadDone = OnCadDone;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

  // Set Radio RX configuration
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  // Start LoRa
  // Serial.println("Starting Radio.Rx");
  // Radio.Rx(RX_TIMEOUT_VALUE);
  // Radio.Standby();
  // Radio.SetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10, LORA_CAD_ONLY, 0);
  // cadTime = millis();
  // Radio.StartCad();

  Serial.println("setup done");
  // Radio.Rx(RX_TIMEOUT_VALUE);
  // Radio.IrqProcessAfterDeepSleep();
  // delay(200);
}

void loop()
{
  // Serial.println("Loop start");

  // // SX126xSetRx(500);
  // delay(10);
  // Serial.print("SX126xGetOperatingMode: ");
  // Serial.println(SX126xGetOperatingMode());
  // Serial.print("SX126xGetRandom: ");
  // Serial.println(SX126xGetRandom());

  // // read_registers(0x300, 0x9FF);

  // Serial.print("getDeviceErrors: ");
  // RadioError_t err = SX126xGetDeviceErrors();
  // // 		uint8_t Rc64kCalib : 1; //!< RC 64kHz oscillator calibration failed
  // // 		uint8_t Rc13mCalib : 1; //!< RC 13MHz oscillator calibration failed
  // // 		uint8_t PllCalib : 1;	//!< PLL calibration failed
  // // 		uint8_t AdcCalib : 1;	//!< ADC calibration failed
  // // 		uint8_t ImgCalib : 1;	//!< Image calibration failed
  // // 		uint8_t XoscStart : 1;	//!< XOSC oscillator failed to start
  // // 		uint8_t PllLock : 1;	//!< PLL lock failed
  // // 		uint8_t BuckStart : 1;	//!< Buck converter failed to start
  // // 		uint8_t PaRamp : 1;		//!< PA ramp failed
  // // 		uint8_t : 7;			//!< Reserved
  // Serial.println(err.Value, HEX);
  // Serial.print("Rc64kCalib: ");
  // Serial.println(err.Fields.Rc64kCalib);
  // Serial.print("Rc13mCalib: ");
  // Serial.println(err.Fields.Rc13mCalib);
  // Serial.print("PllCalib: ");
  // Serial.println(err.Fields.PllCalib);
  // Serial.print("AdcCalib: ");
  // Serial.println(err.Fields.AdcCalib);
  // Serial.print("ImgCalib: ");
  // Serial.println(err.Fields.ImgCalib);
  // Serial.print("XoscStart: ");
  // Serial.println(err.Fields.XoscStart);
  // Serial.print("PllLock: ");
  // Serial.println(err.Fields.PllLock);
  // Serial.print("BuckStart: ");
  // Serial.println(err.Fields.BuckStart);
  // Serial.print("PaRamp: ");
  // Serial.println(err.Fields.PaRamp);

  // Serial.println("Loop end\n");

  // Serial.println(SX126xGetIrqStatus());
  // Radio.IrqProcess();
  Radio.SetRxDutyCycle(1280, 128);
  delay(1000);
  Radio.IrqProcessAfterDeepSleep();
  delay(500);
  Radio.Standby();
  // SX126xSetRx(RX_TIMEOUT_VALUE);
  delay(1500);
  Radio.Sleep();
  delay(500);
}

void read_registers(uint16_t start_addr, uint16_t end_addr)
{
  // read registers from sx126x
  uint8_t buf[16];
  Serial.print(F("Register         0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"));
  Serial.println();
  uint16_t addr = start_addr;
  for (; addr <= end_addr; addr += 16)
  {
    Serial.print("Register 0x");
    Serial.print(addr, HEX);
    Serial.print(": ");

    SX126xReadRegisters(addr, buf, sizeof(buf));

    for (uint16_t i = 0; i < sizeof(buf); i++)
    {
      if (buf[i] < 0x10)
      {
        Serial.print("0");
      }
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

/**@brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void)
{
  Serial.println("OnTxDone");
  // Radio.Rx(RX_TIMEOUT_VALUE);
  // SX126xSetRx(RX_TIMEOUT_VALUE);
}

/**@brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  Serial.println("OnRxDone");
  delay(10);
  BufferSize = size;
  memcpy(RcvBuffer, payload, BufferSize);

  Serial.printf("RssiValue=%d dBm, SnrValue=%d\n", rssi, snr);

  for (int idx = 0; idx < size; idx++)
  {
    Serial.printf("%02X ", RcvBuffer[idx]);
  }
  Serial.println("");

  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("Received a SOMETHING in OnRxDone as Master");

  // Wait 500ms before sending the next package
  delay(500);

  // Check if our channel is available for sending
  Radio.Standby();
  Radio.SetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10, LORA_CAD_ONLY, 0);
  cadTime = millis();
  Radio.StartCad();
  // Sending next Ping will be started when the channel is free
}

/**@brief Function to be executed on Radio Tx Timeout event
 */
void OnTxTimeout(void)
{
  // Radio.Sleep();
  Serial.println("OnTxTimeout");
  digitalWrite(LED_BUILTIN, LOW);

  Radio.Rx(RX_TIMEOUT_VALUE);
  // SX126xSetRx(RX_TIMEOUT_VALUE);
}

/**@brief Function to be executed on Radio Rx Timeout event
 */
void OnRxTimeout(void)
{
  Serial.println("OnRxTimeout");

  digitalWrite(LED_BUILTIN, LOW);

  // Wait 500ms before sending the next package
  delay(500);

  // Check if our channel is available for sending
  Radio.Standby();
  Radio.SetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10, LORA_CAD_ONLY, 0);
  cadTime = millis();
  Radio.StartCad();
  // Sending the ping will be started when the channel is free
}

/**@brief Function to be executed on Radio Rx Error event
 */
void OnRxError(void)
{
  Serial.println("OnRxError");
  // digitalWrite(LED_BUILTIN, LOW);

  // Wait 500ms before sending the next package
  delay(500);

  // Check if our channel is available for sending
  Radio.Standby();
  Radio.SetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10, LORA_CAD_ONLY, 0);
  cadTime = millis();
  Radio.StartCad();
  // Sending the ping will be started when the channel is free
}

/**@brief Function to be executed on CAD Done event
 */
void OnCadDone(bool cadResult)
{
  time_t duration = millis() - cadTime;
  if (cadResult)
  {
    Serial.printf("CAD returned channel busy after %ldms\n", duration);
  Radio.Rx(RX_TIMEOUT_VALUE);
  // SX126xSetRx(RX_TIMEOUT_VALUE);
  }
  else
  {
    Serial.printf("CAD returned channel free after %ldms\n", duration);
    Serial.println("Sending a PING in OnCadDone as Master");
    // Send the next PING frame
    TxdBuffer[0] = 'P';
    TxdBuffer[1] = 'I';
    TxdBuffer[2] = 'N';
    TxdBuffer[3] = 'G';
    TxdBuffer[4] = ' ';
    TxdBuffer[5] = 'P';
    TxdBuffer[6] = 'O';
    TxdBuffer[7] = 'N';
    TxdBuffer[8] = 'G';
    // We fill the buffer with numbers for the payload
    for (int i = 9; i < BufferSize; i++)
    {
      TxdBuffer[i] = i - 9;
    }

    Radio.Send(TxdBuffer, BufferSize);
  }
}

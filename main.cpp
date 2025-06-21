
#include <meFDCAN.h>

#define LED PB15
#define CAN_SD_PIN PA9
#define CAN_IO_PIN PA10

HardwareSerial Serial1(PC11, PC10);

bool sendCan()
{
  bool success = false;
  uint8_t TxData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  int txId = 200;
  //--- CAN Transmit, using defaults of CAN1,len=8
  success = meFDCAN_transmit(txId, TxData);
  return (success);
}

void reportCAN(int canport, int id, int len, uint8_t data[])
{
  Serial1.print("CAN port = ");
  Serial1.print(canport);
  Serial1.print(" Rx <id> ");
  Serial1.println(id);
  for (int i = len - 1; i > -1; i--)
  {
    Serial1.print(data[i]);
    Serial1.print(" ");
  }
  Serial1.println();
}

int checkMessages()
{
  uint8_t len;
  uint32_t id;
  uint8_t rxdata[8];

  /**
   * @brief   Receive data over the FDCAN module
   *
   * @param   * id      The ID of the received data
   * @param   * len     The length of the received data
   * @param   * rxData  The received data
   * @param   canPort   The port number for the FDCAN module (default = 1)
   * @param   * type    The type of the received message ID(default = nullptr)
   * @param   fifo      The FIFO number to use for receiving (default = 0)
   *
   * @return  The Qty in FIFO
   */

  int qty = meFDCAN_receive(&id, &len, rxdata); //--- using defaults of COM1
  if (qty < 0)
    return -1;
  if (qty > 0)
    reportCAN(1, id, len, rxdata);
  return (qty);
}

void setup()
{
  pinMode(LED, OUTPUT);
  Serial1.begin(9600);
  while (!Serial1)
    ;
  Serial1.println("Started");
  pinMode(CAN_SD_PIN, OUTPUT);
  pinMode(CAN_IO_PIN, OUTPUT);

  // Drive CAN_SD_PIN HIGH to put TJA1042 in Normal mode
  digitalWrite(CAN_SD_PIN, LOW);
  digitalWrite(CAN_IO_PIN, HIGH);

  delay(2000);

  Serial1.println("Attempting FDCAN init...");
  bool init_success = meFDCAN_init(125, 1, PB8, PB9);
  Serial1.print("meFDCAN_init result: ");
  Serial1.println(init_success); // This will print 1 for true, 0 for false
  delay(2000);
}

void loop()
{
  digitalToggle(LED);
  //--- Send a CAN message
  bool result = sendCan();
  Serial1.print("Transmit = ");
  Serial1.println(result);
  //--- Receive CAN message
  int cnt = checkMessages();
  Serial1.print("Receive = ");
  Serial1.println(cnt);
  delay(1000);
}


#include <meFDCAN.h>

#define LED PB9   // QFP48 socket PCB

HardwareSerial Serial1(PA10, PA9);


void setup() 
{
  pinMode(LED,OUTPUT);
  Serial1.begin(9600);

  //----  CAN init:  using defaults,125 (125kbs),CAN1,Rx=PA11,Tx=PA12

  meFDCAN_init();  
  
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

bool sendCan()
{
  bool success = false;
  uint8_t TxData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  int txId = 200;
  //--- CAN Transmit, using defaults of CAN1,len=8
  success = meFDCAN_transmit(txId,TxData);
  return (success);
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

  int qty = meFDCAN_receive(&id,&len,rxdata);//--- using defaults of COM1
  if (qty < 0) return -1;
  if (qty > 0) reportCAN(1,id,len,rxdata);
  return (qty);
}

void reportCAN(int canport,int id,int len,uint8_t data[])
{
  Serial1.print("CAN port = ");
  Serial1.print(canport);
  Serial1.print(" Rx <id> ");
  Serial1.println(id);
  for (int i=len-1;i>-1;i--)
  {
    Serial1.print(data[i]);Serial1.print(" ");
  }
  Serial1.println();
}

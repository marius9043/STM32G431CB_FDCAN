#include "meClock_G473.h"
#include <meFDCAN.h>

#define LED PB9   // QFP48 socket PCB

HardwareSerial Serial1(PA10, PA9);


void setup() 
{
  pinMode(LED,OUTPUT);
  Serial1.begin(115200);

  meFDCAN_init();
  meFDCAN_init(125,2,PB12,PB13);
  meFDCAN_init(125,3,PB3,PB4);
  
}

void loop() 
{
  int cnt;
  bool result;
  digitalToggle(LED);
  //--- Send a CAN message
  result = sendCan1();
  if (!result){Serial1.print("Transmit = ");  Serial1.println(result);}
  //--- Receive CAN message
  cnt = checkMessages();
  delay(20);   // delay for serial output
  cnt = checkMessages2();
  delay(20);
  cnt = checkMessages3();
  delay(200);
  result = sendCan2();
  delay(10);
  cnt = checkMessages();
  delay(20);
  cnt = checkMessages2();
  delay(20);
  cnt = checkMessages3();
  delay(200);
  result = sendCan3();
  delay(10);
  cnt = checkMessages();
  delay(20);
  cnt = checkMessages2();
  delay(20);
  cnt = checkMessages3();
  delay(460);
}

bool sendCan1()
{
  bool success = false;
  uint8_t TxData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  int txId = 200;
  success = meFDCAN_transmit(txId,TxData);
  return (success);
}

bool sendCan2()
{
  bool success = false;
  uint8_t TxData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  int txId = 400;
  success = meFDCAN_transmit(txId,TxData,2);
  return (success);
}

bool sendCan3()
{
  bool success = false;
  uint8_t TxData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  int txId = 600;
  success = meFDCAN_transmit(txId,TxData,3);
  return (success);
}

int checkMessages()
{
  uint8_t len;
  uint32_t id;
  uint8_t rxdata[8];
  int qty = meFDCAN_receive(&id,&len,rxdata);
  if (qty < 0) return -1;
  if (qty > 0) reportCAN(1,id,len,rxdata);
  return (qty);
}

int checkMessages2()
{
  uint8_t len;
  uint32_t id;
  uint8_t rxdata[8];
  int canPort = 2;
  int qty = meFDCAN_receive(&id,&len,rxdata,canPort);
  if (qty < 0) return -1;
  if (qty > 0) reportCAN(canPort,id,len,rxdata);
  return (qty);
}

int checkMessages3()
{
  uint8_t len;
  uint32_t id;
  uint8_t rxdata[8];
  int canPort = 3;
  int qty = meFDCAN_receive(&id,&len,rxdata,canPort);
  if (qty < 0) return -1;
  if (qty > 0) reportCAN(canPort,id,len,rxdata);
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

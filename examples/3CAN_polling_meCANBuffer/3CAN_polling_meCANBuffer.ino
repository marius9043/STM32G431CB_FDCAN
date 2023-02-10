//#include "meClock_G473.h"
#include <meFDCAN.h>
#include <meCANbuffer.h>

#define LED PB9   // QFP48 socket PCB

HardwareSerial Serial1(PA10, PA9);
meCANBuffer canBuf;

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
  sendCan1();
  delay(10);
  checkMessages();
  sendCan2();
  delay(10);
  checkMessages();
  sendCan3();
  delay(10);
  checkMessages();
  delay(100);
  //--- Show Receive CAN message from meCANBuffer
  while(1)
  {
    CAN_Message_t bdata;
    bdata = canBuf.pop();
    if (bdata.port == 0) break;
    reportCAN(bdata.port,bdata.id,bdata.length,bdata.data);
  }
  delay(800);
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

void checkMessages()
{
  while (checkMessages1() > 0 ) {;}
  while (checkMessages2() > 0) {;}
  while (checkMessages3() > 0) {;}
}

int checkMessages1()
{
  uint8_t len;
  uint32_t id;
  uint8_t rxdata[8];
  int qty = meFDCAN_receive(&id,&len,rxdata);
  if (qty < 0) return -1;
  if (qty == 0) return 0;
  
  CAN_Message_t bdata;
  bdata.id= (uint16_t) id;
  bdata.port=1;
  bdata.length = len;
  memcpy(bdata.data,rxdata,8);
  canBuf.push(bdata);
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
  if (qty == 0) return 0;
  
  CAN_Message_t bdata;
  bdata.id= (uint16_t) id;
  bdata.port=2;
  bdata.length = len;
  memcpy(bdata.data,rxdata,8);
  canBuf.push(bdata);
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
  if (qty == 0) return 0;
  
  CAN_Message_t bdata;
  bdata.id= (uint16_t) id;
  bdata.port=3;
  bdata.length = len;
  memcpy(bdata.data,rxdata,8);
  canBuf.push(bdata);
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

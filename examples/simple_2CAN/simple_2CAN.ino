//----- use 8Mhz crystal
#include "meClock_G473.h" 
#include <meFDCAN.h>

#define LED PB9   // QFP48 socket PCB

HardwareSerial Serial1(PA10, PA9);


void setup() 
{
  pinMode(LED,OUTPUT);
  Serial1.begin(9600);

  meFDCAN_init();
 /**
 * @brief   Initialize the FDCAN module
 *
 * @param   bitrate  The desired bitrate for the FDCAN module (default = 125)
 * @param   canPort  The port number for the FDCAN module (default = 1)
 * @param   rxPin    The pin number for the receive pin (default = PA11)
 * @param   txPin    The pin number for the transmit pin (default = PA12)
 *
 * @return  true if the initialization was successful, false otherwise
 */
  meFDCAN_init(125,2,PB12,PB13);
  
}

void loop() 
{
  digitalToggle(LED);
  //--- Send a message on CAN2
  bool result = sendCan2();
  Serial1.print("Transmit = ");
  Serial1.println(result);
  //--- Receive message on CAN 1
  int cnt = checkMessages1();
  delay(1000);

}

/**
 * @brief   Transmit data over the FDCAN module
 *
 * @param   txId     The ID for the transmission
 * @param   txData   The data to be transmitted
 * @param   canPort  The port number for the FDCAN module (default = 1)
 * @param   len      The length of the data to be transmitted (default = 8)
 *
 * @return  true if the transmission was successful, false otherwise
 */
bool sendCan2()
{
  bool success = false;
  uint8_t TxData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  int txId = 200;
  success = meFDCAN_transmit(txId,TxData,2);
  return (success);
}

int checkMessages1()
{
  uint8_t len;
  uint32_t id;
  uint8_t rxdata[8];
  int qty = meFDCAN_receive(&id,&len,rxdata);
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

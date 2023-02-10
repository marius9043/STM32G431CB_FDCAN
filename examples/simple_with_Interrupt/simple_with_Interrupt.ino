



extern "C" void FDCAN1_IT0_IRQHandler();

#include <meFDCAN.h>
#include <meCANbuffer.h>

meCANBuffer canBuf;

#define LED PB9   // QFP48 socket PCB

HardwareSerial Serial1(PA10, PA9);

int mycanPort,myid,myLen;
uint8_t mydata[8];


void setup() 
{
  pinMode(LED,OUTPUT);
  Serial1.begin(9600);

  meFDCAN_init();
 /**
 * @brief   Set the receive interrupt for the FDCAN module
 * 
 * @param   canPort  The port number for the FDCAN module (default = 1)
 * @param   fifo     The FIFO number to use for the interrupt (default = 0)
 * 
 * @return  true if the setting of the receive interrupt was successful, false otherwise
 */
  me_FDCAN_setRxInterrupt();
}

void loop() 
{
  digitalToggle(LED);
  //--- Send a CAN message
  bool result = sendCan1();
  Serial1.print("Transmit = ");
  Serial1.println(result);
  //--- Show Receive CAN message from meCANBuffer
  while(1)
  {
    CAN_Message_t bdata;
    bdata = canBuf.pop();
    if (bdata.port == 0) break;
    reportCAN(bdata.port,bdata.id,bdata.length,bdata.data);
  }
  delay(1000);

}

bool sendCan1()
{
  bool success = false;
  uint8_t TxData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  int txId = 200;
  success = meFDCAN_transmit(txId,TxData);
  return (success);
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

/*
 *    Example CAN ISR
 *    
 *    Do Not use Serial or other long processes, it will crash
 *    if Necessary, use a circular buffer etc
 *    
 */

void CAN1_RX_ISR()
{
  uint8_t len=0;
  uint32_t id=0;
  uint8_t rxdata[8];
  int qty = meFDCAN_receive(&id,&len,rxdata,1);
  if (qty < 0)
  {
    return ; 
  }
  /*
    meCANbuffer : Pushing can messages into buffer.
    
    The "bdata" variable is declared as a structure of type "CAN_Message_t".
    The structure members "id", "port", "length", and "data" are then initialized as follows:
        
    using the "memcpy" function.
    Finally, the "bdata" structure is added to the end of the "canBuf" queue.
    */
    
  CAN_Message_t bdata;
  bdata.id= (uint16_t) id;
  bdata.port=1;
  bdata.length = len;
  memcpy(bdata.data,rxdata,8);
  canBuf.push(bdata);
}

/*
     
    The ISR (Interrupt Service Routine) for the FDCAN1 module is weakly defined
    in the Interrupt Vector Table by the STmicro HAL.
    This specific ISR, named "FDCAN1_IT0_IRQHandler", will handle incoming
    messages on FDCAN1 and call the custom handler "CAN1_RX_ISR".
    Afterwards, it will clear the interrupt flag by setting the
    "FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE" in the FDCAN1 module's IR register.
    */

 
void FDCAN1_IT0_IRQHandler(void)
{
  CAN1_RX_ISR();  //---  my handler 
  FDCAN1->IR |= FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE; //--- clears interrupt
}

#include "stm32f0xx.h"  
#include "SE8R01.h"
#include "main.h"
#include "SSD1306.h"

//unsigned char k=0;
unsigned char TX_ADDRESS[TX_ADR_WIDTH]  = {0x34,0x43,0x10,0x10}; 		// pipe Addr.

unsigned char SPI_RW(unsigned char Byte) 																									{// Strobe command (Write only)

CS_LO();
while (!(SPI1->SR & SPI_SR_TXE)){}; SPI1_DR_8bit = Byte;
while ((SPI1->SR & SPI_SR_BSY)){};	
while (!(SPI1->SR & SPI_SR_RXNE)){}; Byte = SPI1_DR_8bit;//SPI1->DR;
CS_HI();
return Byte;
}

unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value)													{
 unsigned char temp;
CS_LO();

while (!(SPI1->SR & SPI_SR_TXE)){};  
SPI1_DR_8bit = reg;
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){}; 
temp=SPI1_DR_8bit;
	
while (!(SPI1->SR & SPI_SR_TXE)){};	 
SPI1_DR_8bit = value;
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){}; 
temp=SPI1_DR_8bit;	

CS_HI();
return temp;                // return nRF24L01 status unsigned char
}
unsigned char SPI_Read(unsigned char reg)																									{
  unsigned char reg_val;
CS_LO();
	
while (!(SPI1->SR & SPI_SR_TXE)){}; 
SPI1_DR_8bit = reg;
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){}; 
reg = SPI1_DR_8bit;
while (!(SPI1->SR & SPI_SR_TXE)){};	 
SPI1_DR_8bit = 0x00;
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){};
reg_val = SPI1_DR_8bit;

CS_HI();	
  
  return(reg_val);               // return register value
}
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)		{
  unsigned char status,i;
	
CS_LO();
while (!(SPI1->SR & SPI_SR_TXE)){};	SPI1_DR_8bit = (reg);
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){}; status=SPI1_DR_8bit;	
for( i = 0; i < bytes; i ++ )
 {
  while (!(SPI1->SR & SPI_SR_TXE)){};	SPI1_DR_8bit = 0x00; 
	while (SPI1->SR & SPI_SR_BSY){};
	while (!(SPI1->SR & SPI_SR_RXNE)){}; pBuf[i] = SPI1_DR_8bit;		
 }
 CS_HI(); 
 return(status);                  // return nRF24L01 status unsigned char
}
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)	{
  unsigned char status,i;

CS_LO();
while (!(SPI1->SR & SPI_SR_TXE)){};	
SPI1_DR_8bit = (reg);
while (SPI1->SR & SPI_SR_BSY){};		
while (!(SPI1->SR & SPI_SR_RXNE)){}; 
status=SPI1_DR_8bit;	
for( i = 0; i < bytes; i ++ )
 {
  while (!(SPI1->SR & SPI_SR_TXE)){};	
	SPI1_DR_8bit = pBuf[i]; 
	while (SPI1->SR & SPI_SR_BSY){};
	while (!(SPI1->SR & SPI_SR_RXNE)){};
	SPI1_DR_8bit;	
 }
CS_HI();
  return(status);                  // return nRF24L01 status unsigned char
}

void rf_switch_bank(unsigned char bankindex)																							{
    unsigned char temp0,temp1;
    temp1 = bankindex;

    temp0 = SPI_RW(iRF_BANK0_STATUS);

    if((temp0&0x80)!=temp1)
    {
        SPI_RW_Reg(iRF_CMD_ACTIVATE,0x53);
    }
}

void se8r01_powerup(void)																																	{
        rf_switch_bank(iBANK0);
        SPI_RW_Reg(iRF_CMD_WRITE_REG|iRF_BANK0_CONFIG,0x03);
        SPI_RW_Reg(iRF_CMD_WRITE_REG|iRF_BANK0_RF_CH,0x32);
        SPI_RW_Reg(iRF_CMD_WRITE_REG|iRF_BANK0_RF_SETUP,0x48);
        SPI_RW_Reg(iRF_CMD_WRITE_REG|iRF_BANK0_PRE_GURD,0x77); //2450 calibration
}

void se8r01_calibration(void)																															{
        rf_switch_bank(iBANK1);
				unsigned char gtemp[5]={0x40,0x00,0x10,0xe6};																			//gtemp[0]=0x40;gtemp[1]=0x00;gtemp[2]=0x10;gtemp[3]=0xE6;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_PLL_CTL0,gtemp, 4);

        gtemp[0]=0x20;
				gtemp[1]=0x08;
				gtemp[2]=0x50;
				gtemp[3]=0x40;
				gtemp[4]=0x50;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_CAL_CTL, gtemp, 5);

        gtemp[0]=0x00;
				gtemp[1]=0x00;
				gtemp[2]=0x1E;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_IF_FREQ, gtemp, 3);

        gtemp[0]=0x29;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_FDEV, gtemp, 1);

        gtemp[0]=0x00;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_LOW, gtemp, 1);

        gtemp[0]=0x7F;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_HI, gtemp, 1);

        gtemp[0]=0x02;
				gtemp[1]=0xC1;
				gtemp[2]=0xEB;
				gtemp[3]=0x1C;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_AGC_GAIN, gtemp, 4);

        gtemp[0]=0x97;
				gtemp[1]=0x64;
				gtemp[2]=0x00;
				gtemp[3]=0x81;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_RF_IVGEN, gtemp, 4);

        rf_switch_bank(iBANK0);

        CEq_HI();
				delay_ms(30);
        CEq_LO();
        delay_ms(50);	//delayMicroseconds(50);                            // delay 50ms waitting for calibaration.
				CEq_HI();
				delay_ms(30);
        CEq_LO();
        delay_ms(50);	//delayMicroseconds(50);                          // delay 50ms waitting for calibaration.
        // calibration end

   }
  
void se8r01_setup(void)  																																	{
				unsigned char gtemp[5];      
				gtemp[0]=0x28;
				gtemp[1]=0x32;
        gtemp[2]=0x80;
        gtemp[3]=0x90;
        gtemp[4]=0x00;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK0_SETUP_VALUE, gtemp, 5);

				delay_ms(2);	//delayMicroseconds(2);    
	

        rf_switch_bank(iBANK1);

        gtemp[0]=0x40;
        gtemp[1]=0x01;
        gtemp[2]=0x30;
        gtemp[3]=0xE2;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_PLL_CTL0, gtemp, 4);

        gtemp[0]=0x29;
        gtemp[1]=0x89;
        gtemp[2]=0x55;
        gtemp[3]=0x40;
        gtemp[4]=0x50;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_CAL_CTL, gtemp, 5);

        gtemp[0]=0x29;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_FDEV, gtemp, 1);

        gtemp[0]=0x55;
        gtemp[1]=0xC2;
        gtemp[2]=0x09;
        gtemp[3]=0xAC;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_RX_CTRL, gtemp, 4);

        gtemp[0]=0x00;
        gtemp[1]=0x14;
        gtemp[2]=0x08;
        gtemp[3]=0x29;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_FAGC_CTRL_1, gtemp, 4);

        gtemp[0]=0x02;
        gtemp[1]=0xC1;
        gtemp[2]=0xCB;
        gtemp[3]=0x1C;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_AGC_GAIN, gtemp, 4);

        gtemp[0]=0x97;
        gtemp[1]=0x64;
        gtemp[2]=0x00;
        gtemp[3]=0x01;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_RF_IVGEN, gtemp, 4);

        gtemp[0]=0x2A;
        gtemp[1]=0x04;
        gtemp[2]=0x00;
        gtemp[3]=0x7D;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_TEST_PKDET, gtemp, 4);

        rf_switch_bank(iBANK0);
 }
unsigned char init_io(void)																																{
  CEq_LO();			// chip enable
  CS_HI();    	// Spi disable	
	return SPI_Read(STATUS);
}
void RXX(struct se8r01_str* data)																													{
    
			data->status = SPI_Read(STATUS);
			data->rpd = (int8_t)SPI_Read(iRF_BANK0_RPD);
	if(data->status&STA_MARK_RX)                                           // if receive data ready (TX_DS) interrupt
    {
      SPI_Read_Buf(RD_RX_PLOAD, data->rxbuf, TX_PLOAD_WIDTH);             // read playload to rx_buf
      SPI_RW_Reg(FLUSH_RX,0); // clear RX_FIFO
      SPI_RW_Reg(WRITE_SE_REG+STATUS,0xff);
    }
     else{
        SPI_RW_Reg(WRITE_SE_REG+STATUS,0xff);
     }
}


void TXX(struct se8r01_str* data)																													{
       
			data->status = SPI_Read(STATUS); 

      SPI_RW_Reg(FLUSH_TX, 0);
      SPI_Write_Buf(WR_TX_PLOAD, data->txbuf, TX_PLOAD_WIDTH);     
      SPI_RW_Reg(WRITE_SE_REG+STATUS,0xff);   // clear RX_DR or TX_DS or MAX_RT interrupt flag
 }


void radio_settings(void)																																	{ 
   SPI_RW_Reg(WRITE_SE_REG|iRF_BANK0_EN_AA, 0x01);          //enable auto acc on pipe 1
   SPI_RW_Reg(WRITE_SE_REG|iRF_BANK0_EN_RXADDR, 0x01);      //enable pipe 1
   SPI_RW_Reg(WRITE_SE_REG|iRF_BANK0_SETUP_AW, 0x02);       //4 byte adress
        
   SPI_RW_Reg(WRITE_SE_REG|iRF_BANK0_SETUP_RETR, 0b00001010);        //lowest 4 bits 0-15 rt transmisston higest 4 bits 256-4096us Auto Retransmit Delay
   SPI_RW_Reg(WRITE_SE_REG|iRF_BANK0_RF_CH, 40);
   SPI_RW_Reg(WRITE_SE_REG|iRF_BANK0_RF_SETUP, 0x4f);        //2mps 0x4f
   //SPI_RW_Reg(WRITE_SE_REG|iRF_BANK0_DYNPD, 0x01);          //pipe0 pipe1 enable dynamic payload length data
   //SPI_RW_Reg(WRITE_SE_REG|iRF_BANK0_FEATURE, 0x07);        // enable dynamic payload lenght; enbale payload with ack enable w_tx_payload_noack
        
	SPI_Write_Buf(WRITE_SE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);  //from tx
	SPI_Write_Buf(WRITE_SE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // Use the same address on the RX device as the TX device
	SPI_RW_Reg(WRITE_SE_REG + RX_PW_P0, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width       
}

  


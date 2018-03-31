/*
  LoRaWAN.cpp - Library for LoRaWAN protocol, uses RFM95W module
  Created by Leo Korbee, March 31, 2018.
  Released into the public domain.
  @license Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
  Thanks to all the folks who contributed beforme me on this code.

*/

#include "Arduino.h"
#include "LoRaWAN.h"


// constructor
LoRaWAN::LoRaWAN(RFM95 *rfm95)
{
   _rfm95 = rfm95;
}



void LoRaWAN::setKeys(unsigned char NwkSkey[], unsigned char AppSkey[], unsigned char DevAddr[])
{
  _NwkSkey = NwkSkey;
  _AppSkey = AppSkey;
  _DevAddr = DevAddr;
}

/*
*****************************************************************************************
* Description : Function contstructs a LoRaWAN package and sends it
*
* Arguments   : *Data pointer to the array of data that will be transmitted
*               Data_Length nuber of bytes to be transmitted
*               Frame_Counter_Up  Frame counter of upstream frames
*****************************************************************************************
*/
void LoRaWAN::Send_Data(unsigned char *Data, unsigned char Data_Length, unsigned int Frame_Counter_Tx)
{
  //Define variables
  unsigned char i;

  //Direction of frame is up
  unsigned char Direction = 0x00;

  unsigned char RFM_Data[64];
  unsigned char RFM_Package_Length;

  unsigned char MIC[4];

  /*
  https://hackmd.io/s/S1kg6Ymo-

  7…5 bits	4…2 bits	1…0 bits
  MType	    RFU	      Major

  MType	Description
  000	(0x00) Join Request
  001	(0x20) Join Accept
  010	(0x40) Unconfirmed Data Up
  011	(0x60) Unconfirmed Data Down
  100	(0x80) Confirmed Data Up
  101	(0xA0) Confirmed Data Down
  110	(0xC0) RFU
  111	(0xE0) Proprietary




  */


  // Unconfirmed data up
  unsigned char Mac_Header = 0x40;

  // Confirmed data up
  // unsigned char Mac_Header = 0x80;

  unsigned char Frame_Control = 0x00;
  unsigned char Frame_Port = 0x01;

  //Encrypt the data
  Encrypt_Payload(Data, Data_Length, Frame_Counter_Tx, Direction);


  //Build the Radio Package
  RFM_Data[0] = Mac_Header;

  RFM_Data[1] = _DevAddr[3];
  RFM_Data[2] = _DevAddr[2];
  RFM_Data[3] = _DevAddr[1];
  RFM_Data[4] = _DevAddr[0];

  RFM_Data[5] = Frame_Control;

  RFM_Data[6] = (Frame_Counter_Tx & 0x00FF);
  RFM_Data[7] = ((Frame_Counter_Tx >> 8) & 0x00FF);

  RFM_Data[8] = Frame_Port;

  //Set Current package length
  RFM_Package_Length = 9;

  //Load Data
  for(i = 0; i < Data_Length; i++)
  {
    RFM_Data[RFM_Package_Length + i] = Data[i];
  }

  //Add data Lenth to package length
  RFM_Package_Length = RFM_Package_Length + Data_Length;

  //Calculate MIC
  Calculate_MIC(RFM_Data, MIC, RFM_Package_Length, Frame_Counter_Tx, Direction);

  //Load MIC in package
  for(i = 0; i < 4; i++)
  {
    RFM_Data[i + RFM_Package_Length] = MIC[i];
  }

  //Add MIC length to RFM package length
  RFM_Package_Length = RFM_Package_Length + 4;

  //Send Package
  RFM_Send_Package(RFM_Data, RFM_Package_Length);
}


/*
*****************************************************************************************
* Description : Function for sending a package with the RFM
*
* Arguments   : *RFM_Tx_Package Pointer to arry with data to be send
*               Package_Length  Length of the package to send
*****************************************************************************************
*/

void LoRaWAN::RFM_Send_Package(unsigned char *RFM_Tx_Package, unsigned char Package_Length)
{
  unsigned char i;
  // unsigned char RFM_Tx_Location = 0x00;

  //Set RFM in Standby mode wait on mode ready

  _rfm95->RFM_Write(0x01,0x81);
  /*
  while (digitalRead(DIO5) == LOW)
  {
  }
  */
  delay(10);

  //Switch DIO0 to TxDone
  _rfm95->RFM_Write(0x40,0x40);
  //Set carrier frequency

  /*
  fixed frequency
  // 868.100 MHz / 61.035 Hz = 14222987 = 0xD9068B
  _rfm95.RFM_Write(0x06,0xD9);
  _rfm95.RFM_Write(0x07,0x06);
  _rfm95.RFM_Write(0x08,0x8B);
  */

  // TCNT0 is timer0 continous timer, kind of random selection of frequency

  // EU863-870 specifications
  switch (TCNT0 % 8)
  {
      case 0x00: //Channel 0 868.100 MHz / 61.035 Hz = 14222987 = 0xD9068B
          _rfm95->RFM_Write(0x06,0xD9);
          _rfm95->RFM_Write(0x07,0x06);
          _rfm95->RFM_Write(0x08,0x8B);
          break;
        case 0x01: //Channel 1 868.300 MHz / 61.035 Hz = 14226264 = 0xD91358
          _rfm95->RFM_Write(0x06,0xD9);
          _rfm95->RFM_Write(0x07,0x13);
          _rfm95->RFM_Write(0x08,0x58);
          break;
        case 0x02: //Channel 2 868.500 MHz / 61.035 Hz = 14229540 = 0xD92024
          _rfm95->RFM_Write(0x06,0xD9);
          _rfm95->RFM_Write(0x07,0x20);
          _rfm95->RFM_Write(0x08,0x24);
          break;
        // added four more channels
        case 0x03: // Channel 3 867.100 MHz / 61.035 Hz = 14206603 = 0xD8C68B
          _rfm95->RFM_Write(0x06,0xD8);
          _rfm95->RFM_Write(0x07,0xC6);
          _rfm95->RFM_Write(0x08,0x8B);
          break;
        case 0x04: // Channel 4 867.300 MHz / 61.035 Hz = 14209880 = 0xD8D358
          _rfm95->RFM_Write(0x06,0xD8);
          _rfm95->RFM_Write(0x07,0xD3);
          _rfm95->RFM_Write(0x08,0x58);
          break;
        case 0x05: // Channel 5 867.500 MHz / 61.035 Hz = 14213156 = 0xD8E024
          _rfm95->RFM_Write(0x06,0xD8);
          _rfm95->RFM_Write(0x07,0xE0);
          _rfm95->RFM_Write(0x08,0x24);
          break;
        case 0x06: // Channel 6 867.700 MHz / 61.035 Hz = 14216433 = 0xD8ECF1
          _rfm95->RFM_Write(0x06,0xD8);
          _rfm95->RFM_Write(0x07,0xEC);
          _rfm95->RFM_Write(0x08,0xF1);
          break;
        case 0x07: // Channel 7 867.900 MHz / 61.035 Hz = 14219710 = 0xD8F9BE
          _rfm95->RFM_Write(0x06,0xD8);
          _rfm95->RFM_Write(0x07,0xF9);
          _rfm95->RFM_Write(0x08,0xBE);
          break;
        // FSK       868.800 Mhz => not used in this config


    }


  //SF7 BW 125 kHz
  _rfm95->RFM_Write(0x1E,0x74); //SF7 CRC On
  _rfm95->RFM_Write(0x1D,0x72); //125 kHz 4/5 coding rate explicit header mode
  _rfm95->RFM_Write(0x26,0x04); //Low datarate optimization off AGC auto on

  //Set IQ to normal values
  _rfm95->RFM_Write(0x33,0x27);
  _rfm95->RFM_Write(0x3B,0x1D);

  //Set payload length to the right length
  _rfm95->RFM_Write(0x22,Package_Length);

  //Get location of Tx part of FiFo
  //RFM_Tx_Location = RFM_Read(0x0E);

  //Set SPI pointer to start of Tx part in FiFo
  //RFM_Write(0x0D,RFM_Tx_Location);
  _rfm95->RFM_Write(0x0D,0x80); // hardcoded fifo location according RFM95 specs

  //Write Payload to FiFo
  for (i = 0;i < Package_Length; i++)
  {
    _rfm95->RFM_Write(0x00,*RFM_Tx_Package);
    RFM_Tx_Package++;
  }

  //Switch RFM to Tx
  _rfm95->RFM_Write(0x01,0x83);

  //Wait for TxDone
  while(!_rfm95->txDone())
  {
  }

  //Switch RFM to sleep
  _rfm95->RFM_Write(0x01,0x00);
}













/*
   Encryption stuff after this line
*/
void LoRaWAN::Encrypt_Payload(unsigned char *Data, unsigned char Data_Length, unsigned int Frame_Counter, unsigned char Direction)
{
  unsigned char i = 0x00;
  unsigned char j;
  unsigned char Number_of_Blocks = 0x00;
  unsigned char Incomplete_Block_Size = 0x00;

  unsigned char Block_A[16];

  //Calculate number of blocks
  Number_of_Blocks = Data_Length / 16;
  Incomplete_Block_Size = Data_Length % 16;
  if(Incomplete_Block_Size != 0)
  {
    Number_of_Blocks++;
  }

  for(i = 1; i <= Number_of_Blocks; i++)
  {
    Block_A[0] = 0x01;
    Block_A[1] = 0x00;
    Block_A[2] = 0x00;
    Block_A[3] = 0x00;
    Block_A[4] = 0x00;

    Block_A[5] = Direction;

    Block_A[6] = _DevAddr[3];
    Block_A[7] = _DevAddr[2];
    Block_A[8] = _DevAddr[1];
    Block_A[9] = _DevAddr[0];

    Block_A[10] = (Frame_Counter & 0x00FF);
    Block_A[11] = ((Frame_Counter >> 8) & 0x00FF);

    Block_A[12] = 0x00; //Frame counter upper Bytes
    Block_A[13] = 0x00;

    Block_A[14] = 0x00;

    Block_A[15] = i;

    //Calculate S
    AES_Encrypt(Block_A, _AppSkey); //original


    //Check for last block
    if(i != Number_of_Blocks)
    {
      for(j = 0; j < 16; j++)
      {
        *Data = *Data ^ Block_A[j];
        Data++;
      }
    }
    else
    {
      if(Incomplete_Block_Size == 0)
      {
        Incomplete_Block_Size = 16;
      }
      for(j = 0; j < Incomplete_Block_Size; j++)
      {
        *Data = *Data ^ Block_A[j];
        Data++;
      }
    }
  }
}


void LoRaWAN::Calculate_MIC(unsigned char *Data, unsigned char *Final_MIC, unsigned char Data_Length, unsigned int Frame_Counter, unsigned char Direction)
{
  unsigned char i;
  unsigned char Block_B[16];

  unsigned char Key_K1[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  unsigned char Key_K2[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };

  //unsigned char Data_Copy[16];

  unsigned char Old_Data[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  unsigned char New_Data[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };


  unsigned char Number_of_Blocks = 0x00;
  unsigned char Incomplete_Block_Size = 0x00;
  unsigned char Block_Counter = 0x01;

  //Create Block_B
  Block_B[0] = 0x49;
  Block_B[1] = 0x00;
  Block_B[2] = 0x00;
  Block_B[3] = 0x00;
  Block_B[4] = 0x00;

  Block_B[5] = Direction;

  Block_B[6] = _DevAddr[3];
  Block_B[7] = _DevAddr[2];
  Block_B[8] = _DevAddr[1];
  Block_B[9] = _DevAddr[0];

  Block_B[10] = (Frame_Counter & 0x00FF);
  Block_B[11] = ((Frame_Counter >> 8) & 0x00FF);

  Block_B[12] = 0x00; //Frame counter upper bytes
  Block_B[13] = 0x00;

  Block_B[14] = 0x00;
  Block_B[15] = Data_Length;

  //Calculate number of Blocks and blocksize of last block
  Number_of_Blocks = Data_Length / 16;
  Incomplete_Block_Size = Data_Length % 16;

  if(Incomplete_Block_Size != 0)
  {
    Number_of_Blocks++;
  }

  Generate_Keys(Key_K1, Key_K2);

  //Preform Calculation on Block B0

  //Preform AES encryption
  AES_Encrypt(Block_B, _NwkSkey);

  //Copy Block_B to Old_Data
  for(i = 0; i < 16; i++)
  {
    Old_Data[i] = Block_B[i];
  }

  //Preform full calculating until n-1 messsage blocks
  while(Block_Counter < Number_of_Blocks)
  {
    //Copy data into array
    for(i = 0; i < 16; i++)
    {
      New_Data[i] = *Data;
      Data++;
    }

    //Preform XOR with old data
    XOR(New_Data,Old_Data);

    //Preform AES encryption
    AES_Encrypt(New_Data, _NwkSkey);

    //Copy New_Data to Old_Data
    for(i = 0; i < 16; i++)
    {
      Old_Data[i] = New_Data[i];
    }

    //Raise Block counter
    Block_Counter++;
  }

  //Perform calculation on last block
  //Check if Datalength is a multiple of 16
  if(Incomplete_Block_Size == 0)
  {
    //Copy last data into array
    for(i = 0; i < 16; i++)
    {
      New_Data[i] = *Data;
      Data++;
    }

    //Preform XOR with Key 1
    XOR(New_Data,Key_K1);

    //Preform XOR with old data
    XOR(New_Data,Old_Data);

    //Preform last AES routine
    // read NwkSkey from PROGMEM
    AES_Encrypt(New_Data, _NwkSkey);
  }
  else
  {
    //Copy the remaining data and fill the rest
    for(i =  0; i < 16; i++)
    {
      if(i < Incomplete_Block_Size)
      {
        New_Data[i] = *Data;
        Data++;
      }
      if(i == Incomplete_Block_Size)
      {
        New_Data[i] = 0x80;
      }
      if(i > Incomplete_Block_Size)
      {
        New_Data[i] = 0x00;
      }
    }

    //Preform XOR with Key 2
    XOR(New_Data,Key_K2);

    //Preform XOR with Old data
    XOR(New_Data,Old_Data);

    //Preform last AES routine
    AES_Encrypt(New_Data, _NwkSkey);
  }

  Final_MIC[0] = New_Data[0];
  Final_MIC[1] = New_Data[1];
  Final_MIC[2] = New_Data[2];
  Final_MIC[3] = New_Data[3];
}


void LoRaWAN::Generate_Keys(unsigned char *K1, unsigned char *K2)
{
  unsigned char i;
  unsigned char MSB_Key;

  //Encrypt the zeros in K1 with the NwkSkey
  AES_Encrypt(K1,_NwkSkey);

  //Create K1
  //Check if MSB is 1
  if((K1[0] & 0x80) == 0x80)
  {
    MSB_Key = 1;
  }
  else
  {
    MSB_Key = 0;
  }

  //Shift K1 one bit left
  Shift_Left(K1);

  //if MSB was 1
  if(MSB_Key == 1)
  {
    K1[15] = K1[15] ^ 0x87;
  }

  //Copy K1 to K2
  for( i = 0; i < 16; i++)
  {
    K2[i] = K1[i];
  }

  //Check if MSB is 1
  if((K2[0] & 0x80) == 0x80)
  {
    MSB_Key = 1;
  }
  else
  {
    MSB_Key = 0;
  }

  //Shift K2 one bit left
  Shift_Left(K2);

  //Check if MSB was 1
  if(MSB_Key == 1)
  {
    K2[15] = K2[15] ^ 0x87;
  }
}

void LoRaWAN::Shift_Left(unsigned char *Data)
{
  unsigned char i;
  unsigned char Overflow = 0;
  //unsigned char High_Byte, Low_Byte;

  for(i = 0; i < 16; i++)
  {
    //Check for overflow on next byte except for the last byte
    if(i < 15)
    {
      //Check if upper bit is one
      if((Data[i+1] & 0x80) == 0x80)
      {
        Overflow = 1;
      }
      else
      {
        Overflow = 0;
      }
    }
    else
    {
      Overflow = 0;
    }

    //Shift one left
    Data[i] = (Data[i] << 1) + Overflow;
  }
}

void LoRaWAN::XOR(unsigned char *New_Data,unsigned char *Old_Data)
{
  unsigned char i;

  for(i = 0; i < 16; i++)
  {
    New_Data[i] = New_Data[i] ^ Old_Data[i];
  }
}

/*
*****************************************************************************************
* Title         : AES_Encrypt
* Description  :
*****************************************************************************************
*/
void LoRaWAN::AES_Encrypt(unsigned char *Data, unsigned char *Key)
{
  unsigned char Row, Column, Round = 0;
  unsigned char Round_Key[16];
    unsigned char State[4][4];

  //  Copy input to State arry
  for( Column = 0; Column < 4; Column++ )
  {
    for( Row = 0; Row < 4; Row++ )
    {
      State[Row][Column] = Data[Row + (Column << 2)];
    }
  }

  //  Copy key to round key
  memcpy( &Round_Key[0], &Key[0], 16 );

  //  Add round key
  AES_Add_Round_Key( Round_Key, State );

  //  Preform 9 full rounds with mixed collums
  for( Round = 1 ; Round < 10 ; Round++ )
  {
    //  Perform Byte substitution with S table
    for( Column = 0 ; Column < 4 ; Column++ )
    {
      for( Row = 0 ; Row < 4 ; Row++ )
      {
        State[Row][Column] = AES_Sub_Byte( State[Row][Column] );
      }
    }

    //  Perform Row Shift
    AES_Shift_Rows(State);

    //  Mix Collums
    AES_Mix_Collums(State);

    //  Calculate new round key
    AES_Calculate_Round_Key(Round, Round_Key);

        //  Add the round key to the Round_key
    AES_Add_Round_Key(Round_Key, State);
  }

  //  Perform Byte substitution with S table whitout mix collums
  for( Column = 0 ; Column < 4 ; Column++ )
  {
    for( Row = 0; Row < 4; Row++ )
    {
      State[Row][Column] = AES_Sub_Byte(State[Row][Column]);
    }
  }

  //  Shift rows
  AES_Shift_Rows(State);

  //  Calculate new round key
  AES_Calculate_Round_Key( Round, Round_Key );

    //  Add round key
  AES_Add_Round_Key( Round_Key, State );

  //  Copy the State into the data array
  for( Column = 0; Column < 4; Column++ )
  {
    for( Row = 0; Row < 4; Row++ )
    {
      Data[Row + (Column << 2)] = State[Row][Column];
    }
  }
} // AES_Encrypt


/*
*****************************************************************************************
* Title         : AES_Add_Round_Key
* Description :
*****************************************************************************************
*/
void LoRaWAN::AES_Add_Round_Key(unsigned char *Round_Key, unsigned char (*State)[4])
{
  unsigned char Row, Collum;

  for(Collum = 0; Collum < 4; Collum++)
  {
    for(Row = 0; Row < 4; Row++)
    {
      State[Row][Collum] ^= Round_Key[Row + (Collum << 2)];
    }
  }
} // AES_Add_Round_Key


/*
*****************************************************************************************
* Title         : AES_Sub_Byte
* Description :
*****************************************************************************************
*/
unsigned char LoRaWAN::AES_Sub_Byte(unsigned char Byte)
{
//  unsigned char S_Row,S_Collum;
//  unsigned char S_Byte;
//
//  S_Row    = ((Byte >> 4) & 0x0F);
//  S_Collum = ((Byte >> 0) & 0x0F);
//  S_Byte   = S_Table [S_Row][S_Collum];

  //return S_Table [ ((Byte >> 4) & 0x0F) ] [ ((Byte >> 0) & 0x0F) ]; // original
  return pgm_read_byte(&(S_Table [((Byte >> 4) & 0x0F)] [((Byte >> 0) & 0x0F)]));
} //    AES_Sub_Byte


/*
*****************************************************************************************
* Title         : AES_Shift_Rows
* Description :
*****************************************************************************************
*/
void LoRaWAN::AES_Shift_Rows(unsigned char (*State)[4])
{
  unsigned char Buffer;

  //Store firt byte in buffer
  Buffer      = State[1][0];
  //Shift all bytes
  State[1][0] = State[1][1];
  State[1][1] = State[1][2];
  State[1][2] = State[1][3];
  State[1][3] = Buffer;

  Buffer      = State[2][0];
  State[2][0] = State[2][2];
  State[2][2] = Buffer;
  Buffer      = State[2][1];
  State[2][1] = State[2][3];
  State[2][3] = Buffer;

  Buffer      = State[3][3];
  State[3][3] = State[3][2];
  State[3][2] = State[3][1];
  State[3][1] = State[3][0];
  State[3][0] = Buffer;
}   //  AES_Shift_Rows


/*
*****************************************************************************************
* Title         : AES_Mix_Collums
* Description :
*****************************************************************************************
*/
void LoRaWAN::AES_Mix_Collums(unsigned char (*State)[4])
{
  unsigned char Row,Collum;
  unsigned char a[4], b[4];


  for(Collum = 0; Collum < 4; Collum++)
  {
    for(Row = 0; Row < 4; Row++)
    {
      a[Row] =  State[Row][Collum];
      b[Row] = (State[Row][Collum] << 1);

      if((State[Row][Collum] & 0x80) == 0x80)
      {
        b[Row] ^= 0x1B;
      }
    }

    State[0][Collum] = b[0] ^ a[1] ^ b[1] ^ a[2] ^ a[3];
    State[1][Collum] = a[0] ^ b[1] ^ a[2] ^ b[2] ^ a[3];
    State[2][Collum] = a[0] ^ a[1] ^ b[2] ^ a[3] ^ b[3];
    State[3][Collum] = a[0] ^ b[0] ^ a[1] ^ a[2] ^ b[3];
  }
}   //  AES_Mix_Collums



/*
*****************************************************************************************
* Title         : AES_Calculate_Round_Key
* Description :
*****************************************************************************************
*/
void LoRaWAN::AES_Calculate_Round_Key(unsigned char Round, unsigned char *Round_Key)
{
  unsigned char i, j, b, Rcon;
  unsigned char Temp[4];


    //Calculate Rcon
  Rcon = 0x01;
  while(Round != 1)
  {
    b = Rcon & 0x80;
    Rcon = Rcon << 1;

    if(b == 0x80)
    {
      Rcon ^= 0x1b;
    }
    Round--;
  }

  //  Calculate first Temp
  //  Copy laste byte from previous key and subsitute the byte, but shift the array contents around by 1.
    Temp[0] = AES_Sub_Byte( Round_Key[12 + 1] );
    Temp[1] = AES_Sub_Byte( Round_Key[12 + 2] );
    Temp[2] = AES_Sub_Byte( Round_Key[12 + 3] );
    Temp[3] = AES_Sub_Byte( Round_Key[12 + 0] );

  //  XOR with Rcon
  Temp[0] ^= Rcon;

  //  Calculate new key
  for(i = 0; i < 4; i++)
  {
    for(j = 0; j < 4; j++)
    {
      Round_Key[j + (i << 2)]  ^= Temp[j];
      Temp[j]                   = Round_Key[j + (i << 2)];
    }
  }
}   //  AES_Calculate_Round_Key

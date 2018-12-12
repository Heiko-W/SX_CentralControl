// SPDX-License-Identifier: MIT
//
// Author:        Heiko Wilke
// Description:   Selectrix Central Control with Arduino
// Copyright 2018 Heiko Wilke
//
// MIT License
// 
// Copyright (c) 2018 Heiko Wilke
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#define SX0_T0_PORT      PORTD       // SX0 Clock Signal Port
#define SX0_T0_OUT       7           // SX0 Clock Signal Output
#define SX0_T0_OUT_PORT  B10000000   // SX0 Clock Signal Port Bit

#define SX0_T1_PORT      PORTD       // SX0 Data from Central to Devices Port
#define SX0_T1_OUT       6           // SX0 Data from Central to Devices Output
#define SX0_T1_OUT_PORT  B01000000   // SX0 Data from Central to Devices Port Bit

#define SX0_D_PORT       PORTD       // SX0 Data from Devices to Central Port
#define SX0_D_IN         5           // SX0 Data from Devices to Central 
#define SX0_D_IN_PORT    B00100000   // SX0 Data from Devices to Central Port Bit

#define PX0_P0_PORT      PORTB       // Signal 0 Track Power for Booster Port
#define PX0_P0_OUT       12          // Signal 0 Track Power for Booster
#define PX0_P0_OUT_PORT  B00010000   // Signal 0 Track Power for Booster Port Bit

#define PX0_P1_PORT      PORTB       // Signal 1 Track Power for Booster Port
#define PX0_P1_OUT       9           // Signal 1 Track Power for Booster
#define PX0_P1_OUT_PORT  B00000010   // Signal 1 Track Power for Booster Port Bit

#define PX0_PWR_PORT     PORTD       // Track Power ON/OFF Port
#define PX0_PWR_OUT      3           // Track Power ON/OFF
#define PX0_PWR_OUT_PORT B00001000   // Track Power ON/OFF Port Bit

#define CHANNEL_NUMBER  112
#define SYNC_SIGNAL     B0001

#define BAUD_RATE       230400


byte sx0_channels_1[CHANNEL_NUMBER];
byte sx0_channels_2[CHANNEL_NUMBER];
byte *active_channels;
byte *passive_channels;
bool clockcycle = true;
bool databit;
bool trackpower = false;

enum estate {
  eSync,
  ePower,
  eSeparate,
  eAddress,
  eData
};

enum ecomstate {
  eWait,
  eComm,
  eBlock
};

void comFunction(){
  static ecomstate state = eWait;
  static byte buf[10];
  static int incomingByte, i =0;
  static int nbyte = 1;
  
  switch(state){
    case eWait: // Wait for Commands from PC
        if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();
                if (incomingByte >= 0) {
                  buf[i]=(byte) incomingByte;
                }
                i++;
        } else {
          if (i>=3) {
            state = eComm;
            i = 0;
          }
        }
      break;
      
    case eComm: //Analyze Command and send Feedback 
      
      // Trackpower ON/OFF
      if(buf[1] == 0xFF) {
        if(buf[2]==0x00) {
          trackpower = false;
        } else {
          trackpower = true;
        }
        Serial.write(0x00);
      }
      
      // Trackpower Status
      if(buf[1] == 0x7F) {
        if(trackpower) {
          Serial.write(0xFF);
        } else {
          Serial.write(0x00);
        }
      }

      // Read SX1 Channel
      if(buf[1] <= 112) {
        Serial.write(*(passive_channels+buf[1]));
      }

      // Write SX1 Channel - Channel & 0x80 from PC
      if(buf[1] & 0x80) {
        *(passive_channels+(buf[1] & (!0x80))) = buf[2];
        Serial.write(0x00);
      }

      // Block read SX1
      if((buf[0] == 0x78) && (buf[1] == 0x03)) {
        state = eBlock;
      }
      
      break;

    case eBlock: // Block Read Mode
        if (nbyte <= 112) {
          Serial.write(*(passive_channels+nbyte-1));
          nbyte++; 
        }
        if (nbyte == 113) {
          Serial.write((0xFF && trackpower));
          nbyte++;
        }
        if ((nbyte > 113) && (nbyte <= 225)) {
          Serial.write(*(passive_channels+nbyte-114));
          nbyte++;
        }
        if (nbyte == 226) {
          Serial.write((0xFF && trackpower));
          nbyte++;
        }
        if (nbyte > 226) {
          nbyte = 0;
          state = eWait; 
        }
        
      break;
  }
}

// Generate PX Signal 
void setPXSignal(bool signalbit, bool clockbit){
  static bool px0_p0=true;
  if (clockbit) {
    PX0_P0_PORT &= ~(PX0_P0_OUT_PORT|PX0_P1_OUT_PORT);
    PX0_P1_PORT |= PX0_P1_OUT_PORT;
  } else {
    if(signalbit){
      px0_p0 = !px0_p0;
    }
    PX0_P1_PORT &= ~PX0_P1_OUT_PORT;
    if (px0_p0) {
      //PX0_P1_PORT &= ~PX0_P1_OUT_PORT;
      PX0_P0_PORT |=  PX0_P0_OUT_PORT;
    } else {
      PX0_P0_PORT &= ~PX0_P0_OUT_PORT;
      //PX0_P1_PORT |=  PX0_P1_OUT_PORT;
    }
  }
}

/*
void setPXSignal(bool signalbit, bool clockbit){
  static bool px0_p0=true;
  if (clockbit) {
    PX0_P0_PORT &= ~(PX0_P0_OUT_PORT|PX0_P1_OUT_PORT);
    PX0_P1_PORT |= PX0_P1_OUT_PORT;
  } else {
    if(signalbit){
      px0_p0 = !px0_p0;
    }
    PX0_P1_PORT &= ~PX0_P1_OUT_PORT;
    if (px0_p0) {
      //PX0_P1_PORT &= ~PX0_P1_OUT_PORT;
      PX0_P0_PORT |=  PX0_P0_OUT_PORT;
    } else {
      PX0_P0_PORT &= ~PX0_P0_OUT_PORT;
      //PX0_P1_PORT |=  PX0_P1_OUT_PORT;
    }
  }
}
*/

bool nextBit() {
  static byte channel = 0;
  static byte counter = 0;
  static byte bitcounter = 0;
  static byte address = 0;
  static estate state = eSync;
  static bool output;
  byte *help_pointer;

  switch (state) {

    // Sync Bits 0001
    case eSync:
      output = SYNC_SIGNAL >> (3 - counter);
      counter++;
      if (counter > 3) {
        state = ePower;
        counter = 0;
      }
      break;

    // Track Power Enable Bit
    case ePower:
      output = trackpower;
      state = eSeparate;
      break;

    // Separation Bit
    case eSeparate:
      output = true;
      address++;
      if (address > 15) {
        address = 0;

        // Pointer Change
        help_pointer = active_channels;
        active_channels = passive_channels;
        passive_channels = help_pointer;
       
      }
      state = eAddress;
      break;

    // 4 Adress Bits for Frame of 7 Channels/Bytes
    // Stuff Bit after 2 Bits
    case eAddress:
      if (counter == 2) {
        counter = 0;
        output = true;
        if (bitcounter > 3) {
          state = eData;
          counter = 0;
          bitcounter = 0;
        }
      } else {
        // Base Address will be transmitted inverted
        output = (~address >> (3 - bitcounter)) & 0x01;
        counter++;
        bitcounter++;
      }
      break;

    // 7 Bytes/Channels Data
    // Stuff Bit after 2 Bits
    case eData:
      if (counter == 2) {
        if (bitcounter > 7) {
          bitcounter = 0;
          channel++;
          if (channel > 6) {
            state = eSync;
            channel = 0;
          }
        }
        counter = 0;
        output = true;
      } else {
        //output = false;
        output = (*(active_channels+address+channel*16) >> (7 - bitcounter)) & 0x01;
        counter++;
        bitcounter++;
      }
      break;
  }
  return output;
}

void setup() {
  // Init Serial Port
  Serial.begin(BAUD_RATE);

  // Disable interrupts because we are running cyclically without interrupts
  noInterrupts();

  // Setup Timer 1 for exact microsecond timing
  TCCR1A = 0;            //
  TCCR1B = 0;            //
  TCNT1 = 0;            // Timer start value
  TCCR1B |= (1 << CS10);// 0 as Prescale value --> each CPU cycle is 1 tick

  // Initialize Pins
  pinMode(SX0_T0_OUT, OUTPUT);
  pinMode(SX0_T1_OUT, OUTPUT);
  pinMode(SX0_D_IN, INPUT);
  pinMode(PX0_P0_OUT, OUTPUT);
  pinMode(PX0_P1_OUT, OUTPUT);
  pinMode(PX0_PWR_OUT, OUTPUT);

  // Initialise Arrays with 0
  for (int i = 0; i < CHANNEL_NUMBER; i++) {
    sx0_channels_1[i] = 0x00;
  }
  for (int i = 0; i < CHANNEL_NUMBER; i++) {
    sx0_channels_2[i] = 0x00;
  }


  // Test Lok mit Adresse 33
  sx0_channels_1[32]=0xFF;
  sx0_channels_2[32]=0xFF;
  sx0_channels_1[33]=0xFF;
  sx0_channels_2[33]=0xFF;
  
  digitalWrite(PX0_PWR_OUT, HIGH);


  // Initialize Array Pointer
  active_channels = &sx0_channels_1[0];
  passive_channels = &sx0_channels_2[0];
}

void loop() {
  if (clockcycle) {
    // Enter code here

    // Time Consuming loop with nop (only one CPU cycle)
    do {
      __asm__("nop\n\t");
    } while (TCNT1 < 145); // equals roughly 10us --> measured

    // Reset Timer
    TCNT1 = 0;
    // Set Output for clock signal
    SX0_T0_PORT |= SX0_T0_OUT_PORT;
    clockcycle = false;

    setPXSignal(databit, false);
  } else
  {
    databit = nextBit();

    ////////////////////
    // Read Input Signal - ToDo !!!!!
    ////////////////////

    // Time Consuming loop with nop (only one CPU cycle)
    do {
      __asm__("nop\n\t");
    } while (TCNT1 < 630); // equals roughly 40us --> measured

    // Reset Timer
    TCNT1 = 0;

    // Reset Clock Signal Output
    SX0_T0_PORT &= ~SX0_T0_OUT_PORT;
    clockcycle = true;
    setPXSignal(false, true);

    // Set Data Output Bit
    if (databit) {
      SX0_T1_PORT |= SX0_T1_OUT_PORT;
    } else {
      SX0_T1_PORT &= ~SX0_T1_OUT_PORT;
    }
  }
}

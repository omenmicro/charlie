/**************************************************************
 * Arduino TINY BASIC with a 8080 Emulation
 * 
 * Arduino 8080 Virtual Machine with up to 4kB ROM (0x0000-0x0fff) and 1 kB RAM (0x1000-0x13FF)
 * 
 * OUT DFh sends data to serial port
 * IN DFh reads data from serial port
 * OUT 0xFE controls LED on pin13 (bit 0)
 * IN DEh returns serial status: 0x02 for no data on input buffer, 0x22 means data are available
 * 
 * 8080 emulator part is Copyright (C) 2012 Alexander Demin <alexander@demin.ws> under GPL2
 * Tiny Basic and Tiny Basic 2 are copylefted by LI-CHEN WANG
 * Source code for BASIC has been compiled with www.asm80.com
 * 
 */


#include "i8080.h"
#include "i8080_hal.h"

#include <Nokia5110.h>

Nokia5110 lcd(3, 2, 4, 5, 6); // (PIN_SCE, PIN_RESET, PIN_DC, PIN_SDIN, PIN_SCLK)

#define JOYX A0
#define JOYY A1
#define GBTN A2
#define RBTN A3

void bios(){
  while (digitalRead(RBTN)==0) {
    ; //wait until...
  }
  lcd.gotoXY(0,0);
  lcd.string("OMEN Charlie 1.0");
  while(1) {
    if (digitalRead(GBTN)==0) break;
    int joyx, joyy;
    joyx=analogRead(JOYX);
    joyy=analogRead(JOYY);
    Serial.print(joyx);
    Serial.println(joyy);
    delay(500);
  }
}

void setup() {
    Serial.begin(115200);
    pinMode(13, OUTPUT);
    pinMode(RBTN, INPUT_PULLUP);
    pinMode(GBTN, INPUT_PULLUP);
    pinMode(JOYX,INPUT);
    pinMode(JOYY,INPUT);
    lcd.setContrast(0xB1);
    lcd.init();
    lcd.clear();
    if (digitalRead(RBTN)==0) bios();
    i8080_init();
    i8080_jump(0);
}

// for debug purposes
void examine() {
    Serial.print("\nA:");
    Serial.print(i8080_regs_a());
    Serial.print(" BC:");
    Serial.print(i8080_regs_bc());
    Serial.print(" DE:");
    Serial.print(i8080_regs_de());
    Serial.print(" HL:");
    Serial.print(i8080_regs_hl());
    Serial.print(" PC:");
    Serial.print(i8080_pc());
    //Serial.print("\n");
}

void loop() {
    //delay(500);
    //examine();
    i8080_instruction();
    if (digitalRead(RBTN)==0) Serial.println("RED");
}

//// MEMORY DEFINITIONS

//test for 8080 emu
const byte PROGMEM  ROM[4096] = 
{ 0xF3, 0x31, 0x00, 0x84, 0x21, 0x00, 0xFC, 0x0E, 0x11, 0x79, 0x77, 0x07, 0x4F, 0x23, 0x7C, 0xB5, 
/* 0010 */  0xC2, 0x09, 0x00, 0x3E, 0x30, 0xD3, 0x07, 0x3E, 0x48, 0xD3, 0x05, 0x3E, 0x65, 0xD3, 0x05, 0x3E, 
/* 0020 */  0x6C, 0xD3, 0x05, 0x3E, 0x6C, 0xD3, 0x05, 0x3E, 0x6F, 0xD3, 0x05, 0xC3, 0x2B, 0x00};

//Uncomment for TINY BASIC v1
//#include "basic.h"

//Uncomment for TINY BASIC v2
//#include "basic2.h"

#define RAMBEG 0x8000
#define RAMEND 0x8400
#define VIDRAMBEG 0xFC00
#define VIDRAMEND 0xFFFF

//some initial RAM constants for Tiny BASIC 1
byte RAM [RAMEND-RAMBEG] = {0xff,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x15,0x10};
byte VIDRAM[512];

#define RAMBEG 0x8000

int vidram(int addr) {
  // pos Y addr&0x007f
  // pos X (addr&0x0380)>>7
  return ((addr&0x0380)>>7)*84+(addr&0x007f);
}

// 1111 11xx xyyy yyyy
byte readByte(int addr) {
    if (addr>=0 && addr < 0x2000) return pgm_read_byte_near(ROM + addr);
    if (addr>=RAMBEG && addr < RAMEND) return RAM[addr-RAMBEG];
    if (addr>=VIDRAMBEG && addr < VIDRAMEND) return VIDRAM[vidram(addr)];
    return 0xFF; //void memory
}

void writeByte(int addr, byte value) {
    if (addr>=RAMBEG && addr < RAMEND) {RAM[addr-RAMBEG]=value;return;}
    if (addr>=VIDRAMBEG && addr < VIDRAMEND) {
      if ((addr&0x007f)>83) return;
      if ((addr&0x0380)>0x0280) return;
      VIDRAM[vidram(addr)] = value;
      //if (!(addr&0x007f)) Serial.println(vidram(addr));
      lcd.gotoXY(addr&0x007f,(addr>>7)&7);
      lcd.write(LCD_DATA,value);
      return;
    }
}

//// HAL - Hardware Abstraction Layer for Emulator

int i8080_hal_memory_read_byte(int addr) {
    return (int)readByte(addr);
}

void i8080_hal_memory_write_byte(int addr, int value) {
    writeByte(addr,value);
}


int i8080_hal_memory_read_word(int addr) {
    return readByte(addr) | (readByte(addr + 1) << 8);
}

void i8080_hal_memory_write_word(int addr, int value) {
    writeByte(addr, value & 0xff);
    writeByte(addr + 1, (value >> 8) & 0xff);
}

int i8080_hal_io_input(int port) {
    switch (port) {
    case 0xde: //serial status
        return Serial.available() ? 0x03 : 0x02;
        break;
    case 0xdf: //serial input
        return Serial.available() ? Serial.read() : 0;
        break;
    default:
        return 0xff;
    }
}

void i8080_hal_io_output(int port, int value) {
    switch (port) {
    case 0xdf: //serial out
        Serial.print((char)(value & 0x7f));
        break;
    case 0xfe: //led control
        digitalWrite(13, value & 0x01);
        break;
    case 0x05:
        lcd.character(value);
        break;
    case 0x07:
        lcd.gotoXY((value&0x0f)*5,(value>>4)&7);
        break;
    default:
        break;
    }
}

void i8080_hal_iff(int on) {
    //no interrupts implemented
}

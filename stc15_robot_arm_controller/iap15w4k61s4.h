#ifndef IAP15W4K61S4_H
#define IAP15W4K61S4_H

typedef unsigned char  u8;
typedef unsigned int   u16;
typedef unsigned long  u32;

sfr P0   = 0x80;
sfr SP   = 0x81;
sfr DPL  = 0x82;
sfr DPH  = 0x83;
sfr PCON = 0x87;
sfr TCON = 0x88;
sfr TMOD = 0x89;
sfr TL0  = 0x8A;
sfr TL1  = 0x8B;
sfr TH0  = 0x8C;
sfr TH1  = 0x8D;
sfr AUXR = 0x8E;
sfr P1   = 0x90;
sfr P1M1 = 0x91;
sfr P1M0 = 0x92;
sfr P0M1 = 0x93;
sfr P0M0 = 0x94;
sfr P2M1 = 0x95;
sfr P2M0 = 0x96;
sfr SCON = 0x98;
sfr SBUF = 0x99;
sfr IE   = 0xA8;
sfr P3   = 0xB0;
sfr P3M1 = 0xB1;
sfr P3M0 = 0xB2;
sfr P4M1 = 0xB3;
sfr P4M0 = 0xB4;
sfr IP   = 0xB8;
sfr P4   = 0xC0;
sfr IAP_DATA  = 0xC2;
sfr IAP_ADDRH = 0xC3;
sfr IAP_ADDRL = 0xC4;
sfr IAP_CMD   = 0xC5;
sfr IAP_TRIG  = 0xC6;
sfr IAP_CONTR = 0xC7;
sfr P5   = 0xC8;
sfr P5M1 = 0xC9;
sfr P5M0 = 0xCA;

sbit P00 = P0 ^ 0;
sbit P01 = P0 ^ 1;
sbit P02 = P0 ^ 2;
sbit P03 = P0 ^ 3;
sbit P04 = P0 ^ 4;
sbit P05 = P0 ^ 5;
sbit P06 = P0 ^ 6;
sbit P07 = P0 ^ 7;

sbit P10 = P1 ^ 0;
sbit P11 = P1 ^ 1;
sbit P12 = P1 ^ 2;
sbit P13 = P1 ^ 3;
sbit P14 = P1 ^ 4;
sbit P15 = P1 ^ 5;
sbit P16 = P1 ^ 6;
sbit P17 = P1 ^ 7;

sbit P30 = P3 ^ 0;
sbit P31 = P3 ^ 1;
sbit P32 = P3 ^ 2;
sbit P33 = P3 ^ 3;
sbit P34 = P3 ^ 4;
sbit P35 = P3 ^ 5;
sbit P36 = P3 ^ 6;
sbit P37 = P3 ^ 7;

sbit P40 = P4 ^ 0;
sbit P41 = P4 ^ 1;
sbit P42 = P4 ^ 2;
sbit P43 = P4 ^ 3;
sbit P44 = P4 ^ 4;
sbit P45 = P4 ^ 5;
sbit P46 = P4 ^ 6;
sbit P47 = P4 ^ 7;

sbit P50 = P5 ^ 0;
sbit P51 = P5 ^ 1;
sbit P52 = P5 ^ 2;
sbit P53 = P5 ^ 3;
sbit P54 = P5 ^ 4;
sbit P55 = P5 ^ 5;

sbit IT0 = TCON ^ 0;
sbit IE0 = TCON ^ 1;
sbit IT1 = TCON ^ 2;
sbit IE1 = TCON ^ 3;
sbit TR0 = TCON ^ 4;
sbit TF0 = TCON ^ 5;
sbit TR1 = TCON ^ 6;
sbit TF1 = TCON ^ 7;

sbit RI  = SCON ^ 0;
sbit TI  = SCON ^ 1;
sbit RB8 = SCON ^ 2;
sbit TB8 = SCON ^ 3;
sbit REN = SCON ^ 4;
sbit SM2 = SCON ^ 5;
sbit SM1 = SCON ^ 6;
sbit SM0 = SCON ^ 7;

sbit EX0 = IE ^ 0;
sbit ET0 = IE ^ 1;
sbit EX1 = IE ^ 2;
sbit ET1 = IE ^ 3;
sbit ES  = IE ^ 4;
sbit ET2 = IE ^ 5;
sbit EA  = IE ^ 7;

#define AUXR_T0_1T     0x80
#define AUXR_T1_1T     0x40
#define AUXR_UART1_T2  0x01

#endif

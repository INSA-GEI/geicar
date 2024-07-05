# This file describes the main steps to configure the NucleoL476 and the GPS

## Hardware 
Redirection of UART2 to arduino connectors (for communication with GPS) (initially, UART2 was dedicated to ST-Link debug)
Change solder bridges :
* SB13 and SB14 → OFF
* SB62 and SB63 → ON

## Software

**Configuration changes are required on the gps (Rover module).** Using the u-center v1 software (https://www.u-blox.com/en/product/u-center), activate the UBX-NAV-PVT frames on uart1:
* CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1 
* CFG-UART1OUTPROT-UBX = 1

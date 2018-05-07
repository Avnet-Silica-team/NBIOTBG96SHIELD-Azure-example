/**
******************************************************************************
* file    readme.txt
* Version V1.2.0
* date    03-May-2017
******************************************************************************
* Attention
*
* COPYRIGHT(c) 2017 STMicroelectronics
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

Application Description 

This firware is one Boot Loader that must run at the Flash beginning (0x08000000 address) and has the purpouse to apply the Firmware-Over-The-Air (FOTA) updated receveid,
already checked and stored in Flash, or run the program normally if there is not one update.
For Nucleo-L476RG, the 1Mbytes of Flash is split on 2 banks of 51Kbytes each one, and each bank is split on 256 pages of 2Kbytes.

|Page 0->7  | Page 8->255       | Page 256->511 |
|16K        | 496K              | 512K          |
|0x08000000 |0x08004000         |0x08080000     |
|-----------------------------------------------|
|BootLoader | Running Program   |  FOTA         |


This Boot Loader must be Loaded on at the flash beginning (0x08000000) and it checks if there is one FOTA stored on 0x08080000 address:
- If YES:
  - it erases the sectors from 1 to 5
  - it copies the FOTA on that sectors
  - it erases the FOTA region after the update.
- if NOT:
  - it works like a trampoline for executing the normal program stored from address 0x08008000

The FOTA must be less than 496Kbytes


The "Running Program" and the FOTA are compiled for running from 0x08004000 address. If they are placed at the beginning of the FLASH they doesn't work.
And without the BootLoader that programs could not be executed.

For Nucleo-F429ZI the 2Mbytes of Flash memory is split in 2 banks of 1Mbyte each, and each bank is divided into 4 sectors of 16Kbytes, 1 sector of 64 Kbytes, and 7 sectors of 128 Kbytes.

Even if the Flash is 2Mbytes, we use only 1Mbyte and, like for Nucleo-L476, is split on 3 regions:

| sector 0   | sector 1 to sector 7 | sector 8 to sector 11 |
| 16K        | 496K                 | 512K                  |
| 0x08000000 | 0x08004000           | 0x8080000             |
|-----------------------------------------------------------|
| BootLoader | Running Program      | FOTA                  |



                               -------------------
                               | VERY IMPORTANT: |
                               -------------------


Inside the BootLoader.zip archive there is the source code and IAR project for the BootLoader.
For using it, it's necessary to unpack it on the Projects\Multi\Applications folder.


/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
 
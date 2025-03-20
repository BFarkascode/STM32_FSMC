# STM32_FSMC
Bare metal implementation of LCD driving using FSMC. Project showcases screen driving and the touchscreen on the STM32F412ZG Discovery board

## General description
When I was making my digital camera project on the STM32F429 Discovery board, one of the banes of my existence had been the apparent incompatibility between the DCMI driving of the camera and LTDC driving of the in-built LCD screen. The two peripherals on that disco board were just not meant to work together, with multiple critical pins overlapping/muxed together. It took me a rather sketchy creative approach to force them to work side by side in the end (see my STM32 digicam project), a solution that was neither elegant nor particularly efficient.

I noticed though that there are other options to drive the LCD screen differently using the M4 Cortex STM32 mcus, namely something called the Flexible Static Memory Controller (FSMC) or Flexible Memory Controller (FMC) – the two being practically the same just given a different name depending on if we are discussing the F41x or the F42x mcu family. At any rate, the FSMC pins on the F429 were not overlapping with the DCMI, allowing – theoretically – parallel function between the two peripherals.

So, let’s check, how this peripheral works by making it send an image to an LCD screen. And, while at it, let’s add touch screen capabilities to the setup for good measure…like, can we draw something on the our screen?

### A recap on screens
Just a short recap over the pitfalls when we are driving an LCD screen…

…the screen and the screen driver are separate things. We are talking to the driver IC and the driver IC is talking to the screen. We aren’t directly driving the screen.

…there are multiple possible screen driver ICs. We will need a library to match the driver we are using. A skeleton code for each library is provided by ST in its firmware packages for each mcu (look for “Drivers/BSP/Components”).

…we will have a command bus and a data bus. What these busses are is defined by how the driver IC is hard-wired (usually called the IM pins in the schematics) and by programming the IC itself through the command bus. The command bus doesn’t always need to be the same as the data bus.

…we will have a refresh rate of the screen – how fast it is publishing the data stored in its frame memory – and a refresh rate of the frame memory itself – how fast we can replace the frame memory content. The screen refresh is an IC command while the memory refresh is dependent on the data bus specs and the offloading capacity of the mcu we are using.

…the driver IC doesn’t know, what the screen is supposed to show. As such, we will need to place the right pixels at the right places within the IC’s frame memory to receive the output we wish to have. Mind, the IC driver can read out and publish its frame memory in many different directions, depending on what is programmed in it. So, be careful how the memory is loaded and how it is read out. Obviously, the two should be the same.

### F412ZG Disco vs F429ZI Disco
A little disclaimer first: we will be using a different board than before. Why? Because on the F429 Disco the screen is hard-wired to the LTDC pins (while the FSMC is on the SDRAM) so that board cannot be made to tun the LCD using FSMC. I will be using the F412 Disco board instead where the in-built screen is connected to the mcu through FSMC. Also, the F429 is hard-wired to be commanded through the SPI bus, something we don’t want to use here.

Anyway, what does this mean for the project compared to what we have already done using the F429?

Firstly, we have a different screen and a different driver IC: the ST7789. On the surface level, this doesn’t mean much, we can still use the same philosophy we had for the F429 Disco’s driver IC. On the other hand, if we intend to tinker with the screen more profoundly and step out from the ST-defined configuration values, the datasheet of the IC will need to be consulted. We will be using the ST-provided ST7789 driver library to the letter, but I will point out some important registers that could be modified if a different configuration will be needed.

Secondly, we will be using a different peripheral to drive the screen AND to feed the screen compared to the F429. On the F429, the IC driver was hard-wired to demand SPI as the command bus and then we could either use SPI or the RGB parallel interface (LTDC) for the data bus by changing the configuration of the IC. On the F412, we will be using the parallel 8080-Series MCU interface protocol running at 16-bit width. This is what the IC is hard-wired to run with. (Of note, this doesn’t mean the driver isn’t capable to run the RGB parallel interface, most ICs can actually use many different sets of command and data busses, including the ST7789.) As it goes, the FSMC can be set up to exactly match the timing characteristics of the 8080-Series protocol.

Lastly, we will have only 240x240 pixels which is not a VGA standard. This should be kept in mind when generating image output. (Luckily, we will be recycling much from the digicam project where we already were manually dialling back to 240x240). The pixel format will stay as RGB565, though this can be changed by manipulating the driver’s 0x3A register.

Regarding the touch sensor, we have a the FT6x06 sensor built within the screen. Similar to the ST7789, we will be taking the library for it from the ST component list as well and just write the IO functions to interface with it.

### FSMC
So what exactly is the FSMC?

Checking the refman, it wasn’t exactly clear to me, so let me give a definition on my own words: it is a transit buffer or 16-bit parallel DMA than can be loaded as fast as HCLK and then offloaded by a speed that is defined by the FSMCs write timing register and the HCLK.

This transit buffer is actually at pre-defined address within the mcu’s memory bank already and we cannot move it around. We can load it by simply pointing to the start address (of the data section) of the FSMC using a pointer but we can’t write to any random memory position of the transit buffer. Due to this, the FSMC behaves like a linear FIFO when loaded or off-loaded.

The FSMC does have two different sections, the first – the command section – starting at the base of the  FSMC memory, the second – the data section – starting at an offset from the base defined by the register select (RS pin) of the FSMC. Since we are linearly loading the FSMC, this practically means that by selecting the appropriate RS pin, we carve off a part of the transit memory designated for commands and registers. This ensures that the FSMC will always send a register or a command – the size defined by the “RS – to the device the FSMC is interfacing with before any data would be sent over. For 8-bit command registers, A0 will need to be selected as RS.

Mind, at which base address we are engaging the FSMC at defines which memory type of FSMC we are using and which chip select. The chip select should be self-explanatory, where the FSMC is divided into multiple banks, each bank capable to service a different connected device, each bank on the F412 being 64 MB deep. (The banks will be at different addresses though, only bank1 chip select 1 starting at the usually given 0x6000000, so pay attention.)

Regarding the types, we can have NOR, NAND and PCCard memory types. The LCD can be driven uniquely by the NOR type. Why that is, I am not sure, but probably because it is a NOR type flash memory that stores the frame within an LCD driver IC.

There is also an additional funkiness when looking at the “command” and the “data” sections of the FSMC memory when using 16-bit width FSMC: while using 8-bit FSMC, the pointer address should be constructed as addr[31:28]BANKSELECT,  addr[27:26]CHIPSELECT, addr[25:0]FSMCADDRESS with the , for 16-bits, it is addr[31:28]BANKSELECT,  addr[27:26]CHIPSELECT, addr[25:1]FSMCADDRESS, [0]DONTCARE. The A0 as RS for the 8-bit version will be the [0] bit, while for the 16-bit version, it will be bit[1] with bit[0] as “don’t care”. This is probably to consider the FSMC’s internal pointer not moving by 8-bit strides by by 16-bit strides. In our case, the command section of the FSMC will start at 0x60000000 and the data section as 0x60000002 since we will be using the NOR type and a chip select of NE1.

#### FSMC connections
Let’s quickly rush through the FSMC connections and discuss, what they are doing:

RS on PF0: the A0 register select. As mentioned above, it sets the offset between the command/register and the data sections of the FSMC memory

NE1 on PD7: the chip select for the NE1 bank within the FSMC. We will need to pick our base memory address according to this value. Different NE1 values will define the base address with an offset of 64 MB. (NE4 will be at 0x6C000000, for instance.)

NOE on PD4 and NWE on PD5: the read enable and write enable lines. Since the data lines are bidirectional, we need a read and a write enable to tell the device on the receiving end of the FSMC signal if it should send or receive data.

D[0:15] on multiple pins: this is the bidirectional data bus. Mind, these also function as the address bus since that is multiplexed on the same output.

## Previous relevant projects
Many of the findings we had for the original digicam project are worth checking to understand the bigger picture:
- STM32_SCREEN_LTDC_DCMI_OV7670_SPI_SDcard

## To read
I am going through the related ControllersTech project’s first two parts without using HAL: 

https://controllerstech.com/stm32-fmc-how-to-configure-for-lcd/

## Particularities
### Confusing registers
A word of caution: the FSMC registers (BCR and BTR) are organized into an 8-element array on the F412G called the “FSMC_Bank1->BTCR” registers. Despite what the name suggests, the “FSMC_Bank1” register array actually controls all 4 banks of the FSMC within the mcu (BTCR[0 is the BCR, BTCR[1] is the BTR for Bank1 and so on). I guess this might be related to the fact that the F412G only has the NOR type of controller and not the NAND and the PCCard ones that are available on the F429.

At any rate, we only need to set the “FSMC_Bank1->BTCR[0]” and the “FSMC_Bank1->BTCR[1]” registers to make the LCD work. We are driving the LCD in normal mode (EXTMOD is not actived) so the contents of “FSMC_Bank1->BWTR” is ignored.

### Timing the FSMC
For the timing of the FSMC, the calculations are coming from the AN2784 application note.

The thing is that the FSMC will publish whatever is stored within itself on its data pins with a timing that needs to set up locally. This timing has to meet the timing demand of the bus that the peripheral is feeding. In our current case, the 8080-Series MCU interface protocol will have a write cycle of 66 ns and a HIGH/LOW hold rate at 15ns as seen in the ST7789’s data sheet (page 41). This practically means that to pass through one packet of data successfully to the driver, the data packet has to be available on the bus for at least 66 ns. Similarly, and HIGH or LOW signal on the bus must be kept steady for at least 15 ns for the driver to recognise that it is a HIGH or a LOW signal. (This is very similar to how we had to bitbang the neopixel driving in a previous project by defining the PWM signal width as 3 for LOW and 6 for HIGH.) From these values and the calculations mentioned in the application note, we can define the address setup time as “3” and the data setup time as “2” when running at 100 MHz.

All these values are to be written into the Bank1 BTR (well, BTCR[1]) register if HAL is not used with address hold, clock divide and latency kept as default (maximum value, all 1-s). Mind, I simply took these values from a working HAL solution and didn’t change them.

### Non FSMC connections
There are additional pins that we need to deal and connect the screen properly:

LCD_Backlight on PF5: this is a PWM signal that will control the backlight of the screen. As it goes, the driver IC of the screen only sets up the colour of the pixels but how bright they will be will depend on the input from this pin. We can add a PWM signal to increase or decrease the brightness of the screen. Here, we will leave it at simple HIGH, which is maximum brightness.

LCD_Reset on PD11: the reset of the driver IC is broken out and connected to the mcu. It is recommended to make good use of it and reset the screen when powering up the setup.

LCD_TE on PG4: this is the tearing effect output from the driver IC towards the mcu. This is a synchronization signal that could be used to synch up the readout of the driver IC from the IC’s internal memory and the loading of the internal memory by the mcu. We can enable or disable this by manipulating the 0x34 and the 0x35 registers. The provided library does not activate it so we can leave this GPIO undefined.

I2C SDA on PB7 and I2C SCL on PB6: connections to talk with the touch sensor.

CTP reset on PF12: GPIO output to reset the touch sensor.

### Getting the ST7789 and FT6x06 libraries to work
To be fair, there isn’t much to say about the ST-provided library for the ST7789. It works perfectly well, albeit it does configure the screen to a certain setup without any function written to it change them. At any rate, any kind of modification to the driver is done by calling the “LCD_IO_WriteReg()” with the register address and then the “LCD_IO_WriteData()” with the data to be sent over, technically the same as sending image data over to the driver IC.

### A few words on I2C within the F4xx family
Until this very moment, I hardly needed I2C to work on an F4xx device. Even when I did use it – like in the digicam project - I simply set the I2C and its clocking (different compared to the L0xx!!!), wrote a crude I2C Tx and went on to focus on more important (and interesting) matters. Well, for this project, I had to remedy that issue and write a fully functioning I2C driver for the F4xx using bare metal.

I don’t want to beat around the bush, I have simply cloned the HAL solution using bare metal coding. Why? Because for the life of me I could not get a hang of the convoluted flag system the peripheral is using on Rx, how they are set and when (see section 24.3.3 in the F412 refman). In a nutshell, one must set the peripheral before a transmission occurs and that setup MUST occur well ahead of time (2 bytes or so), otherwise the appropriate START/ACK/NACK/STOP signals will not be generated on the bus when doing Rx (Tx, on the other hand, is completely straight-forward, so that’s that). Mind, this was automatically dealt with when running I2C in an L0xx.

Anyway, the solution works now and if we flinch and turn our heads 90 degrees to the left, we can make the connections between what is written in the refman and what I have cloned within my code. I just can’t be bothered to figure them all out.

## User guide
There are two outputs hidden behind the “#ifdef” lines.

The first publishes an image to the screen at around 35 Hz.

The second wipes the screen and allows the user to draw on it with one finger. When the screen is touched by two fingers, the screen is reset. Please note that the touch sensor can not react to fast movements so for a continuous line drawing, one should move the drawing finger slowly.

## Conclusion
This was an interesting and fun little project that has managed to provide me some tools to bypass the issues I have faced with previous screen driving solutions.

Mind, the F412ZG does not have a DCMI driver so it cannot be used to replace the F429ZI used in the digicam project!


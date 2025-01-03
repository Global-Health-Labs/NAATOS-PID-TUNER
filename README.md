NAATOS PID TUNER for MK GENERATION PCB 
Codeset compatible with https://github.com/Global-Health-Labs/NAATOS-V2.git

***KNOWN ISSUES***
sTUNE library is bulky and difficult to fit on ATtiny1604 chipset.
Currently unable to run 2 heater PID tuning algorithms simultaniously (ie. cannot have both OBJECTS at compile time).
Thus, must recomplie for each heater target. Which is a major pain. I was able to optimize the FLASH to about 99% capacity, so the firmware would compile but the MCU would not boot at runtime. 

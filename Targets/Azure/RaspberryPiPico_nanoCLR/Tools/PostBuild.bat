
C:\SysGCC\arm-eabi\bin\arm-none-eabi-objdump -d -EL -S %1.elf  >  %1.lst

C:\SysGCC\arm-eabi\bin\arm-none-eabi-size -A -x %1.elf > %1.size

C:\SysGCC\arm-eabi\bin\arm-none-eabi-nm -C  -l %1.elf > %1.nm

%2Tools\ELF2UF2.exe %1.elf  %1.uf2





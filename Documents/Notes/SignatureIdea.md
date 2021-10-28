#### Add a signature to the Firmware for valid CLR detection

```C
 char __attribute__((section(".signatureSection"))) signatureValue[] = {'n', 'a', 'n', 'o' , 'C' ,'L', 'R' , '_' };
```



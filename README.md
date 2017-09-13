# Sensorless BLDC Driver
Sensorless brushless motor controller using PIC18F1230 (XC8)  
* No ESC  
* PWM  
* ADC  

-------------------------------------------
# Feature(Implemented)  
* Open-loop drive.  
* Closed-loop drive(seamless transfer from open-loop).  
* Closed-loop lock detection(back to open-loop).  

-------------------------------------------
# My parts list  
PIC: PIC18F1230 http://ww1.microchip.com/downloads/en/DeviceDoc/39758D.pdf  
NPN Tr: 2SC1815  
MOSFET: SLA5064 http://www.semicon.sanken-ele.co.jp/sk_content/sla5064_ds_en.pdf  
Motor : A2212 13T 1000KV  

-------------------------------------------
# Video  
Closed-loop test: https://youtu.be/z_nNao-uBho  

-------------------------------------------
# How to use  
1. install MPLAB X / XC8  
2. create PIC18F1230 project  
3. create main.c in "Source Files"  
4. copy one of the main(???).c  

-------------------------------------------
Sorry for the terrible English  

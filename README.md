## Digital Signal Processing using an STM32 Nucleo Board, featuring stereo audio input and output, along with a color display.

Main Components:
- STM32 NUCLEO-L432KC: https://www.st.com/en/evaluation-tools/nucleo-l432kc.html
  - (Remove SB16 and SB18 solder bridges on the STM32 Nucleo-32 board!) 
- Digilent Pmod I2S2: https://digilent.com/shop/pmod-i2s2-stereo-audio-input-and-output/
- 3.2" TouchScreen ILI9314: https://de.aliexpress.com/item/1005003120684423.html

Use the information in the folder "Hardware" to connect the three boards.

The folder "VUmeter_FFT_Demo" contains source files for software that demonstrates some of the hardware's capabilities. STM32 CubeIDE was used as the development platform.

Find a short video on [youtube](https://youtu.be/S4vHsiX3N3Y?si=EG1q6hBVTR3Ls1wu). 

The driver for controlling the display was developed by [Mauro De Vecchi](https://github.com/maudeve-it/ILI9XXX-XPT2046-STM32).

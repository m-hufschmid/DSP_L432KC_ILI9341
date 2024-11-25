## Digital Signal Processing using an STM32 Nucleo Board, featuring stereo audio input and output, along with a color display.

Main Components:
- STM32 NUCLEO-L432KC: https://www.st.com/en/evaluation-tools/nucleo-l432kc.html
  (Remove 
- Digilent Pmod I2S2: https://digilent.com/shop/pmod-i2s2-stereo-audio-input-and-output/
- 3.2" TouchScreen ILI9314: https://de.aliexpress.com/item/1005003120684423.html

Use the information in the folder "Hardware" to connect the three boards.

The folder "VUmeter_FFT_Demo" contains source files for software that demonstrates some of the hardware's capabilities. STM32 CubeIDE was used as the development platform.

The driver for controlling the display was developed by [Mauro De Vecchi](https://github.com/maudeve-it/ILI9XXX-XPT2046-STM32).

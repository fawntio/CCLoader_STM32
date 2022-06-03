This repository was created to flash the HM10 CC254x bluetooth module using the STM32 microcontroller.
Programming was carried out in the STM32CubeIDE application based on the STM32F407T6 microcontroller.
To flash, please follow steps:
1. Download the project folder and open it in the STM32CubeIDE program.
You can also download the stm32_ccloader.hex binary file and open it with ST-LINK Utility. Flash firmware to your microcontroller.
2. Connect the pins of the controller and the bluetooth module as shown in the picture:

 ![image](Connection.png)

3. Run the program CCLoader.exe to load the Demo.bin in the terminal.




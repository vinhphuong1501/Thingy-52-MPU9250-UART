# Thingy-52-MPU9250-UART
This project is created to read raw accelerator values from MPU9520 motion sensor integrated in Thingy52 and send data to UART over BLE. The nRF Toolbox will receive those data through UART.
To run the project:
  1. The Thingy app needs to be download from https://www.nordicsemi.com/eng/Products/Nordic-Thingy-52
  2. I used Segger, so i need the project configuration from https://devzone.nordicsemi.com/f/nordic-q-a/33599/thingy52-debugging-under-ses
  3. Then replace the 3 files m_ble.c, main.c, sdk_config.h with the files provided in this repository.

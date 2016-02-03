# USB_STM32F4
This is a firmware for stereo exciter for FM broadcasting. It receives digital sound over USB, oversamples it 8x times and converts them into MPX
signal. Also a CDC device is implemented in composite USB device for control and transmitting the RDS data.
MPX signal is output via built-in DAC of STM32 and also FM radio signal is output on the air by AD9951 onboard.

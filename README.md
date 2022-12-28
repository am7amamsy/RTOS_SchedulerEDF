Creating an EDF-Scheduler-Based RTOS based on the FreeRTOS library files where the scheduler handles six tasks in total:
- Two tasks are concerned with the levels of two GPIOs respctively.
- One task is concerned with sending a periodic string to the UART channel.
- One task is concerned with displaying the GPIO level changes and the periodic string.
- Two idle tasks in order to calculate the CPU load

The explanation of the system can be found in detail in the videos included in the Documents folder.

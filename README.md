# The purpose of this assignment was to simulate a thermostat on a TI CC3220x-LaunchXL board. This project utilizes GPIO, UART2, I2C, and a timer for peripherals. In this project, GPIO is used to take button presses in as input to adjust the setTemp value. If setTemp is set higher than the observed temperature of the temperature sensor, LED0 is turned on to indicate the heater is being turned on. UART2 is implemented to output information about collected data to the terminal. This includes the current temperature, the setPoint temperature, whether heat is set on or off, and how many seconds have passed. I2C is used to take the current temperature in as input from the temperature sensor. The timer is used in order to create a task manager to allow the system to make checks according to the project specifications.
# In this project, I believe I designed the task manager particularly well. I believe my logic branches to make checks based on elapsed time met the specifications particularly well. I think I can improve on my implementation of some peripherals used by the system. I ran into a challenge with the implementation of UART2. After reviewing the documentation provided by TI, I found the proper implementation for UART2. This project provided me with a new skill, working with embedded systems. This allows me to consider more strenuous constraints when developing software. Understanding and considering system constraints can be applied to other projects and can allow for a deeper understanding of projects in the future. I included proper naming conventions and well-written comments to ensure this project is maintainable, readable, and adaptable.

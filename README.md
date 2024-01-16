# BQ34Z100G1 Driver

By USC Rocket Propulsion Lab  / Arpad Kovesdy, Kyle Marino, Jamie Smith

After lots of development, testing, debugging, dead ends, backtracking, and retracing our steps, we at RPL are proud to present a complete driver for Texas Instruments' BQ34Z100G1 state-of-charge estimator.   This handy chip handles all the complexities of monitoring a battery's charging and discharging cycle, and is one of the only real ways to have a real estimate of how much usable energy is left in a standard lithium-ion battery.

However, in order to perform this function, the IC needs a lot of information, including both the on-paper specifications of your battery system and the results of several different calibration runs of your real battery system.  With one exception*, this driver can handle all the needed configuration, taking you from unconfigured to calibrated in as little time as possible.

* Note: To initially program the Chem ID information into each chip to configure it for your battery, you will need an EV2300 or EV2400 programmer box from TI, as well as a header to plug it in on your board.

## Calibrating the Chip
See [here](https://os.mbed.com/users/MultipleMonomials/code/BQ34Z100G1/wiki/Setup-and-Calibration-Guide).

## Credits
The initial version of this driver was taken from Ralim on GitHub [here](https://github.com/Ralim/BQ34Z100). 

However, we have made a huge number of changes and additions since then, including porting the Arduino library to Mbed, cleaning up the code to use enums, breaking out the configuration to external constants, and fixing looots of bugs.
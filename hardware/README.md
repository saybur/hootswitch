Hootswitch Hardware
===================

This folder has the hardware design being used to develop the firmware. Known
issues with it include:

- Some features are untested, including alternate power modes.
- C1 in the BOM is wrong. Since I am not using the power input at the moment I
  left it unpopulated.
- There is no keepout for the wireless chip, which may negatively impact
  reception.
- Annular rings on the mini-DIN connectors are on the small side.
- Green LEDs are too bright. Adjusted in 2024a with PWM. Future revision needs
  stronger resistors.
- Mounting holes are too small for the standard screws. These are also a tad
  too close to the connectors.
- A connection should be made between host power-on and a free GPIO. This can
  be worked around in software with keyboards.
- It might have been wiser to put bus input pins to GPIOs with PWM B inputs.
  This will probably not be changed, PIO handles everything fine.

Anticipate that a future revision will be made with corrections.
[CHANGES.txt](CHANGES.txt) will have any applicable history.

This project is made available under the CERN Open Hardware Licence
strongly-reciprocal variant version 2, available at [LICENSE](LICENSE).

This is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING OF
MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A PARTICULAR PURPOSE.
Please see the CERN-OHL-S v2 for applicable conditions.

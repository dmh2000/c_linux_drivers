linux-modules
=============

example linux device driver modules I developed around 2006.
They were developed for an embedded system that had a 2.6 kernel. The CAN bus drivers were developed before the Linux kernel had built-in CAN bus support so a custom driver was required. The Multicomm serial driver did not have a compatible driver in the kernel so I developed on for it also.

- SJA1000 Phillips CAN Bus Controller (this was deployed for motor control in a large UAV system)
- i82527 Intel CAN Bus Controller 
- Multicomm PC/104 Multiple RS232/422 Serial Board

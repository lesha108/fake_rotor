# fake_rotor
Antenna AZEL rotator emulation with relays.
 
Emulates Easycom AZEL rotator protocol. Instead of controlling motors, it controls a couple of coaxial relays
 to switch set of uda-yagi antennas directed to dfferent AZ. My antennas have elevation about 30 degrees.
 And about 120 degrees between them.
 I use it for satellite communications with PstRotator. Set my AZEL controller there to K3NG (Easycom). 

Main program is for STM32 blupill board. Change switches.rs to depict your set of relays and uda-yagi directions.
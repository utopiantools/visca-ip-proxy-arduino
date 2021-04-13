# visca-ip-proxy-arduino
Uses a WiFi enabled microcontroller to proxy VISCA commands from UDP to RS-232 and vice versa.

The VISCA binary protocol is exactly the same over UDP as it is over wired connections.
However there are exceptions to this rule. When using VISCA over IP, Sony cameras employ
a header and footer surrounding each datagram. However, over a wired connection, they
do not use such a header and footer, and only send the VISCA packet directly.

This code works more like the PTZ Optics implementation. 

This code connects to WiFi and starts a UDP server.
This code also connects to a VISCA camera over RS-232.
On bootup, it will immediately send to the camera an ADDR_SET and POWER_ON command.

From that point on, the code will simply proxy all data packets between RS-232 and UDP
with very little error checking or validation. Whatever packet is received over UDP
will be sent to the camera without modification, and whatever packet is received from
the camera over RS-232 will be sent to the most recent UDP client without modification.

WITH A FEW EXCEPTIONS:
Cameras and controllers vary in their implementation of speed, and therefore this code
allows the programmer to customize various speed curves for the PTZ operations.

They are commented below.

USAGE:
Change the settings below, compile and upload!


FINALLY:
This code should work with minor modifications on any device that supports WiFi, RS-232,
but it was designed for the M5Stack Atom series devices with the RS-232 attachment.

As an added bonus, the Atom and Atom Matrix devices have RGB LEDs which allow us to
use them as tally lights. This code employs that using the vMix tally protocol. If it
successfully connects to vMix, it will by default make itself tally whatever input is
live when it booted. You can change the tally by pressing the main button on the Atom
or the Matrix. If you hold down the button, it will automatically set itself to whatever
input is PREVIEW.

NOTE: An RS-232 or RS-422 interface is definitely needed. The biggest concern is that
Android devices by default use TTL for serial communications, and that protocol uses
voltages that are incompatible with the RS standards.


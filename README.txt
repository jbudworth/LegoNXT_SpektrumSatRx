Code for the Arduino Pro Mini 5V 16MHz to allow the connection of a Spektrum 
Satellite Receiver to a Lego NXT Mindstorms.

This allows the Spektrum Rx to mimic the HiTechnic IRReceiver and using the same 
NXT-G software module the HiTechnic brick uses the data can be accessed the 
same way.
http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NIR1032

Arduino ground -> NXT pin 2 and 3
Arduino 5V     -> NXT cable pin 4
Arduino pin A4 -> NXT cable pin 6
Arduino pin A5 -> NXT cable pin 5

I got my cables from MindSensors:
http://www.mindsensors.com/index.php?module=pagemaster&PAGE_user_op=view_page&PAGE_id=139&MMN_position=50:50

More information on connecting to the NXT can be found here:
http://www.philohome.com/nxt.htm

The Rx is controlled/powered in much the same way as the DIYDrone Spektrum adapter.
http://store.diydrones.com/ProductDetails.asp?ProductCode=RC-SPEKTRUM-APMADPTR
But cut down to using a 3.3V regulator and a NPN transistor.

The Satellite Rx library can be found here:
https://github.com/jbudworth/SpektrumSatRx

The Arduino Pro Mini 3.3V 8MHz is not fast enough to decode the 115200 baud 
serial from the Rx so cannot be used.
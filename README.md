earthquake.arduino
==================

Arduino Code for a very sensitive seismic detector incl FFT fourier frequency analysis (rough)

Hi - this is my first post to GIT, so apologies for formatting etc..

This code is for an Arduino UNO - it's a hacked version of somebody else's FFT core; (will dig up their name to give them kudos);  the code also allows a simple user interface to adjust settings by serial interface.

Hooked up to a seismic geophone, (also works with solid state accellerometer) it's SUPER sensitive and good at rejecting false positives.  It's sensitive enough to 'fire' (see serial screen) when an apple is dropped on a bed mattress on the other side of the rooom. (or a knock at the door)

I have heaps of documentaiton on this 'for fun' project - feel free to email me for more info;

cheers!
ryan


---
Updates from when I dusted this off in September 2014:
- To 'activate' you need to open up serial comms window
- 115200 Baud (when you open serial comms window and press 's') to spew data
- Analog Pin A0 input
- Analog input MUST be DC biased at around 2.5 volts - and then the input sensor signal is Capacitively coupled  - so basically, find two large value resistors and divide 5v and 0v rails in half - THEN - run your geophone or speaker or whatever input transducer you have, into it.
- 
Example Stream:
-- NORMAL -- MAXPEAK 45 N(mp): 0 Freq(aprx): 0 //// TH, 5 77 THC, 27         DECAY, 5 ALERTBULB, 5

If MAXPEAK mp exceeds the TH(reshold) then the alerti fires.   The Frequency is used to flag dominant frequency component.


# arduino-pal
Generate simple gray scale AV output that can be displayed on CRT monitors and old TVs.

This sketch uses the Arduino Uno to display a dot zapping around the screen of an old CRT TV. It is written with PAL timing in mind and displays gray, black, and white. To use the sketch, connect a 470 Ohm resistor to pin 8, and a 1 kOhm resistor to pin 11, tie the resistors together and feed the tie point into a TV through the yellow RCA plug: center pin goes to the two resistors, and shield connects to Arduino ground.

![Screenshot of a CRT screen showing the result](example.gif)

The purpose of this project was for me to generate a picture on a TV screen. Along the way I've learned something about (the pain of) producing consistent timing. For example, I was surprised to find myself fighting the compiler by moving the declaration of variables around to get the picture free of jitter. Certainly not the mainstream flavor of software engineering. The end product is code that displays a field of white rectangles on a black background on a grid of 86 x 57, surrounded by a gray frame. The display field is held in memory and updated during the field blanking period.

I am grateful for previous work shared online that was very encouraging as it made the task seem easy. In the end I've implemented the "fake progressive" "monochrome PAL" video frames as reported by Batsocks here: https://www.batsocks.co.uk/readme/video_timing.htm

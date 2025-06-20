# AdaFruit NeoTrellis 4x4 RGB Keypad

This is an ESP IDF driver for this device.  It provides similar functionality to that provided by the Arduino Library but ommits all the additional code for other SeeSaw based devices.

Note that the key numbering is a little odd, it seems that the internal SeeSaw representation of the buttons was originally for a 4x8 grid device (M4?) and this has been kept in place
for the 4x4 device, so NeoTrellis Key 4 is actually SeeSaw key 8 and so on.  

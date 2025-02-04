Environment sensor and display for the CYD

A combo BMP280/AHT20 sensor board is attached to the CN1 connector. On the board I'm using, one pin was disconnected (oh the fun of these cheap boards) and needed to be connected to IO22 on P3 to match the normal design of a CYD.

Said sensor is read every now and then, and then put on the screen. Using LVGL to do the UI. Mostly this was just an excuse to play around with LVGL.

FIXME: improve readme
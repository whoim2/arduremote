# arduremote
ARDU Remote: simple USB / PPM RC radio controller transmitter 10 channels with QCZEK LRS

Based on arduino pro micro (atmega32u4), have h/w USB port for use as joystic in any OS.
Work with Open.HD.
Have simple calibrate procedure.
Have start protection and 30-secs warning if not used.

May work with QCZEK LRS and other, have PPM pin 8/10 channels.

QCZEK Project: http://qczek.beyondrc.com/qczek-lrs-433mhz-1w-lora-rc-link/

Low cost RC controller from old RC parts! 3D models present.

![alt text](https://github.com/whoim2/arduremote/blob/master/sheme.bmp?raw=true)

If you want use this only as usb joystic, you not need battery, power module, power switch and not need remove fuse on Arduino Pro Micro.
If you use PPM and LRS, you need set channels count in PPMEncoder.h to 10

=== SBUS TEST VERSION ===

QCZEK >=2.0 have only SBUS input. Try this sketch for it: https://github.com/whoim2/arduremote/blob/SBUS-test/ardu_remote.ino
Not need external inverter, but need resistors for down voltage from 5v to 3.3v. As for ppm pin on sheme. Connect TX pin arduino to RX pin qczek via resistors.

===

Video for russian users: https://youtu.be/x-7y0d95x8k
[![Watch the video](https://github.com/whoim2/arduremote/blob/master/photo_title.jpg?raw=true)](https://youtu.be/x-7y0d95x8k)

===

Build:
1) install Arduino IDE
2) copy folders from used_libs.zip to Arduino/libraries folder
3) open ardu_remote.ino in Arduino IDE, try to compile
4) connect Arduino Pro Micro to usb
5) select pro micro in tools/boards menu, press Upload button in IDE
6) solder all parts to Arduino, using sheme.bmp

===

For calibrate sticks & aux1, press calibrate button and power on, before set roll/pitch/yaw axis in zero position, trottle & aux1 set to minimum. While beeps, move sticks & aux1 from min to max positions.
If no beeps at time, power off device. Calibrate done.

Short beeps after power on - protection, channels value not in minimum. Off all switchs & move throttle to minimum.

===

You may connect jdy-30 / hc-05/06 bluetooth module to qczek and have mavlink telemetry on android device with "Droid Planner 2.8" or "Telemetry Vieawer" app. It work with Inav / mavlink.

Also, you may connect mavlink-display to QCZEK:
https://github.com/whoim2/mavlink-display

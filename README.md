# # Operation-SafeFlight
This repository contains all the necessary code files for my final thesis at the Kantonschule Sursee.

Credits to:
mjs513 (on GitHub): Modified the newPing library to work on teensy 4.x  (I'm mentioning him here anyway even though its fine for him if i don't)
    - https://github.com/mjs513/NewPing_t4/tree/master
(to get it to work i needed to repalce the newPing library content (from the original one) with the content of mjs513's newPing library, was quite easy just under .pio/lipdeps/teensy41/newPing delete all the content and replace it )

# *Warning:*
(The Warning stands here because i realised to late that it won't work and had already printed out the written documentation for the project)
The following is a concept work, meaning it is not fully functional. Due to the sensors not being able to refresh 50 times per second, the entire code won't work properly. However, the code that reads the PPM signals and sends them out again is fully functional and tested. The microcontroller is capable of reading and sending the PPM signals but cannot detect obstacles around it. Another factor is the speed of the microcontroller itself, which is insufficient for running all the sensors simultaneously. 

# *Improvements that need to be made for the system to work:*
1. Use a different obstacle detection system, perhaps a LiDAR system that runs independently from the rest of the code to avoid interference.
2. Use more than one microcontroller to split the workload, or use a multicore microcontroller. One would handle the reading, sending, and manipulating of the PPM signals, while the other would solely measure distances.

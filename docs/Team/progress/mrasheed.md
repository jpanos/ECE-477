#### Week 1:
**Date:** 08/28/2020
**Total hours:** 8

**Description of design efforts:**



#### Week 2:
**Date:** 09/04/2020
**Total hours:** 10

**Description of design efforts:**

This week I debugged the connection issues between our transmitter radio, reciever, PixHawk, and QGroundControl (a computer application used to set up the px4 flight firmware on the PixHawk).
The issue ended up being I was confusing the S.Port pin with the SBUS pin.
The S.Port pin was coming off of the connector port and it was labeled with an S, so I just assumed it was the SBUS port.
The SBUS port was actually one of the throughole wires so we had to solder some connections to it.
The documentation that came with the x4r-sb reciever was pretty bad so I blame it on that.
Ended up searching for blogs of how people connected it to figure it out. 
Anyway, we figured it out, and can now continue with the calibration process, including mapping flightmodes to certain switch combinations on the transmitter.
We are shooting for first flight this weekend, will post pictures in next weeks report (obviously, that's like free points).

![imagine what is described above](Team/progress/img/mrasheed_week2_radio.jpg "Radio")

#### Week 3:
**Date:** 09/10/2020
**Total hours:** 14

**Description of design efforts:**

This week I was busy calibrating the drone's flight controller, getting it ready for a flight test, and debugging flight our flight tests.
This involved hooking up the PixHawk 4 mini flight controller to the QGroundControl app on my computer and working through the calibration settings. 
I initially had trouble this, mainly the flight controller would not arm because the difference in measurment from its two magnetometers differed more than 30 degrees.
I thought this was due to running the calibration routine in the basement.
I ran the calibration outside with the help of Josh and that seemed to resolve the issue.
Other calibration routines for the gyroscope and accelerometer seemed to go without a hitch.

![image of radio and drone in driveway](Team/progress/img/mrasheed_week3_driveway.jpg "driveway3")

![image of calibration screen](Team/progress/img/mrasheed_week3_calibration.png "calibration3")

Another important aspect of setup before the drone can come off the ground is setting up the different flight modes. 
The image below shows what we set the different flight modes to and the setting of a kill switch. 
These two links ([here](https://www.youtube.com/watch?v=scqO7vbH2jo&feature=emb_logo) and [here](https://www.youtube.com/watch?v=r0hKVeba0LI)) helped with the set up the transmitter.
Six logical switches needed to be created and assigned to different PWM values on a single channel so that the PixHawk would know which flight mode is selected based off the PWM duty cycle received on that channel. 
Each logical switch was mapped to a certain combination of one 2 position switch and one 3 position switch (2 * 3 = 6 different flight modes). 

![image of flight modes settings](Team/progress/img/mrasheed_week3_flight_modes.png "flight_modes3")

When it came time to actually fly the drone, we ran into several problems however. 
The first one was that we could not locate the physical safety switch on the drone.
It hapenned to be a push button on the GPS module.
The second is that the nuts on the props would keep spinning off for two of the props.
An explanation why can be found [here](https://oscarliang.com/propeller-shaft-adapter-nuts-cw-ccw/).
Basically, we have all CW threaded nuts, so the props that spin CW keep loosening. 
This is still an issue we are working to resolve and are going to resort to more drastic measures (loctite and locknuts).
The final issue we've run into is that we didn't realize the props came in pairs with an associate direction of spin.
In our first few flight attempts, two of the props were spinning in the wrong direction they were made for, therefore not generating enough lift and causing the drone to not attain flight.
I think the others will have better visuals of what this looked like.

#### Week 4:
**Date:** 09/18/2020
**Total hours:** 16

**Description of design efforts:**

We invited along one of our ME friends to help us attach the props correctly so that they would no longer spin in the shafts.
He secred them real well, maybe too well because when we tried to take off again, the drone flipped around and shattered two of the props. 
We are going to table flight testing the drone any further until we have a more controlled flight testing environment setup.

This week I spent setting up my development environment on my laptop. Our team is working with STM's CubeIDE since that is what they provide substantial documentation on and are kind of pushing for.
I spent a couple hours getting familiar with the differences between the different APIs available such as Standard Peripheral Library (SPL, deprecated), Hardware Abstraction Layer (HAL), Lower Level (LL) and CMSIS.
I am a fan of CMSIS since it is an ARM standard and therefore my skills will be more portable to other ARM devices.
My decision is also based on the problems we faced this week that I'm going to discuss next.

Josh had success programming the board at first, but when he went home, he lost communication with the debugger and the only option to keep working with it was to erase the flash when it booted into reset mode using STM's ST-Link Utility. 
This problem persisted with a completely different nucleo dev board when trying to flash it with an empty MX project. 
This was pretty puzzling.
Joe, one of the course staff, mentioned that if we were to somehow overwrite the GPIO configuration registers for the SWD and SWO pins, the debugger would lose communication and would only be reatainable through a boot into reset mode, which prevents the main microcontroller, the H7, from executing any of its code.
The MX project did not have an empty main function.
It came prefilled with code it had generated that I did not understand.
My first attempt was to manually write into the configuration registers for the SWO and SWD GPIO pins in the off chance the HAL functions were overwriting them. 
This did not resolve the issue.
Since I did not write nor understand what the HAL code was doing, I didn't trust it, so I commented all of it out in the second attempt. 
I also wrote in the simple code snippet below that blinks and LED and stops blinking when you press the user push button on the board. 
This did worked and I was able to successfully flash the board and step through all the lines with the debugger.
I can only assume that the prefilled HAL functions are messing stuff up, so I'll uncomment it line by line and see where it breaks to try to figure it out. 
Otherwise I'm fine with just using CMSIS or manually writing values to registers.

![code snippet](Team/progress/img/mrasheed_week4_code_snippet.png "blink code")

I've also updated the show me a thing blinking light on the [Media](Media/media.html) page since I got it working.
I'll put it here too.

<iframe src="https://drive.google.com/file/d/1T0ZfrEJJzlUZXIQusokpSTAEEk50Bh6e/preview" width="640" height="480"></iframe>

#### Week 5:
**Date:** 09/25/2020
**Total hours:** 20

**Description of design efforts:**
I've figured out why the HAL code was failing.
Our specific nucleo board is configured to use the MCU's SMPS by default only [(schematic pg 4)](https://www.st.com/content/ccc/resource/technical/layouts_and_diagrams/schematic_pack/group1/da/0e/e2/b8/a5/b8/4d/ea/MB1363-H745ZIQ-C01_Schematic/files/MB1363-H745ZIQ-C01_Schematic.PDF/jcr:content/translations/en.MB1363-H745ZIQ-C01_Schematic.PDF).
The offending MX generated code set the power mode to LDO only.
Tracing through the code [here](https://github.com/STMicroelectronics/STM32CubeH7/blob/beced99ac090fece04d1e0eb6648b8075e156c6c/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c#L314), function `HAL_PWREx_ConfigSupply` was being called with `PWR_LDO_SUPPLY`, which cleared the SMPS enable, effectively shutting down the STM32H745ZIT6's own power.
Changing this function call with the argument `PWR_DIRECT_SMPS_SUPPLY` fixed our programming problem from week 4 and all other auto generated MX HAL code in a new project caused no other issues (yet, probably). 

This week I also worked on the CAD design as part of the Mechnical overview assignment, getting UART working, and learning more about the mavlink protocol and its c library functions.

![drone cad](Team/progress/img/mrasheed_week5_drone.jpg "drone cad")

![usart snippet](Team/progress/img/mrasheed_week5_usart_dumb_setup.png "usart dumb")

![mavlink heart snip](Team/progress/img/mrasheed_week5_mavlink_heart.png "mavlink heartbeat")

I am now trying to plan out the specifics of how message sending should be organized on the stm for mavlink.
Should it be event based, use DMA, etc?
Ethan got UART working with the HAL functions available and was able to send a char array. 
This might be a vialbe option if they are using some kind of event system.
If their implementation is loop based (checking if the previous byte has completed sending and then put in the new one), it will not be acceptable. 
My current plan is to use DMA and UART to send the entire char buffer of the mavlink message and use DMA and UART to store incoming messages.

For transmitting, I'm still working out how the queueing system might look look like on the stm.
Mavlink devices are required to send a heartbeat message at 1 Hz.
For the drone to be piloted by and on board computer, pixhawk requires it recieves positional instructions at a rate of 2 Hz.
Right now the main issue I'm running into is avoiding deadlock situations where the 1 Hz timer scheduling a heartbeat message is competing with normal code trying to schedule a position command.
So the interrupt ends up competing for a semaphore lock that the normal code already has, leading to a deadlock.

For recieving, I'm thinking of just using DMA and UART for putting the message into a buffer.
Then triggering an interrupt when the recieve is complete and parsing the message using the mavlink libraries. 
From my understanding, the mavlink library's functions parse the message byte by byte, but I don't want to do that for every incoming USART byte since I feel like that will interrupt normal control flow too often. 

#### Week 6:
**Date:** 10/02/2020
**Total hours:** 20

**Description of design efforts:**
This week I worked on recieving and transmitting mavlink messages and black boxing it into a file.
I had some initial trouble since the compiler started to put the buffer I had allocated in a region of memory that was not accessible by the DMA controller (DMA1 does not have access to DTCM or ITCM).
I drew out a mapping of the memory segments and what was being assigned to which core from both linker scripts (both cores had two linker scripts for some reason).
Here is the drawing.

![memory layout](Team/progress/img/mrasheed_week6_mem_quads.jpg "mem areas")

As I said there were two linker scripts for both cores for some reason, but when I looked at the build logs, apparently only `*_FLASH.ld` was being used.
So in the above diagram, only the segments appended with `_FLASH[4,7]` are important.
I then made some changes to the linker files so that given a specific section of memory, `.dma_buffer` in this case, it would place it in SRAM3 (32KB). 
Below are some images of the chnages made to the linker file and the instantiation of the buffer, telling the compiler where to put it now.

![reduce stack](Team/progress/img/mrasheed_week6_reduce_stack_ram_m4.png "reduce m4 ram")

![m7 mem areas](Team/progress/img/mrasheed_week6_m7_mem_areas.png "m7 mem area")

![dma buff area](Team/progress/img/mrasheed_week6_dma_buff_mem_area.png "buff mem area")

![buff alloc](Team/progress/img/mrasheed_week6_buff_alloc.png "buff alloc")

To summarize.
The first image is me reducing the stack pointer of the M4 so that it only spans SRAM1 and 2.
The second image is me defining a new memory segment for the M7.
The third image is me defining a new code segment `.dma_buffer` and where to put it (RAM3 which is SRAM3).
The fourth image is me allocating the buffer.
The bit in front tells the compiler to put it in the `.dma_buffer` code segment. 
The DMA transfer could now work and I could now send heartbeat messages again at a rate of 1Hz. 
I can also recieve and parse heartbeat messages from the flight controller.
I used an interrupt that fires whenever the receive register is not empty to recieve the multi byte messages and parses it when the mavlink helper function signals a complete message has been received. 
I have learned a lot more about the mavlink protocols and the functions I need to use and am ready to start trying requesting specific information from it and sending it commands in the comming week.

#### Week 7:
**Date:** 10/09/2020
**Total hours:** 12

**Description of design efforts:**
At the beginning of this week we finally got the drone to fly.
We tore it completely apart and put it back together.
Made sure it had the most up-to-date and stable version of the firmware and recalibrated its compass and accelerometers. 
We do not know what exactly changed to get it to fly right now, but we are happy.
Here is a video.

<iframe src="https://drive.google.com/file/d/1uzDKWucbLWMJdvhjJ9FaPcdL2WCJuCA5/preview" width="640" height="480"></iframe>

This week I have been working on figuring it more of the mavlink protocol.
It is a slow process because the documentation does not really spell it out for you except at really obscure pages in the px4 user manual (not the mavlink docs).
I was stuck for a little bit on how to send commands until I realized in the docs that commands were not a specific type of message but a subset of the command message.
In this weeks Show Me A Thing [(SMAT)](https://drive.google.com/file/d/1Grw1sNQxtv4wByAsBzgj5GnToPlBnMfq/view?usp=sharing), I showed that we can send the pixhawk an arm command we can receive an acknowledgement of that command. 
We can also unpack a bunch of other information from the pixhawk as well such as system health, gps coordinates, altitude, velocity, and more. 
A constant issue that has circled around in my brain is if an interrupt goes off while normal code is trying to stage a message to be sent out over UART/DMA. 
I am fairly confident this issue can be alleviated by only having the interrupts fire in one core and have application code run on the other core and just have locks controlling who has access to the UART setup.
I am also committing to having a linked list set up as a queue for messages to be sent out since I am worried of processes overwriting each others setup of the UART so I think having one defined interface will be for the best. 
I think I have the commands I need to send in order to takeoff and controll flight and I will now work on getting the drone to fly under the direction of the STM32H7 this comming week. Below is an image of me listing out all the messages I am receiving when the pixhawk mavlink channel I am connected to is set to minimal. 

![min msgs](Team/progress/img/mrasheed_week7_minimal_mv_msgs.jpg "min msgs")

#### Week 9:
**Date:** 10/23/2020
**Total hours:** 20 

**Description of design efforts:**
Since we had a gap week. I am just going to discuss everything I have been up to for the past two weeks.
First up, I have been helping my fellow partners in some of the GPIO initialization for PWM and I2C. 
Having been a TA for 362, it is not too foreign for me. 
HAL also does not seem to play nice or work at all for most things we want to do.
We must not be using it correctly.
Ethan however got it to work early on for UART communication. 

A big development this week for us was finding out that our battery monitor IC was actually half of what we needed (this is on my progress report because I had raised the concern originally). 

![bq29330_func](Team/progress/img/mrasheed_week9_bq29330_half.png "bq29330 func")

The system diagram above was what was on the bq29330s datasheet.
Apparently, the bq29330 is only the right half of this picture and is referred to as the analog front end (AFE).
The bq20z90 is the over half of this image. 
It is not named anywhere in the bq29330s datasheet, one must look at one (but not the other) user gudie on TIs website to hear about it, which was especially frustrating.
It actually does the processing to calculate the remaining capacity of the lipo based off calculations from the AFE.
It also needs a current sensing resistor to use the coulomb counting functionality. 
Our drone pulls from around 20 - 60 amps, so this was not something we wanted to work into our PCB last minute. 
We have settled on a different IC at Todds recommendation, the bq76920.
This chip also has coulomb counting abilities but we will only be using cause it automatically meaures the voltages of each cell and makes it available through its I2C iterface.
We will then use a high order polynomial to model the battery capacity vs voltage curve to determine the remaining life of the battery while flying. 

On the mavlink side of things, I have been making progress.
I moved all the mavlink IO to core 4 since we want it to handle most of our IO and ISRs.
I also have both cores sharing data.
If core 7 wants to send a mavlink message, it pushes it onto the mavlink message queue.
Then, core 4 will eventually get around to sending it. 
If core 7 wants to send a command, it will halt until core 4 has sent the command and recieved an achnoledgement from the pixhawk (I have not implemented timeouts yet on acknowlegement response)
I tried to get the drone to fly by sending it a takeoff command, but it was not working. 
Based off a forum post, I think I am just going to stick with sending set_posistion_target messages while flying in offboard mode.
This should not take too long with the existing framework that I have been working on and is slowly taking shape.
I should be able to test whether we can fly the drone with the micrcontroller over mavlink this weekend. 


#### Week 10:
**Date:** 10/30/2020
**Total hours:** 30

**Description of design efforts:**
This week I was responsible for writing and submittint the legal analysis.
Other than that, I have been tormented with why offboard control has not been working the way I thought it would. 
I had basically put aside all my other work I had to do this week to figure this out. 
It works now, but here is what hapenned. 

It started with [this](https://discuss.px4.io/t/offboard-automatic-takeoff-landing-using-mav-cmd-land-takeoff-local/1333/18) forum post that I originally thought was super helpful but later learned I would have been much better without. 
It describes some special fields in the PX4 firmware for the bit mask in the set position setpoint message reserved for takeoff and landing.
But when I tried using them, it did not work. 
The drone would just stay on the ground.
I then remembered that one of the forum posters tried repeatedly sending the takeoff command.
So I tried that.
That was a bit more successful but it would not listen to any other positional command info.
I tested this by just lifting the drone up when the motors would ramp up for takeoff.
The stm recieves altitude information from the flight controller and once it reaches a specified height, it switches the flight controller to offboard control mode (so it stops taking off and increasing in altitude).
Then, the stm would hover for a bit and then tell the flight controller to land. 
However it was not landing, but attempting to stay at its same position.
Checking the logs, It appeared that it was not listening to my setpoints and just maintaining the previous one from once the takeoff mode had ended (See figure below. The z setpoint I was trying to set was 0 but it is just maintaining previous z position).

![no listen](Team/progress/img/mrasheed_week10_no_listen_set.png "no listen")

I decided to dig deeper into the log files than what the online log file viewer could do.
I also took a look at the px4 flight controller firmware to get an idea of what was going on.
It turns out the different system components and subsystems communicate with each other based off a publish/subscribe messaging system.
I installed a python module called pyulog that would extract the values of all the messages and put them into CSV files so I could analyze them. 
It turns out, the setpoint triplet that was being set by my messages was being assigned correctly but the type was being set to IDLE, so the system would ultimately ignore them. 
Looking back the firmware source again, I found what my problem was.

![set_pos_firm](Team/progress/img/mrasheed_week10_set_pos_firmware_source.png "set pos handle source")

Those special bit fields the forum post had talked about were actually putting me in Idle!.
Apparently in the discussion link found in the comments of the code in the image, these options are not favorable anymore and are in the works of being refacored as part of a larger offboard control refactoring project.
Anyway, not using any bits above 0x800 in the type mask allowed me to set setpoints correctly and actually get the micro to fly the drone. Heres a vid.  

<iframe src="https://drive.google.com/file/d/1yt7GR4Nt-sJ6UGv2cgX5lwSZMBCkWL-K/preview" width="640" height="480"></iframe>

#### Week 11:
**Date:** 11/06/2020
**Total hours:** 15

**Description of design efforts:**
This week was kind of a deload since I obsessively put in a lot of hours last week trying to get the drone to fly.
That is not to say I did nothing.
This week I tried to get the drone to fly in the firmwares provided simulation environment. 
See image below. 
The simulator is gazebo, apparently a popular robot simulation software.
You can launch gazebo in a ROS node and have it communicate with the px4 firmware, which also is a ROS node. 
Then you can run your offboard code in another ROS node to communicate with the px4 node. 
Most of this launching of nodes is handled by the scripts given by the repository.
But it was still quite the learning curve. My ovsessivness in knowing the details certaintly does not help.
I was able to get the simulator and px4 node to launch and learn quite a bit about ROS development and project structure.

![simulation](Team/progress/img/mrasheed_week11_simulation.png "simulation")

In the end however, we decided that it was not worth the effort to get the simulation working at this point in the semester and instead are just opting for more live tests. 

I also was working on implementing a subset of the [ftp protocol over mavlink](https://mavlink.io/en/services/ftp.html) so that we could have the flight controller write battery data we collect to the sd card it has. 
I was having issues with that however too.
I believe I am formatting the message correctly, but I do not get any response for that message from the flight controller.
It is supposed to return a ftp message with a NACK or ACK opcode. 
I traced through the mavsdk code a little bit and I am pretty much doing the same stuff as them, unless I have overlooked something.
Now I am tracing through the firmware code (sigh, again) to figure out what the issue is. 
I already verified that in the autopilot message that the ftp capability flag was set, so it should support these types of messages. 

#### Week 12:
**Date:** 11/13/2020
**Total hours:** 20

**Description of design efforts:**
This week I waas trying to get the drone to move forward a meter instead of just taking off and landing. 
It did not go very well, multiple times.
Below is an image of the aftermath.

![drone_aftermath](Team/progress/img/mrasheed_week12_drone_aftermath.jpg "drone_aftermath")

We broke 3 props, on three seperate occasions and the mast for the GPS broke.
We have ordered more props and repaired the GPS mast. 
The documentation on how to get the drone moving in offboard mode is not that great, and some of the posts on GitHub I have read seem to indicate it is still a work in progress and in the middle of a major refactor effort.
My current method of trying to direct the drone forward has been setting incremental positional setpoints in the x direction (forward). 
But currently, after the drone takes off and loiters for a little bit, it dive bombs towards the ground.
I have been trying to parse through the logs and see where it is going wrong, but it hard to tell, especially because the drone says it switches from offboard into position control mode for periods of time and also does not always loiter for 5 seconds before attempting to move forwards.
I think this might have to do with the position setpoints I am setting, I do not think the autopilot likes them.
Even in the example code for offboard mode, they only set velocity setpoints.
Looking at the source code, the coordinate frame is only recorded when setting velocity setpoints.
I will try to fly the drone forwards using velocity setpoints once the new props get in.
I did fix the issue where the drone would pivot towards North in loiter mode.
This was easily remedied by adjusting my setpoint mask to ignore yaw and yaw rate setpoints which I though would be ignored in loiter mode anyway but apparently not.

#### Week 13:
**Date:** 11/20/2020
**Total hours:** 15

**Description of design efforts:**
My work this week was kind of symied since I was waiting for the propellers to come in the mail. 
In the meantime, I built this to hopefully mitigate this prob breaking issue in future flights.

![drone bubble](Team/progress/img/mrasheed_week13_drone_bubble.jpg "drone bubble")

Its a protective "bubble" for the drone made out of pex tubing. 
It is kind of heavy, so it adds to the inertia of the drone and makes it wonky to fly, but it still within a 2:1 thrust to weight ratio (each motor can generate 1.3 kg of thrust).
I think I mentioned last week that for some reason all our configurations were reset.
In that moment I also upgraded the firmware to 1.11.1 from 1.11.0.
I think this new firmware update may be the source of the crash.

![crash vel set 13](Team/progress/img/mrasheed_week13_vel_set.png "drone vel set 13")

The plot above shows the velocity setpoints and measured values. 
As you can see, the z setpoint velocity is incremented up and then held steady at -.7 m/s.
This is takeoff and operates correctly. 
But, then you can see for a period of 5 seconds, all velocity setpoints are set to 0, but the drones actual velocity is all over the place.
This is when it crashes.
After the 5 seconds of 0 velocity setpoints, you can see the x velocity setpoint go up for a little bit, this is me trying to go forward.
Apparently, the drone is crashing when it attempts to loiter, which is apparent in our video where it crashes almost immediately after reaching takeoff altitude. 

![crash pos 13](Team/progress/img/mrasheed_week13_crash_pos_set.png "drone vel set 13")

![smat pos 13](Team/progress/img/mrasheed_week13_hover_pos_set.png "drone vel set 13")

The two images above show the position values and setpoints of x,y,z.
The top image is the results from one of the crashes after the firmware update.
The bottom image is the result from the SMAT video of the hover, before the firmware update.
As you can see, when I enter the loiter mode (using the special mask bits from the forum post mentioned in earlier weeks), before the firmware update, the drone would set positional setpoints at the last known position.
Now, no setpoints are set by the drone when I use the special loiter bits. 
This is probably because of the firmware update.
I have read in some of the github issues that they are trying to move offboard away from these special bits and so that may be why the loiter bits no longer work in the firmware update.
Our new props came in today, so tomorrow I will try again but this time not use those loiter bits and just set positional setpoints myself for the loiter period of the test flight. 
This all just goes to show me, I should not listen to idiot forum posters ever again. 

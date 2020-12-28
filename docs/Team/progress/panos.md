#### Week 1:
**Date:** 08/28/2020
**Total hours:** 8

**Description of design efforts:**



#### Week 2:
**Date:** 09/04/2020
**Total hours:** 12

**Description of design efforts:**

During this past week I primarily worked on conducting a more thorough calculation of the power consumed by our drone to ascertain a more accurate prediction of our flight time. To accomplish this task, I began by going through the spec sheets of our power-hungry components mainly the motors and Jetson Nano. However, during our drone flight the motors and Nano will not be running at a constant level, as a result I made several assumptions to over predict the amount of power used by our drone during its flight. For our motors I started off by assuming that our drone will spend 60% of its time hovering. For this assessment I have defined hovering as the drone producing as much thrust as the drone weighs. Then for the other 40% of the time our drone is in the air it will be lifting higher into the air, for the purpose of this assessment I decided to consider lift being our motors producing 1.5 times the amount of thrust required to hover. Once I decided upon these metrics, I created an excel sheet to conduct the rudimentary math to determine how much flight time we can expect with our drone. Since you cannot fully discharge a LiPo battery without potentially damaging its cells we have limited our battery discharge to 90%. This gave us 66.6 Whs to work with in our battery, after calculating the total wattage used by our drone, I determined that we can expect a worst case scenario flight time of 10 minutes which is within our expected use case for our prototype. In addition to this I also finalized the ESC connections to our motors so that once we have finished the calibration of our flight controller our drone will be ready for flight.

![image of power chart](Team/progress/img/panos_week2_power.jpg "Power")

![image of drone connections](Team/progress/img/panos_week2_drone.jpg "Drone")


#### Week 3:
**Date:** 09/11/2020
**Total hours:** 15

**Description of design efforts:**

This past Saturday I worked with moiz and the team to prepare the drone for our first flight test. To do this I worked to reasemble the drone after my team worked on preparing a plan for mounting locations for the various protoyping hardware. Once they finished that I resoldered the PDB connections and preformed a rough mount to the drone frame. This mount was consisting only of zip ties since we were not going to be flying the drone and primarly wanted to determine the correct spin of the motors. Once this was completed Moiz and myself determined the directions that the motors were spinning and needed to be spinning. This resulted in me re soldering two motor connections in reverse polarity to change the direction of the motor. With this done from the weekend I began working on preparing the drone for our first flight test by mounting the rotors and securing the nessesary hardware. This flgiht while not going according to plan gave us a lot of information about the drone and its flgiht operations. With this new information I got new mounting hardware i.e lock washers, lock nuts, and thread locker to attempt to better secure the properllers to the drone. In addition to these flgiht test opperations I also began work on the PCB design for our team. This primarly consisted of me collecting the nessesary datasheets for our board mounted componets and created a new KiCad project. Through this I determeined a rough count of the number of capaciters I am going to need to be able to correctly run the STM32 on our PCB. I also met with Todd to discuss these findings, during this meeting I learned a bit more about PCB deisgn and some optiimzations I can make. One such optiimzation is having a capacitor bank for larger capcitors on the input lines to the chip in order to account for instanteous power draw. I have started the PCB because due to the nature of our project a PCB will give us more flexability and weight savings for our drone. Pictures of the aformentioned activities can be found below. 

![image of drone motor spin](Team/progress/img/Motor_spin.jpg)

![image of drone flight](Team/progress/img/Flight.jpg)

![image of PCB begininngs](Team/progress/img/pcb1.jpg)

#### Week 4:
**Date:** 09/18/2020
**Total hours:** 20

**Description of design efforts:**

This past Saturday I was primarly focused on getting our drone ready for flgiht by making imporvemnts to the mounting system that we are using for the propellers. Due to the previous failed attempts at flight I decided to consult with a Mechanical Engineering grad student friend of mine about proper mounting hardware to be used on our motor shafts. After consulting my friend I discovered a flaw in our mounting system which resulted in little friction between our nut and the base of the shaft. After this new solution was mounted we decided to test it again on Monday and found that it worked extreamly well. However, druing this flight we found another issue in another part of our system and have since refocused on our STM32 and Jetson Nano in lieu of working on the drone for the time being. In addition to this on Saturday I began working with the STM32 to famiarize myself with the IDE and vairous methods of programming for the chip. As seen in the picture below I created several projects within the IDE so that I could test an error that we ran into when debuggin our chip. We lost connection with our debugger chip on the board we were using. I began using the STLINK Utility to wipe and reflash our chip in order to regain connection with the board. Once this connection was regained I attempted to reporgram the chip various times with default code generated by the IDEs HAL libraies. When not doing this the rest of my time was spent working on the PCB schematic for our STM32. During this process I was reviewing the schematics of our STM development board to gain a better understanding at some of the decisions and considerations that were taken with this peticular chip. I also spent time reviewing the datasheets for the additional ICs that would need to be added to our PCB to ensure proper functionality of our PCB. Finally, I also researched an IC that can be used to effectivly read each cells voltage and capacity from our Li-Po battery. We are then going to be communicating with this IC over I2C with our STM32 in order to poll this information and determine the amount of battery life we have ramaining during drone operations. 

![image of STLINK untility](Team/progress/img/ST_week4.jpg)

![image of PCB developments](Team/progress/img/PCB_Week4.jpg)

![image of STM Cube IDE](Team/progress/img/Cube_Week4.jpg)


#### Week 5:
**Date:** 09/25/2020
**Total hours:** 18

**Description of design efforts:**

After Moiz figured out how to fix the HAL programming issue I was able to continue working with the STM32 to get a better understanding of how we will be interacting with the peripherals. During this process I was able to begin interfacing our capacative touch sensor with the STM32. To do this I wired our sensor up to the STM32 as seen in the picture below and used the sensors input as a conditional statement trigger to determine the blinking of LEDs on our board. I ended up running into a problem using GPIOC for the input pins on the STM32. I am still trying to figure out why these pins were not properly working however for the time being I moved over to the GPIOB and was able to make the sensor work with no trouble. I also soldered on all of the pinout connections for my STM32 so that I can have the flexability to use all of the avalable peripherals. I wanted to do this so that when it comes to our PCB design we can determine a specific set of pins that are going to be accesable off our board to help minimize the routing challenges. In addition to that, I also spent a lot of time working on refining our PCB design this week. At our current point all of the required power delivery to our board is on the schematic along with assigned footprints and componet selection. Since I have also ordered all of the compents and aquired a breakout board from the ECE lab I plan to work with mounting some of our devices next week so that I can begin to test the I2C protocol on the STM32.


![image of Pins on STM32 and touch sensor](Team/progress/img/Touch_Sensor.jpg)

![image of PCB developments](Team/progress/img/PCB_Week5.jpg)



#### Week 6:
**Date:** 10/2/2020
**Total hours:** 22

**Description of design efforts:**

During this past week I spent the majority of my time working on our PCB schemeatic. In order to ensure our PCB was as complete as possible I worked with our STM, IC??s, and power supplies schematics to ensure that our shcematic was prepared to the required operating specifications. Once this was all completed I met with our team to ensure I was not missing any componets or connecitons on our PCB. I also worked with Jackie to help get her more comfortable with Surface Mount Device (SMD) soldering so that we could mount our battery monitoring curcuit to a breakout board. Once this was completed I also worked on taking apart our drone and lochtiteing each screw so that our drone would stop vibrating apart during flight. During this process I also took apart our PDB to clean up the soldering connections so that we would not have to deal with massive solder blobs for simple wire connections. Once this was completed I also worked to correct errors brought up during our meeting with course staff for our schematic. 


![Part 1 of our schematic with proper PWR connections](Team/progress/img/Schematic_PWR_Connections.jpg)

![Part 2 of our schematic with the Capacitor banks](Team/progress/img/Schematic_Capacitor_Bank.jpg)

![Part 3 of our schematic with the IC pinout connections](Team/progress/img/Schematic_IC_pinout.jpg)


#### Week 7:
**Date:** 10/2/2020
**Total hours:** 20

**Description of design efforts:**

During this pask week I again spent the majority of my time working on our PCB. I spent some time during the weekend revising some ascpects of our schematic to better fit what was required for our battery monitor IC. This mainly consisted of revising the power develivery scheme for the IC and adding in a CLK line for the chip. I found out this chip is special in that it can only act as a slave and has no clock. Therefore in order to use this chip and get information about our battery we have to supply the IC with a clock signal from our microcontroller. Once this was added to our schematic I began routing our board to create a layout for our PCB. This took a surprisingly long time since I was trying to take extra care about manufaturability of our board since we need to actually build our board for use on our drone. Once I completed the layout I review the layout on my own and with my team to catch any simple mistakes I might missed. After our meeting with course staff I changed our large bank of capacitors to a single large 10uF capactitor with two smaller ones for adequate frequency response. Doing this allowed us to greatly decrease the size of our PCB which is critical for use on our drone. The images below are the routing view of our board and a 3D render of the board used during the presentaion with course staff. 


![A routing view of our schematic](Team/progress/img/PCB.jpg)

![A top down view of our PCB](Team/progress/img/TOP_PCB.jpg)

![A bottom up view of our PCB](Team/progress/img/BOT_PCB.jpg)



#### Week 9:
**Date:** 10/23/2020
**Total hours:** 36

**Description of design efforts:**

During this past week my primary responsibility has been to finalize the PCB and send it out for production as well as enable the rotation of a servero with our STM32 based upon contact with our capacative touch sensor. However, on Wednessday members of my team noticed that the IC I had selected to monitor our battery. This chip was in fact simply an analog front end for a companion circuit that was unlisted on the initial chips IC. Due to this information I spent a lot of time reseaching various ways to monitor the health of our battery during flight operations. The first issue that I ran into was that the LiPo battery we are using does not come with a datasheet or a realiable way to determine the cells that were used in its creation. This meant that I was unable to see a discharge curve from the manuafture of our battery. Without this information I was intially unsure of how we are going to be able to montior our battery. With this in mind I looked into some information on generic LiPo battery cells and their discharges at high C ratings. After seeing these curves and realizing that we cannot use the Coulomb counter in our new battery monitor IC I decided to develop a new method of determining battery health. After evaluating the curves I was able to find for other LiPo cells I determiend that if we cutoff our battery when the cells reach a voltage of 3.3V we will be given a large factor of saftey to begin our testing. I have been developing a plan for testing my new model to be used on our microcontroller and battery. Going forward I am going to test our battery on our drone during flight opperations so that I can obtain datapoints to better refine my model. Once my model is refined we should be able to stop flight operations before our battery approaches the knee of the battery discharge curve. Finally I also updated our PCB to reflect this new IC that is being used to monitor the voltages of each cell within our LiPo battery. Due to this new chip being used we now needed to include pullup resistors for our I2C communication between the STM32 and the IC. 


![New IC circut protection](Team/progress/img/New_IC_Pin.jpg)

![A view of our new IC for battery monitor](Team/progress/img/New_IC.jpg)

![A view of the new addition to our PCB](Team/progress/img/New_PCB.jpg)


#### Week 10:
**Date:** 10/30/2020
**Total hours:** 18

**Description of design efforts:**

During this past week I have focused most of my time on the assembly of our PCB which has arrived from our manufacture. This consisted of validating the boards that came in and proceeding to solder on our components so that we could begin functional testing. After some time, I was able to attach all of our SMD devices and now much finish adding in the through hole devices so that we can use the PCB in flight operations this coming week. I have also spent some time this week prototyping our pollinator so that we can being using it for pollination testing and to ensure that our touch sensor is working as intended. 


![Our PCB in production](Team/progress/img/PCB_Work.jpg)


#### Week 11:
**Date:** 11/6/2020
**Total hours:** 15

**Description of design efforts:**

During this past week I focused on the testing, validating, and programing our PCB. This was however not an easy process due to issues that we have identified with our nucleo board. The first step in this process was pin out the SWD connector on our nucleo board. However, unlike in ECE 362 there was not a simple header to be connected for off board programming. So in order to determine if I was actually sending information from the nucleo board I attached the SWD data and clock lines to our oscilloscope, however, after attempting several different triggering methods I was unable to see anything.  After attempting to solve this issue by reading through the chip, nucleo board, and SWD datasheets and manuals I began working with Todd to try and figure out what I was doing wrong. Todd had informed me that I was not connecting the NRST line which I had assumed I did not need since it was not pinned out on our nucleo board. After this conversation I connected this line and changed a header position and found that the debugger was no longer seeing an STM32 to be programmed. After reconvening with Todd, we found through more testing that our board was not sending out any of the initialization signals from our debugger chip to the PCB. Todd then recommended that I checkout a ST-LINK from the ECE shop. Once I acquired the ST-LINK and hooked it up to our PCB I was able to program our STM32 and produce a blinking LED from our pined out GPIO lines. This was a major sign of relief personally for myself as I was beginning to get concerned that I had made an error with our PCB??s fabrication or development. Since this has been finished I have begun the assembly of our pollination arm so that we can begin testing with our touch sensor on the flowers that we plan on using for testing.


![Our PCB in production](Team/progress/img/STLink.jpg)


#### Week 12:
**Date:** 11/13/2020
**Total hours:** 15

**Description of design efforts:**

Since being able to program the PCB we wanted to use this board to work with our battery monitor. However, this weekend I found that their was an issue with our LDO over heating which lead me to believe it had failed and killed our STM32 since that chip was also very hot. After consulting with Todd we decided to move over to a new PCB which I have yet to place an STM on so that we can ensure the rest of the board is working. The new LDO seems to be working as intended and is maintaining a constant voltage of 3.3V. During this time, I wired up our battery monitor so that Jackie could work with her I2C code on the actual battery. However, after trouble shooting the board in the lab it appears that I might have mis soldered or killed another component on the board as Jackie was no longer able to connect to the battery monitor. I am now working on resoldering the battery monitor on a new PCB in order to trouble shoot that code to ensure it is working as intended. In addition to this last week I prototyped a version of our pollinator that was mounted to our drone. While I was able to make a system that could detect contact of our pollinator it was unfortunately destroyed in a testing mishap. I am now working on refining this pollinator to ensure it can be more resistant to impact and stable on our drone during flight. 


![Our Broken PCB](Team/progress/img/PCB_Damaged.jpg)

![Our New PCB](Team/progress/img/PCB_New.jpg)


#### Week 13:
**Date:** 11/20/2020
**Total hours:** 20

**Description of design efforts:**

After last week we learned that our PCB has two different variants of chips for our battery monitor. A 2000PWR and a 2006PWR version of our chip which is addressable to a different slave address. Since resolving this issue we have been able to communicate with the other battery monitor to continue testing with our other PCBs. In addition to this I have soldered another new PCB however I was unable to program this board with the ST-Link which I believe to be due to a soldering issue on the STM32. When checking for the critical voltages of 5V and 3.3V both appeared to be stable at their intended levels. In addition to this I have worked with moiz to ensure proper flight functionality as well as proper construction of the protective barrier for our drone during flight operations.


![ our Done protection](Team/progress/img/Drone.jpg)



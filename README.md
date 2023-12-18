# Embedded-Project-Weekend-Feeder
The weekend pet feeder prototype optimally of the Texas Instruments Digital Signal Controller TM4C123GH6PMI to orchestrate a sophisticated and seamless pet care experience. Leveraging the controller's capabilities, the system interfaces with essential peripherals that collectively result in a reliable and user-friendly automated pet care solution. Go to the report for the process explanation.

### Tiva C Series 64-pin TM4C123GH6PMI Digital Signal Controller
<img src="https://github.com/CpeCoder/Embedded-Project-Weekend-Feeder/assets/123278927/923abebd-70b3-4649-b26b-812aef1b7804" width="300" height="300">

### Commands
1. time *HH:MM* - sets desired time to RTC
```
time 18:29
```
2. time - gets the current time from hibernation peripheral.
```
time
RTC: 18:29
```
3. feed *index* *duration* *PWM* *HH:MM* - schedules feed time, index form 0-9, duration time to run the auger, PWM 50%-100% recommended
```
feed 0 7 65 9:30
Time: 09:30 added to EEprom
```
4. feed *index* delete - deletes data of the following scheduled index
```
feed 0 delete
Time: 09:30 deleted
```
4. water *level* - sets desired water level in the pet dish all the time. The level will be in milliliters from 50-500.
5. water - simply gets the current water level in the pet dish
6. fill *mode* - there are 2 modes ("fill auto" and "fill motion"). Auto mode checks the pet dish water level with the desired water level and refills if needed. Motion mode freshens up the water in the dish when the pet visits.
7. alert *mode* - 2 modes ("alert ON" and "alert OFF"). When the alert is on and when the water is under desired which is not refilling, the buzzer starts buzzing. In OFF mode buzzer stays off.
8. logs - prints time logs when the pet visited the dish. Do not take logs of the same minute considering the pet stays by the dish for approximately 1 minute.
9. schedule - prints all the time food is supposed to be fetched with all the settings. Also prints the time of the next alarm.

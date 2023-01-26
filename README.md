# SurveyingDevice

This project sets out to create a prototype that is the simplest form of Bluetooth enabled electronic survey instrument capable of achieving grade 5 survey requirments.
The aim of the project is to create a device which can serve as a platform for further development by ourselves and others wanting to develop the technology by adding additional enhancements.

**Minimum device requirements (MVP):**

·         Must have an intuitive user interface.<br/>
·         Be waterproof and robust enough to endure a caving trip without additional protection.<br/>
·         Meet grade 5 survey requirements<br/>
·         Have a simple calibration procedure<br/>
·         Remember calibration settings until recalibrated<br/>
·         Store survey data until sent out to Bluetooth device or deleted<br/>

**External (case) button actions**
While the device is sealed the user will only be able to physically interact with the device by pressing the single button on the back of the case.
Users should be able to power the device on and off without opening the case, particularly in very wet/humid cave environments. It should be possible to power the device on and off (standby mode – laser off and ESP-deep sleep) by long pressing (4-5 seconds) the vandal proof button on the back of the case.
Each press of the button less than 4-5 seconds should take a single reading of the calibrated compass heading, inclination, and distance. This reading should be displayed on the OLED on the front of the case and stored within the devices memory ready for transmission to a survey app via Bluetooth.
Once pressed the display should freeze to allow the user to relay the survey information to their survey partner if they are using a paper method of surveying (the laser should also be powered off and the buzzer sounded to indicate the reading has been taken). The main button can then be pressed again to toggle the device back to live updating survey mode. 
The device should power off automatically if the external button is not pressed within 5 minutes to preserve the battery/laser. 

**Internal case buttons – calibration procedure**

As calibrations are normally performed before a trip in a relatively non-harsh environment these the buttons for controlling the calibration procedure can be kept inside the case. There will be 4 additional buttons for users to interact with the device. In the first version of the device these will just be for controlling the calibration procedure and turning bluetooth on and off.

Suggested calibration procedure:

**Button 1 (start calibration) –** this button initiates the calibration, once pressed the display indicates to the user that the device is in calibration mode and states what part of the calibration procedure they are in (a user document/video guide will explain how to perform the calibration).<br/>
**Calibration stage 1 – compass calibration,** the user waves the device around trying to cover as much of the spherical rotational space as possible – a score is updated periodically on the display indicating how complete the compass calibration is.<br/>
**Button 2 (move to next calibration step) –** once the user is satisfied with the calibration score for the compass, pressing button 2 overwrites the existing calibration data and moves them on to the next calibration step (accelerometer).<br/>
**Calibration stage 2 – accelerometer calibration,** the display will update to let the user know they are in accelerometer calibration mode… tba…<br/>
**Button 2 (move to next calibration step) –** once the user is satisfied with the accelerometer calibration, they press button 2 again which moves them to the final calibration step (laser alignment).<br/>
**Calibration stage 3 – laser alignment,** the display will state that the device is in laser alignment mode. The user guide will instruct the user to take multiple readings between two fixed points while rotating the device 45° each reading. The user can take readings if they wish depending on how accurate they want the calibration/how fast they want the calibration to be over.<br/>
**Button 2 (laser alignment complete) –** once the laser has been aligned and the user presses button 2 again the user is then instructed to check calibration.<br/>
**Calibration stage 4 – calibration checking,** the user is instructed to take back legs from the laser alignment step while rotating the device 45 °. The magnitude of the deviation from the theoretically perfect back leg is reported to the user on the display.<br/>
**Calibration save or leave option -**At any point during the calibration the user should have the option to skip to the end of the calibration without saving (e.g. in case they press the calibrate button by accident, or only want to calibrate the compass).<br/>
**Button 3 (skip calibration step) –** pressing button 3 at any stage of the calibration allows the user to skip a stage of the calibration keeping the original calibration data for that stage in memory. Repeatedly pressing button 3 effectively cancels the calibration procedure.<br/>
**Button 4 –** Reserved for toggling Bluetooth on and off.<br/>
 
**BLE commands**
In the first instance the bluetooth capabilies of the device will be kept to a minimum to allow us to focus on developing the other aspects of the device. 

**Minimum bluetooth requirements:** 
Send compass, clino, distance readings to app.
Within the app it will be possible for the user to set whether a set of readings is to be allocated to a survey leg or a splay. It is also possible within sexytopo/topodroid to automate this by detecting whether 2-3 readings are taken within a set tolerance of each other. 
The on/off status of bluetooth should be kept in device memory and recalled when the device is powered on and off. The device should recconntect to a phone automatically once powered on. 

**Display**
By default the display should show live constantly updating compass clino readings, when the user takes a reading the display should remain static until the user presses the button again to toggle the device back to live updating mode. 
The device should also include a flashing bluetooth symbol in the top left corner to indicate that bluetooth is enabled, this stops flashing once connected to a device. If the blutooth is turned off the blutooth symbol disappers. 
A battery level indication symbol should be shown in the top right hand side of the display. 

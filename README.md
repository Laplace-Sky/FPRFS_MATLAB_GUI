# FPRFS_MATLAB_GUI
This repository contains all codes required for the FPRFS controlling software.

The code is for the paper titled "A Multifunctional Smart Field-Programmable Radio Frequency Surface".

The software for the Field-Programmable Radio Frequency Surface (FPRFS) is a graphic user interface (GUI) developed in MATLAB. The software requires a desktop computer running MATLAB. This MATLAB software controls the behavior of our proposed FPRFS and is not used to run on any data. All the data were collected using professional RF characterization equipment. This FPRFS-configuring software also requires a set of supporting hardware, which includes a custom-designed ESP32 intermediate board, an ESP32 controlled RF meter, and one or more FPRFS boards. The ESP32 is loaded with a custom-designed code.

The operating system for all tests: Windows 10 64-bit.
The MATLAB version for all tests: MATLAB R2020b and Arduino IDE version 1.8.13.
Required non-standard hardware: a custom-designed ESP32 intermediate board and our introduced FPRFS.

Instructions:
1.Make sure the computer is installed with MATLAB with versions not later than R2020b and Arduino IDE 1.18.13. Typical installation time for both software is about 10 minutes and a license is required for MATLAB.
2.In the software folder, find the “FPRFS_GUI_multiboards.m” file and open it in MATLAB.
3.Run the open file and a GUI as shown in Supplementary Fig. 19 will pop up.
4.With the hardware, open the “ESP32_code.ino” in Arduino IDE and load the program to the ESP32 board, which connects to an FPRFS or a series of cascaded FPRFSs. Set the local IP address for the ESP32 and the number of boards cascaded for testing. All the rest of the functions introduced in our work can be achieved and controlled in the GUI. For RF -meter-based self-adapting tests, load the “SPI_read_from_RFMETER_final.ino” to the ESP32 board communicating with the RF meter.
5.Without the hardware, a testing sweeping process can be carried out by ticking the “View sweeping” option in the “Automatic Sweeping” panel at the bottom right corner of the GUI window. Make sure that “nanoVNA” is selected in the “Feedback Source” dropdown menu in the “Feedback Sensing Settings” panel and configure “FPRFS Application” to test. Click “S-param Optimize” to view the testing FPRFS pattern sweeping process. For instance, setting “FPRFS Application” as “Antenna” and “View sweeping” time interval as 0.1 second, click the “S-param Optimize” button and a brute force sweeping through all solid rectangular patch patterns will start and finish in about 8 seconds (80 times 0.1 second). The sweeping process should match Supplementary Video 1. Any pattern can be saved or recovered by clicking “Save Pattern” or “Browse and Load” buttons in the “FPRFS Configuration” panel. In the software folder, the pattern is saved as a .txt file with an array of 1s and 0s representing the DC biasing states of each segment.

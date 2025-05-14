How to run the code:
Once connected to the physical MiRO on a native Linux system, and an Xbox controller is plugged in via USB to the laptop:

1. Navigate to ~/catkin_ws/src/COM3528_2025_Team8
2. Ensure "roscore" is running in a terminal, or that laptop connected to MiRo
3. Source the environment
4. Run "roslaunch com3528_team8 controller.launch" to launch the controller code
5. Run "python3 app.py" to launch the video streaming service
6. Click the link or navigate to "http://<localhost>:5000" in a browser connected to the same local network
7. Click "Start Stream" to see the live feed

Unfortunately we were not able to fully realise the requirements for this project. In theory, it should have been possible to
(with NGrok installed) run "ngrok http <localhost>:5000" to get a HTTPS page ready for loading in VR. This unfortunately has not
been possible due to latency problems and browser restrictions. We have included some additional but untested versions of app.py
and index.html. The files with _hm appended have implementation for tracking head movements, the files with _ngrok_frozen_image
appended have an alternate approach implemented for displaying the video that was not fully completed. They are not run in common
recreations of the program but may be useful for future work.
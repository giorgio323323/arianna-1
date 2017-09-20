Introduction to the files in the Kalman filter simulator .zip file.

After you have unpacked the .zip file you have obtained a directory that contains the Matlab simulator functions that I've used in my thesis on Robot Localization and Kalman Filters. All plots presented in the thesis are generated using the functionality of the simulator.

The simulator does not have a graphical user interface. You will have to look into the different function files to get to know the workings of the simulator, although I don't think that that will give many problems. 

charts/ contains functions to plot charts when data is generated
diff_drive/ contains the truth and KF model that implement the drive system and sensor models from the thesis. In particular, diff_drive_problem.m defines the estimation problem. This file is documented with comments on how to use the file.
kf_methods/ contains functions that implement different KFs
misc/ contains miscalaneous functions needed by the simulator
track_drive_single_and_multiple/ contains the models used when detecting single or multiple landmarks at the same time

To get started, make sure that the pathdef.m includes the subdirectories in this directory. 
Then either run 'demo_fs_ekf' (extended Kalman Filter) or 'demo_fs_iekf' (iterated extended Kalman filter).

To get to know the simulator, it is easiest to try the above commands and step through each line of the code. I have added commentry to the code to make it more understandable. However, if you have questions, feel free to contact me via http://contact.negenborn.net/.

Rudy Negenborn
http://www.negenborn.net/

August 2003
July 2009 (update)
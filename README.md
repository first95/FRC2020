# FRC2020
The code base for our 2020 robot

To open the main robot code, open Visual Studio Code, click "Open Folder", and open the `FRC2019` folder.

## Vision coprocessor

This repository also contains our Vision Coprocessor code left over from 2019.  This application runs on a Raspberry Pi, analyzes video from a camera, and publishes information about the targets it sees.  To open this project, click "open folder" in VSCode and open the `VisionCoprocessor` folder.

To build and run the vision coprocessor:
1. Right click build.gradle, click "Build Robot Code", click "Java".  This should generate a file at `VisionCoprocessor/build/libs/VisionCoprocessor-all.jar`.
2. Connect your computer to the same network as the raspberry Pi, and determine its IP address.  I tend to do this by sharing my laptop's wifi, and running wireshark to learn the DHCP-granted address of the raspberry pi.
3. Go to the pi webconsole and stop the vision application.
4. Use FileZilla (or your favorite SFTP tool) to copy VisionCoprocessor-all.jar to /home/pi on the raspberry pi.
5. Rename VisionCoprocessor-all.jar to uploaded.jar.
6. Go to the pi webconsole and start the vision application.


## Branches

- `master`: This is the "polished" branch.  Code should only be merged into this branch after it has passed the software acceptance test.  The idea here is that, at any time, if we need fully-working code on the robot, we can just put the contents of `master` on.
- `develop`: This is the normal common development branch.  Most of the time we should work out of develop, or out of a feature branch.
- `feature/my_cool_feature`: A feature branch.  Make a new feature branch before starting work on a new feature.  Commit all your work here, and push it every time you have a connection.  Merge your feature into `develop` once you finish it.

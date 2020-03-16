# 2019 
Our 2019 robot had two noteworthy software components:

## Vision coprocessor

The vision coprocessor was a Raspberry Pi model 3 mounted to the robot, connected to a network switch along with the RoboRIO and radio.  We wrote a separate application, which was stored in a VS code project in [this folder](https://github.com/first95/FRC2019/tree/master/VisionCoprocessor).  The raspberry pi was configured with the directions [here](https://docs.wpilib.org/en/latest/docs/software/vision-processing/raspberry-pi/using-the-raspberry-pi-for-frc.html)

Development of the vision coprocessor application was made much easier by writing two `main()` functions: one designed to run on a desktop PC, and one designed to run on the Pi.  The one on the desktop was configured to loop through a number of test input images and display the artifacts of its analysis.  This was great for testing and development, especially prior to having the real-world source of images ready.

## Significant use of the command pattern

This was the first year we really took advantage of the scheduler capabilities of the command pattern.  See this command in particular: [Command to pick up a hatch cover off the ground and transfer it to the gripper](https://github.com/first95/FRC2019/blob/master/FRC2019/src/main/java/frc/robot/commands/compound/PickupAndHandoffGroundHatch.java) Remember to click down in through the dependencies to understand it fully.

# First use of the controller's rumble packs

However, note that driver's station support for this was buggy until 2020 (only supported one controller).  So check out the [implementation in the 2020 codebase](https://github.com/first95/FRC2020/blob/master/FRC2020/src/main/java/frc/robot/commands/RumbleCommand.java).  Note that much of the meat of the rumble support is actually in the [OI class](https://github.com/first95/FRC2020/blob/master/FRC2020/src/main/java/frc/robot/OI.java).


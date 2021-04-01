# BaseFalconSwerve </br>
**Basic Swerve Code for a Swerve Module using Falcon Motors, a CTRE CANCoder, and a CTRE Pigeon Gyro** </br>
This code was designed with SDS MK3 style modules in mind, but should be easily adaptable to other styles of modules.</br>

**Setting Constants**
----
The following things must be adjusted to your robot and module's specific constants in the Constants.java file (all distance units must be in meters, and rotation units in radians):</br>
1. Gyro Settings: ```pigeonID``` and ```invertGyro``` (ensure that the gyro rotation is CCW+ (Counter Clockwise Positive)
2. ```trackWidth``` (Center to Center distance of left and right modules)
3. ```wheelBase``` (Center to Center distance of front and rear module wheels)
4. ```wheelDiameter```
5. ```driveGearRatio``` (for SDS MK3 either: (8.16 / 1) or (6.86 / 1))
6. ```angleGearRatio``` (for SDS MK3: (12.8 / 1))
7. Angle Motor PID Values:
    * To tune start with a low P value (0.01).
    * Multiply by 10 until the module starts oscilating around the set point
    * Scale back by searching for the value (for example, if it starts oscillating at a P of 10, then try (10 -> 5 -> 7.5 -> etc)) until the module overshoots the setpoint but corrects with no oscillation.
    * Repeat the process for D. The D value will basically help prevent the overshoot. Ignore I.
8. Get the drive characterization values (KS, KV, KA) by using the WPILib characterization tool, found [here](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html). You will need to lock your modules straight forward, and complete the characterization as if it was a standard tank drive.
9. Tune drive kP until it doesn't overshoot and doesnt oscilate around a target velocity.
10. For ```maxSpeed``` and ```maxAngularVelocity``` you can use the theoretical values, but it is better to physically drive the robot and find the actual max values.
11. Set ```canCoderInvert``` and ```angleMotorInvert``` such that both are CCW+.
12. In the module specific constants, set the can ID's of the motors and CANCoders for the respective modules.
13. Setting Offsets
    * For finding the offsets, use a piece of 1x1 metal that is straight against the forks of the front and back modules (on the left and right side) to ensure that the modules are straight. 
    * You need to point the bevel gears of all the wheels in the same direction (either facing left or right). And preferably you should have the wheels facing in the direction where a postive input to the drive motor drives forward. If for some reason you set the offsets with the wheels backwards, you can change the ```driveMotorInvert``` to fix.
    * Open smartdashboard (or shuffleboard and go to the smartdashboard tab), you will see 4 printouts called "Mod 0 Cancoder", "Mod 1 Cancoder", etc. If you have already straightened the modules, copy those 4 numbers exactly (to 2 decimal places) to their respective ```angleOffset``` variable in constants.


**Controller Mappings**
----
This code is natively setup to use a xbox controller to control the swerve driver. </br>
The Left Stick controls translation (forwards and sideways movement), and the Right Stick controls rotation. </br>
The Y button is mapped to zero the gyro, useful if the gyro drifts mid match, just rotate the robot forwards, and press Y to rezero.

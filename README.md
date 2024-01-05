**<ins>Note:</ins> While this code intends to provide plug n play configs/tuning numbers for COTS Modules, there are several configs/tunes that are untested with the change to Phoenix 6. If this code is used for one of those untested modules, the module specific configs and tuning will most likely need to be edited.**

**If anyone would like to contribute a config/tune to be added as "tested" please send a message or make a PR.**

**Scroll down for a full list of tested/untested configs.**

# BaseTalonFXSwerve </br>

**Basic Swerve Code for a Swerve Module using TalonFX Controllers, a CTRE CANCoder, and a CTRE Pigeon Gyro** </br>
This code was designed with Swerve Drive Specialties MK3, MK4, MK4i, and WCP SwerveX style modules in mind, but should be easily adaptable to other styles of modules.</br>

**Setting Constants**
----
The following things must be adjusted to your robot and module's specific constants in the Constants.java file (all distance units must be in meters, and rotation units in radians):</br>
These instructions are mostly followable from Step 
1. Gyro Settings: ```pigeonID```(ensure that the gyro rotation is CCW+ (Counter Clockwise Positive)
2. ```chosenModule```: 
<br><b><ins>Note: Not every module's configs are tested. See the note at the top</b></ins>
<br>If you are using a supported COTS Module, set the module and drive ratio you are using here. 
<br>This will automatically set certain constants for the specific module required to function properly. 
<br><b><ins>If you are not using a COTS supported module, you should delete this variable, and fix all the errors that pop up with correct values for the module you are using</b></ins>
<br> Here is a list of the constants that will automatically be set if you are using a supported module:
    * Wheel Circumference
    * Angle Motor Invert
    * Drive Motor Invert
    * CANCoder Sensor Invert
    * Angle Motor Gear Ratio
    * Drive Motor Gear Ratio
    * Angle Falcon Motor PID Values
    
3. ```trackWidth```: Center to Center distance of left and right modules in meters.
4. ```wheelBase```: Center to Center distance of front and rear module wheels in meters.
5. ```wheelCircumference```: Cirumference of the wheel (including tread) in meters. <br><b>If you are using a supported module, this value will be automatically set.</b>
6. ```driveGearRatio```: Total gear ratio for the drive motor. <br><b>If you are using a supported module, this value will be automatically set.</b>
7. ```angleGearRatio```: Total gear ratio for the angle motor. <br><b>If you are using a supported module, this value will be automatically set.</b>
8. ```canCoderInvert``` and ```angleMotorInvert```: Both must be set such that they are CCW+. <br><b>If you are using a supported module, this value will be automatically set.</b>
9. ```driveMotorInvert```: <b>If you are using a supported module, this value will be automatically set.</b>
<br>This can always remain false, since you set your offsets in step 11 such that a positive input to the drive motor will cause the robot to drive forwards.
<br>However this can be set to true if for some reason you prefer the bevel gears on the wheel to face one direction or another when setting offsets. See Step 11 for more information.

10. ```Module Specific Constants```: set the Can Id's of the motors and CANCoders for the respective modules, see the next step for setting offsets.
11. Setting Offsets
    * For finding the offsets, use a piece of 1x1 metal that is straight against the forks of the front and back modules (on the left and right side) to ensure that the modules are straight. 
    * Point the bevel gears of all the wheels in the same direction (either facing left or right), where a postive input to the drive motor drives the robot forward (you can use phoenix tuner to test this). If for some reason you set the offsets with the wheels backwards, you can change the ```driveMotorInvert``` value to fix.
    * Open smartdashboard (or shuffleboard and go to the smartdashboard tab), you will see 4 printouts called "Mod 0 Cancoder", "Mod 1 Cancoder", etc. 
    <br>If you have already straightened the modules, copy those 4 numbers exactly (to 2 decimal places) to their respective ```angleOffset``` variable in constants.
    <br><b>Note:</b> The CANcoder values printed to smartdashboard are in degrees, when copying the values to ```angleOffset``` you must use ```Rotation2d.fromDegrees("copied value")```.

12. Angle Motor PID Values: <br><b>If you are using a supported module, this value will be automatically set. If you are not, or prefer a more or less aggressive response, you can use the below instructions to tune.</b> 
    * To tune start with a low P value (0.01).
    * Multiply by 10 until the module starts oscilating around the set point
    * Scale back by searching for the value (for example, if it starts oscillating at a P of 10, then try (10 -> 5 -> 7.5 -> etc)) until the module doesn't oscillate around the setpoint.
    * If there is any overshoot you can add in some D by repeating the same process, leave at 0 if not. Always leave I at 0.

13. ```maxSpeed```: In Meters Per Second. ```maxAngularVelocity```: In Radians Per Second. For these you can use the theoretical values, but it is better to physically drive the robot and find the actual max values.


14. Get the drive characterization values (KS, KV, KA) by using the WPILib characterization tool, found [here](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html). You will need to lock your modules straight forward, and complete the characterization as if it was a standard tank drive.
15. ```driveKP```: 
<br>After completeing characterization and inserting the KS, KV, and KA values into the code, tune the drive motor kP until it doesn't overshoot and doesnt oscilate around a target velocity.
<br>Leave ```driveKI```, and ```driveKD``` at 0.0.


**Controller Mappings**
----
This code is natively setup to use a xbox controller to control the swerve drive. </br>
* Left Stick: Translation Control (forwards and sideways movement)
* Right Stick: Rotation Control </br>
* Y button: Zero Gyro (useful if the gyro drifts mid match, just rotate the robot forwards, and press Y to rezero)
* Left Bumper: Switches To Robot Centric Control while held

**Tested Modules**
----
| Vendor | Module | Motor | Configs (Motor Inverts etc) | Rotation Motor Tuning |
| :-------------: | :-------------: | :-------------: | :-------------: | :-------------: |
| WCP | SwerveX Standard  | Kraken X60 | <code style="color : red">Untested</code> | <code style="color : red">Untested</code> |
| WCP | SwerveX Standard  | Falcon 500 | <code style="color : red">Untested</code> | <code style="color : red">Untested</code> |
| WCP | SwerveX Flipped | Kraken X60 | <code style="color : red">Untested</code> | <code style="color : red">Untested</code> |
| WCP | SwerveX Flipped | Falcon 500 | <code style="color : red">Untested</code> | <code style="color : red">Untested</code> |
| SDS | MK3 | Kraken X60 | <code style="color : red">Untested</code> | <code style="color : red">Untested</code> |
| SDS | MK3 | Falcon 500 | <code style="color : green">Tested</code> | <code style="color : red">Untested</code> |
| SDS | MK4 | Kraken X60 | <code style="color : red">Untested</code> | <code style="color : red">Untested</code> |
| SDS | MK4 | Falcon 500 | <code style="color : green">Tested</code> | <code style="color : red">Untested</code> |
| SDS | MK4i | Kraken X60 | <code style="color : red">Untested</code> | <code style="color : red">Untested</code> |
| SDS | MK4i | Falcon 500 | <code style="color : green">Tested</code> | <code style="color : green">Tested</code> |


**Changelog**
---
* 1/4/24: 
    * Updated Phoenix to Phoenix 6 - 24.0.0-beta-7
        * Implemented Phoenix 6's `SensorToMechanismRatio` removing the need to convert gear ratios in code
        * Implemented Phoenix 6's `ContinuousWrap` removing the need for a custom `SwerveModuleState.optimize` function.
        * Updated conversions from Phoenix 5's units ("counts") to Phoenix 6's units (rotations).
    * Updated WPILib to 2024.1.1-beta-4
    * Added SwerveX Modules and Kraken X60 Motors
    * Moved from directly using the Gyro angle in `Swerve` for heading (`getGyroYaw()`) to using the odometry output for rotation `getHeading()` as intended by WPILib
    * Moved repository off of 364's Github 

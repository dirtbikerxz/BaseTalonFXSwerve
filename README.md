# Team 3373 2023 competition bot

# TODO

- Implement mode commands
  - DropOffMode
  - LoadingStationMode
  - PickupMode

- Implement handoff between driver and special ops

- Shuffleboard configuration & stats

- Vision subsystem
  - Build
    - Get network switch
    - Get case for Pi
    - Build alternate power for radio
    - Attach all of those to the robot (velcro FTW!)
    - Mount camera(s) to robot

- General wiring
  - Connect to all motors and
    - Update firmware
    - Validate CAN bus ID
    - Validate position (inverted or not)

- Arm subsystem
  - Measurements
    - Arm extension
    - Arm rotation
  - Test
    - Teleop (use this to establish presets - x8)
    - Presets (tune these to be as fast/accurate as possible)
  
- Hand subsystem
  - Test grab/release (tune timing of bumper)
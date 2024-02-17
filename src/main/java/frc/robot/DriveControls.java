package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class DriveControls {
    /* Controllers */
    private final Joystick joystick0 = new Joystick(2); //Left
    private final Joystick joystick1 = new Joystick(3); //Middle
    private final Joystick joystick2 = new Joystick(4); //Right

    /* Driver Buttons */
    public final JoystickButton zeroGyro = new JoystickButton(joystick0, 10);
    public final JoystickButton robotCentric = new JoystickButton(joystick0, 7);
    public final JoystickButton hangExtend = new JoystickButton(joystick1, 10);
    public final JoystickButton hangRetract = new JoystickButton(joystick1, 9);
    public final JoystickButton activateShooter = new JoystickButton(joystick2, 9);
    public final JoystickButton runIntake = new JoystickButton(joystick2, 8);
    public final JoystickButton reverseIntake = new JoystickButton(joystick2, 7);

    public double getForward() { return joystick0.getY(); } //Deleted a negative symbol to flip directions
    public double getStrafe() { return joystick0.getX(); } //^^
    public double getRotation() { return -joystick1.getX(); }  
}
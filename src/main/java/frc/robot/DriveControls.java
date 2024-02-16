package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class DriveControls {
    /* Controllers */
    private final Joystick joystick0 = new Joystick(2); //Middle
    private final Joystick joystick1 = new Joystick(3); //Left
    private final Joystick joystick2 = new Joystick(4); //Right (I know it's messed up to order them like that idk why it's that way)

    /* Driver Buttons */
    public final JoystickButton zeroGyro = new JoystickButton(joystick0, 10);
    public final JoystickButton robotCentric = new JoystickButton(joystick0, 7);    

    public double getForward() { return -joystick0.getY(); }
    public double getStrafe() { return -joystick0.getX(); }
    public double getRotation() { return -joystick1.getX(); }  
}
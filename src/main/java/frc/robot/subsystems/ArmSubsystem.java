package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {

    public final double MAXIMUM_POSITION_ROTATING = 42;
    public final double MININUM_POSITION_ROTATING = 42;
    public final double MAXIMUM_POSITION_EXTENDING = 42;
    public final double MININUM_POSITION_EXTENDING = 42;

    public final double TRAVEL_RANGE_ROTATING = 42;
    public final double TRAVEL_RANGE_EXTEDING = 42;


    public CANSparkMax rotatingMotor = new CANSparkMax(1, MotorType.kBrushless);
    public CANSparkMax extendingMotor = new CANSparkMax(2, MotorType.kBrushless);

    public RelativeEncoder rotatingEncoder = rotatingMotor.getEncoder();
    public RelativeEncoder extendingEncoder = extendingMotor.getEncoder();

    public DigitalInput limitSwitch0 = new DigitalInput(0);
    public DigitalInput limitSwitch1 = new DigitalInput(1);
    public DigitalInput limitSwitch2 = new DigitalInput(2);

    
}

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    public double MAXIMUM_POSITION_ROTATING;
    public double MININUM_POSITION_ROTATING;
    public double MAXIMUM_POSITION_EXTENDING;
    public double MININUM_POSITION_EXTENDING;

    public final double TRAVEL_RANGE_ROTATING = 300;
    public final double TRAVEL_RANGE_EXTEDING = 300;

    public CANSparkMax rotatingMotor = new CANSparkMax(1, MotorType.kBrushless);
    public CANSparkMax extendingMotor = new CANSparkMax(2, MotorType.kBrushless);

    public RelativeEncoder rotatingEncoder = rotatingMotor.getEncoder();
    public RelativeEncoder extendingEncoder = extendingMotor.getEncoder();

    public DigitalInput limitSwitch0 = new DigitalInput(4);
    public DigitalInput limitSwitch1 = new DigitalInput(5);
    public DigitalInput limitSwitch2 = new DigitalInput(6);

    public void periodic() {
        SmartDashboard.putBoolean("LS0", limitSwitch0.get());
        SmartDashboard.putBoolean("LS1", limitSwitch1.get());
        SmartDashboard.putBoolean("LS2", limitSwitch2.get());
        SmartDashboard.putNumber("MAX_EXT", MAXIMUM_POSITION_EXTENDING);
        SmartDashboard.putNumber("MAX_ROT", MAXIMUM_POSITION_ROTATING);
    }

    
}

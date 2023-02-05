package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    public static final double TRAVEL_RANGE_ROTATING = 300;
    public static final double TRAVEL_RANGE_EXTEDING = 300;

    public CANSparkMax rotatingMotor;
    public CANSparkMax extendingMotor;

    public RelativeEncoder rotatingEncoder;
    public RelativeEncoder extendingEncoder;

    public DigitalInput limitSwitch0;
    public DigitalInput limitSwitch1;
    public DigitalInput limitSwitch2;

    public double MAXIMUM_POSITION_ROTATING;
    public double MININUM_POSITION_ROTATING;
    public double MAXIMUM_POSITION_EXTENDING;
    public double MININUM_POSITION_EXTENDING;

    public ArmSubsystem() {

        rotatingMotor = new CANSparkMax(1, MotorType.kBrushless);
        rotatingMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rotatingMotor.setOpenLoopRampRate(0.3);
        rotatingMotor.setInverted(false); // TODO fix me

        rotatingEncoder = rotatingMotor.getEncoder();
        rotatingEncoder.setPositionConversionFactor(1.0); // TODO fix

        extendingMotor = new CANSparkMax(2, MotorType.kBrushless);
        extendingMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        extendingMotor.setOpenLoopRampRate(0.3);
        extendingMotor.setInverted(false); // TODO fix me

        extendingEncoder = extendingMotor.getEncoder();
        extendingEncoder.setPositionConversionFactor(1.0); // TODO fix

        limitSwitch0 = new DigitalInput(4);
        limitSwitch1 = new DigitalInput(5);
        limitSwitch2 = new DigitalInput(6);
    }

    public boolean isExtenderLimitPressed() {
        return limitSwitch0.get();
    }

    public boolean isRotatorLimitPressed() {
        return limitSwitch1.get();
    }

    public void periodic() {
        SmartDashboard.putBoolean("LS0", limitSwitch0.get());
        SmartDashboard.putBoolean("LS1", limitSwitch1.get());
        SmartDashboard.putBoolean("LS2", limitSwitch2.get());
        SmartDashboard.putNumber("Extender Position", extendingEncoder.getPosition());
        SmartDashboard.putNumber("Rotator Position", rotatingEncoder.getPosition());
        SmartDashboard.putNumber("MAX_EXT", MAXIMUM_POSITION_EXTENDING);
        SmartDashboard.putNumber("MAX_ROT", MAXIMUM_POSITION_ROTATING);
    }
}

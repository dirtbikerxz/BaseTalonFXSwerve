package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import static frc.robot.subsystems.swerve.SwerveConfig.*;

public class SwerveModule {

    public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            driveKS,
            driveKV,
            driveKA);

    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANCoder angleEncoder;

    public SwerveModule(int moduleNumber, ModuleConfig moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = createAngleEncoder(moduleConstants.cancoderID);
        angleMotor = createAngleMotor(moduleConstants.angleMotorID);
        driveMotor = createDriveMotor(moduleConstants.driveMotorID);
        resetToAbsolute();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = SwerveUtils.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / maxSpeed;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = SwerveUtils.MPSToFalcon(desiredState.speedMetersPerSecond, wheelCircumference, driveGearRatio);
            driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        angleMotor.set(ControlMode.Position, SwerveUtils.degreesToFalcon(angle.getDegrees(), angleGearRatio));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(SwerveUtils.falconToDegrees(angleMotor.getSelectedSensorPosition(), angleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = SwerveUtils.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), angleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            SwerveUtils.falconToMPS(driveMotor.getSelectedSensorVelocity(), wheelCircumference, driveGearRatio),
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            SwerveUtils.falconToMeters(driveMotor.getSelectedSensorPosition(), wheelCircumference, driveGearRatio),
            getAngle()
        );
    }
}
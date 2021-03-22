package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

public class SwerveModule {
    public int moduleNumber;
    private double offset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;
    private TalonFXConfiguration angleMotorConfig;
    private TalonFXConfiguration driveMotorConfig;
    private CANCoderConfiguration canCoderConfig;
    private boolean invertDrive;
    private boolean invertAngle;
    private double lastAngle;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, double offset, int angleMotorID, int angleEncoderID, int driveMotorID,
    boolean invertDrive, boolean invertAngle, TalonFXConfiguration angleMotorConfig, TalonFXConfiguration driveMotorConfig, CANCoderConfiguration canCoderConfig){
        this.moduleNumber = moduleNumber;
        this.offset = offset;
        this.angleMotorConfig = angleMotorConfig;
        this.driveMotorConfig = driveMotorConfig;
        this.canCoderConfig = canCoderConfig;
        this.invertDrive = invertDrive;
        this.invertAngle = invertAngle;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(angleEncoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(driveMotorID);
        configDriveMotor();

        lastAngle = getAngle().getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        SwerveModuleState state = CTREModuleState.optimize(desiredState, getAngle());

        if(isOpenLoop){
            double percentOutput = state.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(state.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(state.speedMetersPerSecond));
        }

        double angle = (Math.abs(state.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : state.angle.getDegrees();
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.Swerve.angleGearRatio)); 
        lastAngle = angle;
    }

    private void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder() - offset, Constants.Swerve.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(canCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(angleMotorConfig, Constants.ctreTimeout);
        mAngleMotor.setInverted(invertAngle);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        mAngleMotor.setStatusFramePeriod(StatusFrame.Status_1_General, Constants.Swerve.angleStatus1, Constants.ctreTimeout);
        mAngleMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, Constants.Swerve.angleStatus2, Constants.ctreTimeout);
        mAngleMotor.configNeutralDeadband(0.04, Constants.ctreTimeout); //0.001 for tuning
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(driveMotorConfig);
        mDriveMotor.setInverted(invertDrive);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setStatusFramePeriod(StatusFrame.Status_1_General, Constants.Swerve.driveStatus1);
        mDriveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, Constants.Swerve.driveStatus2);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public double getCanCoder(){
        return angleEncoder.getAbsolutePosition();
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    }

    public double getVelocity(){
        return Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getVelocity(), getAngle());
    }
    
}
package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants.DrivetrainConstants.DriveGains;
import frc.robot.Constants.DrivetrainConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveGains.kS, DriveGains.kV, DriveGains.kA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, moduleConstants.cancoderBus);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, moduleConstants.angleMotorBus);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, moduleConstants.driveMotorBus);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / DrivetrainConstants.MAX_SPEED;
            mDriveMotor.setControl(new DutyCycleOut(percentOutput, true, false));
        }
        else {
            double velocity = Conversions.getInputShaftRotations((desiredState.speedMetersPerSecond / DrivetrainConstants.WHEEL_CIRCUMFERENCE), DrivetrainConstants.GEAR_RATIO);
            mDriveMotor.setControl(new VelocityDutyCycle(velocity, true, feedforward.calculate(desiredState.speedMetersPerSecond), 0, false));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (DrivetrainConstants.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        //TODO: i fucked that and be better, maybe unfuck
        mAngleMotor.setControl(new PositionDutyCycle((desiredState.angle.getDegrees() / 360) * DrivetrainConstants.ANGLE_RATIO));
        SmartDashboard.putNumber("desired angle", desiredState.angle.getDegrees());
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromRotations(mAngleMotor.getRotorPosition().getValue() / DrivetrainConstants.ANGLE_RATIO);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = ((getCanCoder().getDegrees() - angleOffset.getDegrees()) * DrivetrainConstants.ANGLE_RATIO)/360;
        mAngleMotor.setRotorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        angleEncoder.getConfigurator().apply(new CTREConfigs().swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.getConfigurator().apply(new TalonFXConfiguration());
        mAngleMotor.getConfigurator().apply(new CTREConfigs().swerveAngleFXConfig);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.getConfigurator().apply(new TalonFXConfiguration());
        mDriveMotor.getConfigurator().apply(new CTREConfigs().swerveDriveFXConfig);
        mDriveMotor.setRotorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.getOutputShaftRotations(mDriveMotor.getRotorVelocity().getValue(), DrivetrainConstants.GEAR_RATIO) * DrivetrainConstants.WHEEL_CIRCUMFERENCE,
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.getOutputShaftRotations(mDriveMotor.getRotorPosition().getValue(), DrivetrainConstants.GEAR_RATIO) * DrivetrainConstants.WHEEL_CIRCUMFERENCE,
            getAngle()
        );
    }
}
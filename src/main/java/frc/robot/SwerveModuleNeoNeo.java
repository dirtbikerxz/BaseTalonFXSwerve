package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModuleNeoNeo implements SwerveModule {

    private Rotation2d angleOffset;
    private Rotation2d lastAngle;
    private Rotation2d angle = new Rotation2d(0);

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    private final SparkPIDController drivePIDController;
    private final SparkPIDController anglePIDController;
    private final RelativeEncoder anglemotorEncoder;
    private final RelativeEncoder driveEncoder;
    private CANcoder angleEncoder;
    private SwerveModuleState mCurrentState = new SwerveModuleState(0, new Rotation2d(0));

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    // private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    // private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    // private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModuleNeoNeo(SwerveModuleConstants moduleConstants) {
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        anglemotorEncoder = mAngleMotor.getEncoder();
        anglePIDController = mAngleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = mDriveMotor.getEncoder();
        drivePIDController = mDriveMotor.getPIDController();
        configDriveMotor();


        lastAngle = Rotation2d.fromDegrees(anglemotorEncoder.getPosition());
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        SparkPIDController pidController = mAngleMotor.getPIDController();
        pidController.setFF(1);
        pidController.setReference(desiredState.angle.getRotations(), CANSparkBase.ControlType.kPosition);
        //mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
        setAngle(desiredState);
    }

    @Override
    public Rotation2d getRotation(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = getRotation().getRotations() - angleOffset.getRotations();

        SparkPIDController pidController = mAngleMotor.getPIDController();
        pidController.setReference(absolutePosition,CANSparkMax.ControlType.kPosition);

        //mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return mCurrentState;
    }

    private void configAngleMotor(){
        anglemotorEncoder.setPositionConversionFactor((1/Constants.Swerve.angleGearRatio) * 360);

        resetToAbsolute();

        anglePIDController.setP(Constants.Swerve.angleKP);
        anglePIDController.setI(Constants.Swerve.angleKI);
        anglePIDController.setD(Constants.Swerve.angleKD);
        anglePIDController.setFF(0.0);

        mAngleMotor.getPIDController().setOutputRange(-0.25, 0.25);
    }

    private void configDriveMotor(){
        mDriveMotor.setOpenLoopRampRate(Constants.Swerve.openLoopRamp);
        mDriveMotor.setClosedLoopRampRate(Constants.Swerve.closedLoopRamp);

        driveEncoder.setVelocityConversionFactor(1/Constants.Swerve.driveGearRatio * Constants.Swerve.wheelCircumference / 60);
        //TODO Make sure conversion factor is correct for position
        driveEncoder.setPositionConversionFactor(1/Constants.Swerve.driveGearRatio * Constants.Swerve.wheelCircumference);
        driveEncoder.setPosition(0);

        drivePIDController.setP(Constants.Swerve.driveKP);
        drivePIDController.setI(Constants.Swerve.driveKI);
        drivePIDController.setD(Constants.Swerve.driveKD);
        drivePIDController.setFF(Constants.Swerve.driveKF);

        drivePIDController.setOutputRange(-0.5, 0.5);

    }    
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getEncoder().getPosition(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mDriveMotor.getEncoder().getPosition())
        );
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(anglemotorEncoder.getPosition());
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        mCurrentState = desiredState;

        if(isOpenLoop){
            double speed = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(speed);

            // driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            //mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            // driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            double voltage = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            // mDriveMotor.setControl(driveVelocity);
            mDriveMotor.setVoltage(voltage);
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        mAngleMotor.getPIDController().setReference(angle.getDegrees(), CANSparkBase.ControlType.kPosition);
        lastAngle = angle;
    }
}

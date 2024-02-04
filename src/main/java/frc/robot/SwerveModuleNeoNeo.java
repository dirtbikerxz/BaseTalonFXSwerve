package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.SwerveModuleConstants;


public class SwerveModuleNeoNeo implements SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;

    private final RelativeEncoder angleEncoder;
    private final RelativeEncoder driveEncoder;

    private final SparkPIDController drivePIDController;
    private final SparkPIDController anglePIDController;

    private CANcoder absoluteEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModuleNeoNeo(SwerveModuleConstants moduleConstants, int moduleNumber){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;


        /* Angle Encoder Config */
        absoluteEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        angleEncoder = mAngleMotor.getEncoder();

        angleEncoder.setPosition(getCanCoder().getDegrees());
        anglePIDController = mAngleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = mDriveMotor.getEncoder();
        drivePIDController = mDriveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
        }
        else {
            mDriveMotor
                    .getPIDController()
                    .setReference(
                            desiredState.speedMetersPerSecond,
                            CANSparkMax.ControlType.kVelocity,
                            0,
                            feedforward.calculate(desiredState.speedMetersPerSecond),
                            SparkPIDController.ArbFFUnits.kVoltage);
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
//        PIDController pid = new PIDController(0.12, 0, 0);
//        pid.enableContinuousInput(-180, 180);
//
//        mAngleMotor.set(MathUtil.clamp(pid.calculate(getCanCoder().getDegrees(), angle.getDegrees()), -0.2, 0.2));
        //angleEncoder.setPosition(getCanCoder().getRotations());
        resetToAbsolute();
        mAngleMotor.getPIDController().setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
        lastAngle = angle;

    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }

    public Rotation2d getCanCoder(){
        double position = absoluteEncoder.getAbsolutePosition().getValue();
        return Rotation2d.fromDegrees(position);
    }

    public void resetToAbsolute(){
        angleEncoder.setPosition(getCanCoder().getDegrees() - angleOffset.getDegrees());
    }

    @Override
    public Rotation2d getRotation() {
        return getCanCoder();
    }

    private void configAngleEncoder(){
        absoluteEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);


        //absoluteEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
        //absoluteEncoder.configMagnetOffset(angleOffset.getDegrees());
    }

    private void configAngleMotor(){
        mAngleMotor.restoreFactoryDefaults();
        mAngleMotor.setSmartCurrentLimit(25);
        mAngleMotor.setSecondaryCurrentLimit(0.1);
        mAngleMotor.setInverted(false);
        //mAngleMotor.setIdleMode(false);

        angleEncoder.setPositionConversionFactor((1/Constants.Swerve.angleGearRatio) * 360);

        resetToAbsolute();

        anglePIDController.setP(0.12);
        anglePIDController.setI(0);
        anglePIDController.setD(0);
        anglePIDController.setFF(0);

        mAngleMotor.getPIDController().setOutputRange(-0.25, 0.25);
    }

    private void configDriveMotor(){
        mDriveMotor.restoreFactoryDefaults();
        mDriveMotor.setSmartCurrentLimit(25);
        mDriveMotor.setSecondaryCurrentLimit(0.1);
        mDriveMotor.setInverted(false);
        //mDriveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
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

    public SwerveModuleState getState(){
        return new SwerveModuleState(
                driveEncoder.getVelocity(),
                getCanCoder()
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
                driveEncoder.getPosition(),
                getCanCoder()
        );
    }

    @Override
    public void dashboardPeriodic() {
        resetToAbsolute();
        //SmartDashboard.putNumber("Mod " + moduleNumber + " CANcoder", this.getRotation().getDegrees());
        SmartDashboard.putNumber("Mod " + moduleNumber + "Relative Angle", this.getAngle().getDegrees());
        SmartDashboard.putNumber("Mod " + moduleNumber + " CANcoder", this.getCanCoder().getDegrees());
        SmartDashboard.putNumber("Mod " + moduleNumber + " Velocity", this.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("Mod " + moduleNumber + " Drive", this.mDriveMotor.getAppliedOutput());
    }
}

//public class SwerveModuleNeoNeo implements SwerveModule {
//
//    private Rotation2d steeringOffset;
//    private Rotation2d lastSteeringAngle;
//    private Rotation2d steering = new Rotation2d(0);
//    private int mModuleNumber;
//
//    private CANSparkMax mAngleMotor;
//    private CANSparkMax mDriveMotor;
//    private final SparkPIDController drivePIDController;
//    private final SparkPIDController anglePIDController;
//    private final RelativeEncoder anglemotorEncoder;
//    private final RelativeEncoder driveEncoder;
//    private CANcoder angleEncoder;
//    private SwerveModuleState mCurrentState = new SwerveModuleState(0, new Rotation2d(0));
//
//    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);
//
//    /* drive motor control requests */
//    // private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
//    // private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
//
//    /* angle motor control requests */
//    // private final PositionVoltage anglePosition = new PositionVoltage(0);
//
//    public SwerveModuleNeoNeo(SwerveModuleConstants moduleConstants, int moduleNumber) {
//        this.steeringOffset = moduleConstants.angleOffset;
//        this.mModuleNumber = moduleNumber;
//
//        /* Angle Encoder Config */
//        angleEncoder = new CANcoder(moduleConstants.cancoderID);
//        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);
//
//        /* Angle Motor Config */
//        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
//        anglemotorEncoder = mAngleMotor.getEncoder();
//        anglePIDController = mAngleMotor.getPIDController();
//        configAngleMotor();
//
//        /* Drive Motor Config */
//        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
//        driveEncoder = mDriveMotor.getEncoder();
//        drivePIDController = mDriveMotor.getPIDController();
//        configDriveMotor();
//
//
//        lastSteeringAngle = Rotation2d.fromDegrees(anglemotorEncoder.getPosition());
//    }
//
//    @Override
//    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
//        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
////        SparkPIDController pidController = mAngleMotor.getPIDController();
////        pidController.setFF(1);
////        pidController.setReference(desiredState.angle.getRotations(), CANSparkBase.ControlType.kPosition);
//        //mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
//        setSpeed(desiredState, isOpenLoop);
//        setSteeringAngle(desiredState);
//    }
//
//    @Override
//    public Rotation2d getRotation(){
//        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
//    }
//
//    public void resetToAbsolute(){
//        double absolutePosition = getRotation().getDegrees() - steeringOffset.getDegrees();
//
//        SparkPIDController pidController = mAngleMotor.getPIDController();
//        pidController.setReference(absolutePosition,CANSparkMax.ControlType.kPosition);
//
//        //mAngleMotor.setPosition(absolutePosition);
//    }
//
//    public SwerveModuleState getState(){
//        return mCurrentState;
//    }
//
//    private void configAngleMotor(){
//        anglemotorEncoder.setPositionConversionFactor((1/Constants.Swerve.angleGearRatio) * 360);
//
//        resetToAbsolute();
//
//        anglePIDController.setP(0.05);
//        anglePIDController.setI(Constants.Swerve.angleKI);
////        anglePIDController.setD(Constants.Swerve.angleKD);
//        anglePIDController.setD(0.0);
//        anglePIDController.setFF(0.0);
//
//        mAngleMotor.getPIDController().setOutputRange(-0.25, 0.25);
//    }
//
//    private void configDriveMotor(){
//        mDriveMotor.setOpenLoopRampRate(Constants.Swerve.openLoopRamp);
//        mDriveMotor.setClosedLoopRampRate(Constants.Swerve.closedLoopRamp);
//
//        driveEncoder.setVelocityConversionFactor(1/Constants.Swerve.driveGearRatio * Constants.Swerve.wheelCircumference / 60);
//        //TODO Make sure conversion factor is correct for position
//        driveEncoder.setPositionConversionFactor(1/Constants.Swerve.driveGearRatio * Constants.Swerve.wheelCircumference);
//        driveEncoder.setPosition(0);
//
//        drivePIDController.setP(0.12);
//        drivePIDController.setI(Constants.Swerve.driveKI);
////        drivePIDController.setD(Constants.Swerve.driveKD);
//        drivePIDController.setD(0);
//        drivePIDController.setFF(Constants.Swerve.driveKF);
//
//        drivePIDController.setOutputRange(-0.5, 0.5);
//
//    }
//    public SwerveModulePosition getPosition(){
//        return new SwerveModulePosition(
//            Conversions.rotationsToMeters(mDriveMotor.getEncoder().getPosition(), Constants.Swerve.wheelCircumference),
//            Rotation2d.fromRotations(mAngleMotor.getEncoder().getPosition())
//        );
//    }
//
//    private Rotation2d getSteeringAngle(){
//        return Rotation2d.fromDegrees(anglemotorEncoder.getPosition());
//    }
//
//    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
//        mCurrentState = desiredState;
//
//        if(isOpenLoop){
//            double speed = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
//            mDriveMotor.set(speed);
//
//            // driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
//            //mDriveMotor.setControl(driveDutyCycle);
//        }
//        else {
//            // driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
//            double voltage = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
//            // mDriveMotor.setControl(driveVelocity);
//            mDriveMotor.setVoltage(voltage);
//        }
//    }
//
//    private void setSteeringAngle(SwerveModuleState desiredState){
//        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastSteeringAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
//
//        mAngleMotor.getPIDController().setReference(angle.getDegrees(), CANSparkBase.ControlType.kPosition);
//        lastSteeringAngle = angle;
//    }
//
//    @Override
//    public void dashboardPeriodic() {
//        SmartDashboard.putNumber("Mod " + mModuleNumber + " CANcoder", this.getRotation().getDegrees());
//        SmartDashboard.putNumber("Mod " + mModuleNumber + " Angle", this.getPosition().angle.getDegrees());
//        SmartDashboard.putNumber("Mod " + mModuleNumber + " Velocity", this.getState().speedMetersPerSecond);
//        SmartDashboard.putNumber("Mod " + mModuleNumber + " Drive", this.mDriveMotor.getAppliedOutput());
//    }
//}

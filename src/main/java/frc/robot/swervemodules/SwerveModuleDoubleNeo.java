package frc.robot.swervemodules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.ModuleState;
import frc.lib.SwerveModuleConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.lib.doubleNeoConfig;
import frc.robot.interfaces.SwerveModule;

/**
 * a Swerve Modules using REV Robotics motor controllers and CTRE CANcoder absolute encoders.
 */
public class SwerveModuleDoubleNeo implements SwerveModule
{
    public int moduleNumber;
    private final Rotation2d angleOffset;
    // private Rotation2d lastAngle;

    private final CANSparkMax mAngleMotor;
    private final CANSparkMax mDriveMotor;




    private final CANcoder angleEncoder;
    private RelativeEncoder relAngleEncoder;
    private RelativeEncoder relDriveEncoder;


    //SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModuleDoubleNeo(int moduleNumber, SwerveModuleConstants moduleConstants)
    {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;


        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID,  MotorType.kBrushless);
        configDriveMotor();

        /* Angle Encoder Config */

        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configEncoders();


        // lastAngle = getState().angle;
    }


    private void configEncoders()
    {
        // absolute encoder

        //angleEncoder.configAllSettings(new doubleNeoConfig().canCoderConfig);

        relDriveEncoder = mDriveMotor.getEncoder();
        relDriveEncoder.setPosition(0);


        relDriveEncoder.setPositionConversionFactor(doubleNeoConfig.driveRevToMeters);
        relDriveEncoder.setVelocityConversionFactor(doubleNeoConfig.driveRpmToMetersPerSecond);


        relAngleEncoder = mAngleMotor.getEncoder();
        relAngleEncoder.setPositionConversionFactor(doubleNeoConfig.DegreesPerTurnRotation);
        // in degrees/sec
        relAngleEncoder.setVelocityConversionFactor(doubleNeoConfig.DegreesPerTurnRotation / 60);


        resetToAbsolute();
        mDriveMotor.burnFlash();
        mAngleMotor.burnFlash();

    }

    private void configAngleMotor()
    {
        mAngleMotor.restoreFactoryDefaults();
        SparkPIDController controller = mAngleMotor.getPIDController();
        controller.setP(doubleNeoConfig.angleKP, 0);
        controller.setI(doubleNeoConfig.angleKI,0);
        controller.setD(doubleNeoConfig.angleKD,0);
        controller.setFF(doubleNeoConfig.angleKF,0);
        controller.setOutputRange(-doubleNeoConfig.anglePower, doubleNeoConfig.anglePower);
        mAngleMotor.setSmartCurrentLimit(doubleNeoConfig.angleContinuousCurrentLimit);
        //mAngleMotor.setInverted(doubleNeoConfig.angleMotorInvert);
        mAngleMotor.setIdleMode(doubleNeoConfig.angleIdleMode);



    }

    private void configDriveMotor()
    {
        mDriveMotor.restoreFactoryDefaults();
        SparkPIDController controller = mDriveMotor.getPIDController();
        controller.setP(doubleNeoConfig.driveKP,0);
        controller.setI(doubleNeoConfig.driveKI,0);
        controller.setD(doubleNeoConfig.driveKD,0);
        controller.setFF(doubleNeoConfig.driveKF,0);
        controller.setOutputRange(-doubleNeoConfig.drivePower, doubleNeoConfig.drivePower);
        mDriveMotor.setSmartCurrentLimit(doubleNeoConfig.driveContinuousCurrentLimit);
        //mDriveMotor.setInverted(doubleNeoConfig.driveMotorInvert);
        mDriveMotor.setIdleMode(doubleNeoConfig.driveIdleMode);




    }



    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {


        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        // CTREModuleState actually works for any type of motor.
        desiredState = ModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);

        if(mDriveMotor.getFault(FaultID.kSensorFault))
        {
            DriverStation.reportWarning("Sensor Fault on Drive Motor ID:"+mDriveMotor.getDeviceId(), false);
        }

        if(mAngleMotor.getFault(FaultID.kSensorFault))
        {
            DriverStation.reportWarning("Sensor Fault on Angle Motor ID:"+mAngleMotor.getDeviceId(), true);
        }
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop)
    {

        if(isOpenLoop)
        {
            double percentOutput = desiredState.speedMetersPerSecond / doubleNeoConfig.maxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }

        double velocity = desiredState.speedMetersPerSecond;

        SparkPIDController controller = mDriveMotor.getPIDController();
        controller.setReference(velocity, ControlType.kVelocity, 0);

    }

    private void setAngle(SwerveModuleState desiredState)
    {
        if(Math.abs(desiredState.speedMetersPerSecond) <= (doubleNeoConfig.maxSpeed * 0.01))
        {
            mAngleMotor.stopMotor();
            return;

        }
        Rotation2d angle = desiredState.angle;
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.

        SparkPIDController controller = mAngleMotor.getPIDController();

        double degReference = angle.getDegrees();



        controller.setReference(degReference, ControlType.kPosition, 0);

    }



    private Rotation2d getAngle()
    {
        return Rotation2d.fromDegrees(relAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder()
    {

        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue());
        //return getAngle();
    }

    public int getModuleNumber()
    {
        return moduleNumber;
    }

    public void setModuleNumber(int moduleNumber)
    {
        this.moduleNumber = moduleNumber;
    }

    @Override
    public void resetToAbsolute()
    {

        double absolutePosition =getCanCoder().getDegrees() - angleOffset.getDegrees();
        relAngleEncoder.setPosition(absolutePosition);
    }

    @Override
    public Rotation2d getRotation() {
        return getAngle();
    }


    public SwerveModuleState getState()
    {
        return new SwerveModuleState(
                relDriveEncoder.getVelocity(),
                getAngle()
        );
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(
                relDriveEncoder.getPosition(),
                getAngle()
        );
    }
}
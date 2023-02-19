package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem2 extends SubsystemBase {

    public static final double MAX_ROTATE = 0.8;
    public static final double MAX_EXTEND = 0.8;

    public static final int ROTATION_CANID = 1;
    public static final int ROTATION_LIMIT_ID = 1;
    public static final boolean ROTATION_INVERTED = false;
    public static final double ROTATION_TRAVEL_LIMIT = 300;
    public static final double ROTATION_FACTOR = 1.0;

    public static final int EXTENSION_CANID = 1;
    public static final int EXTENSION_LIMIT_ID = 1;
    public static final boolean EXTENSION_INVERTED = false;
    public static final double EXTENSION_TRAVEL_LIMIT = 300;
    public static final double EXTENSION_FACTOR = 1.0;

    private final Unit rotator;
    private final Unit extender;

    public ArmSubsystem2() {
        extender = new Unit(EXTENSION_CANID, EXTENSION_LIMIT_ID, EXTENSION_FACTOR, EXTENSION_INVERTED);
        rotator = new Unit(ROTATION_CANID, ROTATION_LIMIT_ID, ROTATION_FACTOR, ROTATION_INVERTED);
        SmartDashboard.putData("Rotator", builder -> {
            builder.addDoubleProperty("Current", () -> rotator.encoder.getPosition(), null);
            builder.addDoubleProperty("Max", () -> rotator.max, null);
            builder.addDoubleProperty("Min", () -> rotator.min, null);
        });
        SmartDashboard.putData("Extender", builder -> {
            builder.addDoubleProperty("Current", () -> extender.encoder.getPosition(), null);
            builder.addDoubleProperty("Max", () -> extender.max, null);
            builder.addDoubleProperty("Min", () -> extender.min, null);
        });
    }

    public void clearLimits() {
        rotator.min = Double.NEGATIVE_INFINITY;
        rotator.max = Double.POSITIVE_INFINITY;
        extender.min = Double.NEGATIVE_INFINITY;
        extender.max = Double.POSITIVE_INFINITY;
    }

    public boolean captureLimits(double percentRotate, double percentExtend) {
        if (rotator.limit.get()) {
            rotator.min = rotator.encoder.getPosition();
            rotator.max = rotator.min + ROTATION_TRAVEL_LIMIT;
            percentRotate = 0.0;
        }
        if (extender.limit.get()) {
            extender.min = extender.encoder.getPosition();
            extender.max = extender.min + EXTENSION_TRAVEL_LIMIT;
            percentExtend = 0.0;
        }
        rotator.motor.set(percentRotate);
        extender.motor.set(percentExtend);
        return percentRotate == 0.0 && percentExtend == 0.0;
    }

    public double getRotatorPosition() {
        return rotator.encoder.getPosition();
    }

    public double getExtenderPosition() {
        return extender.encoder.getPosition();
    }

    public void moveAt(double percentRotate, double percentExtend) {
        rotator.setSafe(percentRotate, MAX_ROTATE);
        extender.setSafe(percentExtend, MAX_EXTEND);
    }

    public static class Unit {

        public final CANSparkMax motor;
        public final RelativeEncoder encoder;
        public final DigitalInput limit;
        public double min;
        public double max;

        public Unit(int motorId, int limitId, double factor, boolean inverted) {

            this.motor = new CANSparkMax(motorId, MotorType.kBrushless);
            motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            motor.setOpenLoopRampRate(0.3);
            motor.setInverted(inverted);

            this.encoder = motor.getEncoder();
            encoder.setPositionConversionFactor(factor);

            this.limit = new DigitalInput(limitId);

            this.min = Double.NEGATIVE_INFINITY;
            this.max = Double.POSITIVE_INFINITY;
        }

        public void setSafe(double percentOutput, double maxOutput) {
            if (encoder.getPosition() <= min && percentOutput < 0.0) {
                percentOutput = 0.0;
            }
            if (encoder.getPosition() >= max && percentOutput > 0.0) {
                percentOutput = 0.0;
            }
            percentOutput = MathUtil.clamp(percentOutput, -maxOutput, maxOutput);
            motor.set(percentOutput);
        }
    }
}

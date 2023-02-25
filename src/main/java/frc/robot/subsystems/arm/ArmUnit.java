package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;


/**
 * Represents a combination of a motor, its encoder, and a limit switch that triggers
 * when the motor is travelling in the negative direciton and hits a defined limit.
 */
public class ArmUnit {

    public final double COLLISON_RANGE = 10;

    public final CANSparkMax motor;
    public final RelativeEncoder encoder;
    public final DigitalInput limit;
    public double min;
    public double max;
    public double currentHeight;
    public double currentWidth;
    public double currentLength;

    public ArmUnit(int motorId, int limitId, double factor, boolean inverted) {

        this.motor = new CANSparkMax(motorId, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setOpenLoopRampRate(0.3);
        motor.setInverted(inverted);

        this.encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(factor);

        this.limit = new DigitalInput(limitId);

        this.min = Double.NEGATIVE_INFINITY;
        this.max = Double.POSITIVE_INFINITY;
    }

    /**
     * Clears the calibrated limits
     */
    public void clearLimits() {
        min = Double.NEGATIVE_INFINITY;
        max = Double.POSITIVE_INFINITY;
    }

    /**
     * Called during calibration. Moves the arm at a (hopefully slow) speed
     * until it hits its minimum limit, then sets the min/max values based
     * on the travel limit.
     * @return true if calibration has finished
     */
    public boolean calibrate(double percentOutput, double travelLimit) {
        
        if (limit.get()) {
            min = encoder.getPosition() + COLLISON_RANGE;
            max = min + travelLimit;
            motor.set(0.0);
            return true;
        } else {
            motor.set(percentOutput);
            return false;
        }
    }

    /**
     * Called to move the unit safely, based on the calibrated min and
     * max limits.
     */
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

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents the pneumatics for the hand. There are two double solenoids that
 * control its behavior:
 *   - PRESSURE (options: low, high or off)
 *   - POSITION (options: open, closed or off)
 */
public class OldHandSubsystem extends SubsystemBase {

    /** CAN bus ID for the pneumatics hub */
    public static final int REVPH_CAN_ID = 5;

    /** Switch IDs on the hub for the solenoids */
    public static final int PRESSURE_FORWARD = 0;
    public static final int PRESSURE_REVERSE = 1;
    public static final int POSITION_FORWARD = 2;
    public static final int POSITION_REVERSE = 3;

    /** Solenoid values for the different options */
    public static final Value LO = Value.kReverse;
    public static final Value HI = Value.kForward;
    public static final Value OPEN = Value.kForward;
    public static final Value CLOSED = Value.kReverse;
    public static final Value OFF = Value.kOff;

    private final DoubleSolenoid pressure;
    private final DoubleSolenoid position;

    public OldHandSubsystem() {
        pressure = new DoubleSolenoid(REVPH_CAN_ID, PneumaticsModuleType.REVPH, PRESSURE_FORWARD, PRESSURE_REVERSE);
        position = new DoubleSolenoid(REVPH_CAN_ID, PneumaticsModuleType.REVPH, POSITION_FORWARD, POSITION_REVERSE);
        SmartDashboard.putData("Hand", builder -> {
            builder.addBooleanProperty("Closed", () -> isClosed(), null);
            builder.addStringProperty("Position", () -> getPositionString(), null);
            builder.addStringProperty("Pressure", () -> getPressureString(), null);
        });
    }

    private String getPressureString() {
        Value val = pressure.get();
        if (val == LO) return "Low";
        if (val == HI) return "High";
        return "Off";
    }

    private String getPositionString() {
        Value val = position.get();
        if (val == OPEN) return "Open";
        if (val == CLOSED) return "Closed";
        return "Off";
    }

    private boolean isClosed() {
        return pressure.get() != OFF && position.get() == CLOSED;
    }

    public void grabCone() {
        pressure.set(HI);
        position.set(CLOSED);
    }

    public void grabCube() {
        pressure.set(LO);
        position.set(CLOSED);
    }

    public void release() {
        pressure.set(LO);
        position.set(OPEN);
    }

    public void turnOff() {
        pressure.set(OFF);
    }
}

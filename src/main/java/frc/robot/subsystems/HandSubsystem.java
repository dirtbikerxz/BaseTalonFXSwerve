package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * Represents the pneumatics for the hand. There are two double solenoids that
 * control its behavior:
 *   - PRESSURE (options: low, high or off)
 *   - POSITION (options: open, closed or off)
 */
public class HandSubsystem extends SubsystemBase {

    /** CAN bus ID for the pneumatics hub */
    public static final int REVPH_CAN_ID = 5;

    /** Switch IDs on the hub for the solenoids */
    public static final int CLAW_FORWARD = 0;
    public static final int CLAW_REVERSE = 1;
    public static final int BUMPER_FORWARD = 2;
    public static final int BUMPER_REVERSE = 3;

    /** Solenoid values for the different options */
    public static final Value EXTEND = Value.kReverse;
    public static final Value RETRACT = Value.kForward;
    public static final Value OPEN = Value.kForward;
    public static final Value CLOSED = Value.kReverse;
    public static final Value OFF = Value.kOff;

    private final DoubleSolenoid claw;
    private final DoubleSolenoid bumper;

    public HandSubsystem() {
        claw = new DoubleSolenoid(REVPH_CAN_ID, PneumaticsModuleType.REVPH, CLAW_FORWARD, CLAW_REVERSE);
        bumper = new DoubleSolenoid(REVPH_CAN_ID, PneumaticsModuleType.REVPH, BUMPER_FORWARD, BUMPER_REVERSE);
        SmartDashboard.putData("Hand", builder -> {
            builder.addStringProperty("Claw", this::getOpenCloseString, null);
            builder.addStringProperty("Bumper", this::getExtendRetractString, null);
        });
    }

    private String getOpenCloseString() {
        Value val = claw.get();
        if (val == OPEN) {
            return "OPEN";
        } else if (val == CLOSED) {
            return "CLOSED";
        } else {
            return "-";
        }
    }

    private String getExtendRetractString() {
        Value val = bumper.get();
        if (val == EXTEND) {
            return "EXTEND";
        } else if (val == RETRACT) {
            return "RETRACT";
        } else {
            return "-";
        }
    }

    public void closeClaw() {
        claw.set(CLOSED);
    }

    public void openClaw() {
        claw.set(OPEN);
    }

    public void extendBumper() {
        bumper.set(EXTEND);
    }

    public void retractBumper() {
        claw.set(RETRACT);
    }
}

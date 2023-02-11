package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.HandSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * Represents the pneumatics for the hand.
 */
public class HandSubsystem extends SubsystemBase {
    /**
     *
     */
    private static final int REVPH_CAN_ID = 5;
    private static final int CHARGE_FWD = 0;
    private static final int CHARGE_REV = 1;
    private static final int FIRE_FWD = 2;
    private static final int FIRE_REV = 3;
    private final DoubleSolenoid _Charge = new DoubleSolenoid(REVPH_CAN_ID, PneumaticsModuleType.REVPH, CHARGE_FWD, CHARGE_REV);
    private final DoubleSolenoid _Fire = new DoubleSolenoid(REVPH_CAN_ID, PneumaticsModuleType.REVPH, FIRE_FWD, FIRE_REV);

    private void setPressure(boolean high) {
        _Charge.set(high ? Value.kForward : Value.kReverse);
    }

    private void setPosition(boolean closed) {
        _Fire.set(closed ? Value.kReverse : Value.kForward);
    }

    public void grabCone() {
        setPressure(true);
        setPosition(true);
        // _Fire.set(DoubleSolenoid.Value.kReverse);
        // _Charge.set(DoubleSolenoid.Value.kForward);
        // _Fire.toggle();
    }

    public void grabCube() {
        setPressure(false);
        setPosition(true);
        // _Fire.set(DoubleSolenoid.Value.kReverse);
        // _Charge.set(DoubleSolenoid.Value.kReverse);
        // _Fire.toggle();
    }

    public void release() {
        setPressure(false);
        setPosition(false);
    //   _Charge.set(DoubleSolenoid.Value.kOff);
    }
}

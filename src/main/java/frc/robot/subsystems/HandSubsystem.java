package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.HandSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * Represents the pneumatics for the hand.
 */
public class HandSubsystem extends SubsystemBase {
    public static final String kDefaultAuto = "Default";
    public static final String kCustomAuto = "My Auto";
    public String m_autoSelected;
    public final SendableChooser<String> m_chooser = new SendableChooser<>();
    public final DoubleSolenoid _Charge = new DoubleSolenoid(5, PneumaticsModuleType.REVPH, 0, 1);
    public final DoubleSolenoid _Fire = new DoubleSolenoid(5, PneumaticsModuleType.REVPH, 2, 3);


}

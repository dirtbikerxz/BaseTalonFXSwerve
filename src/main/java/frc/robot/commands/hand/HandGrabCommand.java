package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HandSubsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HandGrabCommand extends CommandBase {

    public static final int CUBE = 0;
    public static final int CONE = 1;

    final HandSubsystem handSubsystem;
    private final int which;
    // private static final String kDefaultAuto = "Default";
    // private static final String kCustomAuto = "My Auto";
    // private String m_autoSelected;
    // private final SendableChooser<String> m_chooser = new SendableChooser<>();
    // private final DoubleSolenoid _Charge = new DoubleSolenoid(5, PneumaticsModuleType.REVPH, 0, 1);
    // private final DoubleSolenoid _Fire = new DoubleSolenoid(5, PneumaticsModuleType.REVPH, 2, 3);

    public HandGrabCommand(HandSubsystem handSubsystem, int which) {
        this.handSubsystem = handSubsystem;
        this.which = which;
        addRequirements(handSubsystem);
    }

    @Override
    public void initialize() {
        handSubsystem._Fire.set(DoubleSolenoid.Value.kReverse);

    }

    @Override
    public void execute() {
        // TODO what do we do here?
        if (which == CONE) {
            handSubsystem._Charge.set(DoubleSolenoid.Value.kForward);
            handSubsystem._Fire.toggle();
        }
        if (which == CUBE) {
            handSubsystem._Charge.set(DoubleSolenoid.Value.kReverse);
            handSubsystem._Fire.toggle();
        }
    }

    @Override
    public boolean isFinished() {
        // TODO what do we do here?
        return true;
    }
}

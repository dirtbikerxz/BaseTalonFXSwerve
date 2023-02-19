package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.OldArmSubsystem;

public class OldArmCalibrationCommand extends CommandBase {
    public static final int calibrateExtension = 0;
    public static final int calibrateRotation = 1;
    public final int which;
    private final OldArmSubsystem armSubsystem;
    public boolean done;
    
    
    
    public OldArmCalibrationCommand(OldArmSubsystem armSubsystem, int which) {
        this.armSubsystem = armSubsystem;
        this.which = which;
        
    }
    
    
    @Override
    public void initialize() {
        done = false;
    }
    
    @Override
    public void execute() {

        if (which == calibrateExtension) {
            if (armSubsystem.isExtenderLimitPressed()) {
                armSubsystem.MININUM_POSITION_EXTENDING = armSubsystem.extendingEncoder.getPosition();
                armSubsystem.MAXIMUM_POSITION_EXTENDING =
                        armSubsystem.MININUM_POSITION_EXTENDING +
                        OldArmSubsystem.TRAVEL_RANGE_EXTEDING;
                armSubsystem.extendingMotor.set(0);
                armSubsystem.extendingEncoder.setPosition(0);
                done = true;
            } else {
                armSubsystem.extendingMotor.set(-0.2);
            }
        }

        if (which == calibrateRotation) {
            if (armSubsystem.isRotatorLimitPressed()) {
                armSubsystem.MININUM_POSITION_ROTATING = armSubsystem.rotatingEncoder.getPosition();
                armSubsystem.MAXIMUM_POSITION_ROTATING =
                        armSubsystem.MININUM_POSITION_ROTATING +
                        OldArmSubsystem.TRAVEL_RANGE_ROTATING;
                armSubsystem.rotatingMotor.set(0);
                armSubsystem.rotatingEncoder.setPosition(0);
                done = true;
            } else {
                armSubsystem.rotatingMotor.set(-0.2);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}

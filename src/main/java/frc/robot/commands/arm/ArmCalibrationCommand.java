package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCalibrationCommand extends CommandBase {
    public static final int calibrateExtension = 0;
    public static final int calibrateRotation = 1;
    public final int which;
    private final ArmSubsystem armSubsystem;
    public boolean done;
    
    
    
    public ArmCalibrationCommand(ArmSubsystem armSubsystem, int which) {
        this.armSubsystem = armSubsystem;
        this.which = which;
        
    }
    
    
    @Override
    public void initialize() {
        done = false;
    }
    
    @Override
    public void execute() {

        boolean LS0Pressed = !armSubsystem.limitSwitch0.get();
        boolean LS1Pressed = !armSubsystem.limitSwitch1.get();
        System.err.println(LS0Pressed+"/"+LS1Pressed);

        if (which == calibrateExtension) {
            if (LS0Pressed) {
                armSubsystem.MAXIMUM_POSITION_EXTENDING = armSubsystem.extendingEncoder.getPosition();
                armSubsystem.extendingMotor.set(0);
                done = true;
            } else {
                armSubsystem.extendingMotor.set(-0.2);
            }
        }

        if (which == calibrateRotation) {
            if (LS1Pressed) {
                armSubsystem.MAXIMUM_POSITION_ROTATING = armSubsystem.rotatingEncoder.getPosition();
                armSubsystem.rotatingMotor.set(0);
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

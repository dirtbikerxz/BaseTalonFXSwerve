package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.other.Ultrasonic;
import frc.robot.subsystems.Swerve;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class ExecUltrasonic extends CommandBase{
    Swerve s_Swerve = new Swerve();
    Ultrasonic s_Ultrasonic = new Ultrasonic();
    Translation2d direction;
    boolean fieldRelative;
    boolean isOpenLoop;

    ExecUltrasonic(Translation2d direction, boolean fieldRelative, boolean isOpenLoop){
        this.direction = direction;
        this.fieldRelative = fieldRelative;
        this.isOpenLoop = isOpenLoop;
    }

    // @Override
    // public void execute() {
    //     s_Swerve.drive(new Translation2d(direction), new Rotation2d(0), fieldRelative, isOpenLoop);
    // }

    @Override
    public boolean isFinished() {
        if(s_Ultrasonic.getDistanceValue() == 0.0){
            return true;
        } else {
            return false;
        }
    }
    
}

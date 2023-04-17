package frc.robot.commands.AutoCommands;

import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.FieldCentricOffset;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/*
 * <h3>AutoCommand<h3>
 * 
 */
public class AutoCommand extends SequentialCommandGroup {

    private static final double MAX_ACCELERATION = Constants.AUTO_ACC;
    private static final double MAX_VELOCITY = Constants.AUTO_VEL;

    /**
     * <h3>AutoCommand</h3>
     * 
     * adding path constraints and builds auto command
     * 
     * @param preCommand a command that is run before the path starts
     * @param s_Swerve the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param eventCommandMap a command that uses strings to returna command that we want to execute at a marker
     * @param postCommand a command that is run after the path completes
     */
    public AutoCommand(Command preCommand, Swerve s_Swerve, String pathName, Map<String, Command> eventCommandMap, Command postCommand) {
        addRequirements(s_Swerve);

        PathConstraints pathConstraints = PathPlanner.getConstraintsFromPath(pathName);
        if(pathConstraints == null) {
            pathConstraints = new PathConstraints(MAX_VELOCITY, MAX_ACCELERATION);
        }

        // Load Path Group trajectory to follow.  All units in meters.
        List<PathPlannerTrajectory> loadPathGroup = PathPlanner.loadPathGroup(pathName, 
             false, pathConstraints);

             SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                s_Swerve::getPose,
                s_Swerve::resetOdometry,
                Constants.Swerve.swerveKinematics,
                new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0),
                new PIDConstants(Constants.AutoConstants.kPYController, 0.0, 0.0),
                s_Swerve::setModuleStates,
                eventCommandMap,
                true,
                s_Swerve
            );

        // Adding a pre command to autonomous ex. highscore
        if(preCommand != null) {
            addCommands(preCommand);
        }

        // creates a command based on the path group
        Command swerveControllerCommand = autoBuilder.fullAuto(loadPathGroup);
        addCommands(
            // TODO: Use april tags to help set this
            new InstantCommand(() -> FieldCentricOffset.getInstance().setOffset(loadPathGroup.get(0).getInitialHolonomicPose().getRotation().getDegrees())),
            swerveControllerCommand
        );
        if (postCommand != null) {
            addCommands(postCommand);
        }

    }

    /**
     * 
     * <h3>AutoCommand</h3>
     * 
     * adding path constraints and builds auto command
     * 
     * @param s_Swerve the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param eventCommandMap a command that uses strings to returna command that we want to execute at a marker
     * @param postCommand a command that is run after the path completes
     */
    public AutoCommand(Swerve s_Swerve, String pathName, Map<String, Command> eventCommandMap, Command postCommand) {
            this(null, s_Swerve, pathName, eventCommandMap, postCommand);
    }

    /**
     * <h3>AutoCommand</h3>
     * 
     * adding path constraints and builds auto command
     * 
     * @param s_Swerve the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param eventCommandMap a command that uses strings to returna command that we want to execute at a marker
     */
    public AutoCommand(Swerve s_Swerve, String pathName, Map<String, Command> eventCommandMap) {
        this(s_Swerve, pathName, eventCommandMap, null);
    }

    /**
     * 
     * <h3>AutoCommand</h3>
     * 
     * adding path constraints and builds auto command
     * 
     * @param preCommand a command that is run before the path starts
     * @param s_Swerve the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param eventCommandMap a command that uses strings to returna command that we want to execute at a marker
     */
    public AutoCommand(Command preCommand, Swerve s_Swerve, String pathName, Map<String, Command> eventCommandMap) {
        this(preCommand, s_Swerve, pathName, eventCommandMap, null);
    }

    /**
     * time based auto with an offset
     * @param timedCommand a timed based autonomous sequence
     * @param offset
     */
    public AutoCommand(Command timedCommand, double offset) {
        addCommands(
            // TODO: Use april tags to help set this
            new InstantCommand(() -> FieldCentricOffset.getInstance().setOffset(offset)),
            timedCommand
        );
    }

    /**
     * time based auto, 180 degree offset by default (front towards grid)
     * @param timedCommand a timed based autonomous sequence
     */
    public AutoCommand(Command timedCommand) {
        addCommands(
            // TODO: Use april tags to help set this
            new InstantCommand(() -> FieldCentricOffset.getInstance().setOffset(180.0)),
            timedCommand
        );

    }
}
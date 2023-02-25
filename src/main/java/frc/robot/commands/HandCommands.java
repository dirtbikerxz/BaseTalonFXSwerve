package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.HandSubsystem;

/**
 * Factor for hand-related commands
 */
public class HandCommands {

    /**
     * @return a command that will close the claw, and then retract the bumper
     * a tiny bit later
     */
    public static Command grab(HandSubsystem hand) {
        return new InstantCommand(hand::grabCone, hand)
                .andThen(new WaitCommand(0.5))
                .andThen(new InstantCommand(hand::retractDuck, hand));
    }

    /**
     * TODO this will do a low-pressure grab, if we reintroduce different compression levels
    public static Command grab(HandSubsystem hand) {
        return new InstantCommand(hand::grabCube, hand)
                .andThen(new WaitCommand(0.5))
                .andThen(new InstantCommand(hand::retractDuck, hand));
    }
     */

    /**
     * @return a command that will open the claw, and then extend the bumper
     * a tiny bit later
     */
    public static Command release(HandSubsystem hand) {
        return new InstantCommand(hand::release, hand)
                .andThen(new WaitCommand(0.5))
                .andThen(new InstantCommand(hand::extendDuck, hand));
    }
}

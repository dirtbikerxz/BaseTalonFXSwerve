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
        return new InstantCommand(hand::closeClaw, hand)
                .andThen(new WaitCommand(0.5))
                .andThen(new InstantCommand(hand::retractBumper, hand));
    }

    /**
     * @return a command that will open the claw, and then extend the bumper
     * a tiny bit later
     */
    public static Command release(HandSubsystem hand) {
        return new InstantCommand(hand::closeClaw, hand)
                .andThen(new WaitCommand(0.5))
                .andThen(new InstantCommand(hand::extendBumper, hand));
    }
}

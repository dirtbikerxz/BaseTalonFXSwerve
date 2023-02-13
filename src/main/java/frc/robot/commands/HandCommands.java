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
     * @return a command that will grab a cone
     */
    public static Command grabCone(HandSubsystem hand) {
        return new InstantCommand(hand::grabCone, hand);
    }

    /**
     * @return a command that will grab a cube
     */
    public static Command grabCube(HandSubsystem hand) {
        return new InstantCommand(hand::grabCube, hand);
    }

    /**
     * @return a command that will open the hand, then 1s later turn off the pressure solenoid
     */
    public static Command release(HandSubsystem hand) {
        return new InstantCommand(hand::release, hand)
                .andThen(new WaitCommand(1.0))
                .andThen(new InstantCommand(hand::turnOff, hand));
    }
}

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.HandCommands;

/**
 * All of the mapping of controls to commands happens here.
 */
public class TestBenchControlMapping {

    public static void mapControls(TestBench testBench, CommandXboxController controller) {
        controller.y().onTrue(HandCommands.grab(testBench.hand));
        controller.rightBumper().onTrue(HandCommands.release(testBench.hand));
    }
}

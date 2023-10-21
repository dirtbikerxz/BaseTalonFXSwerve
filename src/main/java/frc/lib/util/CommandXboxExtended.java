package frc.lib.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CommandXboxExtended extends CommandXboxController {
  public CommandXboxExtended(int port) {
    super(port);
  }

  public void setRumble(RumbleType type, double value) {
    this.getHID().setRumble(type, value);
  }
}

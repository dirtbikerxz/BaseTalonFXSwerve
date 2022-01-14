package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.NewMotor;
import frc.robot.Constants;

public class ZeroMotorsWaitCommand extends CommandBase {
    protected Timer m_timer = new Timer();
    private final double m_duration;
    private Swerve s_Swerve;
    private NewMotor s_NewMotor;

    public ZeroMotorsWaitCommand(double seconds) {
        this.m_duration = seconds;
        SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
        this.s_Swerve = new Swerve();
        this.s_NewMotor = new NewMotor();
      }

    @Override
    public void initialize() {
      s_NewMotor.stop();
      s_Swerve.setMotorsZero(Constants.Swerve.isOpenLoop, Constants.Swerve.isFieldRelative);
      m_timer.reset();
      m_timer.start();
    }

    @Override
    public void end(boolean interrupted) {
      m_timer.stop(); 
    }

    @Override
    public boolean isFinished() {
      return m_timer.hasElapsed(m_duration);
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
    
}

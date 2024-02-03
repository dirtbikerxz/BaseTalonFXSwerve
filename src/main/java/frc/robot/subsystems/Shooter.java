package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final MotorController lowRoller;
    private final MotorController highRoller;

    /**
     * Initialize shooter motors from motor IDs
     *
     * @param lowRollerId Motor ID of low motor
     * @param highRollerId Motor ID of high motor
     */
    public Shooter(int lowRollerId, int highRollerId) {
        this(
            new CANSparkMax(lowRollerId, CANSparkLowLevel.MotorType.kBrushless),
            new CANSparkMax(highRollerId, CANSparkLowLevel.MotorType.kBrushless)
        );
    }

    /**
     * Initialize shooter with given MotorControllers
     *
     * @param lowRoller MotorController for low roller
     * @param highRoller MotorController for high roller
     */
    public Shooter(MotorController lowRoller, MotorController highRoller) {
        this.lowRoller = lowRoller;
        this.highRoller = highRoller;
    }

    /**
     * Set the power of both shooter motors
     *
     * @param power Power [-1.0, 1.0] of the shooter motors
     */
    public void setPower(double power) {
        lowRoller.set(power);
        highRoller.set(power);
    }
}

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Shufflebot extends TimedRobot {

    private double foo;
    private String bar;
    private boolean baz;
    private TalonFX motor;
    private AHRS gyro;
    private DoubleSolenoid solenoid;
    private DigitalInput input;

    public Shufflebot() {
        reset();
        motor = new TalonFX(1);
        gyro = new AHRS(Port.kUSB);
        solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
        input = new DigitalInput(1);
    }

    public double getFoo() {
        return foo;
    }

    public void setFoo(double foo) {
        this.foo = foo;
    }

    public String getBar() {
        return bar;
    }

    public void setBar(String bar) {
        this.bar = bar;
    }

    public boolean isBaz() {
        return baz;
    }

    public void setBaz(boolean baz) {
        this.baz = baz;
    }

    public void reset() {
        foo = 1;
        bar = "BAR";
    }

    public void robotInit() {
        SmartDashboard.putData("Motor", new MotorWrapper(motor));
        SmartDashboard.putData("Switch", input);
        SmartDashboard.putData("Solenoid", solenoid);
        SmartDashboard.putData("Gyro", gyro);
        SmartDashboard.putData("Item", builder -> {
            builder.addDoubleProperty("Foo", this::getFoo, this::setFoo);
            builder.addStringProperty("Bar", this::getBar, this::setBar);
            builder.addBooleanProperty("Baz", this::isBaz, this::setBaz);
        });
    }

    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    public static CommandBase createCommand(String name) {
        CommandBase c = new InstantCommand(() -> System.err.println("hello from "+name+"!"));
        c.setName(name);
        return c;
    }

    public static class MotorWrapper implements MotorController, Sendable {

        private final TalonFX motor;

        public MotorWrapper(TalonFX motor) {
            this.motor = motor;
        }

        @Override
        public void disable() {
            stopMotor();
        }

        @Override
        public double get() {
            return motor.getMotorOutputPercent();
        }

        @Override
        public boolean getInverted() {
            return motor.getInverted();
        }

        @Override
        public void set(double speed) {
            motor.set(TalonFXControlMode.PercentOutput, speed);
        }

        @Override
        public void setInverted(boolean inverted) {
            motor.setInverted(inverted ? InvertType.InvertMotorOutput : InvertType.None);
        }

        @Override
        public void stopMotor() {
            motor.set(TalonFXControlMode.PercentOutput, 0.0);
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Motor Controller");
            builder.setActuator(true);
            builder.setSafeState(this::disable);
        }
    }

    public static Sendable wrapMotor(TalonFX motor) {
        return (builder) -> {
            builder.setSmartDashboardType("Motor Controller");
            builder.setActuator(true);
            builder.setSafeState(() -> motor.set(TalonFXControlMode.PercentOutput, 0));
            builder.addDoubleProperty("Value",
                    () -> motor.getMotorOutputPercent(),
                    (pct) -> motor.set(TalonFXControlMode.PercentOutput, pct));
        };
    }
}

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.OldArmSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftY;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightY;

public class OldArmTeleopLoopCommand extends CommandBase {

    private final OldArmSubsystem arm;
    private final XboxController inputController;
    private boolean done;

    private double restrictedOutput(double motorPosition, double controllerInput, double maxPosition, double minPosition){
        double restrictedOutput;
        if (controllerInput > 0 && motorPosition > maxPosition ){
            restrictedOutput = 0;
        } else if (controllerInput < 0 && motorPosition < minPosition){
            restrictedOutput = 0;
        } else {
            restrictedOutput = controllerInput;
        }

        return restrictedOutput;
    }

    public OldArmTeleopLoopCommand(OldArmSubsystem armSubsystem, XboxController xboxController){

        this.arm = armSubsystem;
        this.inputController = xboxController;
        addRequirements(armSubsystem);

    }  

    public void initialize(){

        done = false;

    }

    public void execute(){

        double rotatingMotorPosition = arm.rotatingEncoder.getPosition();
        double extendingMotorPosition = arm.extendingEncoder.getPosition();

        //Alternative: convert this to a method

        arm.rotatingMotor.set(restrictedOutput(rotatingMotorPosition, -inputController.getRawAxis(kLeftY.value), arm.MAXIMUM_POSITION_ROTATING, arm.MININUM_POSITION_ROTATING));
        arm.extendingMotor.set(restrictedOutput(extendingMotorPosition, -inputController.getRawAxis(kRightY.value), arm.MAXIMUM_POSITION_EXTENDING, arm.MININUM_POSITION_EXTENDING));
        
        /* 

        if (rotatingMotorPosition < arm.MAXIMUM_POSITION_ROTATING && rotatingMotorPosition > arm.MININUM_POSITION_EXTENDING){       
            arm.rotatingMotor.set(-inputController.getRawAxis(kLeftY.value));
        } else if (rotatingMotorPosition > arm.MAXIMUM_POSITION_ROTATING && -inputController.getRawAxis(kLeftY.value) > 0){
            arm.rotatingMotor.set(0);
            SmartDashboard.getBoolean("", done)
        } else if (rotatingMotorPosition < arm.MININUM_POSITION_ROTATING && -inputController.getRawAxis(kLeftY.value) < 0){
            arm.rotatingMotor.set(0);
        } else {
            arm.rotatingMotor.set(-inputController.getRawAxis(kLeftY.value));
        }

        if (extendingMotorPosition < arm.MAXIMUM_POSITION_EXTENDING && extendingMotorPosition > arm.MININUM_POSITION_EXTENDING){
            arm.extendingMotor.set(-inputController.getRawAxis(kRightY.value));
        } else if (extendingMotorPosition > arm.MAXIMUM_POSITION_EXTENDING && -inputController.getRawAxis(kRightY.value) > 0){
            arm.rotatingMotor.set(0);
        } else if (rotatingMotorPosition < arm.MININUM_POSITION_EXTENDING && -inputController.getRawAxis(kRightY.value) < 0){
            arm.rotatingMotor.set(0);
        } else {
            arm.extendingMotor.set(-inputController.getRawAxis(kRightY.value));
        }

        */

        SmartDashboard.putNumber("rotatingMotorPosition", rotatingMotorPosition);
        SmartDashboard.putNumber("extendingMotorPosition", extendingMotorPosition);
        SmartDashboard.putNumber("rotatingMotorVelocity", arm.rotatingEncoder.getVelocity());
        SmartDashboard.putNumber("extendingMotorVelocity", arm.extendingEncoder.getVelocity());
        SmartDashboard.putNumber("restrictedMotorRotating", restrictedOutput(rotatingMotorPosition, -inputController.getRawAxis(kLeftY.value), arm.MAXIMUM_POSITION_ROTATING, arm.MININUM_POSITION_ROTATING));
        SmartDashboard.putNumber("restrictedOutputExtending", restrictedOutput(extendingMotorPosition, -inputController.getRawAxis(kRightY.value), arm.MAXIMUM_POSITION_EXTENDING, arm.MININUM_POSITION_EXTENDING));

    }

    public boolean isFinished(){

        return done;
        
    }
}

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmPresetCommand extends CommandBase{

    public static final int TRAVEL_PRESET = 0;
    public static final int PICKUP_PRESET = 1;

    public static final double [][] PRESETS = {
        { 150, -150 },   // preset values for "travel mode"
        { 2.0, 3.0 }    // preset values for "pickup" mode
    };


    private double indicatedPositionRotating;
    private double indicatedPositionExtending;
    private final ArmSubsystem arm;
    private boolean done;

    
    public ArmPresetCommand(ArmSubsystem arm, int which){

        this.arm = arm;
        this.indicatedPositionExtending = PRESETS[which][0];
        this.indicatedPositionRotating = PRESETS[which][1];

        addRequirements(arm);

    }

    private boolean withinRangeOfAcceptance(double wantedValue, double toleranceWidth, double input ){
 
        if (input < wantedValue + toleranceWidth && input > wantedValue - toleranceWidth){
            return true;
        } else {
            return false;
        }
    }

    public void initialize(){

        done =false;

    }

    public void execute(){

        double dispositionRotating = indicatedPositionRotating - arm.rotatingEncoder.getPosition();
        double dispositionExtending = indicatedPositionExtending - arm.extendingEncoder.getPosition();
       

        if (withinRangeOfAcceptance(indicatedPositionExtending, 0.5, arm.extendingEncoder.getPosition()) &&
         withinRangeOfAcceptance(indicatedPositionRotating, 0.5, arm.rotatingEncoder.getPosition())){

            done = true;

        } else {    

            if (dispositionRotating > 35){
                arm.rotatingMotor.set(1);
            } else if (dispositionRotating < -35){
                arm.rotatingMotor.set(-1);
            } else if (dispositionRotating > 0){
                arm.rotatingMotor.set(0.15);
            } else if (dispositionRotating < 0){
                arm.rotatingMotor.set(-0.15);
            }

            if (dispositionExtending > 35){
                arm.extendingMotor.set(1);
            } else if (dispositionExtending < -35){
                arm.extendingMotor.set(-1);
            } else if (dispositionExtending > 0){
                arm.extendingMotor.set(0.15);
            } else if (dispositionExtending < 0){
                arm.extendingMotor.set(-0.15);
            }


        }
        

        SmartDashboard.putNumber("rotatingMotorPosition", arm.rotatingEncoder.getPosition());
        SmartDashboard.putNumber("extendingMotorPosition", arm.extendingEncoder.getPosition());
        SmartDashboard.putNumber("rotatingMotorVelocity", arm.rotatingEncoder.getVelocity());
        SmartDashboard.putNumber("extendingMotorVelocity", arm.extendingEncoder.getVelocity());

        
    }

    public boolean isFinished(){

        return done;

    }

    
    
}

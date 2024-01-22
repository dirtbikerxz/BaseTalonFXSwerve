// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class TemplateSubsystem extends SubsystemBase {
  /* Instance Variables
   * This section should contain any variables that need to 
   * be accessed by your entire subsystem/class. This usually includes any phyical motors or sensors that are a part of your subsystem,
   * as well as any data that needs to be accessed by your 
   * entire subsystem/class E.G. the height of the elevator 
   * or angle of the wrist.
   */

  /* These are variable declarations. Their access modifiers,
   * types, and names are specified but they are not given a 
   * value yet. They should be given a value in the 
   * constructor. In this example, they is private, which 
   * means that nothing outside of this class can access it.
   */
  private CANSparkMax motor;
  private double internalData;
  
  /* Constructor
   * The Constructor is a special type of method that gets 
   * called when this subsystem/class is created, usually 
   * when the robot turns on. You should give a value to most
   * instance variables created above in this method. You can
   * also do anything else that you want to happen right away.
   */
  public TemplateSubsystem() {
    /* These are variable initializations, where a variable 
     * is given a value. In this case, the `new` keyword is 
     * used to create a new CANSparkMax object and give it 
     * to `motor` as it's value. 
     */
    motor = new CANSparkMax(0, MotorType.kBrushless);
    internalData = 0.4;
  }

  /* Methods
   * The below section should contain any methods you want 
   * to use in your class.
   */

   /* The below method is a private method, which means that 
    * it can only be used inside of this class. It wants a 
    * single double value as input, which it is going to 
    * store in the variable `num`. It then takes `num` and 
    * multiplies it by 3 and returns the resulting value.
    */
  private double tripleNum(double num) {
    return num * 3;
  }

  /* The below method is public, which means that it can be 
   * accessed outside of this class. It also takes in a 
   * single double value as a parameter. It then takes that 
   * value (called `number`) and multiplies it by 
   * `internalData` to change it's value.
   */
  public void doSomething(double number) {
    internalData = internalData * number;
  }

  /* The below method is included in every Subsystem. You can
   * think of it as an infinite loop that runs constantly 
   * while the robot is on.
   * 
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

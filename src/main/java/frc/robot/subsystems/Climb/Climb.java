// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climb.ClimbIO.ClimbInput;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */

  ClimbIO io ;
  ClimbInputAutoLogged input;
  boolean hasLimit;
  
  
  public Climb() {
    hasLimit = false;
  }

  public void setMaximum(){
    io.setMaximum();
    hasLimit = true;
  }
  public void setMinimum(){
    io.setMinimum();
    hasLimit= true;
  } //TODO: ask Eyal if position equals zero is a problem 

  
  public boolean getHasLimit(){return hasLimit;}
  public void setCoast(){io.setCoast();}
  public void setBrake(){io.setBrake();}
  public void resetEncoder(){io.resetEncoder();}
  public void startMotor(double power){io.setMotorDutyCycle(power);}//TODO: Ask Eyal if power should be in subsystem or commend :3
  
  public double getHookHeight(){
    //TODO: Aproximate hook movement with graph:)
    return 0;
  }
  public boolean getIsHookOnChain(){
    return input.motorCurrent>ClimbConstants.CLIMB_CURRENT_WHEN_ON_HOOK;
  
  }
  public double getPosition(){return input.motorPosition;}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.update(input);
    Logger.recordOutput("Is hook on chain", getIsHookOnChain());
    Logger.recordOutput("hook height",getHookHeight() );
    
  
  }


}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
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
  }

  public boolean hasLimit(){return hasLimit;}
  public void setCoast(){io.setCoast();}
  public void setBrake(){io.setBrake();}
  public void resetEncoder(){io.resetEncoder();}

  public void startMotor(double power){
    MathUtil.clamp(power, -0.8, 0.8);
    io.setMotorDutyCycle(power);
  }
  
  public double getHookHeight(){
    //TODO: Aproximate hook movement with graph:)
    return 0;
  }

  public boolean getIsHookOnChain() {return (input.motorCurrent > ClimbConstants.CLIMB_CURRENT_WHEN_ON_HOOK) 
                                    && (Math.abs(input.motorPosition) > ClimbConstants.CHAIN_HEIGHT);}
  public double getPosition(){return input.motorPosition;}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.update(input);
    Logger.recordOutput("Climb/Is hook on chain", getIsHookOnChain());
    Logger.recordOutput("Climb/Hook Height",getHookHeight() );
  }
}

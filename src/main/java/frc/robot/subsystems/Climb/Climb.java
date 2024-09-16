// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climb.ClimbIO.ClimbInputs;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  ClimbIO io ;
  ClimbInputsAutoLogged input;
  boolean hasLimit;
  Pose3d hookPosition;
  
  public Climb() {  }

  public void startMotor(double power){
    power = MathUtil.clamp(power, 0, 1);
    io.setMotorDutyCycle(power);
  }

  public void stopMotor(){
    io.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.update(input);
    // Logger.recordOutput("Climb/Is hook on chain", getIsHookOnChain());
  }
}

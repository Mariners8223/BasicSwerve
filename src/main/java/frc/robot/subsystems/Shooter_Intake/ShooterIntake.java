// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class ShooterIntake extends SubsystemBase {
  ShooterIntakeIO io;
  ShooterIntakeInputsAutoLogged inputs = new ShooterIntakeInputsAutoLogged();

  boolean isGpLoaded = false;
  /** Creates a new ShooterIntake. */
  public ShooterIntake() {}
  
  @Override
  public void periodic() {
    io.update(inputs);
    // This method will be called once per scheduler run
    Logger.processInputs("Shooter Intake", inputs);
  }
  public boolean isGpLoaded(){
    return isGpLoaded;
  }
  public void setGpLoaded(boolean gpLoaded){
    this.isGpLoaded = gpLoaded;
  }

  public void setIntakeMotorTargetPosition(double rotation){
    io.setIntakeTargetPosition(rotation);
  }

  public double getIntakeMotorPositions(){
    return inputs.intakePosition;
  }

  public void setTargetRPMShooterMotorOnPivot(double speed){
    io.setTargetShooterMotorOnPivotRPM(speed);
  }
  public void setTargetRPMShooterMotorOffPivot(double speed){
    io.setTargetShooterMotorOffPivotRPM(speed);
  }
  public void setTargetIntakeMotorRPM(double speed){
    io.setTargetIntakeMotorRPM(speed);
  }
  
  public boolean getBeamBreakValue(){
    return inputs.BeamBreakValue;
  }
  public void StopMotorOnPivot(){
    io.StopMotorOnPivot();
  }
   public void StopMotorOffPivot(){
    io.StopMotorOffPivot();
  }
  public void stopIntakeMotor(){
    io.stopIntakeMotor();
  }
  public double getIntakeMotorRPM(){
    return inputs.IntakeMotorRPM;
}
public boolean getResistance(){
  return inputs.ResistanceInputMotor;
}
public void getTargetRPMShooterMotorOnPivot(double speed){
    io.getTargetShooterMotorOnPivotRPM(speed);
  }
public void getTargetRPMShooterMotorOffPivot(double speed){
    io.getTargetShooterMotorOffPivotRPM(speed);
  }

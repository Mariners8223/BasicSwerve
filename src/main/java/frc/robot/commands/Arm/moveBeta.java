// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

public class moveBeta extends Command {
  /** Creates a new moveBeta. */
  private final Arm arm;
  private final double wantedBetaPos;
  public moveBeta(Arm arm, double wantedBetaPos) {
    this.arm=arm;  
    this.wantedBetaPos=wantedBetaPos;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.MoveBeta(wantedBetaPos);
  }
  
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (arm.GetBetaPosition()== wantedBetaPos);
  }
}

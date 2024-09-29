// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmConstants.ArmPosition;

public class MoveAlpha extends Command {
  /** Creates a new moveAlpha. */
  private final Arm arm;
  private final double wantedAlphaPos;
  public MoveAlpha(Arm arm, double wantedAlphaPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.wantedAlphaPos = wantedAlphaPos;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.moveAlpha(wantedAlphaPos);
  }

 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){arm.currentPos = ArmPosition.UNKNOWN;}
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (arm.getAlphaPosition() == wantedAlphaPos); //TODO: Check if this case is correct
  }
}

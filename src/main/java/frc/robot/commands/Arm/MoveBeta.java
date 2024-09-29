// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Arm.ArmConstants.ArmPosition;

public class MoveBeta extends Command {
  /** Creates a new moveBeta. */
  private final Arm arm;
  private final double wantedBetaPos;
  public MoveBeta(Arm arm, double wantedBetaPos) {
    this.arm=arm;  
    this.wantedBetaPos=wantedBetaPos;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.moveBeta(wantedBetaPos);
  }
  
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
        arm.moveBeta(arm.getBetaPosition());
        arm.setArmPositionUnknown();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(arm.getBetaPosition() - wantedBetaPos) <= ArmConstants.ARM_POSITION_TOLERANCE); //TODO: Check if this case is correct
  }
}

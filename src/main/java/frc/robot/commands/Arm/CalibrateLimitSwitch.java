// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmConstants;

public class CalibrateLimitSwitch extends Command {
  /** Creates a new CalibrateLimitSwitch. */
  private final Arm arm;
  private CalibrateLimitSwitch(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);
  }

  public static Command getCommand(Arm arm){
    // return new CalibrateLimitSwitch(arm).onlyIf(() -> !arm.isCalibrated());
    return new MoveAlpha(arm, ArmConstants.ArmPosition.FREE_POSITION.getAlpha())
    .andThen(new CalibrateLimitSwitch(arm)).onlyIf(() -> !arm.isCalibrated());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.moveBetaDutyCycle(-0.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopBeta();
    if(!interrupted){
      arm.resetBetaEncoder();
      arm.enableBetaSoftLimits();
      arm.setArmCalibrated();
    } 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(arm.getLimitSwitch());
  }
}

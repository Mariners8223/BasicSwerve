// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.util.function.Supplier;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

public class BetaAim_command extends Command {
  private final Arm arm;
  private final Supplier<Measure<Angle>> betaTarget;
  
  public BetaAim_command(Arm arm, Supplier<Measure<Angle>> betaTarget){
    this.arm = arm;
    this.betaTarget = betaTarget;
      addRequirements(arm);
  } 
  /** Creates a new betaAim_command. */ {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.MoveBeta(Math.PI - betaTarget.get().in(Units.Radian));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

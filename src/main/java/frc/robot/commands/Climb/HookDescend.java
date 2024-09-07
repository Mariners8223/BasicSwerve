// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbConstants;

public class HookDescend extends Command {
  /** Creates a new HookDescend. */
  Climb climb ;
  Timer timer;

  public HookDescend(Climb climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climb = climb; 
    addRequirements(climb);

    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(Timer.getMatchTime() < 120|| !climb.hasLimit()) cancel();

    if(climb.getPosition() > 0) climb.startMotor(-1*ClimbConstants.CLIMB_MOTOR_POWER);
    else {climb.startMotor(ClimbConstants.CLIMB_MOTOR_POWER);}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.setBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

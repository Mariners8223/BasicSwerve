// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climb.Climb;

public class HookAscend extends Command {
  /** Creates a new HookAscend. */
  Climb climb ;
  Timer timer;

  public HookAscend() {
    // Use addRequirements() here to declare subsystem dependencies.
    climb = RobotContainer.climb; 
    addRequirements(climb);

    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(timer.getMatchTime()<120){cancel();}//TODO: Ask Eyal if we can do it in initialize :3
    if(!climb.getHasLimit()){timer.start();}
    climb.setCoast();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(timer.get()>=1){
      if(climb.getPosition()<0){climb.setMaximum();}
      else{climb.setMinimum();}
      
      timer.reset();
      timer.stop();
    }

  }

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

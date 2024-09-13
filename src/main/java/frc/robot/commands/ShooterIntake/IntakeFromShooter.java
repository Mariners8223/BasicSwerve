// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter_Intake.ShooterIntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromShooter extends SequentialCommandGroup {
   private static final double INTAKE_SPEED = ShooterIntakeConstants.PresetSpeeds.SPEED3.RPM;
  private static final double SLOW_INTAKE_SPEED = ShooterIntakeConstants.PresetSpeeds.SPEED5.RPM;
  /** Creates a new IntakeFromShooter. */
  public IntakeFromShooter() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
  private class Step1 extends Command{
    private Step1(){

    }

    @Override
    public void initialize() {
      shooterIntake.setTargetIntakeMotorRPM(SLOW_INTAKE_SPEED);
      shooterIntake.setTargetRPMShooterMotorOnPivot(INTAKE_SPEED);
      shooterIntake.setTargetRPMShooterMotorOffPivot(INTAKE_SPEED);
    }
    public void execute(){

    }

    @Override
    public void end(boolean interrupted) {
    }

  @Override
  public boolean isFinished() {
    return shooterIntake.getResistance();
  }

  }

  private class Step2 extends Command{
    private Step2(){

    }

    @Override
    public void initialize() {
      shooterIntake.setTargetIntakeMotorRPM(SLOW_INTAKE_SPEED);
      shooterIntake.setTargetRPMShooterMotorOffPivot(0);
      shooterIntake.setTargetRPMShooterMotorOnPivot(0);
    }

    @Override
    public void end(boolean interrupted) {
      //speed might not be RPM so idk
      shooterIntake.setTargetIntakeMotorRPM(0);
      setGpLoaded();
    }
  @Override
  public boolean isFinished() {
    return shooterIntake.getBeamBreakValue();
  }

  }
}

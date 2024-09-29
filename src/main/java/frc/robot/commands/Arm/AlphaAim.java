// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.util.function.Supplier;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmConstants.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlphaAim extends SequentialCommandGroup {
  /** Creates a new alphaAim. */
  public AlphaAim(Arm arm, Supplier<Measure<Angle>> alphaTarget) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
     new MoveArmToPosition(arm, ArmPosition.FREE_POSITION),
     new MoveBeta(arm,ArmPosition.AIM_POSITION.getBeta()),
     new AlphaAim_command(arm, alphaTarget)
    );          
  }
}

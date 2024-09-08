// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Arm.ArmConstants.ArmPos;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class moveArmToPos extends SequentialCommandGroup {
  /** Creates a new moveArmToPos. */
  public moveArmToPos(Arm arm, ArmPos targetPos, ArmConstants freePos, ArmConstants collectFloorPos) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new InstantCommand(),
      //new moveAlpha(null, 0).onlyIf(() -> !(arm.GetAlphaPos() == targetPos.getAlpha())),
      new moveBeta(arm, ArmConstants.ArmPos.FREE_POSITION.getBeta()).onlyIf(() -> !(arm.GetBetaPos() == ArmConstants.ArmPos.FREE_POSITION.getBeta())),
      new moveAlpha(arm, ArmConstants.ArmPos.FREE_POSITION.getAlpha()).onlyIf(() -> !(arm.GetAlphaPos() == ArmConstants.ArmPos.FREE_POSITION.getAlpha())),
      
      new moveBeta(arm, targetPos.getBeta()),
      new moveAlpha(arm, targetPos.getAlpha())
    );

  }
}

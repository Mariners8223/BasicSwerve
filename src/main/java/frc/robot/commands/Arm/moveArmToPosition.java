// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmConstants.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class moveArmToPosition extends SequentialCommandGroup {
  /** Creates a new moveArmToPosition. */
  public moveArmToPosition(Arm arm, ArmPosition targetPos) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(  

      new moveAlpha(arm, ArmPosition.FREE_POSITION.getAlpha()).onlyIf(() -> targetPos != arm.getCurrentPos() && arm.getCurrentPos() == ArmPosition.COLLECT_FLOOR_POSITION),
      new moveBeta(arm, ArmPosition.FREE_POSITION.getBeta()).onlyIf(() -> targetPos != arm.getCurrentPos() && arm.getCurrentPos() == ArmPosition.COLLECT_FLOOR_POSITION),

      new moveAlpha(arm, ArmPosition.FREE_POSITION.getAlpha()).onlyIf(() -> targetPos != arm.getCurrentPos() && arm.getCurrentPos() != ArmPosition.COLLECT_FLOOR_POSITION && targetPos == ArmPosition.COLLECT_FLOOR_POSITION),
      new moveBeta(arm, ArmPosition.COLLECT_FLOOR_POSITION.getBeta()).onlyIf(() -> targetPos != arm.getCurrentPos() && arm.getCurrentPos() != ArmPosition.COLLECT_FLOOR_POSITION && targetPos == ArmPosition.COLLECT_FLOOR_POSITION),

      new moveAlpha(arm, targetPos.getAlpha()).onlyIf(() -> targetPos != arm.getCurrentPos())
    );

  }
}

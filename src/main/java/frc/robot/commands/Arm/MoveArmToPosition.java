// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmConstants.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveArmToPosition extends SequentialCommandGroup {

  private ArmPosition currentPosition = ArmPosition.UNKNOWN;
  /** Creates a new moveArmToPosition. */
  private MoveArmToPosition(Arm arm, ArmPosition targetPos) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(

      new MoveAlpha(arm, ArmPosition.FREE_POSITION.getAlpha()).onlyIf(() -> {
        currentPosition = arm.getCurrentPos();

        return currentPosition == ArmPosition.COLLECT_FLOOR_POSITION || currentPosition == ArmPosition.UNKNOWN;
      }),
      new MoveBeta(arm, ArmPosition.FREE_POSITION.getBeta()).onlyIf(() -> currentPosition == ArmPosition.COLLECT_FLOOR_POSITION || currentPosition == ArmPosition.AMP_POSITION || currentPosition == ArmPosition.UNKNOWN),

      new MoveAlpha(arm, ArmPosition.FREE_POSITION.getAlpha()).onlyIf(() -> currentPosition != ArmPosition.COLLECT_FLOOR_POSITION && targetPos == ArmPosition.COLLECT_FLOOR_POSITION),
      new MoveBeta(arm, ArmPosition.COLLECT_FLOOR_POSITION.getBeta()).onlyIf(() -> currentPosition != ArmPosition.COLLECT_FLOOR_POSITION && targetPos == ArmPosition.COLLECT_FLOOR_POSITION),

      new MoveAlpha(arm, targetPos.getAlpha()),
      new MoveBeta(arm, ArmPosition.AMP_POSITION.getBeta()).onlyIf(() -> targetPos == ArmPosition.AMP_POSITION)
    );

  }

  public static Command getCommand(Arm arm, ArmPosition targetPos){
    Command command = new MoveArmToPosition(arm, targetPos).onlyIf(() -> arm.isCalibrated() && arm.getCurrentPos() != targetPos);

    command.setName("Move Arm To " + targetPos);

    return command;
  }
}

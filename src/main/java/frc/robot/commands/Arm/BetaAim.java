// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.util.function.Supplier;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmConstants.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BetaAim extends SequentialCommandGroup {
  /** Creates a new betaAim. */
  private BetaAim(Arm arm, Supplier<Measure<Angle>> betaTarget) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
   addCommands(
     MoveArmToPosition.getCommand(arm, ArmPosition.COLLECT_FLOOR_POSITION),
     new MoveAlpha(arm, ArmPosition.AIM_POSITION.getAlpha()),
     new BetaAim_command(arm, betaTarget)
    );
  }

    public static Command getCommand(Arm arm, Supplier<Measure<Angle>> betaTarget){
        Command command = new BetaAim(arm, betaTarget).onlyIf(arm::isCalibrated);

        command.setName("Beta Aim");

        return command;
    }
}
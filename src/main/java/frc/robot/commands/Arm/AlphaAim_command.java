// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.util.function.Supplier;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

public class AlphaAim_command extends Command {
    /**
     * Creates a new alphaAim_command.
     */
    private final Arm arm;
    private final Supplier<Measure<Angle>> alphaTarget;

    public AlphaAim_command(Arm arm, Supplier<Measure<Angle>> alphaTarget) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.arm = arm;
        this.alphaTarget = alphaTarget;
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        arm.moveAlpha(alphaTarget.get().in(Units.Rotations));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

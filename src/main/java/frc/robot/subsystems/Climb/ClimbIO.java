// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimbIO {

    @AutoLog
    public static class ClimbInputs{
        double motorTemperature;
        double motorAppliedOutput;
    }

    public void setMotorDutyCycle(double power);
    public void stopMotor();

    public void update(ClimbInputsAutoLogged inputs);
}
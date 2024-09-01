// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public interface ClimbIO {

    @AutoLog
    public static class ClimbInput{
        double motorTemperature;
        double motorCurrent;
        double motorSpeed;
        double motorPosition;
    }

    public void setMotorDutyCycle(double power);
    public void setCoast();
    public void setBrake();
    public void setMinimum();
    public void setMaximum();
    public void resetEncoder();

    public void update(ClimbInputAutoLogged inputs);
}
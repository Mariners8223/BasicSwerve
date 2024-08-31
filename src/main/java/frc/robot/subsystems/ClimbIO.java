// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public interface ClimbIO {

    @AutoLog
    public class ClimbInput{
        double motorTemperature;
        double motorCurrent;
        double motorSpeed;
        double hookHeight;
        int motorPosition;
        Pose3d hookPosition;
    }

    public void spinMotor(double speed);
    public void setCoast();
    public void setLimit(boolean isMinimum);
    public void resetEncoder();

    public void update(ClimbInputAutoLogged inputs);
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIntakeIO {

    @AutoLog
    public class ShooterIntakeInputs{
        double RPMShooterMotorOnPivot;
        double RPMShooterMotorOffPivot;
        double IntakeMotorRPM;
        double intakePosition;
        boolean BeamBreakValue;
        boolean ResistanceInputMotor;
    }
//RPM = rotations per min
    public void setTargetShooterMotorOnPivotRPM(double speedRPM);
    public void setTargetShooterMotorOffPivotRPM(double speedRPM);
    public void setTargetIntakeMotorRPM(double speedRPM);
    public void StopMotorOnPivot();
    public void StopMotorOffPivot();
    public void stopIntakeMotor();
    public void setIntakeTargetPosition(double rotation);
    public void getTargetShooterMotorOnPivotRPM(double speedRPM);
    public void getTargetShooterMotorOffPivotRPM(double speedRPM);

    public void update(ShooterIntakeInputsAutoLogged inputs);


}

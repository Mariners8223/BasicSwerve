// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

// import com.ctre.phoeni

/** Add your docs here. */
public class ClimbIOReal implements ClimbIO {
    VictorSPX motorSPX;

    public ClimbIOReal(){
        this.motorSPX = new VictorSPX(ClimbConstants.MOTOR_ID);

        motorSPX.enableVoltageCompensation(true);
        motorSPX.setInverted(ClimbConstants.MOTOR_INVERTED);
    }

    @Override
    public void setMotorDutyCycle(double power){motorSPX.set(VictorSPXControlMode.PercentOutput, power);}

    @Override
    public void stopMotor() {motorSPX.set(VictorSPXControlMode.Disabled, 0);}

    @Override
    public void update(ClimbInputsAutoLogged inputs){
        inputs.motorAppliedOutput = motorSPX.getMotorOutputPercent();
        inputs.motorTemperature = motorSPX.getTemperature();
    }


}
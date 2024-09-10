// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class ClimbIOSim implements ClimbIO {

    DCMotorSim motor;
    boolean max;

    public ClimbIOSim(DCMotorSim motor){
        this.motor = motor;
    }

    @Override
    public void setMotorDutyCycle(double power){ motor.setInputVoltage(power * RobotController.getBatteryVoltage()); }
    
    @Override
    public void setCoast(){ motor.setState(motor.getAngularPositionRotations(), 3); } //TODO: Check actual velocity
    public void setBrake(){ motor.setState(motor.getAngularPositionRotations(), 0); }
    
    @Override
    public void resetEncoder(){ motor.setState(0, motor.getAngularVelocityRadPerSec());}

    @Override
    public void setMaximum(){ max = true; }
    public void setMinimum(){ max = false; }

    @Override
    public void update(ClimbInputAutoLogged inputs){
        motor.update(1/50);

        inputs.motorCurrent = motor.getCurrentDrawAmps();
        inputs.motorPosition = motor.getAngularPositionRotations();
        inputs.motorSpeed = motor.getAngularVelocityRPM();
        inputs.motorTemperature = -274;

        if (max){
            if (inputs.motorPosition > 0) { setBrake(); }
        }
        if (!max){
            if (inputs.motorPosition < 0) { setBrake(); }
        }
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class ClimbIOSim implements ClimbIO {

    DCMotorSim motor;
    DCMotor gearbox;
    boolean max;

    public ClimbIOSim(){
        gearbox = new DCMotor(0, 0, 0, 0, 0, 0);
        motor = new DCMotorSim(gearbox, ClimbConstants.SimConstants.GEARING, 1);
    }

    @Override
    public void setMotorDutyCycle(double power){ motor.setInputVoltage(power * RobotController.getBatteryVoltage()); }

    @Override
    public void stopMotor(){ motor.setState(motor.getAngularPositionRad(), 0); }

    @Override
    public void update(ClimbInputsAutoLogged inputs){
        motor.update(1/50);
        inputs.motorAppliedOutput = motor.getCurrentDrawAmps();
        inputs.motorTemperature = -274;
    }
}

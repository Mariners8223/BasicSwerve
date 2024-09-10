// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class ClimbIOReal implements ClimbIO {

    TalonFX motorFx;
    public ClimbIOReal(TalonFX motorFx){ this.motorFx = motorFx; }

    @Override
    public void setBrake(){ motorFx.setNeutralMode(NeutralModeValue.Brake); }
    public void setCoast(){ motorFx.setNeutralMode(NeutralModeValue.Coast); }
    // ConfigStatorCurrentLimit to slow the brake

    @Override
    public void setMotorDutyCycle(double power){motorFx.set(power);}

    @Override
    public void resetEncoder(){motorFx.setPosition(0);}

    @Override
    public void setMaximum(){
        SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs();
        config.ForwardSoftLimitThreshold = 0;
        config.ForwardSoftLimitEnable = true;
        motorFx.getConfigurator().apply(config);
    }

    @Override
    public void setMinimum(){
        SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs();
        config.ForwardSoftLimitThreshold = 0;
        config.ForwardSoftLimitEnable = true;
        motorFx.getConfigurator().apply(config);
    }

    @Override
    public void update(ClimbInputAutoLogged inputs){
        inputs.motorCurrent = motorFx.getSupplyCurrent().getValueAsDouble() ; 
        inputs.motorPosition = motorFx.getPosition().getValueAsDouble();
        inputs.motorSpeed=  motorFx.getVelocity().getValueAsDouble();
        inputs.motorTemperature = motorFx.getDeviceTemp().getValueAsDouble();
    }


}
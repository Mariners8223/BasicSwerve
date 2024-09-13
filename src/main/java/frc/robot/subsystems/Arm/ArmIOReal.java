// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class ArmIOReal implements ArmIO {
    CANSparkMax alphaMotor;
    CANSparkMax betaMotor;

    SparkAbsoluteEncoder encoder_Abs;
    RelativeEncoder encoder_Relative;
    DigitalInput limitSwitch;

    SparkPIDController alphaPID;

    public ArmIOReal(){
        alphaMotor = configAlphaMotor();
        betaMotor = configBetaMotor();
        encoder_Abs = alphaMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        encoder_Relative = alphaMotor.getAlternateEncoder(ArmConstants.ccountsPerRev);
        limitSwitch = new DigitalInput(ArmConstants.limitSwitchPort);

       // alphaMotor.getPIDController().setP(0);
    }

    public CANSparkMax configAlphaMotor(){
        CANSparkMax alphaMotor;
            alphaMotor = new CANSparkMax(ArmConstants.AlphaConstants.MOTOR_ID, MotorType.kBrushless);
            alphaMotor.setInverted(ArmConstants.AlphaConstants.IS_INVERTED);

            alphaMotor.getPIDController().setP(ArmConstants.AlphaConstants.PID.getP());
            alphaMotor.getPIDController().setI(ArmConstants.AlphaConstants.PID.getI());
            alphaMotor.getPIDController().setD(ArmConstants.AlphaConstants.PID.getD());
            alphaMotor.getPIDController().setFF(ArmConstants.AlphaConstants.PID.getF());

            return alphaMotor;
    }
        
    public CANSparkMax configBetaMotor(){
        CANSparkMax betaMotor;
            betaMotor = new CANSparkMax(ArmConstants.BetaConstants.MOTOR_ID, MotorType.kBrushless);
            betaMotor.setInverted(ArmConstants.BetaConstants.IS_INVERTED);
            betaMotor.getEncoder().setPositionConversionFactor(ArmConstants.BetaConstants.GEAR_RATIO);

            betaMotor.getPIDController().setP(ArmConstants.BetaConstants.PID.getP());
            betaMotor.getPIDController().setI(ArmConstants.BetaConstants.PID.getI());
            betaMotor.getPIDController().setD(ArmConstants.BetaConstants.PID.getD());
            betaMotor.getPIDController().setFF(ArmConstants.BetaConstants.PID.getF());

            return betaMotor;
    }

    public void setAlphaTargetRotation(double alphaTarget){
        alphaMotor.getPIDController().setReference(alphaTarget, ControlType.kPosition);
    }
    public void setBetaTargetRotation(double BetaTarget){
        betaMotor.getPIDController().setReference(BetaTarget,ControlType.kPosition);
    }
    public void resetBetaEncoder(){
        betaMotor.getEncoder().setPosition(0);
    }
    public void update(ArmInputsAutoLogged inputs){
        inputs.motorAlphaPosition = encoder_Relative.getPosition();
        inputs.absAlphaEncoderPosition = encoder_Abs.getPosition();
        inputs.motorBetaPosition = betaMotor.getEncoder().getPosition();
        inputs.betaLimitSwitch = limitSwitch.get();
    }
}   

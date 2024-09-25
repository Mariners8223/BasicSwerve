// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class ArmIOReal implements ArmIO {
    CANSparkMax alphaMotor;
    CANSparkMax betaMotor;

    SparkAbsoluteEncoder absoluteEncoder;
    RelativeEncoder relativeEncoder;
    DigitalInput limitSwitch;

    

    public ArmIOReal(){
        alphaMotor = configAlphaMotor();
        betaMotor = configBetaMotor();
        limitSwitch = new DigitalInput(ArmConstants.limitSwitchPort);
    }

    public CANSparkMax configAlphaMotor(){
        CANSparkMax alphaMotor;
            alphaMotor = new CANSparkMax(ArmConstants.AlphaConstants.MOTOR_ID, MotorType.kBrushless);
            alphaMotor.setInverted(ArmConstants.AlphaConstants.IS_INVERTED);

            alphaMotor.getPIDController().setP(ArmConstants.AlphaConstants.PID.getP());
            alphaMotor.getPIDController().setI(ArmConstants.AlphaConstants.PID.getI());
            alphaMotor.getPIDController().setD(ArmConstants.AlphaConstants.PID.getD());
            alphaMotor.getPIDController().setFF(ArmConstants.AlphaConstants.PID.getF());

            absoluteEncoder = alphaMotor.getAbsoluteEncoder();
            relativeEncoder = alphaMotor.getAlternateEncoder(8192); //according to the documentation, https://docs.revrobotics.com/through-bore-encoder/specifications

            relativeEncoder.setPosition(absoluteEncoder.getPosition());

            absoluteEncoder.setPositionConversionFactor(1 / ArmConstants.AlphaConstants.alphaGearRatio); //just takes the encoder position and multiply it by the value given
            relativeEncoder.setPositionConversionFactor(1 / ArmConstants.AlphaConstants.alphaGearRatio); //just takes the encoder position and multiply it by the value given

            alphaMotor.getPIDController().setFeedbackDevice(absoluteEncoder); //this is the encoder that will be used for the PID loop

            alphaMotor.getPIDController().setOutputRange(ArmConstants.AlphaConstants.minOutputRange_alpha, ArmConstants.AlphaConstants.maxOutputRange_alpha);

            alphaMotor.setSmartCurrentLimit(ArmConstants.AlphaConstants.smartCurrentLimit_alpha);
            alphaMotor.setSecondaryCurrentLimit(ArmConstants.AlphaConstants.cecondaryCurrentLimit_alpha);

            alphaMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.AlphaConstants.alphaForwardSoftLimit);
            alphaMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.AlphaConstants.alphaReverseSoftLimit);

            return alphaMotor;
    }
        
    public CANSparkMax configBetaMotor(){
        CANSparkMax betaMotor;
            betaMotor = new CANSparkMax(ArmConstants.BetaConstants.MOTOR_ID, MotorType.kBrushless);
            betaMotor.setInverted(ArmConstants.BetaConstants.IS_INVERTED);
            

            betaMotor.getPIDController().setP(ArmConstants.BetaConstants.PID.getP());
            betaMotor.getPIDController().setI(ArmConstants.BetaConstants.PID.getI());
            betaMotor.getPIDController().setD(ArmConstants.BetaConstants.PID.getD());
            betaMotor.getPIDController().setFF(ArmConstants.BetaConstants.PID.getF());
            betaMotor.getPIDController().setOutputRange(ArmConstants.BetaConstants.minOutputRange_beta, ArmConstants.BetaConstants.maxOutputRange_beta);

            betaMotor.setSmartCurrentLimit(ArmConstants.BetaConstants.smartCurrentLimit_beta);
            betaMotor.setSecondaryCurrentLimit(ArmConstants.BetaConstants.cecondaryCurrentLimit_beta);

            betaMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.BetaConstants.betaForwardSoftLimit);
            betaMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.BetaConstants.betaReverseSoftLimit);

            return betaMotor;
    }

    public void setAlphaTargetRotation(double alphaTarget){
        alphaMotor.getPIDController().setReference(alphaTarget * ArmConstants.AlphaConstants.alphaGearRatio, ControlType.kPosition);
    }

    public void setBetaTargetRotation(double BetaTarget){
        betaMotor.getPIDController().setReference(BetaTarget * ArmConstants.BetaConstants.BetaGearRatio ,ControlType.kPosition);
    }

    public void resetBetaEncoder(){
        betaMotor.getEncoder().setPosition(0);
    }

    public void update(ArmInputsAutoLogged inputs){
        inputs.motorAlphaPosition = relativeEncoder.getPosition();
        inputs.absAlphaEncoderPosition = absoluteEncoder.getPosition();
        inputs.motorBetaPosition = betaMotor.getEncoder().getPosition();
        inputs.betaLimitSwitch = limitSwitch.get();
        inputs.alphaAppliedOutput = alphaMotor.getAppliedOutput();
        inputs.betaAppliedOutput = alphaMotor.getAppliedOutput();
    }
}   

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
import frc.robot.subsystems.Arm.ArmConstants.BetaConstants;

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
        limitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH_PORT);
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

            absoluteEncoder.setPositionConversionFactor(ArmConstants.AlphaConstants.GEAR_RATIO); //just takes the encoder position and multiply it by the value given
            relativeEncoder.setPositionConversionFactor(ArmConstants.AlphaConstants.GEAR_RATIO); //just takes the encoder position and multiply it by the value given

            alphaMotor.getEncoder().setPosition(absoluteEncoder.getPosition());

            alphaMotor.getPIDController().setFeedbackDevice(relativeEncoder); //this is the encoder that will be used for the PID loop

            alphaMotor.getPIDController().setOutputRange(ArmConstants.AlphaConstants.MIN_OUTPUT_RANGE, ArmConstants.AlphaConstants.MAX_OUTPUT_RANGE);

            alphaMotor.setSmartCurrentLimit(ArmConstants.AlphaConstants.SMART_CURRENT_LIMIT);
            alphaMotor.setSecondaryCurrentLimit(ArmConstants.AlphaConstants.SECONDARY_CURRENT_LIMIT);

            alphaMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.AlphaConstants.FORWARD_SOFT_LIMIT);
            alphaMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.AlphaConstants.REVERSE_SOFT_LIMIT);

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
            betaMotor.getPIDController().setOutputRange(ArmConstants.BetaConstants.MIN_OUTPUT_RANGE, ArmConstants.BetaConstants.MAX_OUTPUT_RANGE);

            betaMotor.getEncoder().setPositionConversionFactor(1);

            betaMotor.setSmartCurrentLimit(ArmConstants.BetaConstants.SMART_CURRENT_LIMIT);
            betaMotor.setSecondaryCurrentLimit(ArmConstants.BetaConstants.SECONDARY_CURRENT_LIMIT);

            betaMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.BetaConstants.FORWARD_SOFT_LIMIT);
            betaMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.BetaConstants.REVERSE_SOFT_LIMIT);

            return betaMotor;
    }

    public void setAlphaTargetRotation(double alphaTarget){
        alphaMotor.getPIDController().setReference(alphaTarget * ArmConstants.AlphaConstants.GEAR_RATIO, ControlType.kPosition);
    }

    public void setBetaTargetRotation(double BetaTarget){
        betaMotor.getPIDController().setReference(BetaTarget * ArmConstants.BetaConstants.GEAR_RATIO ,ControlType.kPosition);
    }

    public void resetBetaEncoder(){
        betaMotor.getEncoder().setPosition(0);
    }

    public void update(ArmInputsAutoLogged inputs){
        inputs.motorAlphaPosition = relativeEncoder.getPosition() / ArmConstants.AlphaConstants.GEAR_RATIO;
        inputs.absAlphaEncoderPosition = absoluteEncoder.getPosition() / ArmConstants.AlphaConstants.GEAR_RATIO;
        inputs.motorBetaPosition = betaMotor.getEncoder().getPosition() / ArmConstants.BetaConstants.GEAR_RATIO;
        inputs.betaLimitSwitch = !limitSwitch.get(); //TODO add qeution for inverted
        inputs.alphaAppliedOutput = alphaMotor.getAppliedOutput();
        inputs.betaAppliedOutput = alphaMotor.getAppliedOutput();
    }
}   

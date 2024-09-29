// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmConstants.ArmPosition;


public class Arm extends SubsystemBase {
    /**
     * Creates a new arm.
     */
    ArmIO io;
    ArmInputsAutoLogged inputs;
    public boolean isCalibrated;
    public ArmPosition currentPos;

    public Arm() {
        io = new ArmIOReal();
        inputs = new ArmInputsAutoLogged();
        isCalibrated = false;
        currentPos = ArmPosition.HOME_POSITION;
    }

    public void moveAlpha(double AlphaTarget) {
        io.setAlphaTargetRotation(AlphaTarget);
        inputs.wantedAlphaAlngle = AlphaTarget;
    }

    public void moveBeta(double BetaTarget) {
        io.setBetaTargetRotation(BetaTarget);
        inputs.wantedBetaAlngle = BetaTarget;
    }

    public void moveBetaDutyCycle(double speed) {
        io.moveBetaDutyCycle(speed);
    }

    public void StopAlpha() {
        io.stopAlpha();
    }

    public void StopBeta() {
        io.stopBeta();
    }

    public boolean getLimitSwitch() {
        return inputs.betaLimitSwitch;
    }

    public void resetBetaEncoder() {
        io.resetBetaEncoder();
    }

    public double getAlphaPosition() {
        return inputs.motorAlphaPosition;
    }

    public double getBetaPosition() {
        return inputs.motorBetaPosition;
    }

    @Override
    public void periodic() {
        io.update(inputs);

        if (inputs.betaLimitSwitch) {
            io.resetBetaEncoder();
        }

        double alpha = getAlphaPosition();
        double beta = getBetaPosition();

        currentPos = findArmPosition(alpha, beta);


        Logger.processInputs("Arm", inputs);
        Logger.recordOutput("Current Pos", currentPos);
    }

    private ArmPosition findArmPosition(double alpha, double beta) {
        ArmPosition[] positions = ArmPosition.values();

        for (ArmPosition armPos : positions) {
            if (Math.abs(alpha - armPos.getAlpha()) < ArmConstants.ARM_POSITION_TOLERANCE &&
                    Math.abs(beta - armPos.getBeta()) < ArmConstants.ARM_POSITION_TOLERANCE) return armPos;
        }

        return ArmPosition.UNKNOWN;
    }
}

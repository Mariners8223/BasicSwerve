// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmConstants.ArmPosition;


public class Arm extends SubsystemBase {
    /**
     * Creates a new arm.
     */
    private final ArmIO io;
    private final ArmInputsAutoLogged inputs;
    private boolean isCalibrated;
    private ArmPosition currentPos;

    public Arm() {
        io = new ArmIOReal();
        inputs = new ArmInputsAutoLogged();
        isCalibrated = false;
        currentPos = ArmPosition.HOME_POSITION;
    }

    public void moveAlpha(double alphaTarget) {

        double target = Math.max(ArmConstants.AlphaConstants.REVERSE_SOFT_LIMIT,
                Math.min(alphaTarget, ArmConstants.AlphaConstants.FORWARD_SOFT_LIMIT));

        io.setAlphaTargetRotation(target);

        Logger.recordOutput("Arm/Alpha Target", target);
    }

    public void moveBeta(double betaTarget) {

        double target = Math.max(ArmConstants.BetaConstants.REVERSE_SOFT_LIMIT,
                Math.min(betaTarget, ArmConstants.BetaConstants.FORWARD_SOFT_LIMIT));

        io.setBetaTargetRotation(target);

        Logger.recordOutput("Arm/Beta Target", target);
    }

    public void moveBetaDutyCycle(double speed) {
        speed = MathUtil.clamp(speed,
                ArmConstants.BetaConstants.MIN_OUTPUT_RANGE, ArmConstants.BetaConstants.MAX_OUTPUT_RANGE);

        io.moveBetaDutyCycle(speed);
    }

    public void stopAlpha() {
        io.stopAlpha();
    }

    public void stopBeta() {
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

    public boolean isCalibrated() {
        return isCalibrated;
    }

    public void setArmCalibrated() {
        isCalibrated = true;
    }

    public ArmPosition getCurrentPos() {
        return currentPos;
    }

    public void setArmPositionUnknown() {
        currentPos = ArmPosition.UNKNOWN;
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

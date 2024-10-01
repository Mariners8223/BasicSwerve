package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public class ArmInputs{
        public double motorAlphaPosition = 0;
        public double absAlphaEncoderPosition = 0;
        public double relativeEncoderPosition = 0;

        public double motorBetaPosition = 0;
        public boolean betaLimitSwitch = false;
        public double alphaAppliedOutput = 0;
        public double betaAppliedOutput = 0;

        public double alphaAppliedCurrent = 0;
    }

    public void setAlphaTargetRotation(double AlphaTarget);
    public void setBetaTargetRotation(double BetaTarget);
    public void resetBetaEncoder();
    public void moveBetaDutyCycle(double speed);
    public void enableBetaSoftLimits();
    public void stopAlpha();
    public void stopBeta();

    public void update(ArmInputsAutoLogged inputs);
}

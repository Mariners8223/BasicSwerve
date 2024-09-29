package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public class ArmInputs{
        public double motorAlphaPosition;
        public double absAlphaEncoderPosition;

        public double motorBetaPosition;
        public boolean betaLimitSwitch;
        public double alphaAppliedOutput;
        public double betaAppliedOutput;

        public double wantedAlphaAlngle;
        public double wantedBetaAlngle;
    }

    public void setAlphaTargetRotation(double AlphaTarget);
    public void setBetaTargetRotation(double BetaTarget);
    public void resetBetaEncoder();
    public void moveBetaDutyCycle(double speed);
    public void stopAlpha();
    public void stopBeta();

    public void update(ArmInputsAutoLogged inputs);
}

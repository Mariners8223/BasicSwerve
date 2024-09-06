package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public class ArmInputs{
        public double motorAlphaPosition;
        public double absAlphaEncoderPosition;

        public double motorBetaPosition;
        public boolean betaLimitSwitch;


    }

    public void setAlphaTargetRotation(double AlphaTarget);
    public void setBetaTargetRotation(double BetaTarget);
    public void resetBetaEncoder();
    public double GetCurrentAlphaPos();
    public double GetCurrentBetaPos();

    public void update(ArmInputsAutoLogged inputs);
}

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public class ArmInputs{
        public double motorAlphaPosition;
        public double absAlphaEncoderPosition;

        public double motorBetaPosition;
        public boolean betaLimitSwitch;


    }

    public void setAlphaTargetRotation();
    public void setBetaTargetRotation();
    public void resetBetaEncoder();

    public void update(ArmInputsAutoLogged inputs);
}

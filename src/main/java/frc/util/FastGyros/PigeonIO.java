package frc.util.FastGyros;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class PigeonIO implements GyroIO{
    Pigeon2 pigeon;
    PigeonIOInputsAutoLogged inputs = new PigeonIOInputsAutoLogged();

    @AutoLog
    public static class PigeonIOInputs{
        public Rotation2d angle = new Rotation2d();
        public Rotation2d rotationOffset = new Rotation2d();
        public double yaw = 0;
        public double pitch = 0;
        public double roll = 0;
    }

    public PigeonIO(int id){
        pigeon = new Pigeon2(id);
    }


    @Override
    public double getAngleDegrees() {
        return inputs.yaw;
    }

    @Override
    public Rotation2d getRotation2d() {
        return inputs.angle;
    }

    @Override
    public void update() {
        inputs.yaw = pigeon.getAngle();
        inputs.pitch = pigeon.getPitch().getValueAsDouble();
        inputs.roll = pigeon.getRoll().getValueAsDouble();

        inputs.angle = Rotation2d.fromDegrees((-inputs.yaw) - inputs.rotationOffset.getDegrees());

        Logger.processInputs("Pigeon2", inputs);
    }

    @Override
    public double getYaw() {
        return inputs.yaw;
    }

    @Override
    public double getPitch() {
        return inputs.pitch;
    }

    @Override
    public double getRoll() {
        return inputs.roll;
    }

    @Override
    public double getVelocityX() {
        return 0;
    }

    @Override
    public double getVelocityY() {
        return 0;
    }

    @Override
    public double getAccelerationX() {
        return 0;
    }

    @Override
    public double getAccelerationY() {
        return 0;
    }

    @Override
    public void reset(Pose2d var1) {
        pigeon.reset();

        inputs.angle = var1.getRotation();
        inputs.rotationOffset = var1.getRotation().unaryMinus();
    }

    @Override
    public void initSendable(SendableBuilder var1) {
        pigeon.initSendable(var1);
    }
}

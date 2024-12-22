package frc.util.FastGyros;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;

public class NavxIO extends GyroIO{
    private final AHRS navx;

    public NavxIO() {
        navx = new AHRS();
    }

    @Override
    public void update() {
        inputs.yaw = navx.getAngle();
        inputs.pitch = navx.getPitch();
        inputs.roll = navx.getRoll();
        inputs.yawRate = navx.getRate();
        inputs.accelerationX = navx.getWorldLinearAccelX();
        inputs.accelerationY = navx.getWorldLinearAccelY();

        inputs.angle = Rotation2d.fromDegrees(-inputs.yaw);
    }

    @Override
    public Rotation2d getRotation2d() {
        return inputs.angle;
    }

    @Override
    public double getYaw() {
        return inputs.yaw;
    }

    @Override
    public double getYawRate() {
        return inputs.yawRate;
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
    public double getAccelerationX() {
        return inputs.accelerationX;
    }

    @Override
    public double getAccelerationY() {
        return inputs.accelerationY;
    }

    @Override
    public void reset(Rotation2d newAngle) {
        navx.reset();
        navx.setAngleAdjustment(newAngle.getDegrees());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        navx.initSendable(builder);
    }
}

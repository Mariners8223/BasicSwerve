package frc.util.FastGyros;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.function.Supplier;

public class SimGyroIO extends GyroIO{

    private final Supplier<Twist2d> twistSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private double prevVelocityX = 0;
    private double prevVelocityY = 0;

    public SimGyroIO(Supplier<Twist2d> twistSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.twistSupplier = twistSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    }

    @Override
    public void update() {
        Twist2d twist = twistSupplier.get();
        ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();

        inputs.angle = Rotation2d.fromRadians(twist.dtheta + inputs.angle.getRadians());
        inputs.yaw = -inputs.angle.getDegrees();

        inputs.accelerationX = (chassisSpeeds.vxMetersPerSecond - prevVelocityX) / 0.02;
        inputs.accelerationY = (chassisSpeeds.vyMetersPerSecond - prevVelocityY) / 0.02;

        prevVelocityX = chassisSpeeds.vxMetersPerSecond;
        prevVelocityY = chassisSpeeds.vyMetersPerSecond;

        inputs.yawRate = twist.dtheta / 0.02;
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
        return 0;
    }

    @Override
    public double getRoll() {
        return 0;
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
        inputs.angle = newAngle;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("angle", this::getYaw, null);
    }
}

package frc.util.FastGyros;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;


public final class SimGyroIO implements GyroIO {
    private final Supplier<Twist2d> twistSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    private Rotation2d angle;
    private double velocityX;
    private double velocityY;
    private double displacementX;
    private double displacementY;
    private double accelerationX;
    private double accelerationY;
    private double prevVelocityX;
    private double prevVelocityY;

    public SimGyroIO(Supplier<Twist2d> twistSupplier,Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.twistSupplier = twistSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
        this.angle = new Rotation2d();
    }

    public double getAngleDegrees() {
        double var1;
            var1 = this.angle.getDegrees();

        return var1;
    }


    public Rotation2d getRotation2d() {
        Rotation2d var1;
            var1 = this.angle;

        return var1;
    }

    public void update() {
            Object var2 = this.twistSupplier.get();
            Twist2d twist = (Twist2d)var2;
            Rotation2d var6 = Rotation2d.fromRadians(twist.dtheta + this.angle.getRadians());
            this.angle = var6;
            Rotation2d[] var7 = new Rotation2d[]{this.angle};
            Logger.recordOutput("Gyro/angle", (StructSerializable[])var7);
            this.displacementX += twist.dx * this.angle.getCos() - twist.dy * this.angle.getSin();
            this.displacementY += twist.dy * this.angle.getCos() + twist.dx * this.angle.getSin();
            Pose2d[] var8 = new Pose2d[]{new Pose2d(this.displacementX, this.displacementY, this.angle)};
            Logger.recordOutput("Gyro/EstimatedPose", (StructSerializable[])var8);
            Object var3 = this.chassisSpeedsSupplier.get();
            ChassisSpeeds chassisSpeeds = (ChassisSpeeds)var3;
            this.velocityX = chassisSpeeds.vxMetersPerSecond;
            this.velocityY = chassisSpeeds.vyMetersPerSecond;
            this.accelerationX = (this.velocityX - this.prevVelocityX) / 0.01;
            this.accelerationY = (this.velocityY - this.prevVelocityY) / 0.01;
            Logger.recordOutput("Gyro/PrevVelocityX", this.prevVelocityX);
            Logger.recordOutput("Gyro/PrevVelocityY", this.prevVelocityY);
            this.prevVelocityX = this.velocityX;
            this.prevVelocityY = this.velocityY;
            Logger.recordOutput("Gyro/velocityX", this.velocityX);
            Logger.recordOutput("Gyro/velocityY", this.velocityY);
            Logger.recordOutput("Gyro/accelerationX", this.accelerationX);
            Logger.recordOutput("Gyro/accelerationY", this.accelerationY);

    }

    public double getYaw() {
        double var1;
            var1 = -this.angle.getDegrees();

        return var1;
    }

    public double getPitch() {
        return 0.0;
    }

    public double getRoll() {
        return 0.0;
    }

    public double getVelocityX() {
        double var1;
            var1 = this.velocityX;

        return var1;
    }

    public double getVelocityY() {
        double var1;
            var1 = this.velocityY;
        return var1;
    }

    public double getAccelerationX() {
        double var1;
            var1 = this.accelerationX;

        return var1;
    }

    public double getAccelerationY() {
        double var1;
            var1 = this.accelerationY;

        return var1;
    }

    public void reset( Pose2d newPose) {
            Rotation2d var2 = newPose.getRotation();
            this.angle = var2;
            this.displacementX = newPose.getTranslation().getX();
            this.displacementY = newPose.getTranslation().getY();

    }

    public void initSendable(SendableBuilder builder) {
        if (builder != null) {
            builder.addDoubleProperty("angle", this::getAngleDegrees, null);
        }

    }
}

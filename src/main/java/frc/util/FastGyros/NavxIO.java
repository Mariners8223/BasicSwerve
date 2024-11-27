// NavxIO.java
package frc.util.FastGyros;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;


public final class NavxIO implements GyroIO {
    @AutoLog
    public static class NavxInputs{
        public Rotation2d angle = new Rotation2d();
        public Rotation2d rotationOffset = new Rotation2d();
        public double velocityX;
        public double velocityY;
        public double accelerationX;
        public double accelerationY;
        public Pose2d estimatedPose = new Pose2d();
        public double prevVelocityX;
        public double prevVelocityY;
        public long prevTimeStamp;
        public double yaw;
        public double pitch;
        public double roll;
    }


    private final boolean isInverted;
    private final AHRS navx;
    private final NavxInputsAutoLogged inputs;

    public NavxIO(boolean isInverted) {
        this.isInverted = isInverted;
        this.navx = new AHRS();
        this.inputs = new NavxInputsAutoLogged();
   }

    public void reset(Pose2d newPose) {

        this.navx.reset();
        this.inputs.angle = newPose.getRotation();
        this.inputs.rotationOffset = newPose.getRotation().unaryMinus();
        this.inputs.estimatedPose = newPose;

    }

    public double getAngleDegrees() {
        double var1;
        var1 = this.inputs.angle.getDegrees();

        return var1;
    }

    public Rotation2d getRotation2d() {
        Rotation2d var1;
            var1 = this.inputs.angle;

        return var1;
    }

    public void update() {
            // inputs.angle = if(isInverted) Rotation2d.fromDegrees(navx.angle - inputs.rotationOffset.degrees) else Rotation2d.fromDegrees(-navx.angle - inputs.rotationOffset.degrees);
            inputs.angle = isInverted ? Rotation2d.fromDegrees(navx.getAngle() - inputs.rotationOffset.getDegrees()) : Rotation2d.fromDegrees(-navx.getAngle() - inputs.rotationOffset.getDegrees());
      
            inputs.yaw = navx.getYaw();
            inputs.pitch = navx.getPitch();
            inputs.roll = navx.getRoll();
      
            inputs.velocityX = inputs.angle.getCos() * navx.getVelocityX() - inputs.angle.getSin() * navx.getVelocityY();
            inputs.velocityY = inputs.angle.getSin() * navx.getVelocityX() - inputs.angle.getCos() * navx.getVelocityY();
      
            inputs.accelerationX = (inputs.angle.getCos() * navx.getWorldLinearAccelX() - inputs.angle.getSin() * navx.getWorldLinearAccelY()) * 9.18;
            inputs.accelerationY = (inputs.angle.getSin() * navx.getWorldLinearAccelX() - inputs.angle.getCos() * navx.getWorldLinearAccelY()) * 9.18;
      
            inputs.estimatedPose = new Pose2d((inputs.accelerationX + inputs.prevVelocityX) * (navx.getLastSensorTimestamp() - inputs.prevTimeStamp) / 2 + inputs.estimatedPose.getX(),
              (inputs.accelerationY + inputs.prevVelocityY) * (navx.getLastSensorTimestamp() - inputs.prevTimeStamp) / 2 + inputs.estimatedPose.getY(),    
              inputs.angle);
      
            inputs.prevVelocityX = inputs.velocityX;
            inputs.prevVelocityY = inputs.velocityY;
            inputs.prevTimeStamp = navx.getLastSensorTimestamp();

        Logger.processInputs("FastNavx", (LoggableInputs)this.inputs);
    }

    public double getYaw() {
        double var1;
            var1 = this.inputs.yaw;

        return var1;
    }

    public double getPitch() {
        double var1;
            var1 = this.inputs.pitch;

        return var1;
    }

    public double getRoll() {
        double var1;
            var1 = this.inputs.roll;

        return var1;
    }

    public double getVelocityX() {
        double var1;
            var1 = this.inputs.velocityX;

        return var1;
    }

    public double getVelocityY() {
        double var1;
            var1 = this.inputs.velocityY;

        return var1;
    }

    public double getAccelerationX() {
        double var1;
            var1 = this.inputs.accelerationX;

        return var1;
    }

    public double getAccelerationY() {
        double var1;
            var1 = this.inputs.accelerationY;

        return var1;
    }

    public void initSendable(SendableBuilder builder) {
        this.navx.initSendable(builder);
    }
}
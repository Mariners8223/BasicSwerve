package frc.util.FastGyros;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import org.littletonrobotics.junction.AutoLog;

public abstract class GyroIO implements Sendable {

    protected final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    @AutoLog
    static class GyroIOInputs {
        public Rotation2d angle = new Rotation2d();
        public double yaw = 0;
        public double pitch = 0;
        public double roll = 0;
        public double yawRate = 0;
        public double accelerationX = 0;
        public double accelerationY = 0;
    }

    /**
     * Updates the gyro's data
     * This should be called periodically
     */
    public abstract void update();

    /**
     * gets the rotation 2d of the gyro (around the z axis)
     * @return the rotation 2d of the gyro (left is positive)
     */
    public abstract Rotation2d getRotation2d();

    /**
     * gets the angle of the gyro in degrees
     * @return the angle of the gyro in degrees (right is positive)
     */
    public abstract double getYaw();

    /**
     * gets the rate of change of the angle of the gyro in degrees per second
     * @return the rate of change of the angle of the gyro in degrees per second (right is positive)
     */
    public abstract double getYawRate();

    /**
     * gets the pitch of the gyro in degrees
     * @return the pitch of the gyro in degrees (up is positive)
     */
    public abstract double getPitch();

    /**
     * gets the roll of the gyro in degrees
     * @return the roll of the gyro in degrees (right is positive)
     */
    public abstract double getRoll();

    /**
     * gets the acceleration of the gyro in the x direction in m/s^2
     * the value is gyro relative and not field relative
     * be careful with this value, it is not always accurate (depends on the gyro's location)
     * @return the acceleration of the gyro in the x direction in m/s^2 (forward is positive)
     */
    public abstract double getAccelerationX();

    /**
     * gets the acceleration of the gyro in the y direction in m/s^2
     * the value is gyro relative and not field relative
     * be careful with this value, it is not always accurate (depends on the gyro's location)
     * @return the acceleration of the gyro in the y direction in m/s^2 (left is positive)
     */
    public abstract double getAccelerationY();

    /**
     * resets the gyro's angle to the specified angle
     * @param newAngle the angle to reset the gyro to
     */
    public abstract void reset(Rotation2d newAngle);
}

@file:JvmName("FastNavxss") 
package frc.util.FastGyros

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import kotlin.math.cos
import kotlin.math.sin

class FastNavx : FastGyro{

  private val navx: AHRS = AHRS()

  private val inputs: NavxInputs = NavxInputs()


  /**
   * Resets the gyro to the specified pose.
   * @param newPose The new pose to set the gyro to.
   */
  override fun reset(newPose: Pose2d): Unit {
    navx.reset()
    inputs.angle = Rotation2d()
    inputs.rotationOffset = newPose.rotation.unaryMinus()
    inputs.estimatedPose = newPose
  }

  /**
   * returns the angle of the gyro in degrees. (in the Rotation2d range)
   */
  override fun getAngleDegrees(): Double {
    return inputs.angle.degrees
  }

  /**
   * returns the angle of the gyro in Rotation2d.
   */
  override fun getRotation2d(): Rotation2d {
    return inputs.angle
  }

  /**
   * Updates the gyro's inputs.
   * needs to be called periodically
   */
  override fun update(): Unit {
    inputs.angle = navx.rotation2d.minus(inputs.rotationOffset)

    inputs.yaw = navx.yaw.toDouble()
    inputs.pitch = navx.pitch.toDouble()
    inputs.roll = navx.roll.toDouble()

    inputs.velocityX = inputs.angle.cos * navx.velocityX - inputs.angle.sin * navx.velocityY
    inputs.velocityY = inputs.angle.sin * navx.velocityX - inputs.angle.cos * navx.velocityY

    inputs.accelerationX = (inputs.angle.cos * navx.worldLinearAccelX - inputs.angle.sin * navx.worldLinearAccelY) * 9.18
    inputs.accelerationY = (inputs.angle.sin * navx.worldLinearAccelX - inputs.angle.cos * navx.worldLinearAccelY) * 9.18

    inputs.estimatedPose = Pose2d((inputs.accelerationX + inputs.prevVelocityX) * (navx.lastSensorTimestamp - inputs.prevTimeStamp) / 2 + inputs.estimatedPose.x,
      (inputs.accelerationY + inputs.prevVelocityY) * (navx.lastSensorTimestamp - inputs.prevTimeStamp) / 2 + inputs.estimatedPose.y,
      inputs.angle)

    inputs.prevVelocityX = inputs.velocityX
    inputs.prevVelocityY = inputs.velocityY
    inputs.prevTimeStamp = navx.lastSensorTimestamp

    Logger.processInputs("FastNavx", inputs)
  }

  /**
   * gets the yaw of the gyro. (in degrees) not in Rotation2d space
   */
  override fun getYaw(): Double {
    return inputs.yaw
  }

  /**
   * gets the pitch of the gyro. (in degrees) not in Rotation2d space
   */
  override fun getPitch(): Double {
    return inputs.pitch
  }

  /**
   * gets the roll of the gyro. (in degrees) not in Rotation2d space
   */
  override fun getRoll(): Double {
    return inputs.roll
  }

  /**
   * gets the x velocity of the gyro. (in m/s)
   */
  override fun getVelocityX(): Double {
    return inputs.velocityX
  }

  /**
   * gets the y velocity of the gyro. (in m/s)
   */
  override fun getVelocityY(): Double {
    return inputs.velocityY
  }

  /**
   * gets the x acceleration of the gyro. (in m/s^2)
   */
  override fun getAccelerationX(): Double {
    return inputs.accelerationX
  }

  /**
   * gets the y acceleration of the gyro. (in m/s^2)
   */
  override fun getAccelerationY(): Double {
    return inputs.accelerationY
  }

  /**
   * Initializes the sendable.
   */
  override fun initSendable(builder: SendableBuilder?) {
    return navx.initSendable(builder)
  }
}


class NavxInputs : LoggableInputs, Cloneable{
  var angle: Rotation2d = Rotation2d()
  var rotationOffset: Rotation2d = Rotation2d()

  var velocityX: Double = 0.0
  var velocityY: Double = 0.0
  var accelerationX: Double = 0.0
  var accelerationY: Double = 0.0
  var estimatedPose: Pose2d = Pose2d()

  var prevVelocityX: Double = 0.0
  var prevVelocityY: Double = 0.0
  var prevTimeStamp: Long = 0L

  var yaw: Double = 0.0
  var pitch: Double = 0.0
  var roll: Double = 0.0

  override fun toLog(table: LogTable) {
    table.put("angle", angle)
    table.put("rotationOffset", rotationOffset)
    table.put("velocityX", velocityX)
    table.put("velocityY", velocityY)
    table.put("accelerationX", accelerationX)
    table.put("accelerationY", accelerationY)
    table.put("estimatedPose", estimatedPose)

    table.put("prevVelocityX", prevVelocityX)
    table.put("prevVelocityY", prevVelocityY)
    table.put("prevTimeStamp", prevTimeStamp)

    table.put("yaw", yaw)
    table.put("pitch", pitch)
    table.put("roll", roll)
  }

  override fun fromLog(table: LogTable) {
    angle = table.get("angle", angle)[0]
    rotationOffset = table.get("rotationOffset", rotationOffset)[0]
    velocityX = table.get("velocityX", velocityX)
    velocityY = table.get("velocityY", velocityY)
    accelerationX = table.get("accelerationX", accelerationX)
    accelerationY = table.get("accelerationY", accelerationY)
    estimatedPose = table.get("estimatedPose", estimatedPose)[0]

    prevVelocityX = table.get("prevVelocityX", prevVelocityX)
    prevVelocityY = table.get("prevVelocityY", prevVelocityY)
    prevTimeStamp = table.get("prevTimeStamp", prevTimeStamp)

    yaw = table.get("yaw", yaw)
    pitch = table.get("pitch", pitch)
    roll = table.get("roll", roll)
  }

  override fun clone(): NavxInputs {
    val clone = NavxInputs()
    clone.angle = angle
    clone.rotationOffset = rotationOffset
    clone.velocityX = velocityX
    clone.velocityY = velocityY
    clone.accelerationX = accelerationX
    clone.accelerationY = accelerationY
    clone.estimatedPose = estimatedPose

    clone.prevVelocityX = prevVelocityX
    clone.prevVelocityY = prevVelocityY
    clone.prevTimeStamp = prevTimeStamp

    clone.yaw = yaw
    clone.pitch = pitch
    clone.roll = roll

    return clone
  }
}
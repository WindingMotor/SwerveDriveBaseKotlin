// Winding Motor Libary (wmlib) - Created by Isaac S for team 2106

package frc.robot.wmlib
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.math.geometry.Rotation2d
import java.lang.Math

class AbsoluteEncoder(deviceId: Int, val offset: Double){

    // Create encoder on RoboRio DIO port with correct duty cycle
    val encoder: DutyCycleEncoder = DutyCycleEncoder(deviceId).apply{ setDutyCycleRange(1.0/4096.0, 4095.0/4096.0) }

    // @return (Double) The angle in radians with offset applied
    fun getRadians(): Double = (( 1.0 - encoder.getAbsolutePosition()) * (2.0 * Math.PI)) - offset

    // @return (Double) The angle with in degrees with offset
    fun getDegrees(): Double = Math.toDegrees(getRadians())

    // @return (Rotation2d) The angle in Rotaton2d with offset
    fun getRotation2d(): Rotation2d = Rotation2d(getRadians())

}


import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.math.geometry.Rotation2d
import java.lang.Math

class AbsoluteEncoder(deviceId: Int, val offset: Double){

    val encoder: DutyCycleEncoder = DutyCycleEncoder(deviceId).apply{ setDutyCycleRange(1.0/4096.0, 4095.0/4096.0) }

    fun getRadians(): Double = (( 1.0 - encoder.get()) * (2.0 * Math.PI)) - offset

    fun getDegrees(): Double = Math.toDegrees(getRadians())

    fun getRotation2d(): Rotation2d = Rotation2d(getRadians())

}

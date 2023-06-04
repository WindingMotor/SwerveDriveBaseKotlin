package frc.robot.swerve
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.RelativeEncoder
import Motor
import AbsoluteEncoder

// Class Parameters
class SwerveModule(
    driveID: Int,
    turnID: Int,
    driveInverted: Boolean,
    turnInverted: Boolean,
    absoluteEncoderID: Int,
    private val absoluteEncoderOffsetRadians: Double,
    private val moduleName: String
){

    private val driveMotor: Motor = Motor(driveID,driveInverted,false)
    private val turnMotor: Motor = Motor(turnID,turnInverted,false)
    private val absoluteEncoder: AbsoluteEncoder = AbsoluteEncoder(absoluteEncoderID, absoluteEncoderOffsetRadians)

    
}

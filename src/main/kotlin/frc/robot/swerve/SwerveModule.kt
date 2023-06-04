package frc.robot.swerve
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.RelativeEncoder
import Motor
import AbsoluteEncoder
import frc.robot.Constants

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

    private val driveMotor: Motor = Motor(driveID, driveInverted, false, Constants.MK4SDS.DRIVE_ROT_2_METER, Constants.MK4SDS.DRIVE_RPM_2_MPS)
    private val turnMotor: Motor = Motor(turnID, turnInverted, false, Constants.MK4SDS.TURN_ROT_2_RAD, Constants.MK4SDS.TURN_RPM_2_RADPS)
    private val absoluteEncoder: AbsoluteEncoder = AbsoluteEncoder(absoluteEncoderID, absoluteEncoderOffsetRadians)


}

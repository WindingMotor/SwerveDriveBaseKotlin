
package frc.robot.wmlib
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import frc.robot.Constants

class SwerveSubsystem : SubsystemBase() {

    private val frontLeft = SwerveModule(
        Constants.SwerveConstants.FRONT_LEFT_DRIVE_ID, Constants.SwerveConstants.FRONT_LEFT_TURN_ID,
        Constants.SwerveConstants.FRONT_LEFT_DRIVE_INVERTED, Constants.SwerveConstants.FRONT_LEFT_TURN_INVERTED,
        Constants.SwerveConstants.FRONT_LEFT_ABE_ID, Constants.SwerveConstants.FRONT_LEFT_ABE_OFFSET_RAD, "Front Left");
    
    private val frontRight = SwerveModule(
        Constants.SwerveConstants.FRONT_RIGHT_DRIVE_ID, Constants.SwerveConstants.FRONT_RIGHT_TURN_ID,
        Constants.SwerveConstants.FRONT_RIGHT_DRIVE_INVERTED, Constants.SwerveConstants.FRONT_RIGHT_TURN_INVERTED,
        Constants.SwerveConstants.FRONT_RIGHT_ABE_ID, Constants.SwerveConstants.FRONT_RIGHT_ABE_OFFSET_RAD, "Front Right");
    
    private val backLeft = SwerveModule(
        Constants.SwerveConstants.BACK_LEFT_DRIVE_ID, Constants.SwerveConstants.BACK_LEFT_TURN_ID,
        Constants.SwerveConstants.BACK_LEFT_DRIVE_INVERTED, Constants.SwerveConstants.BACK_LEFT_TURN_INVERTED,
        Constants.SwerveConstants.BACK_LEFT_ABE_ID, Constants.SwerveConstants.BACK_LEFT_ABE_OFFSET_RAD, "Back Left");
    
    private val backRight = SwerveModule(
        Constants.SwerveConstants.BACK_RIGHT_DRIVE_ID, Constants.SwerveConstants.BACK_RIGHT_TURN_ID,
        Constants.SwerveConstants.BACK_RIGHT_DRIVE_INVERTED, Constants.SwerveConstants.BACK_RIGHT_TURN_INVERTED,
        Constants.SwerveConstants.BACK_RIGHT_ABE_ID, Constants.SwerveConstants.BACK_RIGHT_ABE_OFFSET_RAD, "Back Right");

    fun stopAll(){ frontLeft.stopMotors(); frontRight.stopMotors(); backLeft.stopMotors(); backRight.stopMotors(); }

    fun resetAllEncoders(){ frontLeft.resetEncoders(); frontRight.resetEncoders(); backLeft.resetEncoders(); backRight.resetEncoders(); }

    fun setAllAngles(angle: Rotation2d){ frontLeft.setAngle(angle); frontRight.setAngle(angle); backLeft.setAngle(angle); backRight.setAngle(angle); }

    fun getAllFaults(): Array<Array<Short>> = arrayOf(frontLeft.getMotorFaults(), frontRight.getMotorFaults(), backLeft.getMotorFaults(), backRight.getMotorFaults())

    fun getAllPositions(): Array<SwerveModulePosition> = arrayOf(frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition())

    fun setStates(states: Array<SwerveModuleState>) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MK4SDS.THEORETICAL_MAX_SPEED)
        frontLeft.setState(states[0]); frontRight.setState(states[1]);
        backLeft.setState(states[2]); backRight.setState(states[3]);
    }

    override fun periodic(){

    }


}
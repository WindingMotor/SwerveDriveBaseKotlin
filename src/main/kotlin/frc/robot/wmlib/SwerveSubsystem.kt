
package frc.robot.wmlib
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants

class SwerveSubsystem : SubsystemBase() {

    // Create a navX gyro from wmlib
    val gyro = NavX()

    // Create robot odometry
    val odometry = SwerveDriveOdometry(Constants.SwerveConstants.DRIVE_KINEMATICS,
    gyro.getRotation2d(), // Should be wrapped from -180 to 180 or 0 to 360 -> wmlib NavX class does this
    getAllPositions())
    
    // Create 4 swerve modules with their respective constants
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
    // ------------ //

    // Stop every module's motors
    fun stopAll(){ frontLeft.stopMotors(); frontRight.stopMotors(); backLeft.stopMotors(); backRight.stopMotors(); }

    // Reset every module's encoders
    fun resetAllEncoders(){ frontLeft.resetEncoders(); frontRight.resetEncoders(); backLeft.resetEncoders(); backRight.resetEncoders(); }

    // @param (angle) (Rotation2d) Set every swerve module to a set angle
    fun setAllAngles(angle: Rotation2d){ frontLeft.setAngle(angle); frontRight.setAngle(angle); backLeft.setAngle(angle); backRight.setAngle(angle); }

    // @return (Short:ARRAY:ARRAY) A 2d array of the motor faults of each module 
    fun getAllFaults(): Array<Array<Short>> = arrayOf(frontLeft.getMotorFaults(), frontRight.getMotorFaults(), backLeft.getMotorFaults(), backRight.getMotorFaults())

    // @return (SwerveModulePositon:ARRAY) The current position of each swerve module
    fun getAllPositions(): Array<SwerveModulePosition> = arrayOf(frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition())

    // @param (states) (SwerveModuleState:ARRAY) The new state to set each swerve module to
    fun setStates(states: Array<SwerveModuleState>) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MK4SDS.THEORETICAL_MAX_SPEED)
        frontLeft.setState(states[0]); frontRight.setState(states[1]);
        backLeft.setState(states[2]); backRight.setState(states[3]);
    }

    // @param (pose) (Pose2d) - The new pose to assign
    fun resetOdometry(pose: Pose2d){odometry.resetPosition(gyro.getRotation2d(), getAllPositions(), pose)}
    
    /* 
    @param (pose) (Pose2d) The new pose to assign
    @param (angle) (Rotation2d) The new angle to assign
    */
    fun resetOdometry(pose: Pose2d, angle: Rotation2d){odometry.resetPosition(angle, getAllPositions(), pose)}

    // @return (Pose2d) The current robot odometry pose
    fun getPoseMeters(): Pose2d = odometry.poseMeters

    // Updates the robot odometry
    override fun periodic(){ odometry.update(gyro.getRotation2d(), getAllPositions()) }

    // Print SmartDashboard data for this class
    private fun updateSmartDashboard(){ SmartDashboard.putString("Odometry Pose2d", getPoseMeters().toString()) }

    // Print all SmartDashboard data for all nested classes
    fun updateAllSmartDashboard(){
        this.updateSmartDashboard(); gyro.updateSmartDashboard();
        frontLeft.updateSmartDashboard(); frontRight.updateSmartDashboard();
        backLeft.updateSmartDashboard(); backRight.updateSmartDashboard();
    }

     // Print all SmartDashboard debug data for all nested classes
    fun updateAllSmartDashboardDebug(){
        this.updateSmartDashboard(); gyro.updateSmartDashboardDebug();
        frontLeft.updateSmartDashboardDebug(); frontRight.updateSmartDashboardDebug();
        backLeft.updateSmartDashboardDebug(); backRight.updateSmartDashboardDebug();
    }

}
// Winding Motor Libary (wmlib) - Created by Isaac S for team 2106

package frc.robot.wmlib
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.Constants

class Odometer(val swerveSubsystem: SwerveSubsystem){

    val gyro = NavX()

    val odometry = SwerveDriveOdometry(Constants.SwerveConstants.DRIVE_KINEMATICS,
    gyro.getRotation2d(), // Should be wrapped from -180 to 180 or 0 to 360 -> wmlib NavX class does this
    swerveSubsystem.getAllPositions()
    )

    fun resetOdometry(pose: Pose2d){odometry.resetPosition(gyro.getRotation2d(), swerveSubsystem.getAllPositions(), pose)}

    fun resetOdometry(pose: Pose2d, angle: Rotation2d){odometry.resetPosition(angle, swerveSubsystem.getAllPositions(), pose)}

    fun getPoseMeters(): Pose2d = odometry.poseMeters
    

}

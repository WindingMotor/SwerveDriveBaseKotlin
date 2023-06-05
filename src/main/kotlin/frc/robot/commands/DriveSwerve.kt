// Winding Motor Libary (wmlib) Base Command - Created by Isaac S for team 2106

package frc.robot.commands
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.robot.wmlib.SwerveSubsystem
import frc.robot.Constants

// Class parameters
class DriveSwerve(
    private val subsystem: SwerveSubsystem,
    private val xInput: () -> Double,
    private val yInput: () -> Double,
    private val rInput: () -> Double,
    private val isFieldOriented: () -> Boolean,
):CommandBase(){

    private val rPID = PIDController(0.1,0.0,0.0)

    init{
        addRequirements(subsystem)
        rPID.enableContinuousInput(0.0, 360.0)
    }

    override fun initialize(){}

    override fun execute(){

        val rCorrection = rPID.calculate(subsystem.gyro.getDegrees())

        val chassisSpeeds = if(isFieldOriented()){
            ChassisSpeeds.fromFieldRelativeSpeeds(xInput(), yInput(), rInput() + rCorrection, subsystem.gyro.getRotation2d())}
            else{ChassisSpeeds(xInput(), yInput(), rInput() + rCorrection)}

        subsystem.setStates(Constants.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds))

    }

    override fun end(interrupted: Boolean){ subsystem.stopAll() }

    override fun isFinished(): Boolean{
        return false
    }

}

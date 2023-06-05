// Winding Motor Libary (wmlib) - Created by Isaac S for team 2106

package frc.robot.wmlib
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.RelativeEncoder
import frc.robot.wmlib.SparkMax
import frc.robot.wmlib.AbsoluteEncoder
import frc.robot.Constants
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.DriverStation

// Class parameters
class SwerveModule(
    driveID: Int,
    turnID: Int,
    driveInverted: Boolean,
    turnInverted: Boolean,
    absoluteEncoderID: Int,
    private val absoluteEncoderOffsetRadians: Double,
    private val moduleName: String
){
    // Create both motors using class from wmlib
    private val driveMotor: SparkMax = SparkMax(driveID, driveInverted, Constants.MK4SDS.DRIVE_ROT_2_METER, Constants.MK4SDS.DRIVE_RPM_2_MPS)
    private val turnMotor: SparkMax = SparkMax(turnID, turnInverted, Constants.MK4SDS.TURN_ROT_2_RAD, Constants.MK4SDS.TURN_RPM_2_RADPS)

    // Create absolute encoder using class from wmlib
    private val absoluteEncoder: AbsoluteEncoder = AbsoluteEncoder(absoluteEncoderID, absoluteEncoderOffsetRadians)

    // Create the turnPID with (PID) values from constants
    private val turnPID: PIDController = PIDController(Constants.SwerveConstants.TURN_MODULE_PID_P, Constants.SwerveConstants.TURN_MODULE_PID_I,
        Constants.SwerveConstants.TURN_MODULE_PID_D).apply { enableContinuousInput(-Math.PI, Math.PI) }

    // Reset both encoders, drive to 0 and turn to absolute value
    fun resetEncoders(){ driveMotor.resetEncoder(); turnMotor.encoder.position = absoluteEncoder.getRadians() }

    // @return (SwerveModuleState) Returns current state of the module
    fun getState(): SwerveModuleState = SwerveModuleState(driveMotor.getEncoderVelocity(), Rotation2d(turnMotor.getEncoderPosition()))

    // @param (state) (SwerveModuleState) Set the current state of the module
    fun setState(state: SwerveModuleState){ if(Math.abs(state.speedMetersPerSecond) > Constants.SwerveConstants.TELEOP_MIN_STATE_CHANGE){
            var optimizedState = SwerveModuleState.optimize(state, getState().angle)
            driveMotor.set(optimizedState.speedMetersPerSecond / Constants.MK4SDS.THEORETICAL_MAX_SPEED)
            turnMotor.set(turnPID.calculate(turnMotor.encoder.position, optimizedState.angle.radians))
        }else{ stopMotors() }
    }

    // Stop both module motors
    fun stopMotors(){ driveMotor.stop(); turnMotor.stop(); }

    // @param (angle) (Rotation2d) Set the angle of the swerve module
    fun setAngle(angle: Rotation2d){ turnMotor.set(turnPID.calculate(turnMotor.encoder.position, angle.radians)) }

    // @return (Double:ARRAY) An array of the motor currents, driveMotor then turnMotor order.
    fun getMotorsCurrent(): Array<Double> = arrayOf(driveMotor.motor.outputCurrent, turnMotor.motor.outputCurrent)

    // @return (Double:ARRAY) An array of the motor temperatures, driveMotor then turnMotor order.
    fun getMotorsTemperature(): Array<Double> = arrayOf(driveMotor.motor.motorTemperature, turnMotor.motor.motorTemperature)

    // @return (Short:ARRAY) An array of the motor faults, driveMotor then turnMotor order.
    fun getMotorFaults(): Array<Short> = arrayOf(driveMotor.motor.faults, turnMotor.motor.faults)
    
    // @return (SwerveModulePosition) The current position of the swerve module 
    fun getPosition(): SwerveModulePosition = SwerveModulePosition(driveMotor.encoder.position, Rotation2d(turnMotor.encoder.position))


    // Print SmartDashboard data for this class
    fun updateSmartDashboard(){
        SmartDashboard.putString("Swerve module $moduleName :", getState().toString()) 
    }

    // Print SmartDashboard debug data for this class
    fun updateSmartDashboardDebug(){ updateSmartDashboard()
        DriverStation.reportWarning("DEBUG is on for $moduleName", true)
        SmartDashboard.putNumber("${moduleName} ABE Raw: ", absoluteEncoder.encoder.absolutePosition)
        SmartDashboard.putNumber("${moduleName} ABE RAD: ", absoluteEncoder.getRadians())
        SmartDashboard.putNumber("${moduleName} ABE Offset RAD: ", absoluteEncoderOffsetRadians)
    }

}
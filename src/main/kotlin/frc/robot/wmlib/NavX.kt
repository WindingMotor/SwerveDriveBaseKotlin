// Winding Motor Libary (wmlib) - Created by Isaac S for team 2106

package frc.robot.wmlib
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import java.lang.Math
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

class NavX(){

    // Create an AHRS on RoboRio SPI port
    val gyro = AHRS(SPI.Port.kMXP)

    init{
        
        // Calibrate the gyro on a new thread
        Thread{
        try{
            Thread.sleep(1000)
            gyro.calibrate()
        }catch(e: Exception){ DriverStation.reportWarning("Gyro (THREAD) calibration error!", true)}
        }.start()
        
    }

    // Zero the gyro 
    fun zeroHeading(){ gyro.reset() }

    /* Old code thats not inline
    fun getDegrees(): Double{
        var degrees = gyro.angle
        if(degrees < 0.0){return(degrees + 360.0)}
        else{return(degrees)}
    }
    */
    
    // @return (Double) The robot heading wrapped from 0 to 360 in degrees
    fun getDegrees(): Double = if((gyro.angle % 360) < 0.0){(gyro.angle % 360) + 360}else{(gyro.angle % 360)}
    
    // @return (Double) The robot heading in radians wrapped
    fun getRadians(): Double = Math.toRadians(getDegrees())

    // @return (Rotation2d) The robot heading in Rotation2d wrao.anglepped
    fun getRotation2d(): Rotation2d = Rotation2d(getRadians())

    // @return (Double) The estimated robot heading (Yaw) -> should be wrapped? -> for testing only!
    private fun getFusedHeading(): Double = gyro.fusedHeading.toDouble()
    
    // Print SmartDashboard data for this class
    fun updateSmartDashboard(){
        SmartDashboard.putNumber("Robot Degrees", getDegrees())
    }

    // Print SmartDashboard debug data for this class
    fun updateSmartDashboardDebug(){
        updateSmartDashboard()
        SmartDashboard.putNumber("Robot Radians", getRadians())
        SmartDashboard.putString("Robot Rotation2d", getRotation2d().toString())
        SmartDashboard.putNumber("NavX Fused Heading", getFusedHeading())
        SmartDashboard.putBoolean("NavX Connected", gyro.isConnected)
        SmartDashboard.putBoolean("NavX Calibrating", gyro.isCalibrating)
        SmartDashboard.putBoolean("NavX Interference", gyro.isMagneticDisturbance)

    }

}

// Winding Motor Libary (wmlib) - Created by Isaac S for team 2106

package frc.robot.wmlib
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import java.lang.Math
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

class NavX(){

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

    fun zeroHeading(){ gyro.reset() }

    /* Old code thats not inline
    fun getDegrees(): Double{
        var degrees = gyro.angle
        if(degrees < 0.0){return(degrees + 360.0)}
        else{return(degrees)}
    }
    */
    
    fun getDegrees(): Double = if((gyro.angle % 360) < 0.0){(gyro.angle % 360) + 360}else{(gyro.angle % 360)}

    fun getRadians(): Double = Math.toRadians(getDegrees())

    fun getRotation2d(): Rotation2d = Rotation2d(getRadians())

    fun getFusedHeading(): Double = gyro.fusedHeading.toDouble()
    
    fun getYaw(): Double = gyro.yaw.toDouble()

    fun updateSmartDashboard(){
        SmartDashboard.putNumber("Robot Degrees", getDegrees())
    }

    fun updateSmartDashboardDebug(){
        updateSmartDashboard()
        SmartDashboard.putNumber("Robot Radians", getRadians())
        SmartDashboard.putString("Robot Rotation2d", getRotation2d().toString())
        SmartDashboard.putBoolean("NavX Connected", gyro.isConnected)
        SmartDashboard.putBoolean("NavX Calibrating", gyro.isCalibrating)
        SmartDashboard.putBoolean("NavX Interference", gyro.isMagneticDisturbance)
        SmartDashboard.putNumber("NavX Fused Heading", getFusedHeading())
    }

}

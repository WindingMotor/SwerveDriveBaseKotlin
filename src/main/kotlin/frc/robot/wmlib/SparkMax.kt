// Winding Motor Libary (wmlib) - Created by Isaac S for team 2106

package frc.robot.wmlib
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.CANSparkMax.IdleMode
import com.revrobotics.RelativeEncoder

class SparkMax(deviceId: Int, motorInverted: Boolean, encoderInverted: Boolean) {

    // Create converison factor variables and set to defualt -1.0 
    private var positionConversionFactor = -1.0
    private var velocityConversionFactor = -1.0

    // Second constructor -> using conversion factors
    constructor(deviceId: Int, motorInverted: Boolean, encoderInverted: Boolean,
    newPositionConversionFactor: Double, newVelocityConversionFactor: Double):this(deviceId,motorInverted,encoderInverted){
        positionConversionFactor = newPositionConversionFactor;
        velocityConversionFactor = newVelocityConversionFactor;
    }

    // Create motor and apply inverted? and idle mode
    val motor: CANSparkMax = CANSparkMax(deviceId, MotorType.kBrushless).apply{ setInverted(motorInverted); setIdleMode(IdleMode.kBrake); }

    // Create encoder and apply coversion factors if necessary 
    val encoder: RelativeEncoder = motor.encoder.apply{ 
        setInverted(encoderInverted);
        if((positionConversionFactor != -1.0) && (velocityConversionFactor != -1.0)){ // Set the converison factors if needed
            setPositionConversionFactor(positionConversionFactor); setVelocityConversionFactor(velocityConversionFactor); }
    }

    // @param (speed) (Double) Sets motor speed from 0.0 to 1.0
    fun set(speed: Double){ motor.set(speed) }

    // Stop motor 
    fun stop(){ motor.stopMotor() }

    // @param (true)-> Set brake mode (false)-> Set idle mode
    fun setBrakeMode(isBrake: Boolean){if (isBrake) motor.setIdleMode(IdleMode.kBrake) else motor.setIdleMode(IdleMode.kCoast)}

    // Resets encoder to 0.0
    fun resetEncoder(){ encoder.position = 0.0 }

    // @return (Double) The enoder positon with converison factor, if applied
    fun getEncoderPosition(): Double = encoder.position

    // @return (Double) The enoder velocity with converison factor, if applied
    fun getEncoderVelocity(): Double = encoder.velocity

}

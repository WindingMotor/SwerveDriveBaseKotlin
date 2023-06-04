import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.CANSparkMax.IdleMode
import com.revrobotics.RelativeEncoder

class Motor(deviceId: Int, motorInverted: Boolean, encoderInverted: Boolean) {

    private var positionConversionFactor = -1.0
    private var velocityConversionFactor = -1.0

    // Second constructor for conversion factors
    constructor(deviceId: Int, motorInverted: Boolean, encoderInverted: Boolean,
    newPositionConversionFactor: Double, newVelocityConversionFactor: Double):this(deviceId,motorInverted,encoderInverted){
        positionConversionFactor = newPositionConversionFactor;
        velocityConversionFactor = newVelocityConversionFactor;
    }

    val motor: CANSparkMax = CANSparkMax(deviceId, MotorType.kBrushless).apply{ setInverted(motorInverted); setIdleMode(IdleMode.kBrake); }

    val encoder: RelativeEncoder = motor.encoder.apply{ 
        setInverted(encoderInverted);
        if((positionConversionFactor > -1.0) && (velocityConversionFactor > -1.0)){ // Set the converison factors if needed
            setPositionConversionFactor(positionConversionFactor); setVelocityConversionFactor(velocityConversionFactor);}
    }

    fun setSpeed(speed: Double){ motor.set(speed) }

    fun stop(){ motor.stopMotor() }

    /* @param true sets brake mode and false sets idle mode */
    fun setIdleMode(isBrake: Boolean){if (isBrake) motor.setIdleMode(IdleMode.kBrake) else motor.setIdleMode(IdleMode.kCoast)}

    fun resetEncoder(){ encoder.position = 0.0 }
    

}

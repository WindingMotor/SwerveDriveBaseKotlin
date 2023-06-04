// Winding Motor Libary (wmlib) Base Command - Created by Isaac S for team 2106

package frc.robot.commands
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ExampleSubsystem

class DriveSwerve(subsystem: ExampleSubsystem) : CommandBase(){

    init{
        addRequirements(subsystem)
    }

    override fun initialize(){}

    override fun execute(){}

    override fun end(interrupted: Boolean) {}

    override fun isFinished(): Boolean{
        return false
    }

}

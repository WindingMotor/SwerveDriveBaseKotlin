
package frc.robot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.Autos
import frc.robot.commands.ExampleCommand
import frc.robot.subsystems.ExampleSubsystem
import frc.robot.wmlib.SwerveSubsystem


class RobotContainer {

    private val exampleSubsystem = ExampleSubsystem()
    private val swerveSubsystem = SwerveSubsystem()

    private val driverController = CommandXboxController(Constants.OperatorConstants.kDriverControllerPort)

    init {
        // Configure the trigger bindings
        configureBindings()
    }


    private fun configureBindings() {

        // Schedule ExampleCommand when exampleCondition changes to true
        Trigger { exampleSubsystem.exampleCondition() }.onTrue(ExampleCommand(exampleSubsystem))

        // Schedule exampleMethodCommand when the Xbox controller's B button is pressed,
        // cancelling on release.
        driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand())
        
    }

    val autonomousCommand: Command
        get() {
            // An example command will be run in autonomous
            return Autos.exampleAuto(exampleSubsystem)
        }
}

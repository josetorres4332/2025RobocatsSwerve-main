package frc.robot;

//import classes
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer
{
  //Driver Xbox Controller
  final CommandXboxController driverXbox = new CommandXboxController(Constants.DriverControllerAssignments.driverControllerPort);

  //Operator Generic Controller
  final CommandXboxController operatorXbox = new CommandXboxController(Constants.OperatorControllerAssignments.operatorControllerPort);

  // The robot's subsystems and commands are defined here...
  //Swerve Subsystem
  private final SwerveSubsystem drivebase  = new SwerveSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final WristSubsystem wrist = new WristSubsystem(); 
  private final ClimberSubsystem climber = new ClimberSubsystem();  
 
  //The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer()
  {    
    NamedCommands.registerCommand("Default", elevator.goToDefaultCommand());
    NamedCommands.registerCommand("Raise Elevator to Level 1", elevator.goToLevel1Command());
    NamedCommands.registerCommand("Raise Elevator to Level 2", elevator.goToLevel2Command());
    NamedCommands.registerCommand("Raise Elevator to Level 3", elevator.goToLevel3Command());
    NamedCommands.registerCommand("Raise Elevator to Level 4", elevator.goToLevel4Command());    
    NamedCommands.registerCommand("Hold Elevator Position", elevator.holdElevatorPositionCommand());
    NamedCommands.registerCommand("Raise Wrist", wrist.goToWDefaultCommand());
    NamedCommands.registerCommand("Drop Wrist", wrist.goToDropCommand());
    NamedCommands.registerCommand("Hold Wrist Position", wrist.holdArmPositionCommand());
     

    // Configure the trigger bindings
    configureBindings();

    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity); //makes Field Oriented drive the default command

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

  /****************************************************************************************************/
  /*                                        BUTTONS                                                   */
  /*__________________________________________________________________________________________________*/
  /*                                                                                                  */
  /*         Button A = 0                                    Button B = 1                             */
  /*         Button X = 2                                    Button Y = 3                             */
  /*         Left Bumper = 4                                 Right Bumper = 5                         */
  /*         Pause Button = 6                                Menu Button = 7                          */
  /*                                                                                                  */
  /****************************************************************************************************/
    
    //use button A to go to Level 1
    operatorXbox.a().whileTrue(elevator.goToLevel1Command());
    operatorXbox.a().onFalse(elevator.holdElevatorPositionCommand());

    //use button B to go to Level 2
    operatorXbox.b().onTrue(elevator.goToLevel2Command());
    operatorXbox.b().onFalse(elevator.holdElevatorPositionCommand());

    //use button X to go to Level 3
    operatorXbox.x().onTrue(elevator.goToLevel3Command());
    operatorXbox.x().onFalse(elevator.holdElevatorPositionCommand());

    //use button Y to go to Level 4
    operatorXbox.y().onTrue(elevator.goToLevel4Command());
    operatorXbox.y().onFalse(elevator.holdElevatorPositionCommand());

    //use LEFT BUMPER to go to Default for traveling
    operatorXbox.leftBumper().onTrue(elevator.goToDefaultCommand()).onTrue(wrist.goToWDefaultCommand());
    operatorXbox.leftBumper().onFalse(elevator.holdElevatorPositionCommand()).onFalse(wrist.holdArmPositionCommand());

    //use D-PAD DOWN to drop coral
    operatorXbox.povDown().onTrue(wrist.goToDropCommand());
    operatorXbox.povDown().onFalse(wrist.holdArmPositionCommand());

    //use D-PAD UP to put wrist in travel mode
    operatorXbox.povUp().onTrue(wrist.goToWDefaultCommand());
    operatorXbox.povUp().onFalse(wrist.holdArmPositionCommand());

    //use RIGHT BUMPER makes climber go UP
    driverXbox.rightBumper().whileTrue(climber.climberGoingDown());  
    driverXbox.rightBumper().whileFalse(climber.climberStop());  

    //use RIGHT BUMPER makes climber go DOWN
    driverXbox.leftBumper().whileTrue(climber.climberGoingUp());  
    driverXbox.leftBumper().whileFalse(climber.climberStop());

  }

  
  //NamedCommands.registerCommand()
  //NamedCommands.registerCommand("Raise Elevator", elevator.goToLevel3Command());

  //Applies deadbands to driver joystick inputs before sending them to the robot movement
  Command driveFieldOrientedAngularVelocity = drivebase.driveCommand(
                          () -> (MathUtil.applyDeadband(driverXbox.getLeftY(), Constants.RIGHT_X_DEADBAND)),
                          () -> (MathUtil.applyDeadband(driverXbox.getLeftX(), Constants.RIGHT_Y_DEADBAND)), 
                          () -> (driverXbox.getRightX() * Constants.TURN_CONSTANT));   

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {

    return null;

  }

}
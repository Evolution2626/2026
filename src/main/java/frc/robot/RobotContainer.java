// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Aim.ShooterTrackingCommand;
import frc.robot.commands.Aim.TurretTrackingCommand;
import frc.robot.commands.Auto.VeryBadAutoV2;
import frc.robot.commands.Drivetrain.DriveCommand;
import frc.robot.commands.Drivetrain.ResetGyroCommand;
import frc.robot.commands.InstantSetter.StartAimingCommand;
import frc.robot.commands.InstantSetter.StartShooterCommand;
import frc.robot.commands.InstantSetter.StartShooterLastResort;
import frc.robot.commands.InstantSetter.StartShootingCommand;
import frc.robot.commands.InstantSetter.StartShootingHomeCommand;
import frc.robot.commands.InstantSetter.StopAimingCommand;
import frc.robot.commands.InstantSetter.StopShooterCommand;
import frc.robot.commands.InstantSetter.StopShootingCommand;
import frc.robot.commands.InstantSetter.DecrementModifierCommand;
import frc.robot.commands.InstantSetter.IncrementModifierCommand;
import frc.robot.commands.InstantSetter.ReverseShootingCommand;
import frc.robot.commands.InstantSetter.StopShootingHomeCommand;
import frc.robot.commands.InstantSetter.Intake.ReverseIntakeCommand;
import frc.robot.commands.InstantSetter.Intake.StartIntakeCommand;
import frc.robot.commands.InstantSetter.Intake.StopIntakeCommand;
import frc.robot.commands.Manual.MoveHoodCommand;
import frc.robot.commands.Manual.MoveTurretCommand;
import frc.robot.commands.Manual.UseSlider;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeSlider;
import frc.robot.subsystems.Roller;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final CommandXboxController controller = new CommandXboxController(0);
   private static final CommandXboxController controller1 = new CommandXboxController(1);
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final Shooter shooter = new Shooter();
  public static final Hood hood = new Hood();
  public static final IntakeSlider intakeSlider = new IntakeSlider();
  public static final Roller roller = new Roller();
  public static final Intake intake = new Intake();
  public static final Turret turret = new Turret();

  private SendableChooser<Command> autoChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData(autoChooser);
    // Configure the trigger bindings
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, controller));

    //auto aim commands
    shooter.setDefaultCommand(new ShooterTrackingCommand(shooter));
    //hood.setDefaultCommand(new HoodTrackingCommand(hood));
    turret.setDefaultCommand(new TurretTrackingCommand(turret));
   

    //Manual control commands Deprecated soon
    intakeSlider.setDefaultCommand(new UseSlider(intakeSlider, controller1));
    hood.setDefaultCommand(new MoveHoodCommand(hood, controller1));
    //turret.setDefaultCommand(new MoveTurretCommand(turret, controller1));

    autoChooser.addOption("fuck all", null);
    autoChooser.addOption("Very bad auto v2", new VeryBadAutoV2(drivetrain, turret, shooter, roller, intakeSlider));


    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //controller1.a().onTrue(new StartShooterCommand(shooter));
    controller1.rightBumper().onFalse(new StopShooterCommand(shooter));
    controller1.y().onTrue(new StartShootingCommand(roller, shooter));
    controller1.y().onFalse(new StopShootingCommand(roller, shooter));
    controller1.a().onTrue(new ReverseShootingCommand(roller, shooter));
    controller1.a().onFalse(new StopShootingCommand(roller, shooter));
    controller1.x().onTrue(new StartShooterLastResort(shooter));
    controller1.x().onFalse(new StopShooterCommand(shooter));
    controller1.rightBumper().onTrue(new StartAimingCommand(turret, shooter));
    controller1.rightBumper().onFalse(new StopAimingCommand(turret, shooter));
    controller1.leftBumper().onTrue(new StartShootingHomeCommand(hood, turret, shooter));
    controller1.leftBumper().onFalse(new StopShootingHomeCommand(hood, turret, shooter));
    controller1.povRight().onTrue(new IncrementModifierCommand(turret));
    controller1.povLeft().onTrue(new DecrementModifierCommand(turret));

    controller1.rightTrigger().onTrue(new StartIntakeCommand(intake));
    controller1.rightTrigger().onFalse(new StopIntakeCommand(intake));
    controller1.leftTrigger().onTrue(new ReverseIntakeCommand(intake));
    controller1.leftTrigger().onFalse(new StopIntakeCommand(intake));

    controller.start().onTrue(new ResetGyroCommand(drivetrain));
  
    
  }
  public void stopThing() {
      shooter.stopShooter();
        turret.setIsTracking(false);
      shooter.setIsTracking(false);
      turret.resetModifier();
          turret.setManual(false);
      intake.setPower(0);;
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}

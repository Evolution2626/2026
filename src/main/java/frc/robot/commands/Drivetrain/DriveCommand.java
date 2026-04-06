// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.util.Range;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
 
  /** Creates a new DrivetrainDri. */
  private Drivetrain drivetrain;
  private CommandXboxController controller;

  public DriveCommand(Drivetrain drivetrain, CommandXboxController controller) {
    
    this.controller = controller;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedX = controller.getLeftX();
    double speedY = controller.getLeftY();
    double speedR = controller.getRightX();

    speedX = Range.threshold(0.01, Math.pow(speedX,3));
    speedY = Range.threshold(0.01, Math.pow(speedY,3));
    speedR = Range.threshold(0.01, Math.pow(speedR,3));

    speedX *= 8;
    speedY *= 8;
    speedR *= 18;

    if (RobotState.isTest()) {
      drivetrain.allMotorAtZero();
    }else {
      drivetrain.driveSwerve(speedX, -speedY, -speedR, true);
      //drivetrain.driveSwerve(0.2, -0, -0, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

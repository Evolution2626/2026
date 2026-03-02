// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.util.Range;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtMovingTargetCommand extends Command {
  Drivetrain drivetrain;
  Limelight limelight;
  CommandXboxController xboxController;
  /** Creates a new AimAtMovingTargetCommand. */
  public AimAtMovingTargetCommand(Drivetrain drivetrain, Limelight limelight, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.xboxController = controller;
    addRequirements(drivetrain, limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    double speedX = xboxController.getLeftX();
    double speedY = xboxController.getLeftY();
    double speedR = xboxController.getRightX();

    speedX = Range.threshold(0.1, speedX);
    speedY = Range.threshold(0.1, speedY);
    speedR = Range.threshold(0.1, speedR);

    speedX *= 8;
    speedY *= 8;
    speedR *= 18;

    if (RobotState.isTest()) {
      drivetrain.allMotorAtZero();
    }else {
      drivetrain.driveSwerve(speedX, -speedY, -limelight.getdegRotationToTarget()/6-speedR/2, true);
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

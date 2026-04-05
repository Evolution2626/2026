// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Aim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Aimbot;
import frc.robot.subsystems.Hood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodTrackingCommand extends Command {
  Hood hood;
  PIDController hoodPID = new PIDController(0.1, 0, 0);// TODO tune PID values

  /** Creates a new HoodTrackingCommand. */
  public HoodTrackingCommand(Hood hood) {
    this.hood = hood;
    addRequirements(hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (hood.getIsTracking()) {
      if (hood.getTarget() != -1) {
        hood.setHoodSpeed(hoodPID.calculate(hood.getTarget(), hood.getEncoderValue()));
      } else {
        hood.setTarget(Aimbot.getHoodAngle());
      }
    } else {
      hood.setTarget(-1);
      hood.setHoodSpeed(0);// TODO check power needed to retract hood for the trench
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

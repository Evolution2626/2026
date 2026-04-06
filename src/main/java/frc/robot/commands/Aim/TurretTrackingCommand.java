// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Aim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Aimbot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretTrackingCommand extends Command {
  Turret turret;

  PIDController turretPID = new PIDController(0.75, 0.05, 0);

  /** Creates a new TurretTrackingCommand. */
  public TurretTrackingCommand(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (turret.getIsTracking()) {
      if (turret.getTarget() == -99) {
        turret.setTarget(
           turret.getEncoderValue() - turret.convertTurretAngleToEncoder(Aimbot.getTurretRotationOffsetToGoal()));
          //turret.setTarget(turret.convertTurretAngleToEncoder(Math.PI/2));
      } else {
        double power = turretPID.calculate(turret.getEncoderValue(), turret.getTarget());
        if(Math.abs(power) < 0.1) turret.setTarget(-99);
        turret.setTurretSpeed(power);
      }

    }
    else if(turret.getIsShootingHome()){
      double targetTurret = turret.convertTurretAngleToEncoder((((Drivetrain.getGyroAngle()+180)%360)-180)/360*2*Math.PI)+0.18;
      turret.setTurretSpeed(turretPID.calculate(turret.getEncoderValue(), targetTurret));

    } else {
      turret.setTarget(-99);
      turret.setTurretSpeed(0);
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.BlockingDeque;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Shooter extends SubsystemBase {
  private SparkFlex shooterMotor;
  private SparkFlexConfig shooterConfig = new SparkFlexConfig();
  private SparkClosedLoopController shooterController;

  private SparkMax feederMotor;
  private SparkMaxConfig feederConfig = new SparkMaxConfig();

  private double targetRPM = 0;
  private boolean isTracking = false;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new SparkFlex(Constants.shooterMotorID, SparkFlex.MotorType.kBrushless);
    shooterConfig.inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(60, 60)
        
     .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
     .p(0.00015)//0.0008
     .i(0)
     .d(0).feedForward.kV(0.000145);
    shooterMotor.configure(shooterConfig, (com.revrobotics.spark.SparkBase.ResetMode) null,
        (com.revrobotics.spark.SparkBase.PersistMode) null);

    shooterController = shooterMotor.getClosedLoopController();

    feederMotor = new SparkMax(Constants.feederMotorID, SparkMax.MotorType.kBrushless);
    feederConfig.inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60)
        .closedLoopRampRate(0.15)
        .openLoopRampRate(0.2);
    feederMotor.configure(feederConfig, (com.revrobotics.spark.SparkBase.ResetMode) null,
        (com.revrobotics.spark.SparkBase.PersistMode) null);

  }

  public void startShooter() {
    // this function should start the shooter at a random power, then the aimbot
    // will adjust the power to the correct value
    // shooterController.setSetpoint(1, ControlType.kDutyCycle);
    //shooterMotor.set(1);
    shooterController.setSetpoint(350, ControlType.kVelocity);

  }

  public void setShooterSpeed(double rpm) {
    // TODO implement closed loop control to target velocity
    targetRPM = rpm;
    shooterController.setSetpoint(targetRPM, ControlType.kVelocity);
  }

  public void stopShooter() {
    // this function should stop the shooter
    // shooterController.setSetpoint(0.05, ControlType.kDutyCycle);
    shooterMotor.set(0.05);
    targetRPM = 0;

  }

  public void startFeeder() {
    feederMotor.set(-1.0);
  }

  public void stopFeeder() {
    feederMotor.set(0);
  }

  public boolean getIsTracking() {
    return isTracking;
  }

  public void setIsTracking(boolean tracking) {
    isTracking = tracking;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter rpm", shooterMotor.getEncoder().getVelocity());
    // SmartDashboard.putNumber("shooter setpoint",
    // shooterController.getSetpoint());
  }
}

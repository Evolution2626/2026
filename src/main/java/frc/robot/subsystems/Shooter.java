// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  public enum ShooterState {
    RAMPING_UP, SHOOTING, STOPPED
  }
  private ShooterState shooterState = ShooterState.STOPPED;
  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new SparkFlex(Constants.shooterMotorID, SparkFlex.MotorType.kBrushless);
    shooterConfig.inverted(false)
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(60, 60)
                    .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .p(0.0007)
                    .i(0)
                    .d(0);
    shooterMotor.configure(shooterConfig, (com.revrobotics.spark.SparkBase.ResetMode) null, (com.revrobotics.spark.SparkBase.PersistMode) null);

    shooterController = shooterMotor.getClosedLoopController();


    feederMotor = new SparkMax(Constants.feederMotorID, SparkMax.MotorType.kBrushless);
    feederConfig.inverted(false)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(60)
                    .closedLoopRampRate(0.15)
                    .openLoopRampRate(0.2);
    feederMotor.configure(feederConfig, (com.revrobotics.spark.SparkBase.ResetMode) null, (com.revrobotics.spark.SparkBase.PersistMode) null);
 
  }

  /*public void setShooterState(ShooterState state) {
    shooterState = state;
    switch (shooterState) {
      case RAMPING_UP://Start the shooter
        startShooter();
        break;
      case SHOOTING://Start the feeder
        feederMotor.set(0.5);
        break;
      case STOPPED://Stop everything
        shooterMotor.set(0);
        feederMotor.set(0);
        break;
    }
  }*/

  public void startShooter(){
    //shooterMotor.set(0.75);//TODO implement closed loop control to target velocity
   // shooterController.setSetpoint(5000, ControlType.kVelocity);
       shooterController.setSetpoint(1, ControlType.kDutyCycle);

  }
  public void stopShooter(){
    shooterController.setSetpoint(0.05, ControlType.kDutyCycle);

  }
  public void startFeeder(){
    feederMotor.set(-1.0);
  }
  public void stopFeeder(){
    feederMotor.set(0);
  }
  
  public ShooterState getShooterState() {
    return shooterState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter rpm", shooterMotor.getEncoder().getVelocity());
    //SmartDashboard.putNumber("shooter setpoint", shooterController.getSetpoint());
  }
}

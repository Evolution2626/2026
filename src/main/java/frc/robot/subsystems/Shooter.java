// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private SparkMax rotationMotor;
  private TalonSRX shooterMotor;
  private TalonSRX feederMotor;
  private SparkMaxConfig rotationConfig;
  /** Creates a new shooter. */
  public Shooter() {
    rotationMotor = new SparkMax(Constants.rotation, MotorType.kBrushless);
    rotationConfig = new SparkMaxConfig();
    rotationConfig
                    .inverted(false)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20)
                    .closedLoopRampRate(0.15)
                    .openLoopRampRate(0.2);

    rotationMotor.configure(rotationConfig, (com.revrobotics.spark.SparkBase.ResetMode) null, (com.revrobotics.spark.SparkBase.PersistMode) null);

    //shooterMotor = new TalonSRX(Constants.shooter);
    //feederMotor = new TalonSRX(Constants.feeder);

   // feederMotor.setInverted(false);
    //shooterMotor.setInverted(false);

    /*feederMotor.configPeakCurrentLimit(40, 5);
    feederMotor.configPeakCurrentDuration(200, 5);
    feederMotor.configContinuousCurrentLimit(30, 5);
    feederMotor.enableCurrentLimit(true);

    shooterMotor.configPeakCurrentLimit(40, 5);
    shooterMotor.configPeakCurrentDuration(200, 5);
    shooterMotor.configContinuousCurrentLimit(30, 5);
    shooterMotor.enableCurrentLimit(true);

    feederMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.setNeutralMode(NeutralMode.Coast);*/
  }

  public void setRotationPower(double power){
    rotationMotor.set(power);
  }
  /*public void setFeederPower(double power){
    feederMotor.set(ControlMode.PercentOutput, power);
    
  }
  public void setShooterPower(double power){
    shooterMotor.set(ControlMode.PercentOutput, power);
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

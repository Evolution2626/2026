// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Range;

public class Turret extends SubsystemBase {
  private SparkMax turretMotor;
  private AbsoluteEncoder turretEncoder;
  private SparkMaxConfig turretConfig = new SparkMaxConfig();

  private double turn = 0;
  private double lastvalue = 0.5;
  private double currentValue = 0;
  private double minTurretRotation = -1.3;
  private double maxTurretRotation = 2.5;

  private boolean isTracking = false;
  private boolean isShootingHome = false;

  private double target = -99;
  private double turretModifier = 0;
  private boolean manual = false;

  /** Creates a new Turret. */
  public Turret() {
    turretMotor = new SparkMax(Constants.turretMotorID, SparkMax.MotorType.kBrushless);
    turretConfig.inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40);
    turretMotor.configure(turretConfig, (com.revrobotics.spark.SparkBase.ResetMode) null,
        (com.revrobotics.spark.SparkBase.PersistMode) null);
    turretEncoder = turretMotor.getAbsoluteEncoder();
    lastvalue = getEncoderValue();
  }

  public void setTurretSpeed(double speed) {
    turretMotor.set(limitTurretRotation(speed));
  }

  public double getEncoderValue() {
    return turn + turretEncoder.getPosition();
  }

  private double limitTurretRotation(double power) {
    if (getEncoderValue() > maxTurretRotation) {
      return Range.coerce(-1, 0, power);
    } else if (getEncoderValue() < minTurretRotation) {
      return Range.coerce(0, 1, power);
    }
    return power;
  }

  public double convertTurretAngleToEncoder(double angle) {
    // take an angle in radians and convert it to the corresponding encoder value
    double turretToEncoderGearRatio = 125 / 22;
    return (angle / (2 * Math.PI)) * turretToEncoderGearRatio;
  }

  public boolean getIsTracking() {
    return isTracking;
  }

  public void setIsTracking(boolean tracking) {
    isTracking = tracking;
  }

  public double getTarget() {
    return target;
  }

  public void setTarget(double target) {
    this.target = target;
  }

  public boolean getIsShootingHome(){
    return isShootingHome;
  }
  public void setIsShootingHome(boolean isShootingHome){
    this.isShootingHome = isShootingHome;
  }
  public double getMin(){
    return minTurretRotation;
  }
  public double getMax(){
    return maxTurretRotation;
  }
    public void incrementModifier(){
    turretModifier += 0.1;
  }
  public void decrementModifier(){
    turretModifier -= 0.1;
  }
  public double getModifier(){
    return turretModifier;
  }
  public void resetModifier(){
    turretModifier = 0;
  }
  public boolean getIfManual(){
    return manual;
  }
  public void setManual(boolean manual){
    this.manual = manual;
  }


  @Override
  public void periodic() {
    currentValue = turretEncoder.getPosition();
    if ((currentValue - lastvalue) > 0.5) {
      turn--;
    } else if ((currentValue - lastvalue) < -0.5) {
      turn++;
    }
    lastvalue = currentValue;

    SmartDashboard.putNumber("turret encoder", getEncoderValue());
    SmartDashboard.putNumber("target rotation", target);
  }
}

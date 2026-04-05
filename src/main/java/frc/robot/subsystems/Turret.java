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
  enum TurretState {
    TRACKING, STOPPED
  }
  private TurretState turretState = TurretState.STOPPED;
  
  private double turn = 0;
  private double lastvalue = 0;
  private double currentValue = 0;
  private double minTurretRotation = -1.5;
  private double maxTurretRotation = 2.5;

  private boolean isTracking = false;
  /** Creates a new Turret. */
  public Turret() {
    turretMotor = new SparkMax(Constants.turretMotorID, SparkMax.MotorType.kBrushless);
    turretConfig.inverted(false)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40)
                    .closedLoopRampRate(0.15)
                    .openLoopRampRate(0.2);
    turretMotor.configure(turretConfig, (com.revrobotics.spark.SparkBase.ResetMode) null, (com.revrobotics.spark.SparkBase.PersistMode) null);
    turretEncoder = turretMotor.getAbsoluteEncoder();
  }
  public void setTurretSpeed(double speed) {
    turretMotor.set(limitTurretRotation(speed));
  }
  public void setTurretState(TurretState state) {
    turretState = state;
  }
  public TurretState getTurretState() {
    return turretState;
  }
  public double getEncoderValue() {
    return turn + turretEncoder.getPosition();
  }
  
  private double limitTurretRotation(double power){//TODO check if rotation power are correct
    if(getEncoderValue() > maxTurretRotation){
      return Range.coerce(-1, 0, power);
    } else if(getEncoderValue() < minTurretRotation){
      return Range.coerce(0, 1, power);
    }
    return power;
  }
  public double convertTurretAngleToEncoder(double angle){
    return (angle/360)*6.25;

  }

  public boolean getIsTracking() {
    return isTracking;
  }
  public void setIsTracking(boolean tracking) {
    isTracking = tracking;
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
  }
}

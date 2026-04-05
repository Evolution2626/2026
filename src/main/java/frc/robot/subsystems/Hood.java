// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Range;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  private SparkMax hoodMotor;
  private AbsoluteEncoder hoodEncoder;
  private SparkMaxConfig hoodConfig = new SparkMaxConfig();

  double maxValue = 0.7;
  double minValue = 0.38;


  private boolean isTracking = false;
  private double target = -1;
  public Hood() {
    hoodMotor = new SparkMax(Constants.hoodMotorID, SparkMax.MotorType.kBrushless);
    hoodConfig.inverted(false)
                    .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
                    .smartCurrentLimit(40)
                    .closedLoopRampRate(0.15)
                    .openLoopRampRate(0.2)
                    .absoluteEncoder.setSparkMaxDataPortConfig();
    hoodMotor.configure(hoodConfig, (com.revrobotics.spark.SparkBase.ResetMode) null, (com.revrobotics.spark.SparkBase.PersistMode) null);
    hoodEncoder = hoodMotor.getAbsoluteEncoder();
  }

    public void setHoodSpeed(double speed) {
      SmartDashboard.putNumber("power", speed);

      if(getEncoderValue() >= maxValue){
        hoodMotor.set(Range.coerce(0, 1, speed));
      }
      else if(getEncoderValue() <= minValue){
        hoodMotor.set(Range.coerce(-1, 0, speed));
      }
      else{
 hoodMotor.set(speed);
      }
     
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

    public double getEncoderValue() {
      return hoodEncoder.getPosition();
    }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("hood encoder value", getEncoderValue());
    //bas 0.70
    //haut 0.38
    
  }
      
}

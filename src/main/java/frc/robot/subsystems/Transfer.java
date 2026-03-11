// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Transfer extends SubsystemBase {
  private SparkMax transferMotor;
  private SparkMaxConfig transferConfig = new SparkMaxConfig();

   public enum TransferState {
    INTAKING, OUTTAKING, STOPPED
  }
  private TransferState transferState = TransferState.STOPPED;
  /** Creates a new Transfer. */
  public Transfer() {
    transferMotor = new SparkMax(Constants.feederMotorID, SparkMax.MotorType.kBrushless);
    transferConfig.inverted(false)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20)
                    .closedLoopRampRate(0.15)
                    .openLoopRampRate(0.2);
    transferMotor.configure(transferConfig, (com.revrobotics.spark.SparkBase.ResetMode) null, (com.revrobotics.spark.SparkBase.PersistMode) null);
  }
  public void setTransferState(TransferState state) {
    transferState = state;
    switch (transferState) {
      case INTAKING:
        transferMotor.set(0.5);
        break;
      case OUTTAKING:
        transferMotor.set(-0.5);
        break;
      case STOPPED:
        transferMotor.set(0);
        break;
    }
  }
  public TransferState getTransferState() {
    return transferState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Range;

public class Drivetrain extends SubsystemBase {
  public static final ADIS16470_IMU gyro = new ADIS16470_IMU();

  private static TalonFX flDriveMotor;
  private static TalonFX frDriveMotor;
  private static TalonFX blDriveMotor;
  private static TalonFX brDriveMotor;

  private static SparkMax flRotationMotor;
  private static SparkMax frRotationMotor;
  private static SparkMax blRotationMotor;
  private static SparkMax brRotationMotor;

  private SparkClosedLoopController flRotationController;
  private SparkClosedLoopController frRotationController;
  private SparkClosedLoopController blRotationController;
  private SparkClosedLoopController brRotationController;

  TalonFXConfiguration drivingConfig;
  FeedbackConfigs feedBackConfig;

  SwerveDriveKinematics kinematics;
  static SwerveDriveOdometry odometry;

  SparkMaxConfig turningConfig;

  // PIDController motorOutputPIDRotation;
  // PIDController motorOutputPIDDrive;

  double currentAngleRAD;
  double targetAngleRAD;

  Pose2d pose;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // motorOutputPIDRotation = new PIDController(0.5, 0, 0.00);
    // motorOutputPIDRotation.enableContinuousInput(-Math.PI, Math.PI);

    flRotationMotor = new SparkMax(Constants.flR, MotorType.kBrushless);
    frRotationMotor = new SparkMax(Constants.frR, MotorType.kBrushless);
    blRotationMotor = new SparkMax(Constants.blR, MotorType.kBrushless);
    brRotationMotor = new SparkMax(Constants.brR, MotorType.kBrushless);

    flDriveMotor = new TalonFX(Constants.flD);
    frDriveMotor = new TalonFX(Constants.frD);
    blDriveMotor = new TalonFX(Constants.blD);
    brDriveMotor = new TalonFX(Constants.brD);

    drivingConfig = new TalonFXConfiguration();
    feedBackConfig = new FeedbackConfigs();

    feedBackConfig.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    feedBackConfig.SensorToMechanismRatio = 5.54;// 13-16

    drivingConfig.withFeedback(feedBackConfig);
    drivingConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    drivingConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    drivingConfig.CurrentLimits.StatorCurrentLimit = 40;
    drivingConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    drivingConfig.CurrentLimits.SupplyCurrentLimit = 80;

    flDriveMotor.getConfigurator().apply(drivingConfig);
    flDriveMotor.getConfigurator()
        .apply(new Slot0Configs().withKP(Constants.driveKp).withKI(Constants.driveKi).withKD(Constants.driveKd));
    frDriveMotor.getConfigurator().apply(drivingConfig);
    frDriveMotor.getConfigurator()
        .apply(new Slot0Configs().withKP(Constants.driveKp).withKI(Constants.driveKi).withKD(Constants.driveKd));
    blDriveMotor.getConfigurator().apply(drivingConfig);
    blDriveMotor.getConfigurator()
        .apply(new Slot0Configs().withKP(Constants.driveKp).withKI(Constants.driveKi).withKD(Constants.driveKd));
    brDriveMotor.getConfigurator().apply(drivingConfig);
    brDriveMotor.getConfigurator()
        .apply(new Slot0Configs().withKP(Constants.driveKp).withKI(Constants.driveKi).withKD(Constants.driveKd));

    turningConfig = new SparkMaxConfig();
    turningConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .closedLoopRampRate(0.01)
        .openLoopRampRate(0.2).closedLoop.feedbackSensor(FeedbackSensor.kAnalogSensor)
        .p(2)
        .i(0)
        .d(0)
        .positionWrappingInputRange(0, 3.3)
        .positionWrappingEnabled(true);

    flRotationMotor.configure(turningConfig, (com.revrobotics.spark.SparkBase.ResetMode) null,
        (com.revrobotics.spark.SparkBase.PersistMode) null);
    frRotationMotor.configure(turningConfig, (com.revrobotics.spark.SparkBase.ResetMode) null,
        (com.revrobotics.spark.SparkBase.PersistMode) null);
    blRotationMotor.configure(turningConfig, (com.revrobotics.spark.SparkBase.ResetMode) null,
        (com.revrobotics.spark.SparkBase.PersistMode) null);
    brRotationMotor.configure(turningConfig, (com.revrobotics.spark.SparkBase.ResetMode) null,
        (com.revrobotics.spark.SparkBase.PersistMode) null);

    Translation2d frontLeftLocation = new Translation2d(-0.2925, 0.2925);
    Translation2d frontRightLocation = new Translation2d(0.2925, 0.2925);
    Translation2d backLeftLocation = new Translation2d(-0.2925, -0.2925);
    Translation2d backRightLocation = new Translation2d(0.2925, -0.2925);

    flRotationController = flRotationMotor.getClosedLoopController();
    frRotationController = frRotationMotor.getClosedLoopController();
    blRotationController = blRotationMotor.getClosedLoopController();
    brRotationController = brRotationMotor.getClosedLoopController();

    // Creating my kinematics object using the module locations
    kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
    odometry = new SwerveDriveOdometry(
        kinematics, getGyroRotation2d(),
        new SwerveModulePosition[] {
            getPosition(0),
            getPosition(1),
            getPosition(2),
            getPosition(3)
        }, new Pose2d(4.0, 4.0, new Rotation2d()));
  }

  public void allMotorAtZero() {
    flDriveMotor.set(0);
    frDriveMotor.set(0);
    blDriveMotor.set(0);
    brDriveMotor.set(0);

    flRotationMotor.set(0);
    frRotationMotor.set(0);
    blRotationMotor.set(0);
    brRotationMotor.set(0);
  }

  public static void setRobotPose(Pose2d pose) {
    odometry.resetPosition(getGyroRotation2d(), new SwerveModulePosition[] {
        getPosition(0),
        getPosition(1),
        getPosition(2),
        getPosition(3)
    }, new Pose2d(pose.getX(), pose.getY(), getGyroRotation2d()));
  }

  public static SwerveModulePosition getPosition(int moduleNumber) {
    double revolution = 0;
    switch (moduleNumber) {
      case 0:
        revolution = flDriveMotor.getPosition().getValueAsDouble();
        break;
      case 1:
        revolution = frDriveMotor.getPosition().getValueAsDouble();
        break;
      case 2:
        revolution = blDriveMotor.getPosition().getValueAsDouble();
        break;
      case 3:
        revolution = brDriveMotor.getPosition().getValueAsDouble();
        break;

    }
    return new SwerveModulePosition(
        revolution * Constants.RPStoMPS,
        new Rotation2d(returnEncoderAngle(moduleNumber)).minus(new Rotation2d(getEncoderOffset(moduleNumber))));
  }

  public static double getGyroAngle() {
    return gyro.getAngle();
  }

  public void resetGyroAngle() {
    gyro.reset();
    odometry.resetRotation(getGyroRotation2d());
  }

 

  public static Rotation2d getGyroRotation2d() {

    return new Rotation2d(Units.degreesToRadians(getGyroAngle())).minus(new Rotation2d(Math.PI/2));
  }
  public static Rotation2d getGyroRotation2dPose() {

    return new Rotation2d(Units.degreesToRadians(getGyroAngle()));
  }
  public static Pose2d getRobotPose(){
    return odometry.getPoseMeters();
  }

  static double returnEncoderAngle(int encoderNumber) {
    switch (encoderNumber) {
      case 0:
        double v = MathUtil.clamp(flRotationMotor.getAnalog().getPosition(), 0.0, 3.3);
        double angle = (v / 3.3) * 2.0 * Math.PI - Math.PI;
        return angle;
      case 1:
        double v1 = MathUtil.clamp(frRotationMotor.getAnalog().getPosition(), 0.0, 3.3);
        double angle1 = (v1 / 3.3) * 2.0 * Math.PI - Math.PI;
        return angle1;
      case 2:
        double v2 = MathUtil.clamp(blRotationMotor.getAnalog().getPosition(), 0.0, 3.3);
        double angle2 = (v2 / 3.3) * 2.0 * Math.PI - Math.PI;
        return angle2;
      case 3:
        double v3 = MathUtil.clamp(brRotationMotor.getAnalog().getPosition(), 0.0, 3.3);
        double angle3 = (v3 / 3.3) * 2.0 * Math.PI - Math.PI;
        return angle3;
      default:
        return 0;
    }
  }

  private static double getEncoderOffset(int encoderNumber) {
    switch (encoderNumber) {
      case 0:
        return Constants.flEOffset;
      case 1:
        return Constants.frEOffset;
      case 2:
        return Constants.blEOffset;
      case 3:
        return Constants.brEOffset;
      default:
        return 0.0;
    }
  }

  // wrap angle to [-PI, PI]
  private double wrapRadians(double angle) {
    while (angle > Math.PI)
      angle -= 2.0 * Math.PI;
    while (angle <= -Math.PI)
      angle += 2.0 * Math.PI;
    return angle;
  }

  // apply encoder offset before commanding the SparkMax
  void goToAngle(Rotation2d targetAngle, SparkMax rotationMotor, int encoderNumber) {

    double radians = targetAngle.plus(new Rotation2d(getEncoderOffset(encoderNumber))).getRadians();

    // map from [-PI, PI] to [0, 3.3] (analog voltage range used)
    double normalized = ((radians + Math.PI) / (2.0 * Math.PI)) * 3.3;
    normalized = MathUtil.clamp(normalized, 0.0, 3.3);
    SmartDashboard.putNumber("normalized", normalized);
    rotationMotor.getClosedLoopController().setSetpoint(normalized, ControlType.kPosition);
  }

  // fontion qui va faire tourner une roue à une certaine vitesse linéaire en m/s
  /*
   * public double setLinearVelocity(double targetSpeed, TalonFX driveMotor){
   * double currentMotorSpeed = driveMotor.getVelocity().getValueAsDouble();
   * //ajouter un PID qui calcule la vitesse à output dans le moteur pour
   * atteindre le targetSpeed de manière optimale
   * //peut-être utiliser le PID inclut dans les sparkmax pour plus d'efficacité
   * double motorOutput =
   * MathUtil.clamp(motorOutputPIDDrive.calculate(currentMotorSpeed, targetSpeed),
   * -1, 1);
   * 
   * return motorOutput;
   * }
   */
  // fontion qui va orienter la roue vers un certain angle en rotation2d
  /*
   * public double goToAngle(Rotation2d targetAngle, SparkMax rotationMotor, int
   * encoderNumber){
   * currentAngleRAD = returnEncoderAngle(encoderNumber);
   * targetAngleRAD = targetAngle.getRadians();
   * SmartDashboard.putNumber("target angle", targetAngleRAD);
   * 
   * 
   * //ajouter un PID qui calcule la vitesse à output dans le moteur pour
   * atteindre le targetAngle de manière optimale
   * //peut-être utiliser le PID inclut dans les sparkmax pour plus d'efficacité
   * double motorOutput =
   * MathUtil.clamp(motorOutputPIDRotation.calculate(currentAngleRAD,
   * targetAngleRAD), -1, 1);
   * motorOutput = Range.threshold(0.05, motorOutput);
   * SmartDashboard.putNumber("motor output", motorOutput);
   * return motorOutput;
   * }
   */
  /*
   * void goToAngle(Rotation2d targetAngle, SparkMax rotationMotor){
   * 
   * double radians = targetAngle.getRadians();
   * // map from [-PI, PI] to [0, 1]
   * double normalized = ((radians + Math.PI) / (2.0 * Math.PI))*3.3;
   * normalized = MathUtil.clamp(normalized, 0.0, 3.3);
   * rotationMotor.getClosedLoopController().setSetpoint(normalized,
   * ControlType.kPosition);
   * 
   * }
   */
  // fonction qui va gérer la position et la vitesse d'une roue
  public void driveOneSwerve(SwerveModuleState moduleState, SparkMax rotationMotor, TalonFX driveMotor,
      int encoderNumber) {

    // create a velocity closed-loop request, voltage output, slot 0 configs
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    // set velocity to 8 rps, add 0.5 V to overcome gravity
    driveMotor.setControl(m_request.withVelocity(moduleState.speedMetersPerSecond / Constants.RPStoMPS * 1.5));
    goToAngle(moduleState.angle, rotationMotor, encoderNumber);

    // rotationMotor.set(goToAngle(moduleState.angle, rotationMotor,
    // encoderNumber));
    // goToAngle(moduleState.angle, rotationMotor, encoderNumber);
  }

  public void driveSwerve(double x, double y, double r, boolean fieldRelative) {
    // Example chassis speeds: 1 meter per second forward, 3 meters
    // per second to the left, and rotation at 1.5 radians per second
    // counterclockwise.
    ChassisSpeeds speeds = new ChassisSpeeds();

    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, new Rotation2d(Units.degreesToRadians(getGyroAngle())));

    } else {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, new Rotation2d(0));

    }
    // Convert to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    // Front left module state
    SwerveModuleState frontLeft = moduleStates[0];
    frontLeft.optimize(new Rotation2d(returnEncoderAngle(0)).minus(new Rotation2d(getEncoderOffset(0))));
    // // Front right module state
    SwerveModuleState frontRight = moduleStates[1];
    frontRight.optimize(new Rotation2d(returnEncoderAngle(1)).minus(new Rotation2d(getEncoderOffset(1))));
    // // Back left module state
    SwerveModuleState backLeft = moduleStates[2];
    backLeft.optimize(new Rotation2d(returnEncoderAngle(2)).minus(new Rotation2d(getEncoderOffset(2))));
    // // Back right module state
    SwerveModuleState backRight = moduleStates[3];
    backRight.optimize(new Rotation2d(returnEncoderAngle(3)).minus(new Rotation2d(getEncoderOffset(3))));
    driveOneSwerve(frontLeft, flRotationMotor, flDriveMotor, 0);
    driveOneSwerve(frontRight, frRotationMotor, frDriveMotor, 1);
    driveOneSwerve(backLeft, blRotationMotor, blDriveMotor, 2);
    driveOneSwerve(backRight, brRotationMotor, brDriveMotor, 3);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("analog", blRotationMotor.getAnalog().getPosition());
    // SmartDashboard.putNumber("analog error",
    // blRotationMotor.getClosedLoopController().getSetpoint());
    SmartDashboard.putNumber("Gyro", getGyroAngle());
    SmartDashboard.putNumber("velocity", frDriveMotor.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("FL", returnEncoderAngle(0));
    SmartDashboard.putNumber("FR", returnEncoderAngle(1));
    SmartDashboard.putNumber("BL", returnEncoderAngle(2));
    SmartDashboard.putNumber("BR", returnEncoderAngle(3));
    pose = odometry.update(getGyroRotation2d(),
        new SwerveModulePosition[] {
            getPosition(0),
            getPosition(1),
            getPosition(2),
            getPosition(3)
        });
    SmartDashboard.putNumber("pose x", pose.getX());
    SmartDashboard.putNumber("pose y", pose.getY());
    SmartDashboard.putNumber("pose rotation", pose.getRotation().getDegrees());

  }
}
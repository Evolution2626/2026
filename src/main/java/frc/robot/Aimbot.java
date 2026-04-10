// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.util.Range;

/** Add your docs here. */
public class Aimbot {


    static double[] coordinates = { 4.6, 4 };

    public static Pose2d getRobotPose() {

        LimelightHelpers.PoseEstimate limelightMeasurementBlue = LimelightHelpers
                .getBotPoseEstimate_wpiBlue("limelight");// TODO check the name of the limelight
        LimelightHelpers.PoseEstimate limelightMeasurementRed = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");
        Optional<Alliance> ally = DriverStation.getAlliance();
        LimelightHelpers.PoseEstimate robotLimelightPose = ally.isPresent() && ally.get() == DriverStation.Alliance.Red
                ? limelightMeasurementRed
                : limelightMeasurementBlue;
        if (robotLimelightPose.tagCount > 0) {
            Pose2d robotPose = robotLimelightPose.pose;
            SmartDashboard.putNumber("robot x", robotPose.getX());
            SmartDashboard.putNumber("robot y", robotPose.getY());
            SmartDashboard.putNumber("robot angle", robotLimelightPose.pose.getRotation().getDegrees());
            Drivetrain.setRobotPose(robotPose);

            return robotPose;
        }

        return null;
    }

    public static double getRobotDistanceToGoal() {
        getRobotPose();
        Pose2d robotPose = Drivetrain.getRobotPose();

        double goalX = coordinates[0];
        double goalY = coordinates[1];

        if (robotPose != null) {
            Translation2d robotTranslation = robotPose.getTranslation();
            double distance = Math
                    .sqrt(Math.pow(goalX - robotTranslation.getX(), 2) + Math.pow(goalY - robotTranslation.getY(), 2));
            SmartDashboard.putNumber("distance to goal", distance);
            return distance;
        }
        return 0;
    }

    public static double getTurretRotationOffsetToGoal() {
        getRobotPose();
        Pose2d robotPose = Drivetrain.getRobotPose();
        double goalX = coordinates[0];
        double goalY = coordinates[1];

        if (robotPose != null) {
            Translation2d robotTranslation = robotPose.getTranslation();
            double angleToGoal = Math.atan2(goalY - robotTranslation.getY(), goalX - robotTranslation.getX());
            double turretRotationOffset = angleToGoal - robotPose.getRotation().getRadians();
            return turretRotationOffset;// TODO check the math above
        }
        return 0;
    }

    public static Rotation2d getTurretRotation() {
        getRobotPose();
        Pose2d robotPose = Drivetrain.getRobotPose();
        double goalX = coordinates[0];
        double goalY = coordinates[1];
        SmartDashboard.putNumber("goal x", goalX);
        SmartDashboard.putNumber("goal y", goalY);
        


        if (robotPose != null) {
            Translation2d robotTranslation = robotPose.getTranslation();
             SmartDashboard.putNumber("goal offset x", goalX  - robotTranslation.getX());
        SmartDashboard.putNumber("goal offset y", goalY- robotTranslation.getY());
            Rotation2d rer = new Rotation2d(goalX - robotTranslation.getX(), goalY - robotTranslation.getY());
            SmartDashboard.putNumber("angle to goal(rer)", rer.getRadians());
            //double angleToGoal = Math.atan2(goalY - robotTranslation.getY(), goalX - robotTranslation.getX());
            //return new Rotation2d(angleToGoal - Drivetrain.getGyroRotation2d().getRadians());
            Rotation2d temp = rer.minus(Drivetrain.getGyroRotation2d());
            SmartDashboard.putNumber("returned", temp.getRadians());
            return temp;
            
        }
        return null;
    }

    public static double getShooterRPM() {
        return linearVelocityInterpolation(getRobotDistanceToGoal());
    }

    public static double getHoodAngle() {
        return linearHoodAngleInterpolation(getRobotDistanceToGoal());
    }

    private static double linearVelocityInterpolation(double distance) {
        double[] point1 = { 2.26, 3300 };
        double[] point2 = { 4, 3800 };

        double slope = (point2[1] - point1[1]) / (point2[0] - point1[0]);
        double intercept = point1[1] - slope * point1[0];
        return Range.coerce( 3000,7000,slope * distance + intercept);
    }

    private static double linearHoodAngleInterpolation(double distance) {
        double[] point1 = { 0, 0 };
        double[] point2 = { 0, 0 };

        double slope = (point2[1] - point1[1]) / (point2[0] - point1[0]);
        double intercept = point1[1] - slope * point1[0];
        return slope * distance + intercept;
    }
}

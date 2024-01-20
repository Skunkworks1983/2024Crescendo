// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SwerveTeleop;
import frc.robot.constants.Constants;

public class Drivebase extends SubsystemBase {

  private static Drivebase drivebase;
  OI oi = OI.getInstance();
  AHRS gyro = new AHRS(I2C.Port.kOnboard);
  private final Field2d odometryFieldPos = new Field2d();
  ChassisSpeeds speeds;
  Pose2d pose;
  PIDController headingController = new PIDController(Constants.PIDControllers.HeadingControlPID.KP, Constants.PIDControllers.HeadingControlPID.KI, Constants.PIDControllers.HeadingControlPID.KD);

  // locations of the modules
  Translation2d leftFrontLocation = new Translation2d(Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_X),
      Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_Y));

  Translation2d rightFrontLocation = new Translation2d(Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_X),
      Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_Y));

  Translation2d leftBackLocation = new Translation2d(Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_X),
      Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_Y));

  Translation2d rightBackLocation = new Translation2d(Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_X),
      Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_Y));


  SwerveModule frontLeft = new SwerveModule(Constants.IDS.LEFT_FRONT_DRIVE, Constants.IDS.LEFT_FRONT_TURN,
      Constants.IDS.LEFT_FRONT_CAN_CODER, Constants.DrivebaseInfo.FRONT_LEFT_OFFSET);

  SwerveModule frontRight = new SwerveModule(Constants.IDS.RIGHT_FRONT_DRIVE, Constants.IDS.RIGHT_FRONT_TURN,
      Constants.IDS.RIGHT_FRONT_CAN_CODER, Constants.DrivebaseInfo.FRONT_RIGHT_OFFSET);

  SwerveModule backLeft = new SwerveModule(Constants.IDS.LEFT_BACK_DRIVE, Constants.IDS.LEFT_BACK_TURN,
      Constants.IDS.LEFT_BACK_CAN_CODER, Constants.DrivebaseInfo.BACK_LEFT_OFFSET);

  SwerveModule backRight = new SwerveModule(Constants.IDS.RIGHT_BACK_DRIVE, Constants.IDS.RIGHT_BACK_TURN,
      Constants.IDS.RIGHT_BACK_CAN_CODER, Constants.DrivebaseInfo.BACK_RIGHT_OFFSET);


  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      leftFrontLocation, rightFrontLocation, leftBackLocation, rightBackLocation);


  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      kinematics,
      Rotation2d.fromDegrees(-getGyroAngle()),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
      }); 

  private Drivebase() {
    gyro.reset();
    
    resetOdometry(new Pose2d(Units.feetToMeters(Constants.FIELD_Y_LENGTH-1.895833333), 
    Units.feetToMeters(Constants.FIELD_X_LENGTH-1.895833333),
    Rotation2d.fromDegrees(180)));
    headingController.enableContinuousInput(0, 360);
    SmartDashboard.putData("Field Pos", odometryFieldPos);
  }

  /** run in teleop init to set swerve as default teleop command */
  public void setSwerveAsDefaultCommand() {
    setDefaultCommand(new SwerveTeleop(drivebase, oi));
  }

  public double getGyroAngle() {
    double angle = gyro.getAngle();
    SmartDashboard.putNumber("gyro angle", angle);
    return angle;
  }

  public void resetOdometry(Pose2d position) {
    odometry.resetPosition(
      Rotation2d.fromDegrees(-getGyroAngle()), 
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      }, 
      position
    );

    SmartDashboard.putBoolean("odometry reset pos", true);
  }

  public Pose2d getOdometry() {
    return odometry.getPoseMeters();
  }

  public void setDrive(double xFeetPerSecond, double yFeetPerSecond, double degreesPerSecond, boolean fieldRelative) {
    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          Units.feetToMeters(xFeetPerSecond),
          Units.feetToMeters(yFeetPerSecond),
          Units.degreesToRadians(degreesPerSecond),
          Rotation2d.fromDegrees(-getGyroAngle())); // negative because gyro reads differently than wpilib
    } else {
      speeds = new ChassisSpeeds(
          Units.feetToMeters(xFeetPerSecond),
          Units.feetToMeters(yFeetPerSecond),
          Units.degreesToRadians(degreesPerSecond));
    }

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_MODULE_SPEED); // sets module max speed

    setModuleStates(moduleStates);
  }

  public void setDriveDeadband(double xFeetPerSecond, double yFeetPerSecond, boolean fieldRelative) {

    double degreesPerSecond;
    degreesPerSecond = headingController.calculate(gyro.getAngle());


    if (fieldRelative) {

      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          Units.feetToMeters(xFeetPerSecond),
          Units.feetToMeters(yFeetPerSecond),
          Units.degreesToRadians(degreesPerSecond),
          Rotation2d.fromDegrees(-getGyroAngle()));

    } else {

      speeds = new ChassisSpeeds(
          Units.feetToMeters(xFeetPerSecond),
          Units.feetToMeters(yFeetPerSecond),
          Units.degreesToRadians(degreesPerSecond));
    }

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_MODULE_SPEED);

    setModuleStates(moduleStates);
  }

  public void setHeadingController(double setpoint){

    headingController.setSetpoint(setpoint);
  }


  public void setModuleStates(SwerveModuleState[] states) {
    frontLeft.setState(states[0]);
    frontRight.setState(states[1]);
    backLeft.setState(states[2]);
    backRight.setState(states[3]);
  }


  @Override
  public void periodic() {
    pose = odometry.update(Rotation2d.fromDegrees(-getGyroAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });

    odometryFieldPos.setRobotPose(odometry.getPoseMeters());
  }


  
  public static Drivebase getInstance() {
    if (drivebase == null) {
      drivebase = new Drivebase();
    }
    return drivebase;
  }
}
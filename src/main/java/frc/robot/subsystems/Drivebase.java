// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

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
import frc.robot.utils.SmartPIDController;

public class Drivebase extends SubsystemBase {

  private static Drivebase drivebase;
  OI oi = OI.getInstance();
  AHRS gyro = new AHRS(I2C.Port.kOnboard);
  private final Field2d odometryFieldPos = new Field2d();
  ChassisSpeeds speeds;
  Pose2d pose;
  SmartPIDController headingController = new SmartPIDController(
    Constants.PIDControllers.HeadingControlPID.KP, 
    Constants.PIDControllers.HeadingControlPID.KI, 
    Constants.PIDControllers.HeadingControlPID.KD,
    "Heading Controller",
    true
  );

  // locations of the modules, x positive forward y positive left
  Translation2d leftFrontLocation = new Translation2d(
    Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_X),
    Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_Y)
  );

  Translation2d rightFrontLocation = new Translation2d(
    Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_X),
    Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_Y)
  );

  Translation2d leftBackLocation = new Translation2d(
    Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_X),
    Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_Y)
  );

  Translation2d rightBackLocation = new Translation2d(
    Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_X),
    Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_Y)
  );


  SwerveModule frontLeft = new SwerveModule(
    Constants.IDS.LEFT_FRONT_DRIVE, Constants.IDS.LEFT_FRONT_TURN,
    Constants.IDS.LEFT_FRONT_CAN_CODER, Constants.DrivebaseInfo.FRONT_LEFT_OFFSET,
    "Front Left"
  );

  SwerveModule frontRight = new SwerveModule(
    Constants.IDS.RIGHT_FRONT_DRIVE, Constants.IDS.RIGHT_FRONT_TURN,
    Constants.IDS.RIGHT_FRONT_CAN_CODER, Constants.DrivebaseInfo.FRONT_RIGHT_OFFSET,
    "Front Right"
  );

  SwerveModule backLeft = new SwerveModule(
    Constants.IDS.LEFT_BACK_DRIVE, Constants.IDS.LEFT_BACK_TURN,
    Constants.IDS.LEFT_BACK_CAN_CODER, Constants.DrivebaseInfo.BACK_LEFT_OFFSET,
    "Back Left"
  );

  SwerveModule backRight = new SwerveModule(
    Constants.IDS.RIGHT_BACK_DRIVE, Constants.IDS.RIGHT_BACK_TURN,
    Constants.IDS.RIGHT_BACK_CAN_CODER, Constants.DrivebaseInfo.BACK_RIGHT_OFFSET,
    "Back Right"
  );


  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    leftFrontLocation, 
    rightFrontLocation, 
    leftBackLocation, 
    rightBackLocation
  );


  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    kinematics,
    Rotation2d.fromDegrees(getGyroAngle()),
    new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    }
  ); 

  private Drivebase() {
    gyro.reset();
    
    resetOdometry(
      Constants.START_POSITION
    );
    headingController.enableContinuousInput(0, 360);
    SmartDashboard.putData("Field Pos", odometryFieldPos);
  }

  /** run in teleop init to set swerve as default teleop command */
  public void setSwerveAsDefaultCommand() {
    setDefaultCommand(new SwerveTeleop(drivebase, oi));
  }

  //returns angle going counterclockwise
  public double getGyroAngle() {
    double angle = gyro.getAngle();
    SmartDashboard.putNumber("gyro angle", angle);
    return -angle;
    // negative because gyro reads differently than wpilib
  }

  public void resetOdometry(Pose2d position) {
    odometry.resetPosition(
      Rotation2d.fromDegrees(getGyroAngle()), 
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

  //used when driving without rotating
  public void setDrive(double xFeetPerSecond, double yFeetPerSecond, double degreesPerSecond, boolean fieldRelative) {
    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        Units.feetToMeters(xFeetPerSecond),
        Units.feetToMeters(yFeetPerSecond),
        Units.degreesToRadians(degreesPerSecond),
        Rotation2d.fromDegrees(getGyroAngle())
      ); 
    } else {
      speeds = new ChassisSpeeds(
        Units.feetToMeters(xFeetPerSecond),
        Units.feetToMeters(yFeetPerSecond),
        Units.degreesToRadians(degreesPerSecond)
      );
    }

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_MODULE_SPEED); // sets module max speed

    setModuleStates(moduleStates);
  }

  //used for keeping robot heading in the right direction using PID and the targeting buttion 
  public void setDriveTurnPos(double xFeetPerSecond, double yFeetPerSecond, boolean fieldRelative) {

    double degreesPerSecond;
    degreesPerSecond = headingController.calculate(-getGyroAngle());

    setDrive(xFeetPerSecond, yFeetPerSecond, degreesPerSecond, fieldRelative);
  }

  public void setHeadingController(double setpoint){
    headingController.setSetpoint(setpoint);
    SmartDashboard.putNumber("Heading Setpoint", setpoint);
  }


  public void setModuleStates(SwerveModuleState[] states) {
    frontLeft.setState(states[0]);
    frontRight.setState(states[1]);
    backLeft.setState(states[2]);
    backRight.setState(states[3]);
  } 

  public Pose2d getRobotPose() {
    // intentionally negating XY axes
    Pose2d poseOutput = new Pose2d(-odometry.getPoseMeters().getX(), -odometry.getPoseMeters().getY(), odometry.getPoseMeters().getRotation());

    return poseOutput;
  }

  @Override
  public void periodic() {
    pose = odometry.update(
      Rotation2d.fromDegrees(getGyroAngle()),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      }
    );

    odometryFieldPos.setRobotPose(getRobotPose());
    SmartDashboard.putNumber("X odometry Pos", getRobotPose().getX());
    SmartDashboard.putNumber("Y odometry Pos", getRobotPose().getY());
  }


  
  public static Drivebase getInstance() {
    if (drivebase == null) {
      drivebase = new Drivebase();
    }
    return drivebase;
  }
}
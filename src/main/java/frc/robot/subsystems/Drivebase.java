// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

  // locations of the modules, x positive forward y positive left
  Translation2d leftFrontLocation = new Translation2d(Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_X),
      Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_Y));

  Translation2d rightFrontLocation = new Translation2d(Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_X),
      Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_Y));

  Translation2d leftBackLocation = new Translation2d(Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_X),
      Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_Y));

  Translation2d rightBackLocation = new Translation2d(Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_X),
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
    configurePathPlanner();
    resetOdometry(
      new Pose2d(
        Units.feetToMeters(0), 
        Units.feetToMeters(0),
        Rotation2d.fromDegrees(0)
      )
    );
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
    System.out.println("reset odometery is running");
    System.out.println("position x: "+position.getX());
    System.out.println("position y: "+position.getY());
    System.out.println("position theta: "+position.getRotation());
    SmartDashboard.putBoolean("odometry reset pos", true);
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
    System.out.println(xFeetPerSecond+" "+yFeetPerSecond+" "+degreesPerSecond);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_MODULE_SPEED);

    setModuleStates(moduleStates);
  }

  public void setDriveChassisSpeed(ChassisSpeeds chassisSpeeds){
    
    setDrive(
      Units.metersToFeet(-chassisSpeeds.vxMetersPerSecond),
      Units.metersToFeet(-chassisSpeeds.vxMetersPerSecond),
      Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond),
    false
    );
    System.out.println("setDrive x m/s: " + -chassisSpeeds.vxMetersPerSecond);
    System.out.println("setDrive y m/s: " + -chassisSpeeds.vyMetersPerSecond);
    System.out.println("setDrive theta r/s: " + chassisSpeeds.omegaRadiansPerSecond);
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

  public Pose2d getRobotPose() {
    // intentionally negating XY axes
    Pose2d poseOutput;

    poseOutput = odometry.getPoseMeters();

    Pose2d returnPose = new Pose2d(-poseOutput.getX(), -poseOutput.getY(), poseOutput.getRotation());

    return returnPose;
  }

  @Override
  public void periodic() {
    pose = odometry.update(
      Rotation2d.fromDegrees(-getGyroAngle()),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      }
    );

    odometryFieldPos.setRobotPose(getRobotPose());
  }
  
  public static Drivebase getInstance() {
    if (drivebase == null) {
      drivebase = new Drivebase();
    }
    return drivebase;
  }

public ChassisSpeeds getRobotRelativeSpeeds(){
  
  ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(
    frontLeft.getSwerveState(),
    frontRight.getSwerveState(),
    backLeft.getSwerveState(),
    backRight.getSwerveState()
  );
    ChassisSpeeds chassisSpeedsMinus= new ChassisSpeeds(
    -chassisSpeeds.vxMetersPerSecond,
    -chassisSpeeds.vyMetersPerSecond,
    chassisSpeeds.omegaRadiansPerSecond
  );
  System.out.println("xMetersChassisSpeeds: "+chassisSpeedsMinus.vxMetersPerSecond);
  System.out.println("yMetersChassisSpeeds: "+chassisSpeedsMinus.vyMetersPerSecond);
  System.out.println("thetaMetersChassisSpeeds: "+chassisSpeedsMinus.omegaRadiansPerSecond);

  return chassisSpeedsMinus;
}
public void configurePathPlanner(){

  AutoBuilder.configureHolonomic(
    this::getRobotPose,
    this::resetOdometry,
    this::getRobotRelativeSpeeds, // uses -x,-y
    this::setDriveChassisSpeed, // uses -x,-y
    new HolonomicPathFollowerConfig(
      new PIDConstants(
        Constants.PathPlannerInfo.PATHPLANNER_DRIVE_KP, 
        Constants.PathPlannerInfo.PATHPLANNER_DRIVE_KI, 
        Constants.PathPlannerInfo.PATHPLANNER_DRIVE_KD
      ),
      new PIDConstants(
        Constants.PathPlannerInfo.PATHPLANNER_TURN_KP, 
        Constants.PathPlannerInfo.PATHPLANNER_TURN_KI, 
        Constants.PathPlannerInfo.PATHPLANNER_TURN_KD
      ),
      Constants.PathPlannerInfo.PATHPLANNER_MAX_METERS_PER_SECOND,
      Constants.PathPlannerInfo.PATHPLANNER_MAX_RADIANS_PER_SECOND,
      new ReplanningConfig()
    ), 
    () -> {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
      }  
      return false;
    }, 
    this
  );

}
  public Command followPathCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    return AutoBuilder.followPath(path);
  }
}
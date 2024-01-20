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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Drivebase extends SubsystemBase {

  private static Drivebase Drivebase;
  AHRS gyro = new AHRS(I2C.Port.kOnboard);
  private final Field2d odometryFieldPos = new Field2d();
  ChassisSpeeds speeds;
  Pose2d pose;

  // locations of the modules
  Translation2d leftFrontLocation = new Translation2d(-Constants.FEET_TO_METERS * Constants.TRANSLATION_X,
      Constants.FEET_TO_METERS * Constants.TRANSLATION_Y);

  Translation2d rightFrontLocation = new Translation2d(Constants.FEET_TO_METERS * Constants.TRANSLATION_X,
      Constants.FEET_TO_METERS * Constants.TRANSLATION_Y);

  Translation2d leftBackLocation = new Translation2d(-Constants.FEET_TO_METERS * Constants.TRANSLATION_X,
      -Constants.FEET_TO_METERS * Constants.TRANSLATION_Y);

  Translation2d rightBackLocation = new Translation2d(Constants.FEET_TO_METERS * Constants.TRANSLATION_X,
      -Constants.FEET_TO_METERS * Constants.TRANSLATION_Y);


  SwerveModule frontLeft = new SwerveModule(Constants.LEFT_FRONT_DRIVE, Constants.LEFT_FRONT_TURN,
      Constants.LEFT_FRONT_CAN_CODER, Constants.FRONT_LEFT_OFFSET);

  SwerveModule frontRight = new SwerveModule(Constants.RIGHT_FRONT_DRIVE, Constants.RIGHT_FRONT_TURN,
      Constants.RIGHT_FRONT_CAN_CODER, Constants.FRONT_RIGHT_OFFSET);

  SwerveModule backLeft = new SwerveModule(Constants.LEFT_BACK_DRIVE, Constants.LEFT_BACK_TURN,
      Constants.LEFT_BACK_CAN_CODER, Constants.BACK_LEFT_OFFSET);

  SwerveModule backRight = new SwerveModule(Constants.RIGHT_BACK_DRIVE, Constants.RIGHT_BACK_TURN,
      Constants.RIGHT_BACK_CAN_CODER, Constants.BACK_RIGHT_OFFSET);


  SwerveDriveKinematics kinematics = new SwerveDriveKinematics( // creating kinematics
      leftFrontLocation, rightFrontLocation, leftBackLocation, rightBackLocation);


  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      kinematics,
      Rotation2d.fromDegrees(-getGyroAngle()), // creating odometry
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
      }); 



  public Drivebase() {

    gyro.reset();
    configurePathPlanner();
    resetOdometry(new Pose2d((Constants.FIELD_Y_LENGTH-1.895833333) * Constants.FEET_TO_METERS, 
    (Constants.FIELD_X_LENGTH-1.895833333) * Constants.FEET_TO_METERS,
    Rotation2d.fromDegrees(180)));

    SmartDashboard.putData("Field Pos", odometryFieldPos);
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
      backRight.getPosition()}, 
      position);

    SmartDashboard.putBoolean("odometry reset pos", true);
  }


  public Pose2d getOdometry() {

    return odometry.getPoseMeters();
  }

  
  public void setDrive(double xFeetPerSecond, double yFeetPerSecond, double degreesPerSecond, boolean fieldRelative) {

    if (fieldRelative) {

      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xFeetPerSecond * Constants.FEET_TO_METERS,
          yFeetPerSecond * Constants.FEET_TO_METERS,
          degreesPerSecond * Constants.DEGREES_TO_RADIANS,
          Rotation2d.fromDegrees(-getGyroAngle())); // negative because gyro reads differently than wpilib

    } else {

      speeds = new ChassisSpeeds(
          xFeetPerSecond * Constants.FEET_TO_METERS,
          yFeetPerSecond * Constants.FEET_TO_METERS,
          degreesPerSecond * Constants.DEGREES_TO_RADIANS);
    }

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_MODULE_SPEED); // sets module max speed

    setModuleStates(moduleStates);
  }

  public void setDriveChassisSpeed(ChassisSpeeds chassisSpeeds){
    
    setDrive(chassisSpeeds.vxMetersPerSecond*Constants.METERS_TO_FEET,
    chassisSpeeds.vxMetersPerSecond*Constants.METERS_TO_FEET,
    chassisSpeeds.omegaRadiansPerSecond/Constants.DEGREES_TO_RADIANS,
    false);
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
    if (Drivebase == null) {
      Drivebase = new Drivebase();
    }
    return Drivebase;
  }

public void resetPose(Pose2d pose){}
public Pose2d poseSupplier(){
  return new Pose2d((double)0,(double)0,new Rotation2d((double)0));
}
public ChassisSpeeds getRobotRelativeSpeeds(){
  return new ChassisSpeeds();
}
public void configurePathPlanner(){

  AutoBuilder.configureHolonomic(
    this::poseSupplier, 
    this::resetPose, 
    this::getRobotRelativeSpeeds, 
    Drivebase::setDriveChassisSpeed,
    new HolonomicPathFollowerConfig(
      new PIDConstants(Constants.PATHPLANNER_DRIVE_KP, Constants.PATHPLANNER_DRIVE_KI, Constants.PATHPLANNER_DRIVE_KD),
    new PIDConstants(Constants.PATHPLANNER_TURN_KP, Constants.PATHPLANNER_TURN_KI, Constants.PATHPLANNER_TURN_KD),
    Constants.PATHPLANNER_MAX_METERS_PER_SECOND,Constants.PATHPLANNER_MAX_RADIANS_PER_SECOND,
    new ReplanningConfig()
    ), 
    () -> {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
      }  return false;
    }, 
    Drivebase);

}
  public Command followPathCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    return AutoBuilder.followPath(path);
  }
}
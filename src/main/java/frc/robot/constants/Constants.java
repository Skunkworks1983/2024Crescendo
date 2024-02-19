
package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static class SwerveModuleConstants {
    public int driveMotorId;
    public int turnMotorId;
    public int turnEncoderId;
    public double turnEncoderOffset;
    public String modulePosition;

    public SwerveModuleConstants(int driveMotorId, int turnMotorId, int turnEncoderId,
        double turnEncoderOffset, String modulePosition) {
      this.driveMotorId = driveMotorId;
      this.turnMotorId = turnMotorId;
      this.turnEncoderId = turnEncoderId;
      this.turnEncoderOffset = turnEncoderOffset;
      this.modulePosition = modulePosition;
    }
  }

  // Motor, Encoder, & Joystick IDS
  public class IDS {

    // Collector Motor IDS
    // stub
    public static final int COLLECTOR_MOTOR = 0;
    public static final int COLLECTOR_PIVOT_MOTOR = 0;

    // Indexer Motor IDS
    // stub
    public static final int INDEXER_MOTOR = 0;

    // Climber Motor IDS
    // stub
    public static final int CLIMBER_MOTOR_1 = 0;
    public static final int CLIMBER_MOTOR_2 = 0;

    // Shooter Motor IDS
    public static final int SHOOT_MOTOR = 0;
    public static final int SHOOTER_PIVOT_MOTOR = 0;
    public static final int SHOOTER_INDEXER_MOTOR = 0;
    public static final int NOTE_BREAK = 0;
    public static final int SHOOTER_PIVOT_MOTOR_FORWARD_LIMIT_SWITCH = 0;
    public static final int SHOOTER_PIVOT_MOTOR_REVERSE_LIMIT_SWITCH = 0;


    // Joystick Ids
    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
    public static final int BUTTON_STICK = 2;
    public static final int TARGETING_BUTTION = 11;
  }

  public class Collector {
    // Collector Motor IDS
    // stub
    public static final int COLLECTOR_MOTOR = 0; // TODO:set this!
    public static final int COLLECTOR_PIVOT_MOTOR = 0; // TODO:set this!
    public static final int INTAKE_GEAR_RATIO = 25;
    public static final double INTAKE_ROLLER_DIAMETER = 0.0381; // meters
    public static final double PIVOT_GEAR_RATIO = 20;
    public static final double NOTE_INTAKE_SPEED = 0; // TODO:set this!
    public static final double COLLECTOR_FLOOR_POS = 0; // TODO:set this!
    public static final double COLLECTOR_STOW_POS = 0; // TODO:set this!
  }

  public class DrivebaseInfo {

    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;

    // aproxamation based on travel distance of trajectory. Value is in feet.
    public static final double WHEEL_DIAMETER = .33333333;
    public static final double REVS_PER_FOOT = DRIVE_MOTOR_GEAR_RATIO / (WHEEL_DIAMETER * Math.PI);

    // Module translations
    public static final double TRANSLATION_X = 0.9479166665; // feet
    public static final double TRANSLATION_Y = 0.9479166665; // feet

    public class ModuleConstants {

      public static final SwerveModuleConstants FRONT_LEFT_MODULE =
          new SwerveModuleConstants(3, 9, 4, 0.77856445312, "Front Left");

      public static final SwerveModuleConstants FRONT_RIGHT_MODULE =
          new SwerveModuleConstants(7, 11, 8, 0.12280273437, "Front Right");

      public static final SwerveModuleConstants BACK_LEFT_MODULE =
          new SwerveModuleConstants(1, 12, 2, 0.62231445312, "Back Left");

      public static final SwerveModuleConstants BACK_RIGHT_MODULE =
          new SwerveModuleConstants(5, 10, 6, 0.28784179687, "Back Right");
    }
  }

  public class Shooter {
    public static final double SHOOT_MOTOR_GEAR_RATIO = 6.75;
    public static final double SHOOT_PIVOT_GEAR_RATIO = 149.333333333;
    public static final double TICKS_PER_SHOOT_MOTOR_REV = 48;
    public static final double WHEEL_DIAMETER = 0.1016; // meters
    public static final double SHOOTER_ROTATIONS_PER_METER =
        SHOOT_MOTOR_GEAR_RATIO / (WHEEL_DIAMETER * Math.PI) * TICKS_PER_SHOOT_MOTOR_REV;
    // assuiming backwards on the robot is 0 and straight up is 90, double check messurements on
    // real robot
    public static final double PIVOT_MOTOR_ROTATIONS_TO_DEGREES = SHOOT_PIVOT_GEAR_RATIO / 360;
    public static final double SHOOTER_RESTING_POSITION_ROTATIONS = 27.8 * PIVOT_MOTOR_ROTATIONS_TO_DEGREES;
    public static final double SHOOTER_MAX_POSITION_ROTATIONS = 119.5 * PIVOT_MOTOR_ROTATIONS_TO_DEGREES;
    public static final double SHOOTER_RESTING_POSITION_DEGREES = 27.8;
    public static final double SHOOTER_MAX_POSITION_DEGREES = 119.5;
  }

  public class PIDControllers {

    public static final boolean SMART_PID_ACTIVE = true;

    public class TurnPID {
      // Turning Motor PID Constants
      // currently fairly fast but noisy.
      public static final double KP = .005;
      public static final double KI = 0;
      public static final double KD = 0.0004;
      public static final double KF = 0;
      public static final double PID_LOW_LIMIT = -.8;
      public static final double PID_HIGH_LIMIT = .8;
      public static final double TURN_PID_TOLERANCE = .5;

      public static final boolean SMART_PID_ACTIVE = true;
    }

    public class DrivePID {
      // Velocity Mode PID Constants
      public static final double KP = .25;
      public static final double KI = .003;
      public static final double KD = 0.001;
      public static final double KF = .11;

      public static final boolean SMART_PID_ACTIVE = true;
    }

    public class HeadingControlPID {
      public static final double KP = 9;
      public static final double KI = 0;
      public static final double KD = 0.04;

      public static final boolean SMART_PID_ACTIVE = false;
    }

    public class ShootingPID {
      public static final double KP = 0;
      public static final double KI = 0;
      public static final double KD = 0;
      public static final double KF = 0;

      public static final boolean SMART_PID_ACTIVE = false;
    }

    public class ShooterIndexerPID {
      public static final double KP = 0;
      public static final double KI = 0;
      public static final double KD = 0;
      public static final double KF = 0;

      public static final boolean SMART_PID_ACTIVE = false;
    }

    public class ShooterPivotPID {
      public static final double KP = 0;
      public static final double KI = 0;
      public static final double KD = 0;
      public static final double KF = 0;

      public static final boolean SMART_PID_ACTIVE = false;
    }

    public class CollectorIntakePID {
      public static final double KP = 0;
      public static final double KI = 0;
      public static final double KD = 0;
      public static final double FF = 0;

      public static final boolean SMART_PID_ACTIVE = true;
    }

    public class CollectorPivotPID {
      public static final double KP = 0;
      public static final double KI = 0;
      public static final double KD = 0;
      public static final double FF = 0;

      public static final boolean SMART_PID_ACTIVE = true;
    }
  }


  // Speed & Deadband
  public static final double X_JOY_DEADBAND = .1;
  public static final double Y_JOY_DEADBAND = .1;
  public static final double ROT_JOY_DEADBAND = .2;
  public static final double MAX_MODULE_SPEED = Units.feetToMeters(20);

  // Multiplying joystick output by this value in SwerveTeleop to get x and y feet per second.
  // 14.2 f/s was the max speed we could get in SwerveTeleop.
  // TODO: characterization to find true max speed.
  public static final double OI_DRIVE_SPEED_RATIO = 14.2;

  // Multiplying joystick output by this value in SwerveTeleop to get degrees per second.
  public static final double OI_TURN_SPEED_RATIO = 360;

  public static final double MAX_TRAJECTORY_SPEED = Units.feetToMeters(2.0);
  public static final double MAX_TRAJECTORY_ACCELERATION = Units.feetToMeters(30);
  public static final String CANIVORE_NAME = "Canivore_1";
  public static final Pose2d START_POSITION = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
  public static final double TIME_UNTIL_HEADING_CONTROL = 1; // seconds

  // The position that the targeting buttion will point at
  public static final double TARGETING_POSITION_X = 0;
  public static final double TARGETING_POSITION_Y = 0;

  // Width and length of the field. Converting from feet to meters.
  public static final double FIELD_X_LENGTH = Units.feetToMeters(54.2708333);
  public static final double FIELD_Y_LENGTH = Units.feetToMeters(26.9375);

  // Width and length of the robot with bumpers. Used for calculating start pose if pushed against
  // wall.
  public static final double WIDTH_WITH_BUMPER = Units.feetToMeters(1.416667);


  public class PhotonVision {
    public static final String PHOTON_CAMERA_NAME = "Arducam_OV9281_USB_Camera";
    public static final Transform3d ROBOT_TO_CAMERA =
        new Transform3d(Units.feetToMeters(-1.0), 0, Units.feetToMeters(0.666667),
            new Rotation3d(0, Units.degreesToRadians(40), Units.degreesToRadians(180)));

    // Multplying distance to target by this constant to get X and Y uncertainty when adding a
    // vision measurment.
    public static final double DISTANCE_UNCERTAINTY_PROPORTIONAL = .4;

    // Very high rotational uncertainty. Don't trust the vision measurement, instead trust the gyro.
    // TODO: Experiment with rotational output of vision
    public static final double ROTATIONAL_UNCERTAINTY = 99999999;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // pathplanner PID constants
  public class PathPlannerInfo {

    public static final double PATHPLANNER_DRIVE_KP = 7;
    public static final double PATHPLANNER_DRIVE_KD = .05;
    public static final double PATHPLANNER_DRIVE_KI = .0;
    public static final double PATHPLANNER_DRIVE_KF = .0;

    public static final double PATHPLANNER_TURN_KP = 8;
    public static final double PATHPLANNER_TURN_KD = .0;
    public static final double PATHPLANNER_TURN_KI = .0;
    public static final double PATHPLANNER_TURN_KF = .0;

    public static final double PATHPLANNER_MAX_METERS_PER_SECOND = 5;

    // distance from center to wheel
    public static final double PATHPLANNER_DRIVEBASE_RADIUS_METERS = 0.413;
  }
}

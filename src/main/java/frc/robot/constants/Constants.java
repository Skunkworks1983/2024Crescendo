
package frc.robot.constants;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
    // Climber Motor IDS
    // stub
    public static final int LEFT_CLIMBER_MOTOR = 14;
    public static final int RIGHT_CLIMBER_MOTOR = 13;


    // Shooter Motor IDS

    // Left Flywheel
    public static final int SHOOT_MOTOR1 = 37;

    // Right Flywheel
    public static final int SHOOT_MOTOR2 = 36;

    // Shooter Pivot
    public static final int SHOOTER_PIVOT_MOTOR = 15;
    public static final int SHOOTER_INDEXER_MOTOR = 35;
    public static final int SHOOTER_PIVOT_MOTOR_FORWARD_LIMIT_SWITCH = 0;
    public static final int SHOOTER_PIVOT_MOTOR_REVERSE_LIMIT_SWITCH = 8;
    public static final int SHOOTER_PIVOT_ENCODER_PIN_1 = 3;
    public static final int SHOOTER_PIVOT_ENCODER_PIN_2 = 4;


    // Other Shooter IDS
    public static final int NOTE_BREAK1 = 7;
    public static final int NOTE_BREAK2 = 6;

    // Joystick Ids
    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
    public static final int BUTTON_STICK = 2;


    // button Ids
    public static final int MANUAL_SWITCH = 2;
    public static final int SPEAKER_TARGETING_BUTTON = 1;
    public static final int INTERPOLATION_AIM = 10;
    public static final int AMP_TARGETING_BUTTON = 2;
    public static final int SET_ROBOT_RELATIVE = 5;
    public static final int SHOOTER_TO_PASS = 24;
    public static final int REVERSE_NOTE_BACKWARDS = 16;
    public static final int RESET_COLLECTOR = 8;


    public static final int SMART_AIM = 2;
    public static final int LINEAR_AIM = 3;
    public static final int SHOOT_WHEN_READY = 11;
    public static final int FLYWHEEL_SPINUP = 9;

    public static final int COLLECTOR_STOW = 15;
    public static final int COLLECTOR_DOWN = 13;

    public static final int SHOOTER_TO_AMP = 12;
    public static final int SHOOTER_TO_SPEAKER = 10;

    public static final int NOTE_FLOOR_TO_SHOOTER = 14;

    // Joystick IDs for the climber
    public static final int SMART_CLIMB = 7;
    public static final int MANUAL_LEFT_CLIMBER_UP = 6;
    public static final int MANUAL_LEFT_CLIMBER_DOWN = 5;
    public static final int MANUAL_RIGHT_CLIMBER_UP = 3;
    public static final int MANUAL_RIGHT_CLIMBER_DOWN = 4;

    // Button used to reset the gyro (used on both joysticks)
    public static final int RESET_GYRO_BUTTON = 3;

    // Intake from shooter
    public static final int SHOOTER_INTAKE = 23;
  }

  public class Collector {
    // Collector Motor IDS
    // stub
    public static final int TOP_INTAKE_MOTOR = 30;
    public static final int BOTTOM_INTAKE_MOTOR = 31;
    public static final int RIGHT_PIVOT_MOTOR = 32;
    public static final int LEFT_PIVOT_MOTOR = 34;
    public static final int INTAKE_GEAR_RATIO = 25;
    public static final double INTAKE_ROLLER_DIAMETER = 0.0381; // meters
    public static final double PIVOT_GEAR_RATIO = 20.0 * (42.0 / 24.0);
    public static final double NOTE_INTAKE_SPEED = 0; // TODO:set this!
    public static final double COLLECTOR_FLOOR_POS = 110;
    public static final double COLLECTOR_STOW_POS = 0; // TODO:set this!
    public static final double DEGREES_TO_PIVOT_MOTOR_ROTATIONS = PIVOT_GEAR_RATIO / 360;
    public static final double REVERSE_COLLECTOR_SPEED = -.5;

    // Max collector pivot motor current output.
    public static final int COLLECTOR_PIVOT_MAX_AMPS = 1;
    public static final double COLLECTOR_POS_TOLERANCE = 0; // TODO:set this!
    public static final double COLLECTOR_MANUAL_PERCENT_OUTPUT = 1;

    public static final int COLLECTOR_PIVOT_MOTOR_FORWARD_LIMIT_SWITCH = 1;
    public static final int COLLECTOR_PIVOT_MOTOR_REVERSE_LIMIT_SWITCH = 2;
    public static final double RESET_COLLECTOR_PIVOT_PERCENT_OUTPUT_SPEED = .03;
  }

  public class DrivebaseInfo {

    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double CORRECTIVE_SCALE = 0.1;

    // aproxamation based on travel distance of trajectory. Value is in feet.
    public static final double WHEEL_DIAMETER = 0.32516666666;
    public static final double REVS_PER_FOOT = DRIVE_MOTOR_GEAR_RATIO / (WHEEL_DIAMETER * Math.PI);

    // Module translations feet
    public static final double TRANSLATION_X = 0.925;
    public static final double TRANSLATION_Y = 0.8041666;

    public class ModuleConstants {

      public static final SwerveModuleConstants FRONT_LEFT_MODULE =
          new SwerveModuleConstants(16, 18, 17, 0.311035, "Front Left");

      public static final SwerveModuleConstants FRONT_RIGHT_MODULE =
          new SwerveModuleConstants(10, 12, 11, -0.415283, "Front Right");

      public static final SwerveModuleConstants BACK_LEFT_MODULE =
          new SwerveModuleConstants(25, 23, 24, -0.205566, "Back Left");

      public static final SwerveModuleConstants BACK_RIGHT_MODULE =
          new SwerveModuleConstants(22, 20, 21, 0.308838, "Back Right");
    }
  }

  public class ShooterInterpolationConstants {

    // TODO: retune MINIMUM_SPEED_TO_RE_AIM and NUMBER_OF_TICKS_GOING_TO_FAST_TO_RE_AIM
    public static final double MINIMUM_SPEED_TO_RE_AIM = .045;
    public static final double NUMBER_OF_TICKS_GOING_TO_FAST_TO_RE_AIM = 3;

    public static final double A_COEFFICIENT = 1.25090821 * Math.pow(10,2);
    public static final double B_COEFFICIENT = -1.57056835 * Math.pow(10,1);
    public static final double C_COEFFICIENT = -3.87513617 * Math.pow(10,1);
    public static final double D_COEFFICIENT = 3.54081199;
    public static final double E_COEFFICIENT = 3.49116406;
    public static final double F_COEFFICIENT = 8.47042804;
    public static final double G_COEFFICIENT = -7.44807799 * Math.pow(10,-1);
    public static final double H_COEFFICIENT = 4.82389371 * Math.pow(10,-6);
    public static final double I_COEFFICIENT = -7.63099765 * Math.pow(10,-1);
    public static final double J_COEFFICIENT = -6.09079745 * Math.pow(10,-6);;
  }

  public class Shooter {

    public static final double AUTOAIMING_OFFSET = -3.25;
    // 10 is maximum because 2^10=1024=number of ticks in motor and each time, search space is cut
    // in half.
    public static final int ANGLE_SEARCH_DEPTH = 10;

    // in m/s
    public static final double BASE_FLYWHEEL_AUTOAIMING_SPEED = 5;
    public static final double BASE_FLYWHEEL_AUTOAIMING_SPEED_PER_METER_DISTANCE = 1;

    // 1 indicates aiming at the highest point. 0 indicates aiming at the lowest point. .3 would
    // indicate aiming 30% of the total diffrence in angle away from the lowest possible angle.
    // Shoots slightly low because notes that hit the lower edge can bounce in but notes that hit
    // the hood have no way of getting in.
    public static final double AUTO_AIM_ROTATION_RATIO = .3;
    public static final double TEMP_SHOOT_FLYWHEEL_SPEED_RPS = 25;
    public static final double SHOOT_MOTOR_GEAR_RATIO = 1;
    public static final double INDEXER_MOTOR_GEAR_RATIO = 16;
    public static final double SHOOT_PIVOT_GEAR_RATIO_ENCODER = 12.0 / 30.0;
    public static final double TICKS_PER_SHOOT_MOTOR_REV = 48;
    public static final double TICKS_PER_INDEXER_MOTOR_REV = 48;
    public static final double TICKS_PER_PIVOT_MOTOR_REV_ENCODER = 2048;
    public static final double FLYWHEEL_DIAMETER = 0.1016;
    public static final double ROLLER_DIAMETER = Units.inchesToMeters(1.25);
    public static final double SHOOTER_ROTATIONS_PER_METER =
        SHOOT_MOTOR_GEAR_RATIO / (FLYWHEEL_DIAMETER * Math.PI);
    public static final double INDEXER_ROTATIONS_PER_METER =
        INDEXER_MOTOR_GEAR_RATIO / (ROLLER_DIAMETER * Math.PI);
    // used for the stow command, a variable for how far away from stow it does a slow speed
    public static final double PIVOT_STOW_OFFSET = 5;
    // assuming backwards on the robot is 0 and straight up is 180, double check
    // messurements on
    // real robot
    public static final double PIVOT_MOTOR_TICKS_TO_DEGREES =
        (1 / TICKS_PER_PIVOT_MOTOR_REV_ENCODER) * SHOOT_PIVOT_GEAR_RATIO_ENCODER * 360;
    public static final double SHOOTER_RESTING_POSITION_TICKS = 27.8 / PIVOT_MOTOR_TICKS_TO_DEGREES;
    public static final double SHOOTER_MAX_POSITION_TICKS = 119.5 / PIVOT_MOTOR_TICKS_TO_DEGREES;
    public static final Rotation2d SHOOTER_RESTING_POSITION =
        new Rotation2d(Units.degreesToRadians(27.8));
    public static final Rotation2d SHOOTER_MAX_POSITION =
        new Rotation2d(Units.degreesToRadians(119.5));
    public static final double SHOOTER_PIVOT_SLOW_SPEED = 0.1;
    public static final double SHOOTER_PIVOT_FAST_SPEED = -0.45;
    public static final double SHOOTER_PIVOT_SLOW_DOWN_LOAD_PEICE = -0.05;

    public static final double SHOOTER_MANUAL_INDEXER_PERCENT_OUTPUT = 1;
    public static final double SHOOTER_MANUAL_INDEXER_PERCENT_OUTPUT_SLOW = 0.1;
    public static final double SHOOTER_MANUAL_INDEXER_BACKWARDS = -.5;
    public static final double SHOOTER_MANUAL_PIVOT_PERCENT_OUTPUT = 0.01;

    // z is the distance from the ground to the pivot.
    public static final Translation3d ROBOT_RELATIVE_PIVOT_POSITION =
        new Translation3d(Units.inchesToMeters(11.976378), 0, Units.inchesToMeters(24.586839));

    // TODO: find more exact value
    public static final double PIVOT_TO_FLYWHEEL_DISTANCE = Units.inchesToMeters(7);


    // Set Flywheel speeds for Shooter in m/s
    public static final double STOW_FLYWHEEL_SPEED = 21;
    public static final double AMP_FLYWHEEL_SPEED = 20;
    public static final double DEFUALT_SPEAKER_FLYWHEEL_SPEED = 27.0;
    public static final double PODIUM_FLYWHEEL_SPEED = 27;
    public static final double SOURCE_FLYWHEEL_SPEED = -3;
    public static final double PASS_FLYWHEEL_SPEED = 23;

    public static final double PODIUM_ANGLE_DEGREES = 50;
    public static final double PASS_ANGLE_DEGREES = 50;

    // Indexer speeds for the robot:
    public static final double LOADING_INDEXER_SPEED = 1;
    public static final double BEAMBREAK1_INDEXER_SPEED = 0.5;
    public static final double SHOOTING_INDEXER_SPEED = 1;

    // maximum error for flywheel spinup to consider shooting
    public static final double MAX_FLYWHEEL_ERROR = 0.4;

    // Max shooter pivot motor current output.
    public static final double SHOOTER_PIVOT_MAX_AMPS = 5;

    public static final double SHOOTER_PIVOT_TESTING_ANGLE = 90;
    public static final double SHOOTER_PIVOT_TOLARENCE_DEGREES = 0.4;
    public static final double SHOOTER_PIVOT_TUNING_SUCCESSFUL_TICKS = 5;

    public static final double SHOOTER_FLYWHEEL_TUNING_SUCCESSFUL_TICKS = 15;

    // In the ShootWhenReady command, the shooter pivot must be at the setpoint for this number of
    // ticks before allowed to shoot.
    public static final double SHOOTER_ANGLE_WAIT_TICKS = 3;

    // Setting the tolerance on the shooter pivot PID controller to this number.
    public static final double SHOOTER_PIVOT_PID_TOLERANCE = 1.0;

    public static final double SHOOTER_STOW_WAIT_TICKS = 3;
    public static final double SHOOT_WHEN_READY_SECONDS_BEFORE_SHOOT = 5;
  }
  public class AutoShooting {
    public static double WNOTE1_ANGLE = 53;
    public static double WNOTE2_ANGLE = 45;
    public static double WNOTE3_ANGLE = 50;
  
    
  }

  public class PIDControllers {

    public static final boolean SMART_PID_ACTIVE = false;

    public class TurnPID {
      // Turning Motor PID Constants
      public static final double KP = .0145;
      public static final double KI = 0;
      public static final double KD = .00017;
      public static final double KF = 0;
      public static final double PID_LOW_LIMIT = -.8;
      public static final double PID_HIGH_LIMIT = .8;
      public static final double TURN_PID_TOLERANCE = .5;

      public static final boolean SMART_PID_ACTIVE = false;
    }

    public class DrivePID {
      // Velocity Mode PID Constants
      public static final double KP = 0.01;
      public static final double KI = .000;
      public static final double KD = 0.000;
      public static final double KF = .1160;

      public static final boolean SMART_PID_ACTIVE = false;
    }

    public class HeadingControlPID {
      public static final double KP = 9;
      public static final double KI = 0;
      public static final double KD = 0.04;

      public static final boolean SMART_PID_ACTIVE = false;
    }

    public class ShootingPID {
      public static final double KP = 0.4;
      public static final double KI = 0;
      public static final double KD = 0;
      public static final double KF = 0.12;

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

      public static final double KP = .025;
      public static final double KI = 0;
      public static final double KD = 0.00025;

      public static final boolean SMART_PID_ACTIVE = false;
    }

    public class TopCollectorIntakePID {
      public static final double KP = 0;
      public static final double KI = 0;
      public static final double KD = 0;
      public static final double FF = 0;

      public static final boolean SMART_PID_ACTIVE = false;
    }

    public class CollectorPivotPID {

      // Setting low value for testing.
      public static final double KP = 0.035;
      public static final double KI = 0;
      public static final double KD = 0;
      public static final double FF = 0.005;

      // IN DEGREES
      public static final double MAX_VELOCITY = 500;
      public static final double MAX_ACCELERATION = 1100;


      public static final boolean SMART_PID_ACTIVE = false;
    }

    public class ClimberPID {

      public static final double CLIMBER_KP = 0.4;
      public static final double CLIMBER_KI = 0;
      public static final double CLIMBER_KD = 0;
      public static final double CLIMBER_KF = 0;

      public static final boolean SMART_PID_ACTIVE = false;
    }
  }

  // Speed & Deadband
  public static final double X_JOY_DEADBAND = .1;
  public static final double Y_JOY_DEADBAND = .1;
  public static final double ROT_JOY_DEADBAND = .2;
  public static final double MAX_MODULE_SPEED = Units.feetToMeters(14.76);

  // Multiplying joystick output by this value in SwerveTeleop to get x and y feet
  // per second.
  // 14.2 f/s was the max speed we could get in SwerveTeleop.
  // TODO: characterization to find true max speed.
  public static final double OI_DRIVE_SPEED_RATIO = 14.76;

  // Multiplying joystick output by this value in SwerveTeleop to get degrees per
  // second.
  public static final double OI_TURN_SPEED_RATIO = 360;

  public static final double TURNING_SPEED_CAP = 270;

  public static final double MAX_TRAJECTORY_SPEED = Units.feetToMeters(2.0);
  public static final double MAX_TRAJECTORY_ACCELERATION = Units.feetToMeters(30);
  public static final String CANIVORE_NAME = "1983 Comp Drivebase";
  public static final Pose2d START_POSITION = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
  public static final double TIME_UNTIL_HEADING_CONTROL = 1; // seconds

  // The position that the targeting buttion will point at
  public static final double TARGETING_POSITION_X = 0;
  public static final double TARGETING_POSITION_Y = 0;

  // Width and length of the field. Converting from feet to meters.
  public static final double FIELD_X_LENGTH = Units.feetToMeters(54.2708333);
  public static final double FIELD_Y_LENGTH = Units.feetToMeters(26.9375);

  // Width and length of the robot with bumpers. Used for calculating start pose
  // if pushed against
  // wall.
  public static final double WIDTH_WITH_BUMPER = Units.feetToMeters(1.416667);

  // m/s^2
  public static final double ACCELERATION_DUE_TO_GRAVITY = 9.808;


  public class GyroCrashDetection {
    public static final double GYRO_NOISE_TOLERANCE = .0000000000000000000000000001;
    public static final int GYRO_MEASURMENTS_LIST_SIZE = 30;
    public static final int COUNT_STEP_NUMBER = 3;
    public static final double HAS_DIED_TIME_LIMIT_IF_ENABLED = 160;
    public static final double HAS_DIED_TIME_LIMIT_IF_DISABLED = 250;
  }

  public class PhotonVision {
    public static final String CAMERA_2_NAME = "Side";
    public static final String CAMERA_1_NAME = "Forward";

    // Forwards facing camera
    public static final Transform3d ROBOT_TO_CAMERA_1 = new Transform3d(Units.inchesToMeters(8.256)+.37,
    Units.inchesToMeters(0.901 + .875), Units.inchesToMeters(13.5),//10.727 + 2.088 - 0.175 was 12.64
    new Rotation3d(0, Units.degreesToRadians(12.0), Units.degreesToRadians(-2)));

    // Sideways facing camera
    public static final Transform3d ROBOT_TO_CAMERA_2 =
        new Transform3d(Units.inchesToMeters(6.261 - .875 + 0.5), Units.inchesToMeters(5.901),
            Units.inchesToMeters(11.077 + 2.088 - 0.175 - 0.2),
            new Rotation3d(0, Units.degreesToRadians(13.0), Units.degreesToRadians(90)));


    // Multplying distance to target by this constant to get X and Y uncertainty
    // when adding a
    // vision measurment.
    public static final double DISTANCE_UNCERTAINTY_PROPORTIONAL = .75;

    // Multiplying distance to target by this constant to get rotational uncertainty
    // when adding a
    // vision measurement.
    public static final double ROTATIONAL_UNCERTAINTY_PROPORTIONAL = 2;

    // Used for a SmartDashboard boolean that tells you if the camera is plugged in.
    public static final String CAMERA_STATUS_BOOLEAN = "CAMERA PLUGGED IN";

    // Meters
    public static final double APRILTAG_DISTANCE_CUTOFF = 3;
  }

  public class Targeting {

    public enum FieldTarget {
      // SPEAKER uses middle part of goal for z value.
      SPEAKER(new Translation3d(-0.1, Units.feetToMeters(18.520833) -.1,
          Units.feetToMeters(7.2) + 0.43)), SPEAKER_LOWEST_GOAL_PART(
              new Translation3d(SPEAKER.get().get().getX(), SPEAKER.get().get().getY(),
                  Units.feetToMeters(6.0))), AMP(
                      new Translation3d(Units.feetToMeters(6.0), Units.feetToMeters(999999999),
                          0)), SPEAKER_HOOD(
                              new Translation3d(.47, SPEAKER.get().get().getY(),
                                  Units.feetToMeters(7))), NONE();

      Translation3d target;

      FieldTarget(Translation3d target) {
        this.target = target;
      }

      /** Overload of TargetingPoint constructor used for NONE */
      FieldTarget() {
        target = null;
      }

      /**
       * Returns the Optional<Translation2d> value of the target. If the target is NONE, this will
       * return Optional.empty().
       */
      public Optional<Translation3d> get() {
        if (target == null) {
          return Optional.empty();
        } else {
          return Optional.of(target);
        }
      }
    }

    public static double distanceFromHoodToSpeaker =
        FieldTarget.SPEAKER_HOOD.get().get().getX() - FieldTarget.SPEAKER.get().get().getX();
  }

  // pathplanner PID constants
  public class PathPlannerInfo {

    public static final double PATHPLANNER_DRIVE_KP = 6.5;
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

  public class ClimberConstants {

    public static final double GEAR_RATIO = 2.0 * 20.0 * 10.0 / 14.0;

    // Meters
    public static final double CLIMBER_CORD_CYLINDER_DIAMETER = 0.031369;
    public static final double CLIMBER_METERS_TO_MOTOR_ROTATIONS =
        1 / ((Math.PI * CLIMBER_CORD_CYLINDER_DIAMETER) / GEAR_RATIO);

    public static final double MAX_POSITION = .47 * CLIMBER_METERS_TO_MOTOR_ROTATIONS;
    public static final double MIN_POSITION = .01 * CLIMBER_METERS_TO_MOTOR_ROTATIONS;

    // Tolerance when checking if the climber is at a position setpoint. NOTE: This
    // tolerance is in motor rotations, NOT cm.
    public static final double CLIMBER_MOTOR_POSITION_TOLERANCE = .5;

    // CLIMBER_CHAIN_TORQUE used to be 10. Setting it lower to see if it helps keep robot level.
    public static final double CLIMBER_CHAIN_TORQUE = 15;
    public static final double BASE_PULL_SPEED = -.75;
    public static final double ROLL_DEGREES_TO_OUTPUT = 100;

    public enum ClimbModule {
      LEFT, RIGHT;
    }
  }

  public class IndexerConstants {
    public static final int INDEXER_MOTOR = 33;
    public static final int INDEXER_BEAM_BREAK = 9;
    public static final int INDEXER_WHEEL_DIAMETER = 0;
    public static final double INDEXER_GEAR_RATIO = 16;
    public static final double INDEXER_MOTOR_KP = 0;
    public static final double INDEXER_MOTOR_KI = 0;
    public static final double INDEXER_MOTOR_KD = 0;
    public static final double INDEXER_MOTOR_KF = 0;
    public static final double INDEXER_SPEED = 1;
    public static final double REVERSE_INDEXER_SPEED_PERCENT_OUTPUT = -.5;
    public static final boolean SET_INDEXER_SMART_PID = true;
    public static final double INDEXER_MANUAL_PERCENT_OUTPUT = 1;
  }

  public static final double SECONDS_TO_MINUTES = 1.0 / 60.0;

  public static final double DRIVEBASE_TUNING_TURNING_ANGLE_ONE = 90;
  public static final double DRIVEBASE_TUNING_TURNING_ANGLE_TWO = 180;
  public static final double DRIVEBASE_TUNING_TURNING_TOLERANCE = 0.5;
  public static final double DRIVEBASE_TUNING_TURNING_TOLERANCE_POWER = 0.1;
  public static final double DRIVEBASE_TUNING_TICK_COUNT = 5;

  public static final double DRIVEBASE_TUNING_VELOCITY_TOLERANCE = 0.05;
}

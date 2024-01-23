

package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class Constants{



  // Motor, Encoder, & Joystick IDS
  public class IDS{

    // Drive Motor IDS
    public static final int RIGHT_FRONT_DRIVE = 7;
    public static final int RIGHT_BACK_DRIVE = 5;
    public static final int LEFT_FRONT_DRIVE = 3;
    public static final int LEFT_BACK_DRIVE = 1;

    // Turn Motor IDS
    public static final int RIGHT_FRONT_TURN = 11;
    public static final int RIGHT_BACK_TURN = 10;
    public static final int LEFT_FRONT_TURN = 9;
    public static final int LEFT_BACK_TURN = 12;

      // Turning Encoders
    public static final int RIGHT_FRONT_CAN_CODER = 8;
    public static final int RIGHT_BACK_CAN_CODER = 6;
    public static final int LEFT_FRONT_CAN_CODER = 4;
    public static final int LEFT_BACK_CAN_CODER = 2;

    // Joystick Ids
    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
    public static final int BUTTON_STICK = 2;
  }



  public class DrivebaseInfo{

    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double WHEEL_DIAMETER = .33333333;
    
    //public static final int TALON_TICKS_PER_MOTOR_REV = 2048;
    public static final double REVS_PER_FOOT = DRIVE_MOTOR_GEAR_RATIO / (WHEEL_DIAMETER * Math.PI);

    // Module translations
    public static final double TRANSLATION_X = 1.895833333; // set this
    public static final double TRANSLATION_Y = 1.895833333; // set this

    // Can coder offsets
    public static final double FRONT_LEFT_OFFSET = 0.778564453125;
    public static final double FRONT_RIGHT_OFFSET = 0.122802734375;
    public static final double BACK_LEFT_OFFSET = 0.622314453125;
    public static final double BACK_RIGHT_OFFSET = 0.287841796875;
  }

  public class PathPlannerInfo{

  //pathplanner PID constants

  public static final double PATHPLANNER_DRIVE_KP = 2;
  public static final double PATHPLANNER_DRIVE_KD=.0;
  public static final double PATHPLANNER_DRIVE_KI=.0;
  public static final double PATHPLANNER_DRIVE_KF=.0;

  public static final double PATHPLANNER_TURN_KP = 1;
  public static final double PATHPLANNER_TURN_KD=.0;
  public static final double PATHPLANNER_TURN_KI=.0;
  public static final double PATHPLANNER_TURN_KF=.0;

  public static final double PATHPLANNER_MAX_METERS_PER_SECOND=1;
  public static final double PATHPLANNER_MAX_RADIANS_PER_SECOND=3.5;

  }



  public class PIDControllers{

    public class TurnPID{

      // Turning Motor PID Constants
      public static final double KP = .005;
      public static final double KI = 0;
      public static final double KD = 0;//.003;
      public static final double PID_LOW_LIMIT = -.8;
      public static final double PID_HIGH_LIMIT = .8;
      public static final double TURN_PID_TOLERANCE = 2;
    }

    public class DrivePID{

      // Velocity Mode PID Constants
      public static final double KP = .25;
      public static final double KI = 0;
      public static final double KD = 0; // used to be 0
      public static final double KF = 0;
    }

    public class HeadingControlPID{

      public static final double KP = 0.9;
      public static final double KI = 0;
      public static final double KD = 0;

    }
  }

  
  // Speed & Deadband
  public static final double X_JOY_DEADBAND = .1;
  public static final double Y_JOY_DEADBAND = .1;
  public static final double ROT_JOY_DEADBAND = .2;
  public static final double MAX_MODULE_SPEED = Units.feetToMeters(20);
  public static final double OI_DRIVE_SPEED_RATIO = 7.0; // 7.0 is slow
  public static final double OI_TURN_SPEED_RATIO = 360;  // max turn speed is 360 degrees per second
  public static final double MAX_TRAJECTORY_SPEED = Units.feetToMeters(2.0);
  public static final double MAX_TRAJECTORY_ACCELERATION = Units.feetToMeters(30);
  public static final String CANIVORE_NAME = "Canivore_1";

  // Field dimensions
  public static final double FIELD_X_LENGTH = 26.291667; // feet
  public static final double FIELD_Y_LENGTH = 54.2708333; // feet

  // swerve test directions
  public static enum DIRECTION {
    X,
    Y
  }
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
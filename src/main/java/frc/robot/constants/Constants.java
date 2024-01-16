

package frc.robot.constants;




public class Constants{

  // Motor, Encoder, & Joystick IDS

  // Drive Motors
  public static final int RIGHT_FRONT_DRIVE = 7;
  public static final int RIGHT_BACK_DRIVE = 5;
  public static final int LEFT_FRONT_DRIVE = 3;
  public static final int LEFT_BACK_DRIVE = 1;

  // Turn Motors
  public static final int RIGHT_FRONT_TURN = 11;
  public static final int RIGHT_BACK_TURN = 10;
  public static final int LEFT_FRONT_TURN = 9;
  public static final int LEFT_BACK_TURN = 12;

  // Driving Encoders
  public static final double TALON_GEAR_RATIO = 6.75;
  public static final double WHEEL_DIAMETER = .33333333;
  //public static final int TALON_TICKS_PER_MOTOR_REV = 2048;
  public static final double REVS_PER_FOOT = TALON_GEAR_RATIO / WHEEL_DIAMETER * Math.PI;

  // Turning Encoders
  public static final int RIGHT_FRONT_CAN_CODER = 8;
  public static final int RIGHT_BACK_CAN_CODER = 6;
  public static final int LEFT_FRONT_CAN_CODER = 4;
  public static final int LEFT_BACK_CAN_CODER = 2;

  // Joystick Ids
  public static final int LEFT_JOYSTICK = 0;
  public static final int RIGHT_JOYSTICK = 1;


  // Turning Motor PID Constants
  public static final double TURN_KP = .005;
  public static final double TURN_KD = 0;//.003;
  public static final double TURN_KI = 0;
  public static final double TURN_PID_LOW_LIMIT = -.8;
  public static final double TURN_PID_HIGH_LIMIT = .8;

  // Velocity Mode PID Constants
  public static final double DRIVE_KP = .1;
  public static final double DRIVE_KD = .02; // used to be 0
  public static final double DRIVE_KI = 0;
  public static final double DRIVE_KF = 0;

  // Module translations
  public static final double TRANSLATION_X = 1.895833333; // set this
  public static final double TRANSLATION_Y = 1.895833333; // set this

  // Conversions
  public static final double METERS_TO_FEET = 3.28084;
  public static final double FEET_TO_METERS = 1 / 3.28084;
  public static final double DEGREES_TO_RADIANS = 1 / 57.2958;
  public static final double RADIANS_TO_DEGREES = 57.2958;

  // Can coder offsets
  public static final double FRONT_LEFT_OFFSET = 190.283203125; // set this
  public static final double FRONT_RIGHT_OFFSET = 314.208984375; // set this
  public static final double BACK_LEFT_OFFSET = 134.033203125; // set this
  public static final double BACK_RIGHT_OFFSET = 13.623046875; // set this

  // Speeds
  public static final double MAX_MODULE_SPEED = 20 * FEET_TO_METERS; // used to be 10 - max speed that the module can go
  public static final double OI_DRIVE_SPEED_RATIO = 7.0; // max speed input is 15 fps in direction -- FAST
  public static final double OI_TURN_SPEED_RATIO = 360;  // max turn input in 360 degrees per second
  public static final double MAX_TRAJECTORY_SPEED = 2.0 * FEET_TO_METERS; // max is 10 feet per second trajectory
  public static final double MAX_TRAJECTORY_ACCELERATION = 30 * FEET_TO_METERS; // max acceleration is 10 fps squared

  // Field dimensions
  public static final double FIELD_X_LENGTH = 26.291667; // feet
  public static final double FIELD_Y_LENGTH = 54.2708333; // feet
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
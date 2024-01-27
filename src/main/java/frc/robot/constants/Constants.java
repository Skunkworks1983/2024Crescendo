

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

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
  public static final double WHEEL_DIAMETER = .33333333; // feet
  public static final double REVS_PER_FOOT = TALON_GEAR_RATIO / (WHEEL_DIAMETER * Math.PI);

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
  public static final double TURN_KI = 0;
  public static final double TURN_KD = 0;
  public static final double TURN_PID_LOW_LIMIT = -.8;
  public static final double TURN_PID_HIGH_LIMIT = .8;
  public static final double TURN_PID_TOLERANCE = 2;

  // Velocity Mode PID Constants
  public static final double DRIVE_KP = .25;
  public static final double DRIVE_KI = 0;
  public static final double DRIVE_KD = 0;
  public static final double DRIVE_KF = 0;

  // Module translations
  public static final double MODULE_TRANSLATION_X = 1.895833333; // feet
  public static final double MODULE_TRANSLATION_Y = 1.895833333; // feet

  // Conversions
  public static final double METERS_TO_FEET = 3.28084;
  public static final double FEET_TO_METERS = 1 / METERS_TO_FEET;
  public static final double RADIANS_TO_DEGREES = 57.2958;
  public static final double DEGREES_TO_RADIANS = 1 / RADIANS_TO_DEGREES;
  

  // Can coder offsets
  // Offsets are in rotations, 1 = 1 rotation
  public static final double FRONT_LEFT_OFFSET = 0.528564453125;
  public static final double FRONT_RIGHT_OFFSET = 0.872802734375;
  public static final double BACK_LEFT_OFFSET = 0.372314453125;
  public static final double BACK_RIGHT_OFFSET = 0.037841796875;

  // CANivore
  public static final String CANIVORE_NAME = "Canivore_1";

  // Speed & Deadband
  public static final double X_JOY_DEADBAND = .1;
  public static final double Y_JOY_DEADBAND = .1;
  public static final double ROT_JOY_DEADBAND = .2;
  public static final double MAX_MODULE_SPEED = 20 * FEET_TO_METERS;
  public static final double OI_DRIVE_SPEED_RATIO = 7.0; // 7.0 is slow
  public static final double OI_TURN_SPEED_RATIO = 360;  // max turn speed is 360 degrees per second
  public static final double MAX_TRAJECTORY_SPEED = 2.0 * FEET_TO_METERS;
  public static final double MAX_TRAJECTORY_ACCELERATION = 30 * FEET_TO_METERS;

  // Field dimensions
  public static final double FIELD_X_LENGTH = 26.291667; // feet
  public static final double FIELD_Y_LENGTH = 54.2708333; // feet

  // Photon Camera to Robot
  public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(0, 0, 0, new Rotation3d());
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
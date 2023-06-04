package frc.robot
import edu.wpi.first.math.util.Units
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics

class Constants {

    object OperatorConstants{

      const val kDriverControllerPort = 1;
  
    }

    object MK4SDS{

      const val WHEEL_DIAMETER = 0.1016 //4 in
      const val DRIVE_GEAR_RATIO = 1 / 8.14
      const val TURN_GEAR_RATIO = 1 / 12.8

      const val DRIVE_ROT_2_METER = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER
      const val TURN_ROT_2_RAD = TURN_GEAR_RATIO * 2 * Math.PI

      const val DRIVE_RPM_2_MPS = DRIVE_ROT_2_METER / 60;
      const val TURN_RPM_2_RADPS = TURN_ROT_2_RAD / 60;
    
    }


    object SwerveConstants{

    /* 
           MODULE ID ORDER
     front left       front right
     *- D1 T2 A0 -*   *- D3 T4 A1 -*

     back left        back right
     *- D7 T8 A3 -*   *- 5D 6T A2 -*
     */

    // IDs
    const val FRONT_LEFT_DRIVE_ID =  1;
    const val FRONT_RIGHT_DRIVE_ID = 3;
    const val BACK_LEFT_DRIVE_ID =   7;
    const val BACK_RIGHT_DRIVE_ID =  5;

    const val FRONT_LEFT_TURN_ID =  2;
    const val FRONT_RIGHT_TURN_ID = 4;
    const val BACK_LEFT_TURN_ID =   8;
    const val BACK_RIGHT_TURN_ID =  6;

    const val FRONT_LEFT_ABE_ID =  0;
    const val FRONT_RIGHT_ABE_ID = 1;
    const val BACK_LEFT_ABE_ID =   3;
    const val BACK_RIGHT_ABE_ID =  2;

    // Inversions
    const val FRONT_LEFT_DRIVE_INVERTED =  true;
    const val FRONT_RIGHT_DRIVE_INVERTED = true;
    const val BACK_LEFT_DRIVE_INVERTED =   true;
    const val BACK_RIGHT_DRIVE_INVERTED =  true;

    const val FRONT_LEFT_TURN_INVERTED =  true;
    const val FRONT_RIGHT_TURN_INVERTED = true;
    const val BACK_LEFT_TURN_INVERTED =   true;
    const val BACK_RIGHT_TURN_INVERTED =  true;

    // Numerics
    const val FRONT_LEFT_ABE_OFFSET_RAD =  1.658063;
    const val FRONT_RIGHT_ABE_OFFSET_RAD = 1.39626;
    const val BACK_LEFT_ABE_OFFSET_RAD =   1.04719;
    const val BACK_RIGHT_ABE_OFFSET_RAD =  -0.122173;

    const val PHYSICAL_MAX_VELOCITY_MS = 3.6;
    const val PHYSICAL_MAX_ANGULAR_RADS =  2 * 2 * Math.PI;

    const val TELEOP_MAX_VELOCITY_MS = PHYSICAL_MAX_VELOCITY_MS;
    const val TELEOP_MAX_ANGULAR_RADS = PHYSICAL_MAX_ANGULAR_RADS;

    const val TELEOP_SLEW_VELOCITY_UNITS = 8;
    const val TELEOP_SLEW_ANGULAR_UNITS = 2;

    // PIDs
    const val TURN_MODULE_PID_P = 0.5;
    const val TURN_MODULE_PID_I = 0.0;
    const val TURN_MODULE_PID_D = 0.0;

    const val ROBOT_GYRO_PID_P = 0.001;
    const val ROBOT_GYRO_PID_I = 0.0;
    const val ROBOT_GYRO_PID_D = 0.0;
    
    // Odometry
    const val TRACK_WIDTH = 0.53975 // Distance between RIGHT and LEFT wheel centers
    const val WHEEL_BASE = 0.53975; // Distance between FRONT and BACK wheel centers
    
    val DRIVE_KINEMATICS = SwerveDriveKinematics(
      Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),    // Front Left
      Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),   // Front Right
      Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),   // Back Left
      Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); // Back Right
      
    }


}

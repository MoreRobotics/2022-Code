// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int DRIVE_TRAIN_FRONT_LEFT_ID = 3;
    public static final int DRIVE_TRAIN_FRONT_RIGHT_ID = 4;
    public static final int DRIVE_TRAIN_REAR_LEFT_ID = 2;
    public static final int DRIVE_TRAIN_REAR_RIGHT_ID = 1;
    public static final int DRIVE_TRAIN_MID_LEFT_ID = 5;
    public static final int DRIVE_TRAIN_MID_RIGHT_ID = 0;
    

    //shooter motors (change later)
    public static final int SHOOTER_LEFT_ID = 6;
    public static final int SHOOTER_RIGHT_ID = 7;
    public static final int TURRET_MOTOR_ID = 8;

    //intake motor
    public static final int INTAKE_MOTOR_ID = 12;

    //climber motors
    public static final int CLIMBER_LEFT_MOTOR_ID = 13;
    public static final int CLIMBER_RIGHT_MOTOR_ID = 15;
    public static final int CLIMBER_MID_MOTOR_ID = 14;
    public static final int CLIMBER_SLOT_INDEX_ID = 0;

    //transporter motors
    public static final int TRANSPORTER_MOTOR_LEFT = 9;
    public static final int TRANSPORTER_MOTOR_RIGHT = 10;
    public static final int TOWER_MOTOR_ID = 11;

    //controllers
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final double EDGES_PER_REVOLUTION = 21448.15;
    //public static final double EDGES_PER_REVOLUTION = 2048;
    public static final double WHEEL_DIAMETER = .152;

    //Shooter
    public static final int kTimeoutMs = 30;
    public static final double SHOOTER_SPEED = 1;
    public static final double ENCODER_UNITS_TO_DEGREES = 2048.0/360.0 * 10;
    public static final int SHOOTER_SLOT_INDEX_ID = 0;
    public static final int SHOOTER_TARGET_RPM = 4000;
    public static final double RPM_TO_ENCODER_UNITS_PER_100_MS = .1 * 2048.0 / 60.0;

    //Vision
    public static final double CAMERA_HEIGHT_METERS = 0.65;
    public static final double TARGET_HEIGHT_METERS = 2.64;
    public static final double CAMERA_PITCH_RADIANS = -Math.PI/60;
    public static final double GOAL_RANGE_METERS = 2;
    
    //Ranges
    public static final int RANGE1 = 5;
    public static final int RANGE2 = 10;
    public static final int RANGE3 = 15;
    public static final int RANGE4 = 20;

    //Shooter hood
    public static final int ACTUATOR1_PORT =  8;
    public static final int ACTUATOR2_PORT = 9;
    
    public static final int ACTUATOR_SPEED = 32;
    public static final int ACTUATOR_LENGTH = 118;


    //Speeds
    public static final double INTAKE_SPEED = 1.0;
    public static final double TRANSPORTER_SPEED = 1.0;

    public static final Gains kGains_Shooter_Velocity = new Gains(0.2, 0.0, 0.0, 1023.0/53000.0, 300, 1.0);
    public static final Gains kGains_Climber_Rotation_Speed = new Gains(0.1, 0.0, 0.0, 1023.0/8192.0, 300, 1.0);

}

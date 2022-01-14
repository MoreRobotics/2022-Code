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
    public static final int DRIVE_TRAIN_REAR_RIGHT_ID = 5;

    //shooter motors (change later)
    public static final int SHOOTER_LEFT_ID = -1;
    public static final int SHOOTER_RIGHT_ID = -1;
    public static final int TURRET_MOTOR_ID = -1;

    //intake motor
    public static final int INTAKE_MOTOR_ID = -1;


    //transporter motor
    public static final int TRANSPORTER_MOTOR_ID = -1;

    //controllers
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final double EDGES_PER_REVOLUTION = 21448.15;
    //public static final double EDGES_PER_REVOLUTION = 2048;
    public static final double WHEEL_DIAMETER = .152;

    //Shooter
    public static final int kTimeoutMs = 30;
    public static final double SHOOTER_SPEED = 1;
    public static final double ENCODER_UNITS_TO_DEGREES = 4096.0/360.0 * 10;

    //Speeds
    public static final double INTAKE_SPEED = 1.0;
    public static final double TRANSPORTER_SPEED = 1.0;
}

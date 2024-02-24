// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //PC Specs: 26.5 in

  //Maybe need maxVolts later?

  //PID or feedforward? BOTH!

  //Test Swerve Auto IDs and stuff
  //mod 3: FL
  /* 
  public static final int mod0DriveMotor = 59;        //59    //10
  public static final int mod0TurningMotor = 55;      //55    //11
  public static final int mod0CANCoder = 7;           //3   //2
  public static final Rotation2d mod0TurningOffset = Rotation2d.fromDegrees(226.0);
  */
  //mod 0: FR
  /* 
  public static final int mod1DriveMotor = 54;       //58    //9
  public static final int mod1TurningMotor = 58;       //54    //8
  public static final int mod1CANCoder = 5;       //0       //0
  public static final Rotation2d mod1TurningOffset = Rotation2d.fromDegrees(67.3);
  */
  //mod 2: BL
  /* 
  public static final int mod2DriveMotor = 53;        //53    //12
  public static final int mod2TurningMotor = 61;        //61    //13
  public static final int mod2CANCoder = 4;         //2       //1
  public static final Rotation2d mod2TurningOffset = Rotation2d.fromDegrees(103.8);
*/
//mod 1: BR
/* 
  public static final int mod3DriveMotor = 36;       //36    //9
  public static final int mod3TurningMotor = 52;       //52    //8
  public static final int mod3CANCoder = 6;       //1       //3
  public static final Rotation2d mod3TurningOffset = Rotation2d.fromDegrees(271.2);
  */

  //NOTE: RENAME MOD NUMBERS TO POSITION ON BOT!!!!

  //EVERYTHING IS FLIP-FLOPPED HORIZONTALLY

  //Bevel gears should face towards the right when back if lifted up

  //Question marked module numbers are what to use for readouts

  //module 3 constants: Front Left    Module 0???                //MG    //PC

  public static final int mod0DriveMotor = 37;        //59    //10
  public static final int mod0TurningMotor = 60;      //55    //11
  public static final int mod0CANCoder = 3;           //3   //2
  public static final Rotation2d mod0TurningOffset = Rotation2d.fromDegrees(148.2);

  //module 0 constants: Front Right Module 1????
  public static final int mod1DriveMotor = 39;       //58    //9
  public static final int mod1TurningMotor = 32;       //54    //8
  public static final int mod1CANCoder = 0;       //0       //0
  public static final Rotation2d mod1TurningOffset = Rotation2d.fromDegrees(156.0);


  //module 2 constants: Back Left
  public static final int mod2DriveMotor = 45;        //53    //12
  public static final int mod2TurningMotor = 43;        //61    //13
  public static final int mod2CANCoder = 2;         //2       //1
  public static final Rotation2d mod2TurningOffset = Rotation2d.fromDegrees(166.1);


  //module 1 constants: Back Right mod 3???
  public static final int mod3DriveMotor = 42;       //36    //9
  public static final int mod3TurningMotor = 31;       //52    //8
  public static final int mod3CANCoder = 1;       //1       //3
  public static final Rotation2d mod3TurningOffset = Rotation2d.fromDegrees(187.3);


  //PID Constants
  public static final double drivekP = 0.005; 
  public static final double drivekI = 0;
  public static final double drivekD = 0;

  public static final double turningkP = 0.0066; //init value 1
  public static final double turningkI = 0.0006; //init value 0.1
  public static final double turningkD = 0.0001; //init value 0

  


  //Gear Ratios (for the conversion factors)
  public static final double driveMotorRatio = 6.12; //L3: 6.12, L2: 6.75
  public static final double turningMotorRatio = 150 / 7;

  //Conversion Factors
  public static final double driveMotorPosFactor = Math.PI * driveMotorRatio * 9.5;
  public static final double driveMotorVelFactor = driveMotorPosFactor / 60;
  public static final double turningMotorPosFactor = 360 / turningMotorRatio;

  
  //Max drive of the robot (in meters per second or radians per second)
  public static final double maxSpeed = 5;
  public static final double maxAutoSpeed = 2.5;
  public static final double maxAutoAcceleration = 0.5;
  public static final double maxAngularVelocity = 7.0;
  public static final double maxAutoAngularVelocity = Math.PI;
  

  


  //Lengths of the drivetrain, as well as the map of where the wheels are on the robot (like coordinates)
  public static final double robotLength = Units.inchesToMeters(20.381); //23.625? or .675?
  public static final double robotWidth = Units.inchesToMeters(20.381);


  public static final SwerveDriveKinematics SwerveMap = new SwerveDriveKinematics(
    new Translation2d(robotLength / 2, robotWidth / 2), //++
    new Translation2d(robotLength / 2, -robotWidth /2),  //+-
    new Translation2d(-robotLength / 2, robotWidth / 2), //-+
    new Translation2d(-robotLength / 2, -robotWidth / 2) //--

    );



    //Intake Constants

    public static final int intakeMotorID = 44;

    public static final double intakeMotorSpeed = 1;

        //Pivot Angles

        public static final double intakePos = 20;
        public static final double subwooferPos = 60;
        public static final double ampPos = 100;
        public static final double podiumPos = 32.807;
        public static final double sourcePos = 60;


    //Shooter Constants

    public static final int fastShooterMotorID = 1;
    public static final int slowShooterMotorID = 2;
    public static final int shooterHoldMotorID = 3;

    public static final int shooterPotID = 0;

    public static final double shooterkP = 1;
    public static final double shooterkI = 0;
    public static final double shooterkD = 0;

    public static final double shooterIntakeSpeed = 0;

    public static final double shooterHighSpeed = 1;
    public static final double shooterLowSpeed = 0.5;
    public static final double shooterBumpSpeed = 0.2;
     


    //Pivot Constants

    public static final int pivotMotorID = 9;

    public static final double pivotkP = 1;
    public static final double pivotkI = 0;
    public static final double pivotkD = 0;



    //Climber Constants

    public static final int climberMotorLeftID = 4;
    public static final int climberMotorRightID = 5;

    public static final double climberUpSpeed = 0.5;
    public static final double climberDownSpeed = 0.3;

    public static final int climberLeftServoID = 0;
    public static final int climberRightServoID = 1;


  //auto constants
  //change PID for testing swerve auto
    public static final double autoTurningP = .88;
    public static final double autoTurningI = 0;
    public static final double autoTurningD = 0;

    public static final double autoXP = 1;
    public static final double autoXI = 0;
    public static final double autoXD = 0;

    public static final double autoYP = 1;
    public static final double autoYI = 0;
    public static final double autoYD = 0;

    public static final TrapezoidProfile.Constraints autoTurnController = new TrapezoidProfile.Constraints(maxAutoAngularVelocity, maxAutoAngularVelocity);

    //feedforward constants
  public static final double ffkS = 0.99;
  public static final double ffkV = 0.5; //5 m/s 2.44
  public static final double ffkA = 0.1; //accel to 5 m/s 0.27





}


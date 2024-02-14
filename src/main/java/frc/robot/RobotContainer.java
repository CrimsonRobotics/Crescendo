// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ClimberLeftDown;
import frc.robot.commands.ClimberRightDown;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.Drive;
import frc.robot.commands.PivotHoldCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterIntake;
import frc.robot.commands.SpinUpAmp;
import frc.robot.commands.SpinUpShooter;
import frc.robot.commands.intakeSpin;
 
 import frc.robot.commands.ShootCommand;
 import frc.robot.commands.ShooterIntake;
import frc.robot.commands.SwerveAuto;
import frc.robot.commands.SwerveTeleOp;
 import frc.robot.commands.intakeSpin;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Joystick driverL = new Joystick(0);
  private final Joystick driverR = new Joystick(1);
   private final Joystick operatorL = new Joystick(2);
   private final Joystick operatorR = new Joystick(3);

  private final JoystickButton resetYawButton = new JoystickButton(driverL, 1);

  private final JoystickButton shootButton = new JoystickButton(operatorR, 1);
  private final JoystickButton intakePositionButton = new JoystickButton(operatorR, 2);
  private final JoystickButton subwooferPositionButton = new JoystickButton(operatorR, 4);
  private final JoystickButton ampPositionButton = new JoystickButton(operatorL, 3);
  private final JoystickButton ampShootButton = new JoystickButton(operatorL, 1);
  private final JoystickButton podiumPositionButton = new JoystickButton(operatorR, 3);
  private final JoystickButton sourceIntakeButton = new JoystickButton(operatorR, 3);
//Climber up: low buttons left side, operatorL 11-16
  private final JoystickButton climberUpButton = new JoystickButton(operatorL, 11);
  private final JoystickButton climberUp2 = new JoystickButton(operatorL, 12);
  private final JoystickButton climberUp3 = new JoystickButton(operatorL, 13);
  private final JoystickButton climberUp4 = new JoystickButton(operatorL, 14);
  private final JoystickButton climberUp5 = new JoystickButton(operatorL, 15);
  private final JoystickButton climberUp6 = new JoystickButton(operatorL, 16);

  //Climber down: low buttons right side, operatorR 11-16
  private final JoystickButton climberDownButton = new JoystickButton(operatorR, 11);
  private final JoystickButton climberDown2 = new JoystickButton(operatorR, 12);
  private final JoystickButton climberDown3 = new JoystickButton(operatorR, 13);
  private final JoystickButton climberDown4 = new JoystickButton(operatorR, 14);
  private final JoystickButton climberDown5 = new JoystickButton(operatorR, 15);
  private final JoystickButton climberDown6 = new JoystickButton(operatorR, 16);


  private final JoystickButton intakeStopButton = new JoystickButton(operatorL, 5);
  private final JoystickButton intakeDumpButton = new JoystickButton(operatorL, 2);

  
  

  public final SwerveDrive driveSwerve;
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  
  public final Intake inTake = new Intake();
  public final Shooter noteShooter = new Shooter();
  public final Pivot shooterPivot = new Pivot();
  public final Climber chainClimber = new Climber();
   

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    this.driveSwerve = new SwerveDrive(driverL, driverR);
    this.driveSwerve.setDefaultCommand(
        new SwerveTeleOp(
            driveSwerve,
            driverL,
            driverR
            
          )
        );
    //this.inTake.setDefaultCommand(new intakeSpin(inTake));
    //this.noteShooter.setDefaultCommand(new ShooterIntake(noteShooter));
    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

     resetYawButton.onTrue(new InstantCommand(() -> driveSwerve.zeroGyro()));
     
     
      shootButton.whileTrue(new ShootCommand(noteShooter));
      shootButton.onFalse(new intakeSpin(inTake, Constants.intakeMotorSpeed));
      
       
    intakePositionButton.onTrue(new PivotHoldCommand(shooterPivot, Constants.intakePos).alongWith(new intakeSpin(inTake, Constants.intakeMotorSpeed)).alongWith(new ShooterIntake(noteShooter)));
    subwooferPositionButton.onTrue(new PivotHoldCommand(shooterPivot, Constants.subwooferPos).alongWith(new SpinUpShooter(noteShooter, Constants.shooterHighSpeed, Constants.shooterLowSpeed)).alongWith(new intakeSpin(inTake, 0)));
    ampPositionButton.onTrue(new PivotHoldCommand(shooterPivot, Constants.ampPos).alongWith(new intakeSpin(inTake, 0)).alongWith(new SpinUpAmp(noteShooter)));
    podiumPositionButton.onTrue(new PivotHoldCommand(shooterPivot, Constants.podiumPos).alongWith(new SpinUpShooter(noteShooter, Constants.shooterHighSpeed, Constants.shooterLowSpeed)).alongWith(new intakeSpin(inTake, 0)));

    climberUpButton.whileTrue(new ClimberUp(chainClimber));
    climberUp2.whileTrue(new ClimberUp(chainClimber));
    climberUp3.whileTrue(new ClimberUp(chainClimber));
    climberUp4.whileTrue(new ClimberUp(chainClimber));
    climberUp5.whileTrue(new ClimberUp(chainClimber));
    climberUp6.whileTrue(new ClimberUp(chainClimber));

    climberDownButton.onTrue(new ClimberRightDown(chainClimber).alongWith(new ClimberLeftDown(chainClimber)));
    climberDown2.onTrue(new ClimberRightDown(chainClimber).alongWith(new ClimberLeftDown(chainClimber)));
    climberDown3.onTrue(new ClimberRightDown(chainClimber).alongWith(new ClimberLeftDown(chainClimber)));
    climberDown4.onTrue(new ClimberRightDown(chainClimber).alongWith(new ClimberLeftDown(chainClimber)));
    climberDown5.onTrue(new ClimberRightDown(chainClimber).alongWith(new ClimberLeftDown(chainClimber)));
    climberDown6.onTrue(new ClimberRightDown(chainClimber).alongWith(new ClimberLeftDown(chainClimber)));

    
     



    
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new SwerveAuto(driveSwerve);
  }
}

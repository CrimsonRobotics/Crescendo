// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CloseRightAuto extends SequentialCommandGroup {
  /** Creates a new CloseLeftAuto. */
  public CloseRightAuto(SwerveDrive driveSwerve, Joystick driverL, Joystick driverR, Shooter noteShooter, Intake inTake, Pivot shooterPivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new WaitCommand(3),
      (new PivotHoldCommand(shooterPivot, Constants.subwooferPos).alongWith(new SpinUpShooter(noteShooter, Constants.shooterHighSpeed, 0)).alongWith(new intakeSpin(inTake, 0))).raceWith(new WaitCommand(1)),
      (new ShootCommand(noteShooter, Constants.shooterHighSpeed, Constants.shooterBumpSpeed).raceWith(new WaitCommand(1))),
      //ask if okay
      (new PivotHoldCommand(shooterPivot, Constants.intakePos).raceWith(new WaitCommand(0.5).alongWith(new intakeSpin(inTake, Constants.intakeMotorSpeed)).alongWith(((new ShootCommand(noteShooter, 0, 0)).raceWith(new WaitCommand(3))).andThen(new ShooterIntake(noteShooter, Constants.shooterIntakeSpeed))))).raceWith(new WaitCommand(1)),
      //new PivotHoldCommand(shooterPivot, Constants.intakePos).alongWith(new intakeSpin(inTake, Constants.intakeMotorSpeed)).alongWith(new ShooterIntake(noteShooter)),
      new Drive(driveSwerve, driverL, driverR, 0, .5, 0, true).raceWith(new WaitCommand(1.5)),
      new InstantCommand(() -> driveSwerve.zeroGyroAuto(-30)),
      new Drive(driveSwerve, driverL, driverR, 0, 0, 0, true).alongWith(new intakeSpin(inTake, 0)).alongWith(new ShooterIntake(noteShooter, 0))

    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveAuto extends SequentialCommandGroup {
  /** Creates a new SwerveAuto. */
  public SwerveAuto(SwerveDrive driveSwerve) {
    
    TrajectoryConfig configuration = new TrajectoryConfig(Constants.maxAutoSpeed, Constants.maxAutoAcceleration).setKinematics(Constants.SwerveMap);

    Trajectory newTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(1, 1), new Translation2d(2, -1)), new Pose2d(0, 0, new Rotation2d(0)), configuration);
    ProfiledPIDController turnController = new ProfiledPIDController(Constants.autoTurningP, Constants.autoTurningI, Constants.autoTurningD, Constants.autoTurnController);
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand autoController = new SwerveControllerCommand(newTrajectory, driveSwerve::getPose, Constants.SwerveMap, new PIDController(Constants.autoXP, Constants.autoXI, Constants.autoXD), new PIDController(Constants.autoYP, Constants.autoYI, Constants.autoYD), turnController, driveSwerve::setModuleStates, driveSwerve);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> driveSwerve.resetOdometry(newTrajectory.getInitialPose())), autoController);
    
    
  }
}

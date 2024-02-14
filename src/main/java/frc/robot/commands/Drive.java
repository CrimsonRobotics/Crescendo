// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class Drive extends Command {

  private final SwerveDrive driveSwerve;
  // private final BooleanSupplier robotCentricSupply;

  private final double translation;
  private final double strafe;
  private final double rotation;
  private final boolean isAuto;

  /** Creates a new SwerveTeleOp. */
  public Drive(SwerveDrive driveSwerve, double translation, double strafe, double rotation, boolean isAuto) {
    this.driveSwerve = driveSwerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSwerve);
    this.translation = translation;
    this.strafe = strafe;
    this.rotation = rotation;
    this.isAuto = isAuto;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isAuto) {
      driveSwerve.drive(
      new Translation2d(this.translation, this.strafe).times(Constants.maxSpeed), 
      this.rotation * Constants.maxAngularVelocity, true, 
      true
    );
    }
    else {
      driveSwerve.drive(
      new Translation2d(this.translation, this.strafe).times(Constants.maxSpeed), 
      this.rotation * Constants.maxAngularVelocity, true, 
      false
    );
    }
    
    // TODO: wait for wheels to turn before driving

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

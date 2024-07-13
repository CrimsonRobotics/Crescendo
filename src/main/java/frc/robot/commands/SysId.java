// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SwerveDrive;

public class SysId extends Command {
  /** Creates a new SysId. */

  private final SwerveDrive driveSwerve;
  private final SysIdRoutine.Direction direction;
  private final int type;

  public SysId(SwerveDrive driveSwerve, SysIdRoutine.Direction direction, int type) {
    this.driveSwerve = driveSwerve;
    this.direction = direction;
    this.type = type;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (type == 0) {
      driveSwerve.sysIdQuasistatic(direction);
    } else if (type == 1) {
      driveSwerve.sysIdDynamic(direction);
    }
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

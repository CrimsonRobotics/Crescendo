// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class PivotHoldCommand extends Command {
  Pivot pivot;
  double position;
  
  /** Creates a new PivotHoldCommand. */
  public PivotHoldCommand(Pivot pivot, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.pivot);
    this.pivot = pivot;
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterPivotValue = this.pivot.pivotPot.get();

    double motorSpeed = MathUtil.clamp(this.pivot.pivotPID.calculate(shooterPivotValue, position), 0, 100);
    motorSpeed /= 100;
    this.pivot.setSpeed(motorSpeed);
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

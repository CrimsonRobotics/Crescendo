// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Pivot;

public class PivotVoltSet extends Command {
  Pivot pivot;
  LinearFilter filter;
  double position;
  /** Creates a new PivotVoltSet. */
  public PivotVoltSet(Pivot pivot, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
        this.pivot = pivot;
    addRequirements(this.pivot);

    this.position = position;
    filter = LinearFilter.singlePoleIIR(0.1, 0.02);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.pivot.m_controller.reset(this.pivot.pivotPot.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterPivotValue = this.pivot.pivotPot.get();

    //double pivot_voltage = (MathUtil.clamp(this.pivot.pivotPID.calculate(this.filter.calculate(shooterPivotValue), this.position),-100,100))/100 + this.pivot.pivot_feed_forward.calculate(this.position, 0);
    //double pivot_voltage = this.pivot.pivot_feed_forward.calculate(this.position, 0);
    //pivot_voltage /= 100;
    double pivot_voltage = (MathUtil.clamp(this.pivot.m_controller.calculate(this.filter.calculate(shooterPivotValue), this.position), -100, 100))/100 + this.pivot.pivot_feed_forward.calculate(this.position, 0);
    
    this.pivot.setSpeed(pivot_voltage);
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

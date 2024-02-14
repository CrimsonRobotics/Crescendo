// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
  CANSparkMax pivotMotor;

  public AnalogPotentiometer pivotPot;

  public PIDController pivotPID;

  

  LinearFilter filter;
  /** Creates a new Pivot. */
  public Pivot() {
    pivotMotor = new CANSparkMax(Constants.pivotMotorID, MotorType.kBrushless);
    pivotPot = new AnalogPotentiometer(Constants.shooterPotID, 180, 0);

    pivotPID = new PIDController(Constants.pivotkP, Constants.pivotkI, Constants.pivotkD);

    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setInverted(false);

    pivotPID.setIntegratorRange(0, 100);

    filter = LinearFilter.singlePoleIIR(0.1, 0.02);

  }

  public void setSpeed(double speed) {
    pivotMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

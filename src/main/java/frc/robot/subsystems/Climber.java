// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax climberMotorLeft;
  CANSparkMax climberMotorRight;

  boolean climberOn;
  public Climber() {
    climberMotorLeft = new CANSparkMax(Constants.climberMotorLeftID, MotorType.kBrushless);
    climberMotorRight = new CANSparkMax(Constants.climberMotorRightID, MotorType.kBrushless);

    climberMotorLeft.setIdleMode(IdleMode.kBrake);
    climberMotorRight.setIdleMode(IdleMode.kBrake);

    climberMotorLeft.setInverted(false);
    climberMotorRight.setInverted(false);

    climberOn = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

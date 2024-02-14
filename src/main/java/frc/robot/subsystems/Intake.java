// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  CANSparkMax intakeMotor;
  DigitalInput shooterLimitSwitch;
  
  /** Creates a new Intake. */
  public Intake() {
  intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);
  intakeMotor.setIdleMode(IdleMode.kBrake);
  intakeMotor.setInverted(false);

  shooterLimitSwitch = new DigitalInput(0);
  }

  
  public void intakeSpin(double speed) {
    if (shooterLimitSwitch.get()) {
      intakeMotor.set(0);
    }
    else {
    intakeMotor.set(speed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

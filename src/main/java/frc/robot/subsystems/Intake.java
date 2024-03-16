// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  CANSparkMax intakeMotor;
  public DigitalInput shooterLimitSwitch;
  DigitalOutput led0;
  
  /** Creates a new Intake. */
  public Intake() {
  intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);
  intakeMotor.setIdleMode(IdleMode.kBrake);
  intakeMotor.setInverted(false);

  shooterLimitSwitch = new DigitalInput(0);

  //led0 = new DigitalOutput(0);

  
  }

  
  public void intakeSpin(double speed) {
    if (shooterLimitSwitch.get() == false) {
      intakeMotor.set(0);
      //led0.set(true);
    }
    else {
    intakeMotor.set(speed);
      //led0.set(false);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putBoolean("Bottom Limit Switch", shooterLimitSwitch.get());
  }
}

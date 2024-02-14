// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  
  CANSparkMax fastShooterMotor;
  CANSparkMax slowShooterMotor;
  CANSparkMax shooterHoldMotor;


  PIDController shooterPID;

  DigitalInput shooterLimitSwitch;

  boolean intakeState;


  /** Creates a new Shooter. */
  public Shooter() {
    
    fastShooterMotor = new CANSparkMax(Constants.fastShooterMotorID, MotorType.kBrushless);
    slowShooterMotor = new CANSparkMax(Constants.slowShooterMotorID, MotorType.kBrushless);
    shooterHoldMotor = new CANSparkMax(Constants.shooterHoldMotorID, MotorType.kBrushless);

    shooterLimitSwitch = new DigitalInput(0);

    fastShooterMotor.setIdleMode(IdleMode.kBrake);
    slowShooterMotor.setIdleMode(IdleMode.kBrake);
    shooterHoldMotor.setIdleMode(IdleMode.kBrake);

    fastShooterMotor.setInverted(false);
    slowShooterMotor.setInverted(false);
    shooterHoldMotor.setInverted(false);


    shooterPID = new PIDController(Constants.shooterkP, Constants.shooterkI, Constants.shooterkD);
    shooterPID.setIntegratorRange(0, 1);

    shooterIntake(Constants.shooterIntakeSpeed);

  }
 
  public void setIntakeState() {
    if (shooterLimitSwitch.get()) {
      intakeState = true;
    }
    else {
      intakeState = false;
    }
  }

  public void shooterIntake(double speed) {
    
      if (shooterLimitSwitch.get()) {
        fastShooterMotor.set(0);
        slowShooterMotor.set(0);
      }
      else {
      fastShooterMotor.set(speed);
      slowShooterMotor.set(speed);
      }
    
  }

  public void spinUpShooter(double highSpeed, double lowSpeed) {
    fastShooterMotor.set(highSpeed);
    slowShooterMotor.set(highSpeed);
    shooterHoldMotor.set(-Constants.shooterBumpSpeed);
  }

  public void shootCommand(double highSpeed, double lowSpeed, double bumpSpeed) {
    fastShooterMotor.set(highSpeed);
    slowShooterMotor.set(highSpeed);
    shooterHoldMotor.set(bumpSpeed);
  }

  public void spinUpAmp(double highSpeed, double lowSpeed) {
    fastShooterMotor.set(highSpeed);
    slowShooterMotor.set(lowSpeed);
    shooterHoldMotor.set(-Constants.shooterBumpSpeed);
  }
  public void ampShootCommand(double highSpeed, double lowSpeed) {
    fastShooterMotor.set(highSpeed);
    slowShooterMotor.set(lowSpeed);
    shooterHoldMotor.set(Constants.shooterBumpSpeed);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
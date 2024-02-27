// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  PhotonCamera camera;
  PhotonPipelineResult cameraFeed;
  List<PhotonTrackedTarget> targets;
  double error = 0;
  public VisionSubsystem() {
    camera = new PhotonCamera("C270_HD_WEBCAM");
  }

  public double getError() {
    cameraFeed = camera.getLatestResult();
    targets = cameraFeed.getTargets();
    for(PhotonTrackedTarget target : targets) {
      if(target.getFiducialId()==Constants.speakerTagID) {
        error = target.getYaw();
      }
      else {
        error = 0;
      }
    }
    return error;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Target Error", error);
  }
}

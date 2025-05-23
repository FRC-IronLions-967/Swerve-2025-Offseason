// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants;

public class ObjectDetectionCamera extends SubsystemBase {
  /** Creates a new ObjectDetectionCamera. */

  private PhotonCamera objectDetectionCamera;
  private PhotonTrackedTarget bestTarget;

  public ObjectDetectionCamera() {

    objectDetectionCamera = new PhotonCamera(Constants.objectDetectionCamera);

  }

  public PhotonTrackedTarget getBestTarget() {
    return bestTarget;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = objectDetectionCamera.getAllUnreadResults();
    for (PhotonPipelineResult i : result) {
      bestTarget = i.getBestTarget();
    }
  }
}

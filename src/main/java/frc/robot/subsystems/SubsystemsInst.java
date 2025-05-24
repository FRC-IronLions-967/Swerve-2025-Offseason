package frc.robot.subsystems;

import frc.robot.Utils.Constants;

public class SubsystemsInst {
    public Drivetrain drivetrain;
    public AprilTagCamera aprilTagCamera1;
    public AprilTagCamera aprilTagCamera2;
    public ObjectDetectionCamera objectDetectionCamera;
   
   
    private static SubsystemsInst inst;

    private SubsystemsInst() {
        drivetrain = new Drivetrain();
        aprilTagCamera1 = new AprilTagCamera(drivetrain::addVisionMeasurement, Constants.aprilTagCameraName1, Constants.robotToAprilTagCamera1);
        aprilTagCamera2 = new AprilTagCamera(drivetrain::addVisionMeasurement, Constants.aprilTagCameraName2, Constants.robotToAprilTagCamera2);
        objectDetectionCamera = new ObjectDetectionCamera();

    }

    public static SubsystemsInst getInst () {
        if(inst == null) inst = new SubsystemsInst();

        return inst;

    }
    
}
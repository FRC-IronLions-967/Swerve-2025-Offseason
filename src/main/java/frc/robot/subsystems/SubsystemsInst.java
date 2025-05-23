package frc.robot.subsystems;

import frc.robot.Utils.Constants;

public class SubsystemsInst {
    public Drivetrain drivetrain;
    public AprilTagVision aprilTagVision1;
    public AprilTagVision aprilTagVision2;
   
   
    private static SubsystemsInst inst;

    private SubsystemsInst() {
        drivetrain = new Drivetrain();
        aprilTagVision1 = new AprilTagVision(drivetrain::addVisionMeasurement, Constants.aprilTagCameraName1, Constants.robotToAprilTagCamera1);
        aprilTagVision2 = new AprilTagVision(drivetrain::addVisionMeasurement, Constants.aprilTagCameraName2, Constants.robotToAprilTagCamera2);

    }

    public static SubsystemsInst getInst () {
        if(inst == null) inst = new SubsystemsInst();

        return inst;

    }
    
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ObjectDetectionCamera;
import frc.robot.subsystems.SubsystemsInst;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunOverObject extends Command {
  /** Creates a new RunOverObject. */

  private Drivetrain drive;
  private ObjectDetectionCamera vision;
  private double speed;

  public RunOverObject(double speed) {
    this.speed = speed;
    drive = SubsystemsInst.getInst().drivetrain;
    vision = SubsystemsInst.getInst().objectDetectionCamera;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.moveTowardsObject(vision.getRotation(), speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

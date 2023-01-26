// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

// Not finished yet


public class PoseEstimatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final PhotonCamera photonCamera = new PhotonCamera("camera");
  private final Drivetrain driveTrainSubsytem = new Drivetrain();
  private final GyroSubystem gyro = new GyroSubystem();

   private static final List<Pose3d> targetPose = Collections.unmodifiableList
   (List.of(new Pose3d(0, 0, 0, new Rotation3d(0, 0, degreesToRadians(180))),
    (new Pose3d(0, 0, 0, new Rotation3d(0, 0, degreesToRadians(180)))))); 
    
    private static final Vector<N7> stateStDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05, 0.05, 0.05);
    private static final Vector<N5> local = VecBuilder.fill(Units.degreesToRadians(0.01), 0.01, 0.01, 0.01, 0.01);
    private static final Vector<N3> measurment = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    private static DifferentialDrivePoseEstimator poseEstimator;
    private final Field2d field2d = new Field2d();
    private double previousPipelineTimestamp = 0;

    // public PoseEstimatorSubsystem(PhotonCamera camera, Drivetrain drive, GyroSubystem gyro){
    //     poseEstimator = new DifferentialDrivePoseEstimator<N7, N7, N5>(
    //         Nat.N7(), 
    //         Nat.N7(),
    //         Nat.N5(),
    //         gyro.getRoll(),
    //         Constants.DRIVE_KINEMATICS, 

    // }

  public static double degreesToRadians(int degrees){
    return (degrees/180) * Math.PI;
  }
}
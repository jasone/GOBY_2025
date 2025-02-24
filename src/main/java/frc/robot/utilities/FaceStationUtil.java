// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class FaceStationUtil {
  private final ArrayList<Pose2d> m_coralStations;

  public FaceStationUtil() {
    Alliance alliance = getAlliance();
    m_coralStations = getStations(alliance);
  }

  private static Alliance getAlliance() {
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    Alliance alliance = allianceOpt.isPresent() ? allianceOpt.get() : Alliance.Blue;
    return alliance;
  }

  private static ArrayList<Pose2d> getStations(Alliance alliance) {
    ArrayList<Pose2d> coralStations;
    switch (alliance) {
      default: assert false;
      case Blue: { 
        coralStations = FieldConstants.kBlueCoralStations;
        break;
      }
      case Red: {
        coralStations = FieldConstants.kRedCoralStations;
        break;
      }
    }
    return coralStations;
  }
  private static Translation2d getRobotToCoralStation(Pose2d robotPose, Pose2d station) {
    Translation2d robotToStation = station.getTranslation().minus(robotPose.getTranslation());
    return robotToStation;
  }

  private Pose2d closestStation(Pose2d robotPose) {
    Translation2d robotPos = robotPose.getTranslation();
    Pose2d nearestStation = m_coralStations.get(0);

    for (Pose2d station : m_coralStations) {
      if (station.getTranslation().getDistance(robotPos) < nearestStation.getTranslation().getDistance(robotPos)) {
        nearestStation = station;
      }
    }
    return nearestStation;
  }

  public Rotation2d getRotationDeviation(Pose2d robotPose) {
    Rotation2d currentRotation = robotPose.getRotation();
    Rotation2d desiredRotation = closestStation(robotPose).getRotation();
    Rotation2d rotationDeviation = currentRotation.minus(desiredRotation);
    return rotationDeviation;
  }

  public double getStationDistance(Pose2d robotPose) {
    Translation2d robotToStation = getRobotToCoralStation(robotPose, closestStation(robotPose));
    double stationDistance = robotToStation.getNorm();
    return stationDistance;
  }
}

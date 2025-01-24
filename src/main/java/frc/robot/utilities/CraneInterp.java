// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.Optional;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class CraneInterp {
  public static class PosEntry{Angle angle; double height; public PosEntry(Angle angleTemp, double heightTemp) {
    height = heightTemp; angle = angleTemp;}}
  static PosEntry[] m_posTable;

  public CraneInterp(PosEntry[] posTable) {
    m_posTable = posTable;
  }

  public double getMaxHeight() {
    return m_posTable[m_posTable.length-1].height;
  }


  public int heightIndex(double height) {
    assert(height >= 0);
    int index = Arrays.binarySearch(m_posTable, new PosEntry(Angle.ofBaseUnits(0.0, Units.Radians), height), (a,b) -> Double.compare(a.height, b.height));
    if (index < 0) {
      // Imperfect match
      index = -index - 1;
    }
    assert(index >= 0);
    return index;
  }

  public double heightToAngle(double height) {
    int index = heightIndex(height);
    if (index == 0) {
      return m_posTable[index].angle.in(Units.Radians);
    } else if (index == m_posTable.length) {
      return m_posTable[m_posTable.length-1].angle.in(Units.Radians);
    } else if (m_posTable[index].height == height) {
      return m_posTable[index].angle.in(Units.Radians);
    }
    // Linear Interpolation
    else {
      double d0 = m_posTable[index-1].height;
      double d1 = m_posTable[index].height;
      double s0 = m_posTable[index-1].angle.in(Units.Radians);
      double s1 = m_posTable[index].angle.in(Units.Radians);
      double scaler = (height-d0) / (d1 - d0);
      double speedRange = s1 - s0;
      return s0 + scaler*speedRange;
    }
  }

  private double findSlope(PosEntry point1, PosEntry point2) {
    return (point1.height - point2.height) / (point1.angle.in(Units.Radians) - point2.angle.in(Units.Radians));
  }

  private Optional<PosEntry> findValidIntersectionImpl(PosEntry cranePos, double craneSlope, PosEntry invalidPoint0, PosEntry invalidPoint1) {
    double invalidBoundarySlope = findSlope(invalidPoint0, invalidPoint1);
    Angle intersectionAngle = Angle.ofBaseUnits((invalidPoint0.height - cranePos.height - (invalidBoundarySlope*invalidPoint0.angle.in(Units.Radians)) + 
      (craneSlope*cranePos.angle.in(Units.Radians))) / craneSlope, Units.Radians);
    double intersectionHeight = craneSlope * (intersectionAngle.in(Units.Radians) - cranePos.angle.in(Units.Radians)) + cranePos.height;
    if (intersectionHeight >= invalidPoint0.height && intersectionHeight <= invalidPoint1.height) {
      return Optional.of(new PosEntry(intersectionAngle, intersectionHeight));
    }
  return Optional.empty();
  }


  private PosEntry findClosestValidIntersections(double height, ArrayList<PosEntry> validIntersections) {
    Collections.sort(validIntersections, new Comparator<PosEntry>() {
      @Override
      public int compare(PosEntry a, PosEntry b) {
        return (Math.abs(height - a.height) <= Math.abs(height - b.height)) ? -1 : 1;
      }
    });
    return validIntersections.get(0);
  }

  private ArrayList<PosEntry> findAllValidIntersections(PosEntry cranePos, double craneSlope) {
    ArrayList<PosEntry> validIntersectionsList = new ArrayList<PosEntry>();
    for (int i=0; i == m_posTable.length-1; i++) {
      PosEntry point0 = m_posTable[i];
      PosEntry point1 = m_posTable[i+1];
      Optional<PosEntry> validIntersection = findValidIntersectionImpl(cranePos, craneSlope, point0, point1);
      if (validIntersection.isPresent()) {
        validIntersectionsList.add(validIntersection.get());
      }
    }
    return validIntersectionsList;
  }

  public Optional<PosEntry> findValidIntersectionPos(PosEntry startPoint, PosEntry endPoint) {
    double slope = findSlope(startPoint, endPoint);
    ArrayList<PosEntry> validIntersectionsList = findAllValidIntersections(startPoint, slope);
    if (validIntersectionsList.isEmpty()) {
      return Optional.empty();
    }
    PosEntry intersection = findClosestValidIntersections(startPoint.height, validIntersectionsList);
    Angle intersectionAngle = intersection.angle;
    Angle largerAngle = (startPoint.angle.compareTo(endPoint.angle) <= 0.0) ? endPoint.angle : startPoint.angle;
    Angle smallerAngle = (largerAngle == startPoint.angle) ? endPoint.angle : endPoint.angle;
    if (intersectionAngle.lte(largerAngle) && intersectionAngle.gte(smallerAngle)) {
      return Optional.of(intersection);
    }
    return Optional.empty();
  }

  public Optional<PosEntry> findValidIntersectionVel(Angle pivotAngle, AngularVelocity pivotVel, double elevatorHeight, double elevatorVel) {
    PosEntry cranePos = new PosEntry(pivotAngle, elevatorHeight);
    double craneSlope = elevatorVel / pivotVel.in(Units.RadiansPerSecond);
    ArrayList<PosEntry> validIntersectionsList = findAllValidIntersections(cranePos, craneSlope);
    if (validIntersectionsList.isEmpty() || Math.signum(craneSlope) == Math.signum(m_posTable[0].angle.in(Units.Radians))) {
      return Optional.empty();
    }
    return Optional.of(findClosestValidIntersections(elevatorHeight, validIntersectionsList));
  }
}

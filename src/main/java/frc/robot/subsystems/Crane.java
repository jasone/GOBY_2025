// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CraneConstants;
import frc.robot.utilities.SparkUtil;

public class Crane extends SubsystemBase {
  private final SparkFlex m_pivotMotor;
  private final SparkFlex m_leftElevatorMotor;
  private final SparkFlex m_rightElevatorMotor;

  private final RelativeEncoder m_pivotEncoder;
  private final RelativeEncoder m_elevatorEncoder;

  private final SparkClosedLoopController m_pivotPID;
  private final SparkClosedLoopController m_leftElevatorPID;

  private double m_angleSetpoint;
  private double m_heightSetpoint;

  private double m_angleTolerance;
  private double m_heightTolerance;

  private int m_currentSerialNum = 0;
  private double m_startTime = 0.0;
  private boolean m_isVelControlled = false;

  public Crane() {
    m_pivotMotor = new SparkFlex(CraneConstants.kPivotMotorID, MotorType.kBrushless);
    SparkUtil.configureMotor(m_pivotMotor, CraneConstants.kPivotMotorConfig);

    m_leftElevatorMotor = new SparkFlex(CraneConstants.kLeftElevatorMotorID, MotorType.kBrushless);
    SparkUtil.configureMotor(m_leftElevatorMotor, CraneConstants.kElevatorMotorConfig);

    m_rightElevatorMotor = new SparkFlex(CraneConstants.kRightElevatorMotorID, MotorType.kBrushless);
    SparkUtil.configureFollowerMotor(m_rightElevatorMotor, CraneConstants.kElevatorMotorConfig, m_leftElevatorMotor);

    m_pivotEncoder = m_pivotMotor.getEncoder();
    m_elevatorEncoder = m_leftElevatorMotor.getEncoder();

    m_pivotPID = m_pivotMotor.getClosedLoopController();
    m_leftElevatorPID = m_leftElevatorMotor.getClosedLoopController();
  }

  private int allocatePosSerialNum() {
    m_isVelControlled = false;
    int serialNum = m_currentSerialNum;
    m_currentSerialNum++;
    return serialNum;
  }

  // Only increases serial number once while controlling with velocity to avoid the serial num getting too large.
  private void allocateVelSerialNum() {
    if (!m_isVelControlled) {
      m_isVelControlled = true;
      m_currentSerialNum++;
    }
  }

  private void moveElevatorToImpl(double height, double heightTolerance) {
    m_heightSetpoint = height;
    m_heightTolerance = heightTolerance;
    m_leftElevatorPID.setReference(height, ControlType.kMAXMotionPositionControl,
      CraneConstants.kElevatorPosPIDFSlot.slot());
  }

  private void movePivotToImpl(double angle, double angleTolerance) {
    m_angleSetpoint = angle;
    m_angleTolerance = angleTolerance;
    m_pivotPID.setReference(angle, ControlType.kMAXMotionPositionControl,
      CraneConstants.kPivotPosPIDFSlot.slot());
  }

  public int moveElevatorTo(double height, double heightTolerance) {
    moveElevatorToImpl(height, heightTolerance);
    return allocatePosSerialNum();
  }

  public int movePivotTo(double angle, double angleTolerance) {
    movePivotToImpl(angle, angleTolerance);
    return allocatePosSerialNum();
  }

  // Tolerance is in absolute difference
  public int moveTo(double height, double angle, double heightTolerance, double angleTolerance) {
    m_heightSetpoint = height;
    m_angleSetpoint = angle;
    m_heightTolerance = heightTolerance;
    m_angleTolerance = angleTolerance;
    moveElevatorToImpl(height, heightTolerance);
    movePivotToImpl(angle, angleTolerance);
    return allocatePosSerialNum();
  }

  public void moveElevatorVel(double velocity) {
    m_leftElevatorPID.setReference(velocity, ControlType.kMAXMotionVelocityControl, CraneConstants.kElevatorVelPIDFSlot.slot());
    allocateVelSerialNum();;
  }

  public void movePivotVel(double velocity) {
    m_pivotPID.setReference(velocity, ControlType.kMAXMotionVelocityControl, CraneConstants.kPivotVelPIDFSlot.slot());
    allocateVelSerialNum();
  }

  public Optional<Integer> craneAtSetpoint() {
    double height = m_elevatorEncoder.getPosition();
    double angle = m_pivotEncoder.getPosition();
    double currentTime = (double)RobotController.getFPGATime() / 1000000.0;
    if (Math.abs(height - m_heightSetpoint) <= m_heightTolerance &&
      Math.abs(angle - m_angleSetpoint) <= m_angleTolerance) {
        if (currentTime - m_startTime >= CraneConstants.kDebouncingTime) {
          m_startTime = 0.0;
          return Optional.of(m_currentSerialNum);
        } else if (m_startTime == 0.0) {
          m_startTime = currentTime;
        }
    }
    return Optional.empty();
  }

  @Override
  public void periodic() {}
}
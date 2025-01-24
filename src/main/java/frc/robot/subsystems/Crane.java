// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CraneConstants;

public class Crane extends SubsystemBase {
  private final SparkFlex m_leftElevatorMotor;
  private final SparkFlex m_rightElevatorMotor;
  private final SparkFlex m_pivotMotor;

  private final RelativeEncoder m_leftElevatorEncoder;
  private final RelativeEncoder m_rightElevatorEncoder;
  private final RelativeEncoder m_pivotEncoder;

  private final SparkClosedLoopController m_leftElevatorPID;
  private final SparkClosedLoopController m_pivotPID;

  private double m_heightTolerance;
  private double m_angleTolerance;
  private double m_heightSetpoint;
  private double m_angleSetpoint;

  private int m_currentSerialNum = 0;
  private double m_startTime = 0.0;
  private boolean m_isVelControlled = false;

  public Crane() {
    m_leftElevatorMotor = new SparkFlex(CraneConstants.kLeftElevatorMotorID, MotorType.kBrushless);
    m_rightElevatorMotor = new SparkFlex(CraneConstants.kRightElevatorMotorID, MotorType.kBrushless);
    m_pivotMotor = new SparkFlex(CraneConstants.kPivotMotorID, MotorType.kBrushless);

    m_leftElevatorEncoder = m_leftElevatorMotor.getEncoder();
    m_rightElevatorEncoder = m_rightElevatorMotor.getEncoder();
    m_pivotEncoder = m_pivotMotor.getEncoder();

    m_leftElevatorPID = m_leftElevatorMotor.getClosedLoopController();
    m_pivotPID = m_pivotMotor.getClosedLoopController();

    SparkFlexConfig leftElevatorConfig = new SparkFlexConfig();
    leftElevatorConfig.smartCurrentLimit(CraneConstants.kElevatorMotorsCurrentLimit)
      .inverted(CraneConstants.kLeftElevatorMotorReversed)
      .idleMode(IdleMode.kBrake);
    leftElevatorConfig.encoder
      .velocityConversionFactor(CraneConstants.kElevatorVelConversionFactor)
      .positionConversionFactor(CraneConstants.kElevatorPosConversionFactor);
    leftElevatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .maxMotion
        .maxVelocity(CraneConstants.kElevatorMaxVel)
        .maxAcceleration(CraneConstants.kElevatorMaxAccel);

    SparkFlexConfig rightElevatorConfig = new SparkFlexConfig();
    rightElevatorConfig.smartCurrentLimit(CraneConstants.kElevatorMotorsCurrentLimit)
      .idleMode(IdleMode.kBrake)
      .follow(m_leftElevatorMotor, true);
    leftElevatorConfig.encoder
      .velocityConversionFactor(CraneConstants.kElevatorVelConversionFactor)
      .positionConversionFactor(CraneConstants.kElevatorPosConversionFactor);
    leftElevatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .maxMotion
        .maxVelocity(CraneConstants.kElevatorMaxVel)
        .maxAcceleration(CraneConstants.kElevatorMaxAccel);

    SparkFlexConfig pivotConfig = new SparkFlexConfig();
    rightElevatorConfig.smartCurrentLimit(CraneConstants.kPivotMotorCurrentLimit)
      .inverted(CraneConstants.kPivotMotorReversed)
      .idleMode(IdleMode.kBrake);
    leftElevatorConfig.encoder
      .velocityConversionFactor(CraneConstants.kPivotVelConversionFactor)
      .positionConversionFactor(CraneConstants.kPivotPosConversionFactor);
    leftElevatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .maxMotion
        .maxVelocity(CraneConstants.kPivotMaxVel)
        .maxAcceleration(CraneConstants.kPivotMaxAccel);

    CraneConstants.kLeftElevatorVelPIDF.controllerSet(leftElevatorConfig.closedLoop, CraneConstants.kLeftElevatorVelPIDFSlotID);
    CraneConstants.kLeftElevatorPosPIDF.controllerSet(leftElevatorConfig.closedLoop, CraneConstants.kElevatorPosSlot);
    CraneConstants.kPivotVelPIDF.controllerSet(pivotConfig.closedLoop, CraneConstants.kPivotVelPIDFSlotID);
    CraneConstants.kPivotPosPIDF.controllerSet(pivotConfig.closedLoop, CraneConstants.kPivotPosSlot);
  }

  private double getAvgElevatorEncoderPos() {
    return (m_leftElevatorEncoder.getPosition() + m_rightElevatorEncoder.getPosition()) / 2.0;
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
    m_leftElevatorPID.setReference(height, ControlType.kMAXMotionPositionControl, CraneConstants.kElevatorPosSlot);
  }

  private void movePivotToImpl(double angle, double angleTolerance) {
    m_angleSetpoint = angle;
    m_angleTolerance = angleTolerance;
    m_pivotPID.setReference(angle, ControlType.kMAXMotionPositionControl, CraneConstants.kPivotPosSlot);
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
    m_leftElevatorPID.setReference(velocity, ControlType.kMAXMotionVelocityControl, CraneConstants.kLeftElevatorVelPIDFSlotID);
    allocateVelSerialNum();;
  }

  public void movePivotVel(double velocity) {
    m_leftElevatorPID.setReference(velocity, ControlType.kMAXMotionVelocityControl, CraneConstants.kLeftElevatorVelPIDFSlotID);
    allocateVelSerialNum();
  }

  public Optional<Integer> craneAtSetpoint() {
    double height = getAvgElevatorEncoderPos();
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

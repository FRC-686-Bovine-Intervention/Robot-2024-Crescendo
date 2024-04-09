// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NoteVisualizer;
import frc.robot.RobotState;
import frc.robot.util.EdgeDetector;
import frc.robot.util.LoggedTunableNumber;

public class Pivot extends SubsystemBase {
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  public static final LoggedTunableNumber toleranceDeg = new LoggedTunableNumber("Pivot/PID/Position Tolerance Deg", 1);

  private static final Translation3d robotToPivotTranslation = 
    new Translation3d(
      Inches.of(-7.5),
      Inches.of(0),
      Inches.of(19.01)
    )
  ;
  public static Transform3d getRobotToPivot(double angle) {
    return new Transform3d(
      robotToPivotTranslation,
      new Rotation3d(
        0,
        Goal.IDLE.getRads()-angle,
        0
      )
    );
  }
  public Transform3d getRobotToPivot() {
    return getRobotToPivot(inputs.pivotEncoder.positionRad);
  }

  public static enum Goal {
    IDLE(new LoggedTunableNumber("Pivot/Angles/Zero", 9)),
    AIM(() -> RobotState.getInstance().aimingParameters.pivotAltitude()),
    AMP(new LoggedTunableNumber("Pivot/Angles/Amp", 109)),
    SUPER_PASS(new LoggedTunableNumber("Pivot/Angles/Super Pass", 50+5.09765625)),
    // RECAL(() -> 0){
    //   private final LoggedTunableNumber recalVolts = new LoggedTunableNumber("Pivot/Recal Volts", -1);
    //   @Override
    //   public void runGoal(PivotIO pivotIO) {
    //     pivotIO.setPivotVoltage(recalVolts.get());
    //   }
    // },
    ;
    private final DoubleSupplier altitudeDeg;
    Goal(DoubleSupplier altitudeDeg) {
      this.altitudeDeg = altitudeDeg;
    }
    public double getRads() {
      return Units.degreesToRadians(altitudeDeg.getAsDouble());
    }
    public void runGoal(PivotIO pivotIO) {
      pivotIO.setPivotPos(getRads());
    }
  }

  @AutoLogOutput(key = "Pivot/Goal")
  private Goal goal = Goal.IDLE;

  private final EdgeDetector increaseEdgeDetector;
  private final EdgeDetector decreaseEdgeDetector;

  public Pivot(PivotIO pivotIO, BooleanSupplier increaseRuntimeOffset, BooleanSupplier decreaseRuntimeOffset) {
    System.out.println("[Init Pivot] Instantiating Pivot");
    this.pivotIO = pivotIO;
    System.out.println("[Init Pivot] Pivot IO: " + this.pivotIO.getClass().getSimpleName());
    SmartDashboard.putData("Subsystems/Pivot", this);
    this.increaseEdgeDetector = new EdgeDetector(increaseRuntimeOffset);
    this.decreaseEdgeDetector = new EdgeDetector(decreaseRuntimeOffset);
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
    Logger.recordOutput("Mechanism3d/Shooter", getRobotToPivot());
    NoteVisualizer.robotToPivot = getRobotToPivot();
    if(increaseEdgeDetector.risingEdge()) {
      runtimeOffset += 0.5;
    }
    if(decreaseEdgeDetector.risingEdge()) {
      runtimeOffset -= 0.5;
    }

    if(increaseEdgeDetector.risingEdge() || decreaseEdgeDetector.risingEdge()) {
      pivotIO.setRotorOffset(Units.degreesToRadians(runtimeOffset));
    }
    goal.runGoal(pivotIO);
  }

  @AutoLogOutput(key = "Pivot/Runtime Offset")
  private double runtimeOffset = 0;

  // public Command recal() {
  //   var subsystem = this;
  //   return new Command() {
  //     {
  //       addRequirements(subsystem);
  //       setName("Recal");
  //     }
  //     @Override
  //     public void initialize() {
  //       pivotIO.enableSoftLimits(false);
  //     }
  //     @Override
  //     public void execute() {
  //       pivotIO.setPivotVoltage(-1);
  //     }
  //     @Override
  //     public void end(boolean interrupted) {
  //       pivotIO.enableSoftLimits(true);
  //       pivotIO.stop();
  //     }
  //   };
  // }

  public Command setGoalCommand(Goal goal) {
    return startEnd(
        () -> this.goal = goal,
        () -> this.goal = Goal.IDLE
    )
    .withName("Pivot " + goal.name());
}

  public boolean readyToShoot() {
    return atPos() && goal != Goal.IDLE;
  }

  public boolean atPos() {
    return inputs.atGoal;
  }

  public boolean isAtAngle(double angleRad) {
    return MathUtil.isNear(angleRad, inputs.pivotEncoder.positionRad, Units.degreesToRadians(toleranceDeg.get()));
  }

  public void setCoast(boolean coast) {
    pivotIO.setCoast(coast);
  }
}

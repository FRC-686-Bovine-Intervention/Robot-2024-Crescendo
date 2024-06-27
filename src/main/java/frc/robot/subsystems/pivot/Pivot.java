// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AimingParameters;
import frc.robot.NoteVisualizer;
import frc.robot.util.LoggedTunableMeasure;
import frc.robot.util.SuppliedEdgeDetector;

public class Pivot extends SubsystemBase {
    private final PivotIO pivotIO;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public static final LoggedTunableMeasure<Angle> tolerance = new LoggedTunableMeasure<>("Pivot/PID/Position Tolerance Deg", Degrees.of(1));
    public static final LoggedTunableMeasure<Angle> idleAltitude = new LoggedTunableMeasure<>("Pivot/Angles/Zero", Degrees.of(0));
    public static final LoggedTunableMeasure<Angle> ampAltitude = new LoggedTunableMeasure<>("Pivot/Angles/Amp", Degrees.of(100));
    public static final LoggedTunableMeasure<Angle> superPassAltitude = new LoggedTunableMeasure<>("Pivot/Angles/Super Pass", Degrees.of(50+5.09765625));

    public final Trigger atPos = new Trigger(() -> inputs.atGoal);

    private static final Translation3d robotToPivotTranslation = 
        new Translation3d(
            Inches.of(-7.5),
            Inches.of(0),
            Inches.of(19.01)
        )
    ;

    private final SuppliedEdgeDetector increaseEdgeDetector;
    private final SuppliedEdgeDetector decreaseEdgeDetector;

    @AutoLogOutput(key = "Pivot/Runtime Offset")
    private MutableMeasure<Angle> runtimeOffset = MutableMeasure.zero(Degrees);

    public Pivot(PivotIO pivotIO, BooleanSupplier increaseRuntimeOffset, BooleanSupplier decreaseRuntimeOffset) {
        System.out.println("[Init Pivot] Instantiating Pivot");
        this.pivotIO = pivotIO;
        System.out.println("[Init Pivot] Pivot IO: " + this.pivotIO.getClass().getSimpleName());
        SmartDashboard.putData("Subsystems/Pivot", this);
        this.increaseEdgeDetector = new SuppliedEdgeDetector(increaseRuntimeOffset);
        this.decreaseEdgeDetector = new SuppliedEdgeDetector(decreaseRuntimeOffset);
    }

    @Override
    public void periodic() {
        pivotIO.updateInputs(inputs);
        Logger.processInputs("Inputs/Pivot", inputs);
        Logger.recordOutput("Mechanism3d/Shooter", getRobotToPivot());
        NoteVisualizer.robotToPivot = getRobotToPivot();
        increaseEdgeDetector.update();
        decreaseEdgeDetector.update();
        if(increaseEdgeDetector.risingEdge()) {
            runtimeOffset.mut_acc(0.5);
        }
        if(decreaseEdgeDetector.risingEdge()) {
            runtimeOffset.mut_acc(-0.5);
        }

        if(increaseEdgeDetector.risingEdge() || decreaseEdgeDetector.risingEdge()) {
            pivotIO.setRotorOffset(runtimeOffset.in(Radians));
        }
    }

    public static Transform3d getRobotToPivot(double angle) {
        return new Transform3d(
            robotToPivotTranslation,
            new Rotation3d(
                0,
                idleAltitude.in(Radians)-angle,
                0
            )
        );
    }

    public Transform3d getRobotToPivot() {
        return getRobotToPivot(inputs.pivotEncoder.positionRad);
    }

    public void setCoast(boolean coast) {
        pivotIO.setCoast(coast);
    }

    private Command genCommand(String name, Supplier<Measure<Angle>> angleSupplier) {
        var subsystem = this;
        return new Command() {
            {
                setName(name);
                addRequirements(subsystem);
            }

            @Override
            public void execute() {
                pivotIO.setPivotPos(angleSupplier.get().in(Radians));
            }
        };
    }

    public Command idle() {
        return genCommand(
            "Idle",
            idleAltitude
        );
    }
    public Command aim() {
        return genCommand(
            "Aim",
            AimingParameters::pivotAltitude
        );
    }
    public Command amp() {
        return genCommand(
            "Amp",
            ampAltitude
        );
    }
    public Command superPass() {
        return genCommand(
            "Super Pass",
            superPassAltitude
        );
    }

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
}

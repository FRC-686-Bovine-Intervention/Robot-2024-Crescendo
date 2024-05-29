package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SuppliedEdgeDetector {
    private final Supplier<boolean[]> source;

    public SuppliedEdgeDetector(BooleanSupplier source) {
        this(() -> new boolean[]{source.getAsBoolean()});
    }

    public SuppliedEdgeDetector(Supplier<boolean[]> source) {
        this.source = source;
    }

    private boolean prevVal;
    private boolean risingEdge;
    private boolean fallingEdge;

    public void update() {
        update(source.get());
    }
    public void update(boolean... history) {
        risingEdge = false;
        fallingEdge = false;
        for(var source : history) {
            var val = source;
            if(val && !prevVal) {
                risingEdge = true;
            }
            if(!val && prevVal) {
                fallingEdge = true;
            }
            prevVal = val;
        }
    }

    public boolean getValue() {
        return prevVal;
    }
    public boolean risingEdge() {
        return risingEdge;
    }
    public boolean fallingEdge() {
        return fallingEdge;
    }
    public boolean changed() {
        return risingEdge || fallingEdge;
    }
}

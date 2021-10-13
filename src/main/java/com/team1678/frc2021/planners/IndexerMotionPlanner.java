package com.team1678.frc2021.planners;

import com.team1678.frc2021.Constants;

public class IndexerMotionPlanner {
    private boolean[] slots = {false, false, false};

    public IndexerMotionPlanner() {
        throw new UnsupportedOperationException("Unsupported Orientation");
    }

    public double findPrepDistance(int slotNumber) {
        double distanceToIntake = slotNumber * Constants.kDistancePerSlot;
        return -(Math.abs(Constants.kTotalDistance - distanceToIntake));
    }

    public double findIntakingDistance(boolean slotFull) {
        if (!slotFull) {
            return Constants.kDistancePerSlot;
        } else {
            return 0;
        }
    }

    public boolean[] updateSlotStatus(boolean[] raw_slots) {
        int idx = 0;
        for (int i = 0; i < 5; i++) {
            slots[idx] = raw_slots[i];
            idx++;
            if (idx > 4) {
                idx = 0;
            }
        }

        return slots;
    }

    public boolean isAtGoal(boolean[] slots) {
        return slots[0];
    }

    public static double findDistanceGoal(int slotNumber){
        return Constants.kTotalDistance - (slotNumber * Constants.kDistancePerSlot);
    }
}

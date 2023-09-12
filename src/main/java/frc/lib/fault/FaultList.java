
package frc.lib.fault;

import java.util.ArrayList;

public class FaultList extends ArrayList<Fault> {

    public int getCurrentFaultCount() {
        int count = 0;
        for (Fault fault : this) {
            if (fault.get()) {
                count += fault.getCount();
            }
        }
        return count;
    }

    public int getStickyFaultCount() {
        int count = 0;
        for (Fault fault : this) {
            if (fault.getSticky()) {
                count += 1;
            }
        }
        return count;
    }

    public boolean hasFaults() {
        return getCurrentFaultCount() > 0 | getStickyFaultCount() > 0;
    }

    /**
     * plus: merge two faultlists into one and return it
     * @param other FaultList to merge with
     * @return new FaultList with both FaultLists
     */
    public FaultList plus(FaultList other) {
        FaultList newList = new FaultList();
        newList.addAll(this);
        newList.addAll(other);
        return newList;
    }

    /**
     * plus: merge another faultlist into this one
     * @param other FaultList to merge
     */
    public void merge(FaultList other) {
        this.addAll(other);
    }
}

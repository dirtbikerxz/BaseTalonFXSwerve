package frc.lib.fault;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

public class Fault {
    private String name;
    private BooleanSupplier current;
    private BooleanSupplier sticky;
    private IntSupplier count;

    public Fault(String name, BooleanSupplier current, BooleanSupplier sticky) {
        this(name, current, sticky, () -> -1);
    }

    /**
     * A Fault with no Sticky
     * @param name
     * @param current
     */
    public Fault(String name, BooleanSupplier current) {
        this(name, current, () -> false, () -> -1);
    }

    /**
     * A Fault with a count, current, and sticky
     * @param name
     * @param current
     * @param sticky
     * @param count
     */
    public Fault(String name, BooleanSupplier current, BooleanSupplier sticky, IntSupplier count) {
        this.name = name;
        this.current = current;
        this.sticky = sticky;
        this.count = count;
    }

    /**
     * A Fault with a count, no current or sticky
     * @param name
     * @param current
     * @param count
     */
    public Fault(String name, IntSupplier count) {
        this(name, () -> count.getAsInt() > 1, () -> false, count);
    }

    public String getName() {
        return name;
    }

    public boolean get() {
        return current.getAsBoolean();
    }

    public boolean getSticky() {
        return sticky.getAsBoolean();
    }

    /**
     * Get the number of active faults, typically 1 or 0 for boolean faults, but can faults are counted individually
     * @return a positive integer for the number of faults
     */
    public int getCount() {
        if (count.getAsInt() < 0) {
            return get() ? 1 : 0;
        }
        return count.getAsInt();
    }
}

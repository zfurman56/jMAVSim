package me.drton.jmavsim;

/**
 * Updater for the visualizer's simulation state's report.
 */
public class ReportUpdater extends WorldObject {
    private static final long UPDATE_FREQ_MS = 250;

    private static final StringBuilder builder = new StringBuilder();
    private final Visualizer3D visualizer;
    private long lastUpdateT;


    public ReportUpdater(World world, Visualizer3D visualizer) {
        super(world);
        this.visualizer = visualizer;
    }

    @Override
    public void update(long t) {
        if (t < lastUpdateT + UPDATE_FREQ_MS) {
            return;
        }

        lastUpdateT = t;
        builder.setLength(0);

        for (WorldObject object : getWorld().getObjects()) {
            if (object instanceof ReportingObject) {
                ((ReportingObject) object).report(builder);
            }
        }

        visualizer.setReportText(builder.toString());
    }
}

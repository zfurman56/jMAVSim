package me.drton.jmavsim.vehicle;

import me.drton.jmavsim.DynamicObject;
import me.drton.jmavsim.ReportUtil;
import me.drton.jmavsim.ReportingObject;
import me.drton.jmavsim.Sensors;
import me.drton.jmavsim.World;

import javax.vecmath.Vector3d;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Abstract vehicle class, should be used for creating vehicle of any type.
 * Child class should use member 'control' as control input for actuators.
 * 'update()' method of AbstractVehicle must be called from child class implementation if overridden.
 */
public abstract class AbstractVehicle extends DynamicObject implements ReportingObject {
    protected List<Double> control = Collections.emptyList();
    protected Sensors sensors = null;

    public AbstractVehicle(World world, String modelName) throws FileNotFoundException {
        super(world);
        modelFromFile(modelName);
        position.set(0.0, 0.0, world.getEnvironment().getGroundLevel(new Vector3d(0.0, 0.0, 0.0)));
    }

    public void report(StringBuilder builder) {
        Vector3d tv;
        builder.append("VEHICLE");
        builder.append(newLine);
        builder.append("===========");
        builder.append(newLine+newLine);

        builder.append("Attitude:");
        builder.append(newLine+newLine);
        if (sensors != null) {
            tv = sensors.getAcc();
            builder.append(String.format("ACC X: %.5f Y: %.5f Z: %.5f", tv.x, tv.y, tv.z));
            builder.append(newLine);
            builder.append(String.format("    Magnitude: %.5f", Math.sqrt(tv.x*tv.x + tv.y*tv.y + tv.z*tv.z)));
            builder.append(newLine);
            builder.append(String.format("    Pitch: %.5f; Roll: %.5f", Math.toDegrees(Math.atan2(tv.x, -tv.z)), Math.toDegrees(Math.atan2(-tv.y, -tv.z))));
            builder.append(newLine);

            tv = sensors.getGyro();
            builder.append(String.format("GYO X: %.5f Y: %.5f Z: %.5f", tv.x, tv.y, tv.z));
            builder.append(newLine);
            
            tv = sensors.getMag();
            builder.append(String.format("MAG X: %.5f Y: %.5f Z: %.5f", tv.x, tv.y, tv.z));
            builder.append(newLine);
            builder.append(String.format("    Magnitude: %.5f", Math.sqrt(tv.x*tv.x + tv.y*tv.y + tv.z*tv.z)));
            builder.append(newLine);

            builder.append(newLine);
        }
        
        builder.append("Position:");
        builder.append(newLine+newLine);
        if (sensors != null) {
            if (sensors.getGNSS() != null && sensors.getGNSS().position != null) {
                builder.append(String.format("  Lat: %.8f;\n  Lon: %.8f\n  Alt: %.3f", sensors.getGNSS().position.lat, sensors.getGNSS().position.lat, sensors.getGNSS().position.alt));
                builder.append(newLine);
            }
            builder.append(String.format("  Baro Alt: %.3f;\n  Pa: %.2f", sensors.getPressureAlt(), sensors.getPressure()));
            builder.append(newLine);
        }
        builder.append(String.format("  X: %.5f Y: %.5f Z: %.5f", position.x, position.y, position.z));
        builder.append(newLine+newLine);

        builder.append("Velocity: ");
        builder.append(ReportUtil.toShortString(velocity));
        builder.append(newLine);
        
        builder.append("Acceleration: ");
        builder.append(ReportUtil.toShortString(acceleration));
        builder.append(newLine);

        builder.append("Rotation Rate: ");
        builder.append(ReportUtil.toShortString(rotationRate));
        builder.append(newLine);

        builder.append(newLine);
    }

    public void setControl(List<Double> control) {
        this.control = new ArrayList<Double>(control);
    }

    public List<Double> getControl() {
        return control;
    }

    /**
     * Set sensors object for the vehicle.
     *
     * @param sensors
     */
    public void setSensors(Sensors sensors) {
        this.sensors = sensors;
        sensors.setObject(this);
    }

    public Sensors getSensors() {
        return sensors;
    }
    
    @Override
    public void resetObjectParameters() {
        super.resetObjectParameters();
        if (sensors != null)
            sensors.setReset(true);
    }

    @Override
    public void update(long t) {
        super.update(t);
        if (sensors != null) {
            sensors.update(t);
        }
    }
}

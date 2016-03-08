package me.drton.jmavsim.vehicle;

import me.drton.jmavsim.ReportUtil;
import me.drton.jmavsim.ReportingObject;
import me.drton.jmavsim.Rotor;
import me.drton.jmavsim.World;

import javax.vecmath.Vector3d;
import java.io.FileNotFoundException;

/**
 * Abstract multicopter class. Does all necessary calculations for multirotor with any placement of rotors.
 * Only rotors on one plane supported now.
 */
public abstract class AbstractMulticopter extends AbstractVehicle implements ReportingObject {
    private double dragMove = 0.0;
    private double dragRotate = 0.0;
    protected Rotor[] rotors;

    public AbstractMulticopter(World world, String modelName) throws FileNotFoundException {
        super(world, modelName);
        rotors = new Rotor[getRotorsNum()];
        for (int i = 0; i < getRotorsNum(); i++) {
            rotors[i] = new Rotor();
        }
    }

    public void report(StringBuilder builder) {
        if (sensors.getGNSS() != null) {
        builder.append("VEHICLE");
        builder.append(newLine);
        builder.append("===========");
        builder.append(newLine);
        builder.append(newLine);

        builder.append("Position:\n");
        builder.append(newLine);
        builder.append(String.format("  Lat: %.5f;\n  Lon: %.5f\n  Alt: %.5f", sensors.getGNSS().position.lat, sensors.getGNSS().position.lat, sensors.getGNSS().position.alt));
        builder.append(newLine);
        builder.append(String.format("  Baro Alt: %.5f;\n  Pa: %.5f", sensors.getPressureAlt(), sensors.getPressure()));
        builder.append(newLine);
        builder.append(String.format("  X: %.5f Y: %.5f Z: %.5f", position.x, position.y, position.z));
        builder.append(newLine);
        builder.append(newLine);
        }
        
        builder.append("MULTICOPTER");
        builder.append(newLine);
        builder.append("===========");
        builder.append(newLine);

        builder.append("CONTROLS");
        builder.append(newLine);
        builder.append("--------");
        builder.append(newLine);

        if (getControl().size() > 0) {
            for (int i = 0; i < getControl().size(); i++) {
                builder.append(String.format("%.5f", getControl().get(i)));
                builder.append(newLine);
            }
        } else {
            builder.append("n/a");
            builder.append(newLine);
        }

        builder.append(newLine);

        for (int i = 0; i < getRotorsNum(); i++) {
            reportRotor(builder, i);
            builder.append(newLine);
        }

        builder.append(ReportingObject.newLine);
    }

    private void reportRotor(StringBuilder builder, int rotorIndex) {
        Rotor rotor = rotors[rotorIndex];

        builder.append("ROTOR #");
        builder.append(rotorIndex);
        builder.append(newLine);
        builder.append("--------");
        builder.append(newLine);

        builder.append("Control: ");
        builder.append(String.format("%.6f", rotor.getControl()));
        builder.append(newLine);

        builder.append("Thrust: ");
        builder.append(String.format("%.6f", rotor.getThrust()));
        builder.append(" / ");
        builder.append(String.format("%.6f", rotor.getFullThrust()));
        builder.append(" [N]");
        builder.append(newLine);

        builder.append("Torque: ");
        builder.append(String.format("%.6f", rotor.getTorque()));
        builder.append(" / ");
        builder.append(String.format("%.6f", rotor.getFullTorque()));
        builder.append(" [Nm]");
        builder.append(newLine);

        builder.append("Spin up: ");
        builder.append(String.format("%.5f", rotor.getTimeConstant()));
        builder.append(" [s]");
        builder.append(newLine);

        builder.append("Position: ");
        builder.append(ReportUtil.toShortString(getRotorPosition(rotorIndex)));
        builder.append(newLine);

    }

    /**
     * Get number of rotors.
     *
     * @return number of rotors
     */
    protected abstract int getRotorsNum();

    /**
     * Get rotor position relative to gravity center of vehicle.
     *
     * @param i rotor number
     * @return rotor radius-vector from GC
     */
    protected abstract Vector3d getRotorPosition(int i);

    public void setDragMove(double dragMove) {
        this.dragMove = dragMove;
    }

    public void setDragRotate(double dragRotate) {
        this.dragRotate = dragRotate;
    }

    @Override
    public void update(long t) {
        for (Rotor rotor : rotors) {
            rotor.update(t);
        }
        super.update(t);
        for (int i = 0; i < rotors.length; i++) {
            double c = control.size() > i ? control.get(i) : 0.0;
            rotors[i].setControl(c);
        }
    }

    @Override
    protected Vector3d getForce() {
        int n = getRotorsNum();
        Vector3d f = new Vector3d();
        for (int i = 0; i < n; i++) {
            f.z -= rotors[i].getThrust();
        }
        rotation.transform(f);
        Vector3d airSpeed = new Vector3d(getVelocity());
        airSpeed.scale(-1.0);
        airSpeed.add(getWorld().getEnvironment().getWind(position));
        f.add(getAirFlowForce(airSpeed));
        return f;
    }

    @Override
    protected Vector3d getTorque() {
        int n = getRotorsNum();
        Vector3d torque = new Vector3d();
        Vector3d m = new Vector3d();
        Vector3d t = new Vector3d();
        for (int i = 0; i < n; i++) {
            // Roll / pitch
            t.z = -rotors[i].getThrust();
            m.cross(getRotorPosition(i), t);
            // Yaw
            m.z -= rotors[i].getTorque();
            torque.add(m);
        }
        Vector3d airRotationRate = new Vector3d(rotationRate);
        airRotationRate.scale(-1.0);
        torque.add(getAirFlowTorque(airRotationRate));
        return torque;
    }

    protected Vector3d getAirFlowForce(Vector3d airSpeed) {
        Vector3d f = new Vector3d(airSpeed);
        f.scale(f.length() * dragMove);
        return f;
    }

    protected Vector3d getAirFlowTorque(Vector3d airRotationRate) {
        Vector3d f = new Vector3d(airRotationRate);
        f.scale(f.length() * dragRotate);
        return f;
    }
}

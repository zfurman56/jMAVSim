package me.drton.jmavsim.vehicle;

import me.drton.jmavsim.ReportUtil;
import me.drton.jmavsim.World;

import javax.vecmath.Vector3d;

/**
 * Rocket class.
 */
public class Rocket extends AbstractVehicle {
    private double dragMove = 0.0;
    private double launch_time;

    public Rocket(World world, String modelName) {
        super(world, modelName);
    }

    public void setDragMove(double dragMove) {
        this.dragMove = dragMove;
    }

    @Override
    protected Vector3d getForce() {
        Vector3d f = new Vector3d();
        Vector3d airSpeed = new Vector3d(getVelocity());
        airSpeed.scale(-1.0);
        if (!ignoreWind)
            airSpeed.add(getWorld().getEnvironment().getCurrentWind(position));
        f.add(getAirFlowForce(airSpeed));
        return f;
    }

    @Override
    protected Vector3d getTorque() {
        Vector3d torque = new Vector3d(0, 0, 0);
        return torque;
    }

    protected Vector3d getAirFlowForce(Vector3d airSpeed) {
        Vector3d f = new Vector3d(airSpeed);
        f.scale(f.length() * dragMove);
        return f;
    }
}

package me.drton.jmavsim.vehicle;

import me.drton.jmavsim.ReportUtil;
import me.drton.jmavsim.World;

import javax.vecmath.Vector3d;

/**
 * Rocket class.
 */
public class Rocket extends AbstractVehicle {
    private double dragMove = 0.0;
    private double launch_time = 0.0;
    private RocketEngine engine = new RocketEngine("/rocket_engines/AeroTech_F52.eng");

    public Rocket(World world, String modelName) {
        super(world, modelName);
    }

    public void setDragMove(double dragMove) {
        this.dragMove = dragMove;
    }

    public void launch() {
        // Launch 5 seconds in the future
        launch_time = System.currentTimeMillis()+5000;
    }

    @Override
    protected Vector3d getForce() {
        // If the rocket hasn't started the launch sequence, start it
        if (launch_time==0.0) {
            launch();
        }

        Vector3d f = new Vector3d();
        Vector3d airSpeed = new Vector3d(getVelocity());
        airSpeed.scale(-1.0);
        if (!ignoreWind)
            airSpeed.add(getWorld().getEnvironment().getCurrentWind(position));
        f.add(getAirFlowForce(airSpeed));

        Vector3d thrust_force = new Vector3d(0, 0, -engine.thrust((System.currentTimeMillis()-launch_time)/1000.0));
        f.add(thrust_force);
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

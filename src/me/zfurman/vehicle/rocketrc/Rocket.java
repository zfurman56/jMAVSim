package me.zfurman.vehicle.rocketrc;

import java.util.List;

import me.drton.jmavsim.ReportUtil;
import me.drton.jmavsim.World;
import me.drton.jmavsim.vehicle.AbstractVehicle;

import javax.vecmath.Vector3d;

public class Rocket extends AbstractVehicle {
    private double dragMove = 0.0;
    private double launch_time = 0.0;
    private double dragBrakeAngle = 0.0;
    private int dragGain = 7;
    private RocketEngine engine = new RocketEngine("/rocket_engines/AeroTech_F52.eng");

    public Rocket(World world, String modelName) {
        super(world, modelName);
    }

    public void setDragMove(double dragMove) {
        this.dragMove = dragMove;
    }

    public void launch() {
        // Launch 5 seconds in the future
        launch_time = System.currentTimeMillis() + 15000;
    }

    @Override
    protected Vector3d getForce() {
        // If the rocket hasn't started the launch sequence, start it
        if (launch_time == 0.0) {
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
        List<Double> control = getControl();
        if (control.size() > 0) {
            dragBrakeAngle = control.get(0) * (Math.PI / 2);
        }
        Vector3d f = new Vector3d(0, 0, (dragMove * (1 + (dragGain * Math.pow(Math.sin(dragBrakeAngle), 2))) * Math.pow(airSpeed.z, 2) * Math.signum(airSpeed.z)));
        return f;
    }
}

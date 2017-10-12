package me.zfurman.jmavsim;

import java.io.IOException;

import javax.vecmath.Matrix3d;

import me.drton.jmavsim.AbstractSimulator;
import me.drton.jmavsim.SimpleSensors;
import me.drton.jmavsim.World;
import me.drton.jmavsim.vehicle.AbstractVehicle;

import me.zfurman.jmavsim.vehicle.Rocket;

public class RocketSimulator extends AbstractSimulator
{
    public static final String DEFAULT_VEHICLE_MODEL = "models/rocket.obj";

    public RocketSimulator() throws IOException, InterruptedException {}

    @Override
    protected AbstractVehicle buildVehicle(World world) {
        Rocket vehicle = new Rocket(world, DEFAULT_VEHICLE_MODEL);
        Matrix3d I = new Matrix3d();
        // Moments of inertia
        I.m00 = 0.005;  // X
        I.m11 = 0.005;  // Y
        I.m22 = 0.009;  // Z
        vehicle.setMomentOfInertia(I);
        vehicle.setMass(0.625);
        vehicle.setDragMove(0.0011);
        SimpleSensors sensors = new SimpleSensors();
        sensors.setGPSInterval(50);
        sensors.setGPSDelay(200);
        sensors.setGPSStartTime(System.currentTimeMillis() + 1000);
        sensors.setNoise_Acc(0.05f);
        sensors.setNoise_Gyo(0.01f);
        sensors.setNoise_Mag(0.005f);
        sensors.setNoise_Prs(0.0f);
        vehicle.setSensors(sensors);
        //v.setDragRotate(0.1);
        
        return vehicle;
    }
}
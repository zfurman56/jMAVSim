package me.drton.jmavsim;

import me.drton.jmavsim.vehicle.AbstractVehicle;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import java.util.List;

/**
 * User: ton Date: 21.03.14 Time: 23:22
 */
public class CameraGimbal2D extends KinematicObject {
    private DynamicObject baseObject;
    private int pitchChannel = -1;
    private double pitchScale = 1.0;
    private int rollChannel = -1;
    private double rollScale = 1.0;

    public CameraGimbal2D(World world) {
        super(world);
    }

    public void setBaseObject(DynamicObject object) {
        this.baseObject = object;
    }

    public void setPitchChannel(int pitchChannel) {
        this.pitchChannel = pitchChannel;
    }

    public void setPitchScale(double pitchScale) {
        this.pitchScale = pitchScale;
    }

    public void setRollChannel(int channel) {
        this.rollChannel = channel;
    }

    public void setRollScale(double scale) {
        this.rollScale = scale;
    }

    @Override
    public void update(long t) {
        this.position = baseObject.position;
        this.velocity = baseObject.velocity;
        this.acceleration = baseObject.acceleration;
        double yaw = Math.atan2(baseObject.rotation.getElement(1, 0), baseObject.rotation.getElement(0, 0));
        this.rotation.rotZ(yaw);
        if ((pitchChannel >= 0 || rollChannel >= 0) && baseObject instanceof AbstractVehicle) {
            // Control camera pitch/roll
            List<Double> control = ((AbstractVehicle) baseObject).getControl();
            Matrix3d r = new Matrix3d();
            if (pitchChannel >= 0 && control.size() > pitchChannel)
                r.rotY(control.get(pitchChannel) * pitchScale);
            if (rollChannel >= 0 && control.size() > rollChannel)
                r.rotX(control.get(rollChannel) * rollScale);
            this.rotation.mul(r);
        }
    }
}

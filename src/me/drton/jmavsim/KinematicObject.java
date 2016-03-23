package me.drton.jmavsim;

import com.sun.j3d.loaders.Scene;
import com.sun.j3d.loaders.objectfile.ObjectFile;

import javax.media.j3d.BranchGroup;
import javax.media.j3d.Transform3D;
import javax.media.j3d.TransformGroup;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import java.io.FileNotFoundException;
import java.net.MalformedURLException;
import java.net.URL;

/**
 * Abstract kinematic object class.
 * Stores all kinematic parameters (attitude, attitude rates, position, velocity, acceleration) but doesn't calculate it.
 * These parameters may be set directly for objects moving by fixed trajectory or simulated from external forces (see DynamicObject).
 */
public abstract class KinematicObject extends WorldObject {
    protected boolean ignoreGravity = false;
    protected Vector3d position = new Vector3d();
    protected Vector3d velocity = new Vector3d();
    protected Vector3d acceleration = new Vector3d();
    protected Matrix3d rotation = new Matrix3d();
    protected Vector3d rotationRate = new Vector3d();

    private Transform3D transform;
    protected TransformGroup transformGroup;
    private BranchGroup branchGroup;

    public KinematicObject(World world) {
        super(world);
        rotation.setIdentity();
        transformGroup = new TransformGroup();
        transformGroup.setCapability(TransformGroup.ALLOW_TRANSFORM_WRITE);
        transform = new Transform3D();
        transformGroup.setTransform(transform);
        branchGroup = new BranchGroup();
        branchGroup.addChild(transformGroup);
    }

    /**
     * Helper method to create model from .obj file.
     *
     * @param modelFile file name
     * @throws java.io.FileNotFoundException
     */
    protected void modelFromFile(String modelFile) throws FileNotFoundException {
        URL file = null;
        try {
            file = new URL("file:./" + modelFile);
        } catch (MalformedURLException e) {
            System.err.println(e);
            System.exit(1);
        }
        ObjectFile objectFile = new ObjectFile();
        Scene scene = objectFile.load(file);
        transformGroup.addChild(scene.getSceneGroup());
    }

    public BranchGroup getBranchGroup() {
        return branchGroup;
    }

    public void updateBranchGroup() {
        transform.setTranslation(position);
        transform.setRotationScale(rotation);
        transformGroup.setTransform(transform);
    }

    public void setIgnoreGravity(boolean ignoreGravity) {
        this.ignoreGravity = ignoreGravity;
    }

    public Vector3d getPosition() {
        return position;
    }

    public void setPosition(Vector3d position) {
        this.position = position;
    }

    public Vector3d getVelocity() {
        return velocity;
    }

    public void setVelocity(Vector3d vel) {
        this.velocity = vel;
    }

    public Vector3d getAcceleration() {
        return acceleration;
    }

    public void setAcceleration(Vector3d acc) {
        this.acceleration = acc;
    }

    public Matrix3d getRotation() {
        return rotation;
    }

    public void setRotation(Matrix3d rotation) {
        this.rotation = rotation;
    }

    public Vector3d getRotationRate() {
        return rotationRate;
    }

    public void setRotationRate(Vector3d rate) {
        this.rotationRate = rate;
    }

    public void resetObjectParameters() {
        position = new Vector3d();
        velocity = new Vector3d();
        acceleration = new Vector3d();
        rotation = new Matrix3d();
        rotationRate = new Vector3d();
        
        rotation.rotX(0);
    }
}

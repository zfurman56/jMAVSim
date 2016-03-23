package me.drton.jmavsim;

import com.sun.j3d.utils.geometry.Sphere;
import com.sun.j3d.utils.image.TextureLoader;
import com.sun.j3d.utils.universe.SimpleUniverse;

import javax.media.j3d.*;
import javax.swing.*;
import javax.vecmath.*;

import java.awt.*;
import java.awt.event.*;
import java.util.BitSet;
import java.util.Enumeration;

/**
 * 3D Visualizer, works in own thread, synchronized with "world" thread.
 */
public class Visualizer3D extends JFrame {
    private static final long serialVersionUID = 1L;
    private static final int KEYCHECK_INTERVAL = 50;  // [ms]

    public static enum ViewTypes { VIEW_STATIC, VIEW_FPV, VIEW_GIMBAL }
    
    private static Color3f white = new Color3f(1.0f, 1.0f, 1.0f);
    private final World world;
    private boolean reportPaused = false;
    private long nextKeycheck = 0L;
    private SimpleUniverse universe;
    private BoundingSphere sceneBounds = new BoundingSphere(new Point3d(0, 0, 0), 1000000.0);
    private Vector3d viewerPosition = new Vector3d(0.0, 0.0, 0.0);
    private Vector3d viewerPositionOffset = new Vector3d(0.0, 0.0, 0.0);
    private Transform3D viewerTransform = new Transform3D();
    private TransformGroup viewerTransformGroup;
    private KinematicObject viewerTargetObject;
    private KinematicObject viewerPositionObject;
    private KinematicObject vehicleViewObject;
    private KinematicObject gimbalViewObject;
    private ReportPanel reportPanel;
    private MAVLinkHILSystem hilSystem;
    private KeyboardHandler keyHandler;

    public Visualizer3D(World world) {
        this.world = world;

        setSize(800, 600);
        setDefaultCloseOperation(EXIT_ON_CLOSE);  // HIDE_ON_CLOSE
        setTitle("jMAVSim");
        
        keyHandler = new KeyboardHandler();
        
        GraphicsConfiguration gc = SimpleUniverse.getPreferredConfiguration();
        Canvas3D canvas = new Canvas3D(gc);
        canvas.setFocusable(true);
        canvas.addKeyListener(keyHandler);
        getContentPane().add(canvas);
        
        universe = new SimpleUniverse(canvas);
        universe.getViewer().getView().setBackClipDistance(100000.0);
        viewerTransformGroup = universe.getViewingPlatform().getViewPlatformTransform();
        createEnvironment();
        
        for (WorldObject object : world.getObjects()) {
            if (object instanceof KinematicObject) {
                BranchGroup bg = ((KinematicObject) object).getBranchGroup();
                if (bg != null) {
                    universe.addBranchGraph(bg);
                }
            }
        }
        setVisible(true);
        resetView();

    }

    /**
     * Target object to point camera, has effect only if viewerPositionObject is not set.
     *
     * @param object
     */
    public void setViewerTargetObject(KinematicObject object) {
        this.viewerTargetObject = object;
    }

    /**
     * Object to place camera on, if nullptr then camera will be placed in fixed point set by setViewerPosition().
     *
     * @param object
     */
    public void setViewerPositionObject(KinematicObject object) {
        this.viewerPositionObject = object;
    }

    /**
     * Fixed camera position, has effect only if viewerPositionObject not set.
     *
     * @param position
     */
    public void setViewerPosition(Vector3d position) {
        this.viewerPositionObject = null;
        this.viewerPosition = position;
        viewerTransform.setTranslation(viewerPosition);
    }

    /**
     * Camera position offset from object position when viewer placed on some object
     *
     * @param offset position offset
     */
    public void setViewerPositionOffset(Vector3d offset) {
        this.viewerPositionOffset = offset;
    }

    /**
     * Set the "vehicle" object to use for switching views.
     *
     * @param object
     */
    public void setVehicleViewObject(KinematicObject object) {
        this.vehicleViewObject = object;
    }

    /**
     * Set the "gimbal" object to use for switching views.
     *
     * @param object
     */
    public void setGimbalViewObject(KinematicObject object) {
        this.gimbalViewObject = object;
    }

    /**
     * Set the system being controlled.
     *
     * @param system
     */
    public void setHilSystem(MAVLinkHILSystem system) {
        this.hilSystem = system;
    }

    /**
     * Sets the text of the simulation report.
     *
     * @param text
     */
    public void setReportText(String text) {
    	if (this.reportPanel != null && !reportPaused)
    	    this.reportPanel.setText(text);
    }

    /**
     * Show/hide the simulation report.
     *
     * @param text
     */
    public void toggleReportPanel(boolean on) {
        if (on && this.reportPanel == null) {
            this.reportPanel = new ReportPanel();
            this.reportPanel.setFocusable(false);
            reportPaused = false;
            add(this.reportPanel, "West");
        } else if (!on && this.reportPanel != null) {
            remove(this.reportPanel);
            this.reportPanel = null;
        }
        revalidate();
    }
    
    public void toggleReportPanel() {
        this.toggleReportPanel(this.reportPanel == null);
    }

    /**
     * Toggles updates of the report panel text.
     *
     * @param pause
     */
    public void setReportPaused(boolean pause) {
        reportPaused = pause;
        reportPanel.setIsFocusable(pause);
    }

    public void setViewType(ViewTypes v) {
        switch (v) {
            case VIEW_STATIC :
                // Put camera on static point and point to vehicle
                if (vehicleViewObject != null) {
                    this.setViewerPosition(new Vector3d(-5.0, 0.0, -1.7));
                    this.setViewerTargetObject(vehicleViewObject);
                }
                break;

            case VIEW_FPV :
                // Put camera on vehicle (FPV)
                if (vehicleViewObject != null) {
                    this.setViewerPositionObject(vehicleViewObject);
                    this.setViewerPositionOffset(new Vector3d(-0.6f, 0.0f, -0.3f));   // Offset from vehicle center
                }
                break;

            case VIEW_GIMBAL :
                if (gimbalViewObject != null) {
                    this.setViewerPositionObject(gimbalViewObject);
                    this.setViewerPositionOffset(new Vector3d(0.0f, 0.0f, 0.0f));
                }
                break;
        }
    }

    public void resetView() {
        Matrix3d mat = new Matrix3d();
        Matrix3d mat1 = new Matrix3d();
        mat.rotZ(Math.PI);
        mat1.rotY(Math.PI / 2);
        mat.mul(mat1);
        mat1.rotZ(-Math.PI / 2);
        mat.mul(mat1);
        viewerTransform.setRotation(mat);
    }
 
    private void createEnvironment() {
        BranchGroup group = new BranchGroup();
        // Sky
        BoundingSphere bounds = new BoundingSphere(new Point3d(0.0, 0.0, 0.0), 1000.0);
        Background bg = new Background();
        bg.setApplicationBounds(bounds);
        BranchGroup backGeoBranch = new BranchGroup();
        Sphere skySphere = new Sphere(1.0f, Sphere.GENERATE_NORMALS | Sphere.GENERATE_NORMALS_INWARD | Sphere.GENERATE_TEXTURE_COORDS, 32);
        Texture texSky = new TextureLoader("environment/sky.jpg", null).getTexture();
        skySphere.getAppearance().setTexture(texSky);
        Transform3D transformSky = new Transform3D();
        //transformSky.setTranslation(new Vector3d(0.0, 0.0, -0.5));
        Matrix3d rot = new Matrix3d();
        rot.rotX(Math.PI / 2);
        transformSky.setRotation(rot);
        TransformGroup tgSky = new TransformGroup(transformSky);
        tgSky.addChild(skySphere);
        backGeoBranch.addChild(tgSky);
        bg.setGeometry(backGeoBranch);
        group.addChild(bg);
        //group.addChild(tgSky);
        // Ground
        QuadArray polygon1 = new QuadArray(4, QuadArray.COORDINATES | GeometryArray.TEXTURE_COORDINATE_2);
        polygon1.setCoordinate(0, new Point3f(-1000f, 1000f, 0f));
        polygon1.setCoordinate(1, new Point3f(1000f, 1000f, 0f));
        polygon1.setCoordinate(2, new Point3f(1000f, -1000f, 0f));
        polygon1.setCoordinate(3, new Point3f(-1000f, -1000f, 0f));
        polygon1.setTextureCoordinate(0, 0, new TexCoord2f(0.0f, 0.0f));
        polygon1.setTextureCoordinate(0, 1, new TexCoord2f(10.0f, 0.0f));
        polygon1.setTextureCoordinate(0, 2, new TexCoord2f(10.0f, 10.0f));
        polygon1.setTextureCoordinate(0, 3, new TexCoord2f(0.0f, 10.0f));
        Texture texGround = new TextureLoader("environment/grass2.jpg", null).getTexture();
        Appearance apGround = new Appearance();
        apGround.setTexture(texGround);
        Shape3D ground = new Shape3D(polygon1, apGround);
        Transform3D transformGround = new Transform3D();
        transformGround.setTranslation(
                new Vector3d(0.0, 0.0, 0.005 + world.getEnvironment().getGroundLevel(new Vector3d(0.0, 0.0, 0.0))));
        TransformGroup tgGround = new TransformGroup(transformGround);
        tgGround.addChild(ground);
        group.addChild(tgGround);

        // Light
        DirectionalLight light1 = new DirectionalLight(white, new Vector3f(4.0f, 7.0f, 12.0f));
        light1.setInfluencingBounds(sceneBounds);
        group.addChild(light1);
        AmbientLight light2 = new AmbientLight(new Color3f(0.9f, 0.9f, 0.9f));
        light2.setInfluencingBounds(sceneBounds);
        group.addChild(light2);

        // Update behavior
        Behavior b = new UpdateBehavior();
        b.setSchedulingBounds(bounds);
        group.addChild(b);
        universe.addBranchGraph(group);
    }

    private void updateVisualizer() {
//        synchronized (world) { // Synchronize with "world" thread
            // Update branch groups of all kinematic objects
            for (WorldObject object : world.getObjects()) {
                if (object instanceof KinematicObject) {
                    BranchGroup bg = ((KinematicObject) object).getBranchGroup();
                    if (bg != null) {
                        ((KinematicObject) object).updateBranchGroup();
                    }
                }
            }

            // Update view platform
            if (viewerPositionObject != null) {
                // Camera on object
                viewerPosition.set(viewerPositionOffset);
                viewerPositionObject.getRotation().transform(viewerPosition);
                viewerPosition.add(viewerPositionObject.getPosition());
                viewerTransform.setTranslation(viewerPosition);

                Matrix3d mat = new Matrix3d();
                Matrix3d mat1 = new Matrix3d();
                mat.set(viewerPositionObject.getRotation());
                mat1.rotZ(Math.PI / 2);
                mat.mul(mat1);
                mat1.rotX(-Math.PI / 2);
                mat.mul(mat1);
                viewerTransform.setRotation(mat);
            } 
            else if (viewerTargetObject != null) {
                // Fixed-position camera, point camera to target
                Vector3d pos = viewerTargetObject.getPosition();
                Vector3d dist = new Vector3d();
                dist.sub(pos, viewerPosition);

                Matrix3d mat = new Matrix3d();
                Matrix3d mat1 = new Matrix3d();
                mat.rotZ(Math.PI);
                mat1.rotY(Math.PI / 2);
                mat.mul(mat1);
                mat1.rotZ(-Math.PI / 2);
                mat.mul(mat1);
                mat1.rotY(-Math.atan2(pos.y - viewerPosition.y, pos.x - viewerPosition.x));
                mat.mul(mat1);
                mat1.rotX(-Math.asin((pos.z - viewerPosition.z) / dist.length()));
                mat.mul(mat1);
                viewerTransform.setRotation(mat);
            }
            viewerTransformGroup.setTransform(viewerTransform);
//        }
        if (System.currentTimeMillis() > nextKeycheck) {
            nextKeycheck = System.currentTimeMillis() + KEYCHECK_INTERVAL;
            keyHandler.checkCumulativeKeys();
        }
    }

    private void rotateObject(KinematicObject obj, Vector3f vec, float deg) {
        Matrix3d rot = obj.getRotation();
        Matrix3d r = new Matrix3d();
        AxisAngle4f aa = new AxisAngle4f(vec, (float)Math.toRadians(deg));
        r.set(aa);
        rot.mulNormalize(r);
        obj.setRotation(rot);
    }
    
    private void moveObject(KinematicObject obj, Vector3f vec) {
        Vector3d pos = obj.getPosition();
        pos.add(new Vector3d(vec));
        obj.setIgnoreGravity(pos.z < 0.0);
//        if (pos.z >= 0.0)
//            //world.getEnvironment().setG(null);
//        else
//            world.getEnvironment().setG(new Vector3d());
        obj.setPosition(pos);
    }

    private void rotationRateObject(KinematicObject obj, Vector3f vec) {
        if (vec == null)
            obj.setRotationRate(new Vector3d());
        else {
            // if still on ground, move it up so it can rotate
            if (obj.getPosition().z >= 0)
                moveObject(obj, new Vector3f(0f, 0f, -2.0f));
            Vector3d rot = obj.getRotationRate();
            rot.add(new Vector3d(vec));
            obj.setRotationRate(rot);
        }
    }
    
    class UpdateBehavior extends Behavior {
        private WakeupCondition condition = new WakeupOnElapsedFrames(0, false);

        @Override
        public void initialize() {
            wakeupOn(condition);
        }

        @Override
        public void processStimulus(Enumeration wakeup) {
            Object w;
            while (wakeup.hasMoreElements()) {
                w = wakeup.nextElement();
                if (w instanceof WakeupOnElapsedFrames) {
                    updateVisualizer();
                }
                wakeupOn(condition);
            }
        }
    }

    
    /////// KeyboardHandler ///////
    
    public class KeyboardHandler extends KeyAdapter {
        public BitSet keyBits = new BitSet(256);

        @Override
        public void keyReleased(KeyEvent e) {
            keyBits.clear(e.getKeyCode());
            
            switch (e.getKeyCode()) {
            
            case KeyEvent.VK_F :
                setViewType(ViewTypes.VIEW_FPV);
                break;

            case KeyEvent.VK_S :
                setViewType(ViewTypes.VIEW_STATIC);
                break;

            case KeyEvent.VK_G :
                setViewType(ViewTypes.VIEW_GIMBAL);
                break;
                
            case KeyEvent.VK_R :
                toggleReportPanel();
                break;

            case KeyEvent.VK_T :
                setReportPaused(!reportPaused);
                break;

            case KeyEvent.VK_I :
                if (hilSystem != null)
                    hilSystem.initMavLink();
                break;

            case KeyEvent.VK_Q :
                if (hilSystem != null)
                    hilSystem.endSim();
                break;
                
            case KeyEvent.VK_SPACE :
                if (vehicleViewObject != null) {
                    vehicleViewObject.resetObjectParameters();
                    vehicleViewObject.position.set(0.0, 0.0, world.getEnvironment().getGroundLevel(new Vector3d(0.0, 0.0, 0.0)));
                }
                if (gimbalViewObject != null)
                    gimbalViewObject.resetObjectParameters();
                
                resetView();
                break;

            case KeyEvent.VK_ESCAPE :
                dispatchEvent(new WindowEvent(getWindows()[0], WindowEvent.WINDOW_CLOSING));
                break;
                
            }
            
        }

        @Override
        public void keyPressed(KeyEvent e) {
            keyBits.set(e.getKeyCode());
            
//            checkCumulativeKeys();

            Vector3f dir = new Vector3f(0f, 0f, 0f);
            float m = keyHandler.keyBits.get(KeyEvent.VK_CONTROL) ? 1.0f : 0.5f;

            if (!keyBits.get(KeyEvent.VK_NUMPAD5)) {
                if (keyBits.get(KeyEvent.VK_NUMPAD4))
                    dir.setX(-m);
                if (keyBits.get(KeyEvent.VK_NUMPAD6))
                    dir.setX(m);
                if (keyBits.get(KeyEvent.VK_NUMPAD8))
                    dir.setY(-m);
                if (keyBits.get(KeyEvent.VK_NUMPAD2))
                    dir.setY(m);
                if (keyBits.get(KeyEvent.VK_NUMPAD1))
                    dir.setZ(-m);
                if (keyBits.get(KeyEvent.VK_NUMPAD3))
                    dir.setZ(m);
            }
            if (dir.length() != 0.0)
                rotationRateObject(vehicleViewObject, dir);
            else if (keyBits.get(KeyEvent.VK_NUMPAD5))
                rotationRateObject(vehicleViewObject, null);
         
        }

        public void checkCumulativeKeys() {
            Vector3f dir = new Vector3f(0f, 0f, 0f);
            float deg = keyBits.get(KeyEvent.VK_CONTROL) ? 5.0f : 1.0f;
            float m = keyBits.get(KeyEvent.VK_SHIFT) ? deg / 5.0f : 1.0f;

            if (keyBits.get(KeyEvent.VK_LEFT) || keyBits.get(KeyEvent.VK_KP_LEFT))
                dir.setX(-m);

            if (keyBits.get(KeyEvent.VK_RIGHT) || keyBits.get(KeyEvent.VK_KP_RIGHT))
                dir.setX(m);

            if (keyBits.get(KeyEvent.VK_UP) || keyBits.get(KeyEvent.VK_KP_UP))
                dir.setY(-m);

            if (keyBits.get(KeyEvent.VK_DOWN) || keyBits.get(KeyEvent.VK_KP_DOWN))
                dir.setY(m);

            if (keyBits.get(KeyEvent.VK_END) || keyBits.get(KeyEvent.VK_INSERT))
                dir.setZ(-m);

            if (keyBits.get(KeyEvent.VK_PAGE_DOWN) || keyBits.get(KeyEvent.VK_DELETE))
                dir.setZ(m);

            if (dir.length() != 0.0) {
                if (keyBits.get(KeyEvent.VK_SHIFT)) {
                    dir.set(-dir.y, dir.x, dir.z);
                    moveObject(vehicleViewObject, dir);
                } else
                    rotateObject(vehicleViewObject, dir, deg);
            }
        }
        
    }
}

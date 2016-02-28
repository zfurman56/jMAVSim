package me.drton.jmavsim;

import me.drton.jmavlib.geo.LatLonAlt;
import me.drton.jmavlib.mavlink.MAVLinkSchema;
import me.drton.jmavsim.vehicle.AbstractMulticopter;
import me.drton.jmavsim.vehicle.Quadcopter;
import org.xml.sax.SAXException;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import java.lang.Math;
import javax.xml.parsers.ParserConfigurationException;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.HashSet;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;;

/**
 * User: ton Date: 26.11.13 Time: 12:33
 */
public class Simulator implements Runnable {

    public static boolean USE_SERIAL_PORT = false;
    public static boolean COMMUNICATE_WITH_QGC = true;
    public static boolean SHOW_REPORT_PANEL = false;
    
    public static final int    DEFAULT_SIM_SPEED = 500; // Hz
    public static final String DEFAULT_AUTOPILOT_TYPE = "generic";  // eg. "px4" or "aq"
    public static final int    DEFAULT_AUTOPILOT_PORT = 14560;
    public static final int    DEFAULT_QGC_BIND_PORT = 0;
    public static final int    DEFAULT_QGC_PEER_PORT = 14550;
    public static final String DEFAULT_SERIAL_PATH = "/dev/tty.usbmodem1";
    public static final int    DEFAULT_SERIAL_BAUD_RATE = 230400;
    public static final String LOCAL_HOST = "127.0.0.1";

    public static final int    DEFAULT_CAM_PITCH_CHAN =  4;    // Control gimbal pitch from autopilot, -1 to disable
    public static final int    DEFAULT_CAM_ROLL_CHAN  = -1;    // Control gimbal roll from autopilot, -1 to disable
    public static final Double DEFAULT_CAM_PITCH_SCAL = 1.57;  // channel value to physical movement (+/-90 deg)
    public static final Double DEFAULT_CAM_ROLL_SCAL  = 1.57;  // channel value to physical movement (+/-90 deg)

    
    private static int sleepInterval = (int)1e6 / DEFAULT_SIM_SPEED;  // Main loop interval, in us
    private static String autopilotType = DEFAULT_AUTOPILOT_TYPE;
    private static String autopilotIpAddress = LOCAL_HOST;
    private static int autopilotPort = DEFAULT_AUTOPILOT_PORT;
    private static String qgcIpAddress = LOCAL_HOST;
    private static int qgcBindPort = DEFAULT_QGC_BIND_PORT;
    private static int qgcPeerPort = DEFAULT_QGC_PEER_PORT;
    private static String serialPath = DEFAULT_SERIAL_PATH;
    private static int serialBaudRate = DEFAULT_SERIAL_BAUD_RATE;

    private static HashSet<Integer> monitorMessageIds = new HashSet<Integer>();
    private static boolean monitorMessage = false;

    Runnable keyboardWatcher;
    Visualizer3D visualizer;
    AbstractMulticopter vehicle;
    CameraGimbal2D gimbal;

    private World world;
//    private int simDelayMax = 500;  // Max delay between simulated and real time to skip samples in simulator, in ms
    private ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
    public volatile boolean shutdown = false;

    public Simulator() throws IOException, InterruptedException, ParserConfigurationException, SAXException {
        // Create world
        world = new World();
        // Set global reference point
        // Zurich Irchel Park: 47.397742, 8.545594, 488m
        // Seattle downtown (15 deg declination): 47.592182, -122.316031, 86m
        // Moscow downtown: 55.753395, 37.625427, 155m
        LatLonAlt referencePos = new LatLonAlt(47.397742, 8.545594, 488.0);
        world.setGlobalReference(referencePos);

        MAVLinkSchema schema = new MAVLinkSchema("mavlink/message_definitions/common.xml");

        // Create MAVLink connections
        MAVLinkConnection connHIL = new MAVLinkConnection(world);
        world.addObject(connHIL);
        MAVLinkConnection connCommon = new MAVLinkConnection(world);
        // Don't spam ground station with HIL messages
        connCommon.addSkipMessage(schema.getMessageDefinition("HIL_CONTROLS").id);
        connCommon.addSkipMessage(schema.getMessageDefinition("HIL_SENSOR").id);
        connCommon.addSkipMessage(schema.getMessageDefinition("HIL_GPS").id);
        world.addObject(connCommon);

        // Create ports
        MAVLinkPort autopilotMavLinkPort;
        if (USE_SERIAL_PORT) {
            //Serial port: connection to autopilot over serial.
            SerialMAVLinkPort port = new SerialMAVLinkPort(schema);
            port.setup(serialPath, serialBaudRate, 8, 1, 0);
            autopilotMavLinkPort = port;
        } else {
            UDPMavLinkPort port = new UDPMavLinkPort(schema);
            //port.setDebug(true);
            port.setup(0, autopilotIpAddress, autopilotPort); // default source port 0 for autopilot, which is a client of JMAVSim
            // monitor certain mavlink messages.
            if (monitorMessage)  port.setMonitorMessageID(monitorMessageIds);
            autopilotMavLinkPort = port;
        }

        // allow HIL and GCS to talk to this port
        connHIL.addNode(autopilotMavLinkPort);
        connCommon.addNode(autopilotMavLinkPort);
        // UDP port: connection to ground station
        UDPMavLinkPort udpGCMavLinkPort = new UDPMavLinkPort(schema);
        //udpGCMavLinkPort.setDebug(true);
        if (COMMUNICATE_WITH_QGC) {
            udpGCMavLinkPort.setup(qgcBindPort, qgcIpAddress, qgcPeerPort);
            //udpGCMavLinkPort.setDebug(true);
            if (monitorMessage && USE_SERIAL_PORT)
                udpGCMavLinkPort.setMonitorMessageID(monitorMessageIds);
            connCommon.addNode(udpGCMavLinkPort);
        }

        // Create environment
        SimpleEnvironment simpleEnvironment = new SimpleEnvironment(world);

        // Mag vector in earth field, loosely based on the earth field in Zurich,
        // but without declination. The declination will be added below based on
        // the origin GPS position.
        Vector3d magField = new Vector3d(0.21523, 0.0f, 0.42741);

        //simpleEnvironment.setWind(new Vector3d(0.0, 5.0, 0.0));
        //simpleEnvironment.setWindDeviation(0.0);
        simpleEnvironment.setGroundLevel(0.0f);
        world.addObject(simpleEnvironment);

        // Set declination based on the initialization position of the Simulator
        // getMagDeclination() is in degrees, GPS position is in radians and the result
        // variable decl is back in radians.
        double decl = (world.getEnvironment().getMagDeclination(referencePos.lat / Math.PI * 180.0, referencePos.lon / Math.PI * 180.0) / 180.0) * Math.PI;

        Matrix3d magDecl = new Matrix3d();
        magDecl.rotZ(decl);
        magDecl.transform(magField);
        simpleEnvironment.setMagField(magField);

        // Create vehicle with sensors
        vehicle = buildMulticopter();

        // Create MAVLink HIL system
        // SysId should be the same as autopilot, ComponentId should be different!
        MAVLinkHILSystem hilSystem = new MAVLinkHILSystem(schema, 1, 51, vehicle);
        connHIL.addNode(hilSystem);
        world.addObject(vehicle);

        // Put camera on vehicle with gimbal
        if (DEFAULT_CAM_PITCH_CHAN > -1 || DEFAULT_CAM_ROLL_CHAN > -1) {
            gimbal = buildGimbal();
            world.addObject(gimbal);
        }

        // Create 3D visualizer
        visualizer = new Visualizer3D(world);

        // default camera view
        setFPV();

        // Create simulation report updater
        world.addObject(new ReportUpdater(world, visualizer));
        visualizer.toggleReportPanel(SHOW_REPORT_PANEL);

        // Open ports
        autopilotMavLinkPort.open();

        if (autopilotType == "px4" && autopilotMavLinkPort instanceof SerialMAVLinkPort) {
            // Special handling for PX4: Start MAVLink instance
            SerialMAVLinkPort port = (SerialMAVLinkPort) autopilotMavLinkPort;
            port.sendRaw("\nsh /etc/init.d/rc.usb\n".getBytes());
        }

        if (COMMUNICATE_WITH_QGC)
            udpGCMavLinkPort.open();

        keyboardWatcher = new Runnable() {
            InputStreamReader fileInputStream = new InputStreamReader(System.in);
            BufferedReader bufferedReader = new BufferedReader(fileInputStream);
            @Override
            public void run() {
                try {
                    // get user input as a String
                    if (bufferedReader.ready()) {
                        String input = bufferedReader.readLine();
                        if (input.equals("f"))
                            setFPV();
                        else if (input.equals("g"))
                            setGimbal();
                        else if (input.equals("s"))
                            setStaticCamera();
                        else if (input.equals("x"))
                            shutdown = true;
                        else if (input.equals("i"))
                            hilSystem.initMavLink();
                        else if (input.equals("q"))
                            hilSystem.endSim();
                        else if (input.equals("r"))
                            visualizer.toggleReportPanel();
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        };

        final ScheduledFuture<?> thisHandle = executor.scheduleAtFixedRate(this, 0, sleepInterval, TimeUnit.MICROSECONDS);
        final ScheduledFuture<?> kwHandle = executor.scheduleAtFixedRate(keyboardWatcher, 0, 1, TimeUnit.MILLISECONDS);

        while(true) { 
            Thread.sleep(100);
        
            if (shutdown)
                break;
        }

        System.out.println("Shutdown.");
        hilSystem.endSim();
        // Close ports
        autopilotMavLinkPort.close();
        udpGCMavLinkPort.close();
        kwHandle.cancel(true);
        thisHandle.cancel(true);
        executor.shutdown();
        System.exit(0);
    }

    private AbstractMulticopter buildMulticopter() throws IOException {
        Vector3d gc = new Vector3d(0.0, 0.0, 0.0);  // gravity center
        AbstractMulticopter vehicle = new Quadcopter(world, "models/3dr_arducopter_quad_x.obj", "x", 0.33 / 2, 4.0,
                0.05, 0.005, gc);
        vehicle.setMass(0.8);
        Matrix3d I = new Matrix3d();
        // Moments of inertia
        I.m00 = 0.005;  // X
        I.m11 = 0.005;  // Y
        I.m22 = 0.009;  // Z
        vehicle.setMomentOfInertia(I);
        SimpleSensors sensors = new SimpleSensors();
        sensors.setGPSDelay(200);
        sensors.setGPSStartTime(System.currentTimeMillis() + 1000);
        vehicle.setSensors(sensors);
        vehicle.setDragMove(0.02);
        //v.setDragRotate(0.1);

        return vehicle;
    }

    private CameraGimbal2D buildGimbal() {
        gimbal = new CameraGimbal2D(world);
        gimbal.setBaseObject(vehicle);
        gimbal.setPitchChannel(DEFAULT_CAM_PITCH_CHAN);
        gimbal.setPitchScale(DEFAULT_CAM_PITCH_SCAL); 
        gimbal.setRollChannel(DEFAULT_CAM_ROLL_CHAN);
        gimbal.setRollScale(DEFAULT_CAM_ROLL_SCAL);
        return gimbal;
    }

    public void setFPV() {
        // Put camera on vehicle (FPV)
        visualizer.setViewerPositionObject(vehicle);
        visualizer.setViewerPositionOffset(new Vector3d(-0.6f, 0.0f, -0.3f));   // Offset from vehicle center
    }

    public void setGimbal() {
        visualizer.setViewerPositionOffset(new Vector3d(0.0f, 0.0f, 0.0f));
        visualizer.setViewerPositionObject(gimbal);
    }

    public void setStaticCamera() {
        // Put camera on static point and point to vehicle
        visualizer.setViewerPosition(new Vector3d(-5.0, 0.0, -1.7));
        visualizer.setViewerTargetObject(vehicle);
    }

    public void run() {
        try {
            //keyboardWatcher.run();
            long t = System.currentTimeMillis();
            world.update(t);
        }
        catch (Exception e) {
            executor.shutdown();
        }
    }

    public final static String PRINT_INDICATION_STRING = "-m [<MsgID[, MsgID]...>]";
    public final static String UDP_STRING = "-udp <mav ip>:<mav port>";
    public final static String QGC_STRING = "-qgc"; // <qgc ip address>:<qgc peer port> <qgc bind port>
    public final static String SERIAL_STRING = "-serial [<path> <baudRate>]";
    public final static String REP_STRING = "-rep";
    public final static String AP_STRING = "-ap <autopilot_type>";
    public final static String SPEED_STRING = "-r <Hz>";
    public final static String CMD_STRING = "java -cp lib/*:out/production/jmavsim.jar me.drton.jmavsim.Simulator";
    public final static String CMD_STRING_JAR = "java -jar jmavsim_run.jar";
    public final static String USAGE_STRING = CMD_STRING_JAR + " [-h] [" + UDP_STRING + " | " + SERIAL_STRING + "] [" + SPEED_STRING + "] [" + AP_STRING + "] " + 
                                              "[" + QGC_STRING + "] [" + REP_STRING + "] [" + PRINT_INDICATION_STRING + "]";

    public static void main(String[] args)
            throws InterruptedException, IOException, ParserConfigurationException, SAXException {

        // default is to use UDP.
        if (args.length == 0) {
            USE_SERIAL_PORT = false;
        }
        if (args.length > 8) {
            System.err.println("Incorrect number of arguments. \n Usage: " + USAGE_STRING);
            return;
        }

        int i = 0;
        while (i < args.length) {
            String arg = args[i++];
            if (arg.equalsIgnoreCase("-h") || arg.equalsIgnoreCase("--help")) {
                handleHelpFlag();
                return;
            }
            if (arg.equalsIgnoreCase("-m")) {
                monitorMessage = true;
                if (i < args.length) {
                    String nextArg = args[i++];
                    try {
                        if (nextArg.startsWith("-")) {
                            // if user ONLY passes in -m, monitor all messages.
                            i--;
                            continue;
                        }
                        if (nextArg.contains(",")) {
                            String split[] = nextArg.split(",");
                            for (String s : split) {
                                monitorMessageIds.add(Integer.parseInt(s));
                            }
                        } else {
                            monitorMessageIds.add(Integer.parseInt(nextArg));
                        }
                    } catch (NumberFormatException e) {
                        System.err.println("Expected: " + PRINT_INDICATION_STRING + ", got: " + Arrays.toString(args));
                        return;
                    }
                } else {
                    // if user ONLY passes in -m, monitor all messages.
                    continue;
                }
            }
            else if (arg.equalsIgnoreCase("-udp")) {
                USE_SERIAL_PORT = false;
                if (i == args.length) {
                    // only arg is -udp, so use default values.
                    break;
                }
                if (i < args.length) {
                    String nextArg = args[i++];
                    if (nextArg.startsWith("-")) {
                        // only turning on udp, but want to use default ports
                        i--;
                        continue;
                    }
                    try {
                        // try to parse passed-in ports.
                        String[] list = nextArg.split(":");
                        if (list.length != 2) {
                            System.err.println("Expected: " + UDP_STRING + ", got: " + Arrays.toString(list));
                            return;
                        }
                        autopilotIpAddress = list[0];
                        autopilotPort = Integer.parseInt(list[1]);
                    } catch (NumberFormatException e) {
                        System.err.println("Expected: " + USAGE_STRING + ", got: " + e.toString());
                        return;
                    }
                } else {
                    System.err.println("-udp needs an argument: " + UDP_STRING);
                    return;
                }
            } else if (arg.equals("-serial")) {
                USE_SERIAL_PORT = true;
                if (i >= args.length) {
                    // only arg is -serial, so use default values
                    break;
                }
                String nextArg = args[i++];
                if (nextArg.startsWith("-")) {
                    i--;
                    continue;
                }
                if ( (i+1) <= args.length) {
                    try {
                        serialPath = nextArg;
                        serialBaudRate = Integer.parseInt(args[i++]);
                    } catch (NumberFormatException e) {
                        System.err.println("Expected: " + USAGE_STRING + ", got: " + e.toString());
                        return;
                    }
                } else {
                    System.err.println("-serial needs two arguments. Expected: " + SERIAL_STRING + ", got: " + Arrays.toString(args));
                    return;
                }
            } else if (arg.equals("-qgc")) {
                COMMUNICATE_WITH_QGC = true;
                // if (i < args.length) {
                //     String firstArg = args[i++];
                //     try {
                //         String[] list = firstArg.split(":");
                //         if (list.length == 1) {
                //             // Only one argument turns off QGC if the arg is -1
                //             //qgcBindPort = Integer.parseInt(list[0]);
                //             if (qgcBindPort < 0) {
                //                 COMMUNICATE_WITH_QGC = false;
                //                 continue;
                //             } else {
                //                 System.err.println("Expected: " + QGC_STRING + ", got: " + Arrays.toString(args));
                //                 return;
                //             }
                //         } else if (list.length == 2) {
                //             qgcIpAddress = list[0];
                //             qgcPeerPort = Integer.parseInt(list[1]);
                //         } else {
                //             System.err.println("-qgc needs the correct number of arguments. Expected: " + QGC_STRING + ", got: " + Arrays.toString(args));
                //             return;
                //         }
                //         if (i < args.length) {
                //             // Parsed QGC peer IP and peer Port, or errored out already
                //             String secondArg = args[i++];
                //             qgcBindPort = Integer.parseInt(secondArg);
                //         } else {
                //             System.err.println("Wrong number of arguments. Expected: " + QGC_STRING + ", got: " + Arrays.toString(args));
                //         }
                //     } catch (NumberFormatException e) {
                //         System.err.println("Expected: " + USAGE_STRING + ", got: " + e.toString());
                //         return;
                //     }
                // } else {
                //     System.err.println("-qgc needs an argument: " + QGC_STRING);
                //     return;
                // }
            } else if (arg.equals("-ap")) {
                if (i < args.length) {
                    autopilotType = args[i++];
                } else {
                    System.err.println("-ap requires the autopilot name as an argument.");
                    return;
                }
            } else if (arg.equals("-r")) {
                if (i < args.length) {
                    int t;
                    try {
                        t = Integer.parseInt(args[i++]);
                    } catch (NumberFormatException e) {
                        System.err.println("Expected numeric argument after -r: " + SPEED_STRING);
                        return;
                    }
                    sleepInterval = (int)1e6 / t;
                } else {
                    System.err.println("-r requires Hz as an argument.");
                    return;
                }
            } else if (arg.equals("-rep")) {
                SHOW_REPORT_PANEL = true;
            } else {
                System.err.println("Unknown flag: " + arg + ", usage: " + USAGE_STRING);
                return;
            }
        }

        if (i != args.length) {
            System.err.println("Usage: " + USAGE_STRING);
            return;
        } else { System.out.println("Options parsed, starting Sim.  Press x<ENTER> to exit."); }

        new Simulator();
    }

    private static void handleHelpFlag() {
        System.out.println("\nUsage: " + USAGE_STRING + "\n");
        System.out.println("Command-line options:\n");
        System.out.println(UDP_STRING);
        System.out.println("      Open a TCP/IP UDP connection to the MAV (default: " + autopilotIpAddress + ":" + autopilotPort + ").\n");
        System.out.println(SERIAL_STRING);
        System.out.println("      Open a serial connection to the MAV (default: " + serialPath + " @ " + serialBaudRate + ").\n");
        System.out.println(SPEED_STRING);
        System.out.println("      Refresh rate at which jMAVSim runs. This dictates the frequency of the HIL_SENSOR messages.");
        System.out.println("      Default is " + DEFAULT_SIM_SPEED + " Hz\n");
        System.out.println(AP_STRING);
        System.out.println("      Specify a specific MAV type. E.g. 'px4' or 'aq'. Default is: " + autopilotType + "\n");
        System.out.println(QGC_STRING);
        System.out.println("      Forward message packets to QGC via UDP at " + qgcIpAddress + ":" + qgcPeerPort + " bind:" + qgcBindPort + "\n");
        System.out.println(REP_STRING);
        System.out.println("      Start with data report visible (once started, use 'r' in console to toggle).\n");
        System.out.println(PRINT_INDICATION_STRING);
        System.out.println("      Monitor (echo) all/selected MAVLink messages to the console.");
        System.out.println("      If no MsgIDs are specified, all messages are monitored.\n");
        System.out.println("Key commands (in the console, press the key and then <ENTER>):");
        System.out.println(" Views:");
        System.out.println("   f - First-person camera.");
        System.out.println("   s - Stationary camera.");
        System.out.println("   g - Gimbal camera.");
        System.out.println(" Other:");
        System.out.println("   q - Disable sim on MAV.");
        System.out.println("   i - Enable sim on MAV.");
        System.out.println("   r - Show/hide data reports in visualizer window.");
        System.out.println("   x - Exit jMAVSim.");
        //System.out.println("\n Note: if <qgc <port> is set to -1, JMavSim won't generate Mavlink messages for GroundControl.");
    }

}

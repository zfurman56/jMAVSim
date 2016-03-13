package me.drton.jmavsim;

import me.drton.jmavlib.geo.LatLonAlt;
import me.drton.jmavlib.mavlink.MAVLinkSchema;
import me.drton.jmavsim.Visualizer3D.ViewTypes;
import me.drton.jmavsim.vehicle.AbstractMulticopter;
import me.drton.jmavsim.vehicle.Quadcopter;

import org.xml.sax.SAXException;

import javax.swing.JFrame;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import javax.xml.parsers.ParserConfigurationException;

import java.io.IOException;
import java.io.InputStream;
import java.lang.Math;
import java.net.URL;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Scanner;
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
    public static boolean DO_MAG_FIELD_LOOKUP = false;
    public static boolean SHOW_REPORT_PANEL = false;
    public static boolean GUI_START_MAXIMIZED = false;
    public static ViewTypes GUI_START_VIEW = ViewTypes.VIEW_FPV;
    
    public static final int    DEFAULT_SIM_SPEED = 800; // Hz
    public static final int    DEFAULT_AUTOPILOT_SYSID = -1; // System ID of autopilot to communicate with. -1 to auto set ID on first received heartbeat.
    public static final String DEFAULT_AUTOPILOT_TYPE = "generic";  // eg. "px4" or "aq"
    public static final int    DEFAULT_AUTOPILOT_PORT = 14560;
    public static final int    DEFAULT_QGC_BIND_PORT = 0;
    public static final int    DEFAULT_QGC_PEER_PORT = 14550;
    public static final String DEFAULT_SERIAL_PATH = "/dev/tty.usbmodem1";
    public static final int    DEFAULT_SERIAL_BAUD_RATE = 230400;
    public static final String LOCAL_HOST = "127.0.0.1";

    // Set global reference point
    // Zurich Irchel Park: 47.397742, 8.545594, 488m
    // Seattle downtown: 47.592182, -122.316031, 86m
    // Moscow downtown: 55.753395, 37.625427, 155m
    public static final LatLonAlt DEFAULT_ORIGIN_POS = new LatLonAlt(47.397742, 8.545594, 488);

    // Mag vector in earth field. If Y value is left as zero, then the
    //   declination will be added later based on the origin GPS position.
    // If DO_MAG_FIELD_LOOKUP = true or -automag switch is used then this value is ignored.
    // 
    // Zurich:  (0.44831f, 0.01664f, 0.89372f)
    // Seattle: (0.34252f, 0.09805f, 0.93438f)
    // Moscow:  (0.31337f, 0.06030f, 0.94771f)
    public static final Vector3d  DEFAULT_MAG_FIELD = new Vector3d(0.44831f, 0.01664f, 0.89372f);
    
    public static final int    DEFAULT_CAM_PITCH_CHAN = 7;     // Control gimbal pitch from autopilot, -1 to disable
    public static final int    DEFAULT_CAM_ROLL_CHAN  = 6;     // Control gimbal roll from autopilot, -1 to disable
    public static final Double DEFAULT_CAM_PITCH_SCAL = 1.57;  // channel value to physical movement (+/-90 deg)
    public static final Double DEFAULT_CAM_ROLL_SCAL  = 1.57;  // channel value to physical movement (+/-90 deg)

    private static int sleepInterval = (int)1e6 / DEFAULT_SIM_SPEED;  // Main loop interval, in us
    private static int autopilotSysId = DEFAULT_AUTOPILOT_SYSID;
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

    Visualizer3D visualizer;
    AbstractMulticopter vehicle;
    CameraGimbal2D gimbal;
    MAVLinkHILSystem hilSystem;
    MAVLinkPort autopilotMavLinkPort;
    UDPMavLinkPort udpGCMavLinkPort;
    ScheduledFuture<?> thisHandle;

    private World world;
//    private int simDelayMax = 500;  // Max delay between simulated and real time to skip samples in simulator, in ms
    private ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
    public volatile boolean shutdown = false;

    public Simulator() throws IOException, InterruptedException, ParserConfigurationException, SAXException {
        // Create world
        world = new World();
        LatLonAlt referencePos = DEFAULT_ORIGIN_POS;
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
            if (monitorMessage)
                port.setMonitorMessageID(monitorMessageIds);
            autopilotMavLinkPort = port;
        }

        // allow HIL and GCS to talk to this port
        connHIL.addNode(autopilotMavLinkPort);
        connCommon.addNode(autopilotMavLinkPort);
        // UDP port: connection to ground station
        udpGCMavLinkPort = new UDPMavLinkPort(schema);
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

        //simpleEnvironment.setWind(new Vector3d(0.0, 5.0, 0.0));
        simpleEnvironment.setWindDeviation(0.0);
        simpleEnvironment.setGroundLevel(0.0f);
        world.addObject(simpleEnvironment);

        // Set up magnetic field deviations 
        // (do this after environment already has a reference point in case we need to look up declination manually)
        Vector3d magField = DEFAULT_MAG_FIELD;
        if (DO_MAG_FIELD_LOOKUP)
            magField = magFieldLookup(referencePos);
        else if (magField.y == 0.0f) {
            // Set declination based on the initialization position of the Simulator
            // getMagDeclination() returns degrees and variable decl is in radians.
            double decl = Math.toRadians(world.getEnvironment().getMagDeclination(referencePos.lat, referencePos.lon));
            //System.out.println("Declination: " + (Math.toDegrees(decl)));
            Matrix3d magDecl = new Matrix3d();
            magDecl.rotZ(decl);
            magDecl.transform(magField);
        }
        simpleEnvironment.setMagField(magField);

        // Create vehicle with sensors
        vehicle = buildMulticopter();

        // Create MAVLink HIL system
        // SysId should be the same as autopilot, ComponentId should be different!
        hilSystem = new MAVLinkHILSystem(schema, autopilotSysId, 51, vehicle);
        hilSystem.setHeartbeatInterval(0);
        connHIL.addNode(hilSystem);
        world.addObject(vehicle);

        // Put camera on vehicle with gimbal
        if (DEFAULT_CAM_PITCH_CHAN > -1 || DEFAULT_CAM_ROLL_CHAN > -1) {
            gimbal = buildGimbal();
            world.addObject(gimbal);
        }

        // Create 3D visualizer
        visualizer = new Visualizer3D(world);
        visualizer.setHilSystem(hilSystem);
        visualizer.setVehicleViewObject(vehicle);
        if (gimbal != null)
            visualizer.setGimbalViewObject(gimbal);
        if (GUI_START_MAXIMIZED)
            visualizer.setExtendedState(JFrame.MAXIMIZED_BOTH);
        
        // set default view
        visualizer.setViewType(GUI_START_VIEW);

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

        thisHandle = executor.scheduleAtFixedRate(this, 0, sleepInterval, TimeUnit.MICROSECONDS);
        
        Runtime.getRuntime().addShutdownHook(new Thread() {
            @Override
            public void run() {
                try {
                    Thread.sleep(200);
                    
                    System.out.println("Shutting down...");
                    if (hilSystem != null)
                        hilSystem.endSim();
                    
                    // Close ports
                    if (autopilotMavLinkPort != null)
                        autopilotMavLinkPort.close();
                    if (udpGCMavLinkPort != null)
                        udpGCMavLinkPort.close();
                    
                    if (thisHandle != null)
                        thisHandle.cancel(true);
                    executor.shutdown();

                } catch (InterruptedException | IOException e) {
                    e.printStackTrace();
                }
            }
        });

        while(true) { 
            Thread.sleep(100);
        
            if (shutdown)
                break;
        }
        
        System.exit(0);
    }

    private AbstractMulticopter buildMulticopter() throws IOException {
        Vector3d gc = new Vector3d(0.0, 0.0, 0.0);  // gravity center
        AbstractMulticopter vehicle = new Quadcopter(world, "models/3dr_arducopter_quad_x.obj", "x", "cw_fr", 0.1, 1.35, 0.02, 0.0005, gc);
        vehicle.setMass(0.25);
        Matrix3d I = new Matrix3d();
        // Moments of inertia
        I.m00 = 0.0017;  // X
        I.m11 = 0.0017;  // Y
        I.m22 = 0.002;  // Z
        vehicle.setMomentOfInertia(I);
        SimpleSensors sensors = new SimpleSensors();
        sensors.setGPSInterval(50);
        sensors.setGPSDelay(10);
        //sensors.setGPSStartTime(-1);
        sensors.setPressureAltOffset(world.getGlobalReference().alt);
        vehicle.setSensors(sensors);
        vehicle.setDragMove(0.01);
        //v.setDragRotate(0.1);
        
        return vehicle;
    }

    private CameraGimbal2D buildGimbal() {
        CameraGimbal2D g = new CameraGimbal2D(world);
        g.setBaseObject(vehicle);
        g.setPitchChannel(DEFAULT_CAM_PITCH_CHAN);
        g.setPitchScale(DEFAULT_CAM_PITCH_SCAL); 
        g.setRollChannel(DEFAULT_CAM_ROLL_CHAN);
        g.setRollScale(DEFAULT_CAM_ROLL_SCAL);
        return g;
    }

    public void run() {
        try {
            world.update(System.currentTimeMillis());
        }
        catch (Exception e) {
            System.err.println("Exception in Simulator.world.update() : ");
            e.printStackTrace();
            executor.shutdown();
        }
    }

    /**
     * Look up the magnetic inclination and declination for a given Lat/Lon/Alt using a NOAA Web service.
     * If successful, returns a valid Vector3d() suitable for setting the magnetic field in the simulated environment.
     * Also displays the resulting vector so it can be copied to the DEFAULT_MAG_FIELD setting to avoid future lookups.
     *
     * @param pos {@link me.drton.jmavlib.geo.LatLonAlt} object of reference point.
     * @return Vector3d The magnetic field variance vector, or Vector3d(0,0,0) if lookup failed.
     */
    public static Vector3d magFieldLookup(LatLonAlt pos) {

        Double decl;
        Double incl;
        Vector3d magField = new Vector3d(0.0f, 0.0f, 0.0f);
        String resp, vals[];
        
        String reqUrl = "http://www.ngdc.noaa.gov/geomag-web/calculators/calculateIgrfwmm?";
        reqUrl += "resultFormat=csv&coordinateSystem=M&lat1=" + pos.lat + "&lon1=" + pos.lon + "&elevation=" + pos.alt / 1e3;
        System.out.println("Attempting magnetic field data lookup from NOAA...");
        try {
            InputStream instr = new URL(reqUrl).openStream();
            Scanner scan = new Scanner(instr, "UTF-8");
            resp = scan.useDelimiter("\\A").hasNext() ? scan.next() : "";
            String lines[] = resp.split("\n");
            vals = lines[lines.length-1].split(",");
            scan.close();
            instr.close();
        } catch (IOException e) {
            System.err.println("Error requesting URL: " + reqUrl + "\n");
            return magField;
        }

        if (vals.length > 3) {
            try {
                decl = new Double(vals[1]);
                incl = new Double(vals[2]);
            } catch (NumberFormatException e) {
                System.err.println("Error parsing response: " + resp + "\n");
                return magField;
            }
            System.out.println("Lookup Declination: " + decl + "; Inclination: " + incl);
            decl = Math.toRadians(decl);
            incl = Math.toRadians(incl);
            magField = new Vector3d(Math.cos(incl), 0.0f, Math.sin(incl));
            Matrix3d declMtx = new Matrix3d();
            declMtx.rotZ(decl);
            declMtx.transform(magField);
            System.out.printf("Result Vectors: Vector3d(%.5f, %.5f, %.5f) \n", magField.x, magField.y, magField.z);
            System.out.printf("       Declination: %.5f; Inclination: %.5f \n", Math.toDegrees(Math.atan2(magField.y, magField.x)), Math.toDegrees(Math.atan2(magField.z, magField.x)));
        } else
            System.err.println("Error parsing response: " + resp + "\n");
        
        return magField;
    }
    
    public final static String PRINT_INDICATION_STRING = "-m [<MsgID[, MsgID]...>]";
    public final static String UDP_STRING = "-udp <mav ip>:<mav port>";
    public final static String QGC_STRING = "-qgc"; // <qgc ip address>:<qgc peer port> <qgc bind port>
    public final static String SERIAL_STRING = "-serial [<path> <baudRate>]";
    public final static String MAG_STRING = "-automag";
    public final static String REP_STRING = "-rep";
    public final static String GUI_MAX_STRING = "-max";
    public final static String GUI_VIEW_STRING = "-view (fpv|grnd|gmbl)";
    public final static String AP_STRING = "-ap <autopilot_type>";
    public final static String SPEED_STRING = "-r <Hz>";
    public final static String CMD_STRING = "java -cp lib/*:out/production/jmavsim.jar me.drton.jmavsim.Simulator";
    public final static String CMD_STRING_JAR = "java -jar jmavsim_run.jar";
    public final static String USAGE_STRING = CMD_STRING_JAR + " [-h] [" + UDP_STRING + " | " + SERIAL_STRING + "] [" + SPEED_STRING + "] [" + AP_STRING + "] [" + MAG_STRING + "] " + 
                                              "[" + QGC_STRING + "] [" + GUI_MAX_STRING + "] [" + GUI_VIEW_STRING + "] [" + REP_STRING + "] [" + PRINT_INDICATION_STRING + "]";

    public static void main(String[] args)
            throws InterruptedException, IOException, ParserConfigurationException, SAXException {

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
            } else if (arg.equals("-view")) {
                String t;
                if (i < args.length) {
                    t = args[i++];
                    if (t.equals("fpv"))
                        GUI_START_VIEW = ViewTypes.VIEW_FPV;
                    else if (t.equals("grnd"))
                        GUI_START_VIEW = ViewTypes.VIEW_STATIC;
                    else if (t.equals("gmbl"))
                        GUI_START_VIEW = ViewTypes.VIEW_GIMBAL;
                    else
                        System.out.println("Warning: Unrecognized value for -view option, ignoring.");
                } else {
                    System.err.println("-view requires an argument: " + GUI_VIEW_STRING);
                    return;
                }
            } else if (arg.equals("-automag")) {
                DO_MAG_FIELD_LOOKUP = true;
            } else if (arg.equals("-rep")) {
                SHOW_REPORT_PANEL = true;
            } else if (arg.equals("-max")) {
                GUI_START_MAXIMIZED = true;
            } else {
                System.err.println("Unknown flag: " + arg + ", usage: " + USAGE_STRING);
                return;
            }
        }

        if (i != args.length) {
            System.err.println("Usage: " + USAGE_STRING);
            return;
        } else { 
            System.out.println("Options parsed, starting Sim."); 
        }

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
        System.out.println("      Specify the MAV type. E.g. 'px4' or 'aq'. Default is: " + autopilotType + "\n");
        System.out.println(MAG_STRING);
        System.out.println("      Attempt automatic magnetic field inclination/declination lookup for starting position via NOAA Web service.\n");
        System.out.println(QGC_STRING);
        System.out.println("      Forward message packets to QGC via UDP at " + qgcIpAddress + ":" + qgcPeerPort + " bind:" + qgcBindPort + "\n");
        System.out.println(GUI_VIEW_STRING);
        System.out.println("      Start with the specified view type. One of: 'fpv', 'grnd', or 'gmbl'. Default is 'fpv'.\n");
        System.out.println(GUI_MAX_STRING);
        System.out.println("      Start with the visualizer GUI window maximized.\n");
        System.out.println(REP_STRING);
        System.out.println("      Start with data report visible (once started, use 'r' in console to toggle).\n");
        System.out.println(PRINT_INDICATION_STRING);
        System.out.println("      Monitor (echo) all/selected MAVLink messages to the console.");
        System.out.println("      If no MsgIDs are specified, all messages are monitored.\n");
        System.out.println("Key commands (in the visualizer window):");
        System.out.println(" Views:");
        System.out.println("   F - First-person camera.");
        System.out.println("   S - Stationary camera.");
        System.out.println("   G - Gimbal camera.");
        System.out.println(" Other:");
        System.out.println("   Q   - Disable sim on MAV.");
        System.out.println("   I   - Enable sim on MAV.");
        System.out.println("   R   - Show/hide data reports.");
        System.out.println("   T   - Pause/resume data report updates.");
        System.out.println(" SPACE - Reset vehicle & view to start position.");
        System.out.println("  ESC  - Exit jMAVSim.");
        System.out.println("");
        //System.out.println("\n Note: if <qgc <port> is set to -1, JMavSim won't generate Mavlink messages for GroundControl.");
    }

}

package me.drton.jmavsim;

import me.drton.jmavlib.mavlink.MAVLinkSchema;
import me.drton.jmavlib.mavlink.MAVLinkStream;
import me.drton.jmavlib.mavlink.MAVLinkMessage;

import java.io.IOException;
import java.net.*;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.util.*;

/**
 * User: ton Date: 02.12.13 Time: 20:56
 */
public class UDPMavLinkPort extends MAVLinkPort {
    private MAVLinkSchema schema;
    private DatagramChannel channel = null;
    private ByteBuffer rxBuffer = ByteBuffer.allocate(8192);
    private SocketAddress bindPort = null;
    private SocketAddress peerPort;
    private MAVLinkStream stream;
    private boolean debug = false;

    private boolean monitorMessage = false;
    private HashSet<Integer> monitorMessageIDs;
    private HashMap<Integer, Integer> messageCounts = new HashMap<Integer, Integer>();
    
    static String[] LOCAL_HOST_TERMS = { "localhost", "127.0.0.1" };
    static int MONITOR_MESSAGE_RATE = 100; // rate at which to print message info
    static int TIME_PASSING = 10;         // change the print so it's visible to the user.
    static int time = 0;


    public UDPMavLinkPort(MAVLinkSchema schema) {
        super(schema);
        this.schema = schema;
        rxBuffer.flip();
    }

    public void setMonitorMessageID(HashSet<Integer> ids) {
        this.monitorMessageIDs  = ids;
        for (int id : ids) {
            messageCounts.put(id, 0);
        }
        this.monitorMessage = true;
    }

    public void setDebug(boolean debug) {
        this.debug = debug;
    }

    public void setup(int bindPort, String peerAddress, int peerPort) throws UnknownHostException, IOException {
        this.peerPort = new InetSocketAddress(peerAddress, peerPort);
        // If PX4 is running on localhost, we should connect on local host as well.
        for (String term : LOCAL_HOST_TERMS) {
            if (peerAddress.equalsIgnoreCase(term)) {
                this.bindPort = new InetSocketAddress(term, bindPort);
            }
        }
        // Otherwise, we should attempt to find the external IP address and connect over that.
        if (this.bindPort == null) {
            InetAddress localHostExternalIPAddress = getMyHostIPAddress();
            this.bindPort = new InetSocketAddress(localHostExternalIPAddress, bindPort);
        }
        if (debug) {
            System.out.println("peerAddress: " + peerAddress + ", bindAddress: " + this.bindPort.toString());
        }
    }

    /**
     * Searches for the externally-reachable IP address for this machine. Note that if PX4 is running on
     * a private network, this method may or may not work to setup communication.
     *
     * @return the best possible address found.
     * @throws UnknownHostException
     * @throws SocketException
     * @throws IOException
     */
    private static InetAddress getMyHostIPAddress() throws UnknownHostException, SocketException, IOException {
        InetAddress possibleAddress = null;
        // Look over all the network interfaces for the appropriate interface.
        Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();
        while (networkInterfaces.hasMoreElements()) {
            NetworkInterface iface = networkInterfaces.nextElement();
            Enumeration<InetAddress> addresses = iface.getInetAddresses();
            while (addresses.hasMoreElements()) {
                InetAddress address = addresses.nextElement();
                if (!address.isLoopbackAddress()) {
                    if (address.isSiteLocalAddress()) {
                        // probably a good one!
                        return address;
                    } else {
                        // Found a non-loopback address that isn't site local.
                        // Might be link local (private network), but probably better than nothing.
                        possibleAddress = address;
                        // Return the first IPV4 address we find.
                        if(address instanceof Inet4Address) {
                            return address;
                        } else {
                            System.out.println("Found a non-IPv4 address: " + address);
                            // IPv6 may cause the system to crash
                            possibleAddress = null;
                        }
                    }
                }
            }
        }
        // At this point, if we haven't found a better option, we better just take whatever Java thinks is best.
        if (possibleAddress == null) {
            possibleAddress = InetAddress.getLoopbackAddress();
        }
        return possibleAddress;
    }

    public void open() throws IOException {
        channel = DatagramChannel.open();
        channel.socket().bind(bindPort);
        channel.configureBlocking(false);
        channel.connect(peerPort);
        stream = new MAVLinkStream(schema, channel);
    }

    @Override
    public void close() throws IOException {
        if (channel != null) {
            channel.close();
        }
    }

    @Override
    public boolean isOpened() {
        return channel != null && channel.isOpen();
    }

    @Override
    public void handleMessage(MAVLinkMessage msg) {
        if (debug) System.out.println("[handleMessage] msg.name: " + msg.getMsgName() + ", type: " + msg.getMsgType());

        try {
            /*SocketAddress remote =*/ channel.getRemoteAddress();
        } catch (IOException e) {
            System.err.println(e.toString());
        }


        if (isOpened()) {
            try {
                stream.write(msg);
                IndicateReceivedMessage(msg.getMsgType());
            } catch (IOException ignored) {
                // Silently ignore this exception, we likely just have nobody on this port yet/already
            }
        }
    }

    private void IndicateReceivedMessage(int type) {
        if (monitorMessage) {
            boolean shouldPrint = false;
            int count = 0;
            // if the list of messages to monitor is empty, but the flag is on, monitor all messages.
            if (monitorMessageIDs.isEmpty()) {
                if (messageCounts.containsKey(type)) count = messageCounts.get(type);
                shouldPrint = count >= MONITOR_MESSAGE_RATE;
            } else {
                // otherwise, only print messages in the list of message IDs we're monitoring.
                if (messageCounts.containsKey(type)) count = messageCounts.get(type);
                shouldPrint = count >= MONITOR_MESSAGE_RATE && monitorMessageIDs.contains(type);
            }
            printMessage(shouldPrint, count, type);
        }
    }

    private void printMessage(boolean should, int count, int type) {
        if (should) {
            System.out.println(type);
            messageCounts.put(type, 0);
            if (time >= TIME_PASSING) {
                System.out.println("---");
                time = 0;
            } else {
                time++;
            }
        } else {
            messageCounts.put(type, count+1);
        }
    }

    @Override
    public void update(long t) {
        while (isOpened()) {
            try {
                MAVLinkMessage msg = stream.read();
                if (msg == null)
                    break;
                if (debug) 
                    System.out.println("[update] msg.name: " + msg.getMsgName() + ", type: " + msg.getMsgType());
                IndicateReceivedMessage(msg.getMsgType());
                sendMessage(msg);
            } catch (IOException e) {
                // Silently ignore this exception, we likely just have nobody on this port yet/already
                return;
            }
        }
    }
}

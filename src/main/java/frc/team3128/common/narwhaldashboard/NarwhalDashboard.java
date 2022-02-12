package frc.team3128.common.narwhaldashboard;

import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.LinkedHashMap;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import frc.team3128.common.utility.Log;
import frc.team3128.common.hardware.limelight.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class NarwhalDashboard extends WebSocketServer {
    private static final int PORT = 5805;
    private final static int UPDATE_WAVELENGTH = 100;

    public static int getUpdateWavelength() {
        return UPDATE_WAVELENGTH;
    }

    private static HashMap<String, String> debugValues = new HashMap<String, String>();
    private static LinkedHashMap<String, Command> autoPrograms = new LinkedHashMap<String, Command>();

    private static HashMap<String, DashButtonCallback> buttons = new HashMap<String, DashButtonCallback>();
    private static HashMap<String, NumericalDataCallback> numDataCallbacks = new HashMap<String, NumericalDataCallback>();

    private static HashMap<String, Limelight> limelights = new HashMap<String, Limelight>();

    private static String selectedAuto = null;
    private static String selectedLimelight = null;
    private static boolean pushed = false;

    public NarwhalDashboard(int port) throws UnknownHostException {
        super(new InetSocketAddress(port));
    }

    public NarwhalDashboard(InetSocketAddress address) {
        super(address);
    }

    /**
     * Publishes a numerical value to NarwhalDashboard
     */
    public static void put(String key, double value) {
        put(key, Double.toString(value));
    }

    /**
     * Publishes a boolean value to NarwhalDashboard
     */
    public static void put(String key, boolean value) {
        put(key, Boolean.toString(value));
    }

    /**
     * Publishes a string value to NarwhalDashboard
     */
    public static void put(String key, String value) {
        debugValues.put(key, value);
    }

    public static void addButton(String key, DashButtonCallback callback) {
        buttons.put(key, callback);
    }

    public static void addNumDataListener(String key, NumericalDataCallback callback) {
        numDataCallbacks.put(key, callback);
    }

    /**
     * Clears the list of autonomous programs.
     */
    public static void clearAutos() {
        autoPrograms = new LinkedHashMap<String, Command>();
    }

    /**
     * Adds an autonomous program to NarwhalDashboard's auto picker
     * 
     * @param name    - The human-readable name of the autonomous program
     * @param program - The auto program to run if this element is chosen
     */
    public static void addAuto(String name, Command program) {
        autoPrograms.put(name, program);
    }

    /**
     * Sends new set of initial data to NarwhalDashboard.
     */
    public static void pushData() {
        pushed = false;
    }

    public static void addLimelight(Limelight light) {
        limelights.put(light.hostname, light);
    }

    /**
     * Returns the currently selected auto program
     */
    public static Command getSelectedAuto() {
        if (selectedAuto == null)
            return null;

        // Redundant to onMessage()
        // if (!autoPrograms.containsKey(selectedAuto)) {
        //     Log.recoverable("NarwhalDashboard", "Auto program \"" + selectedAuto
        //             + "\" does not exist. Perhaps it was deleted between its selection and the beginning of the autonomous period?");
        // }

        return autoPrograms.get(selectedAuto);
    }

    /**
     * Starts the NarwhalDashboard server. This opens it up to be able to be
     * connected to by client devices (the DS Laptop, a tablet controller, etc) and
     * begins streaming data.
     */
    public static void startServer() {
        NarwhalDashboard s;
        try {
            s = new NarwhalDashboard(PORT);
            s.setReuseAddr(true);
            s.start();

            Log.info("NarwhalDashboard", "Server has started on port " + PORT);
        } catch (UnknownHostException e) {
            e.printStackTrace();
        }
    }

    // Called once on connection with web server
    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        Log.info("NarwhalDashboard", conn.getRemoteSocketAddress().getHostName() + " has opened a connection.");

        pushed = false;

        (new Thread(() -> {
            while (conn.isOpen()) {
                JSONObject obj = new JSONObject();

                SmartDashboard.putNumber("Debug", debugValues.size());

                JSONArray debugArr = new JSONArray();
                for (String key : debugValues.keySet()) {
                    JSONObject pair = new JSONObject();
                    pair.put("key", key);
                    pair.put("value", debugValues.get(key));
                    debugArr.add(pair);
                }
                obj.put("debug", debugArr);
               
                obj.put("selected_auto", selectedAuto);
                obj.put("selected_limelight", selectedLimelight);
                if(selectedLimelight != null) {
                    obj.put("selected_pipeline", limelights.get(selectedLimelight).getSelectedPipeline());
                }
                if(!pushed) {
                    JSONArray autoProgramArr = new JSONArray();
                    for (String autoName : autoPrograms.keySet()) {
                        autoProgramArr.add(autoName);
                    }
                    obj.put("auto", autoProgramArr);

                    JSONArray limelightsArr = new JSONArray();
                    for(Limelight lime : limelights.values()) {
                        limelightsArr.add(lime.hostname);
                    }
                    obj.put("limelight", limelightsArr);

                    JSONArray limelightsOptionsArr = new JSONArray();
                    for(Pipeline pipeline : Pipeline.values()) {
                        limelightsOptionsArr.add(pipeline.toString());
                    }
                    obj.put("pipeline", limelightsOptionsArr);

                    pushed = true;
                }
                
                conn.send(obj.toJSONString());
                
                try {
                    Thread.sleep(UPDATE_WAVELENGTH);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

        })).start();
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        // Log.info("NarwhalDashboard", conn.getRemoteSocketAddress().getHostName() + "
        // has closed its connection.");
    }

    // Called by request from web server 
    @Override
    public void onMessage(WebSocket conn, String message) {
        Log.info("NarwhalDashboard", message);
        String[] parts = message.split(":");

        // Receive auto selection
        if (parts[0].equals("selectauto")) {
            String programName = parts[1];

            if (programName.equals("null")) {
                selectedAuto = null;
            } else if (autoPrograms.containsKey(programName)) {
                selectedAuto = programName;
                SmartDashboard.putString("Auto", programName);
                Log.info("NarwhalDashboard", "Selected auto program: \"" + selectedAuto + "\"");
            } else {
                Log.recoverable("NarwhalDashboard", "Auto program \"" + programName + "\" does not exist.");
            }

        // Receive numerical data (for debug only afaik)
        } else if (parts[0].equals("numData")) {
            String key = parts[1];
            String list = parts[2];

            String[] stringData = list.split(",");
            double[] data = new double[stringData.length];

            for (int i = 0; i < stringData.length; i++) {
                data[i] = Double.parseDouble(stringData[i]);
            }

            if (numDataCallbacks.containsKey(key)) {
                numDataCallbacks.get(key).process(data);
            } else {
                Log.info("NarwhalDashboard", "Recieved, but will not process, numerical data: " + key + " = " + data);
            }

        // Receive input data (clicking a button on the dash to activate commands)
        } else if (parts[0].equals("button")) {
            String key = parts[1];
            boolean down = parts[2].equals("down");

            if (buttons.containsKey(key)) {
                buttons.get(key).action(down);
            } else {
                Log.recoverable("NarwhalDashboard", "Button \"" + parts[1] + "\" was never added.");
            }

        // Receive limelight selection (could be consolidated with pipeline)
        } else if(parts[0].equals("selectlimelight")){
                selectedLimelight = parts[1];

                if(selectedLimelight.equals("null")){
                    selectedLimelight = null;
                } else {
                    Log.info("NarwhalDashboard", "Unable to Parse Limelight Change Request from Dashboard");
                }

        // Receive pipeline selection (could be consolidated with limelight)
        } else if(parts[0].equals("selectpipeline")) {
                String pipelineStr = parts[1];

                if(pipelineStr.equals("null")){
                    pipelineStr = null;
                } else if(limelights.containsKey(selectedLimelight)) {
                    limelights.get(selectedLimelight).setPipeline(Pipeline.valueOf(pipelineStr));
                }
                else {
                    Log.info("NarwhalDashboard", "Unable to Parse Pipeline Change Request from Dashboard");
                }
        } else {
            Log.info("NarwhalDashboard", "Message recieved: " + message);
        }
        
    }

    @Override
    public void onError(WebSocket conn, Exception ex) {
        ex.printStackTrace();
    }

    @Override
    public void onStart() {

    }
}

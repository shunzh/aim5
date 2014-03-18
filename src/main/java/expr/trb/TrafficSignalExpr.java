package expr.trb;

import aim4.config.Condor;
import aim4.config.Debug;
import aim4.config.SimConfig;
import aim4.driver.pilot.V2IPilot;
import aim4.gui.Viewer;
import aim4.map.BasicMap;
import aim4.sim.Simulator;
import aim4.sim.setup.ApproxNPhasesTrafficSignalSimSetup;
import aim4.sim.setup.ApproxStopSignSimSetup;
import aim4.sim.setup.AutoDriverOnlySimSetup;
import aim4.sim.setup.BasicSimSetup;
import aim4.util.Util;


public class TrafficSignalExpr {

  private enum SIM_TYPE {
    FCFS,
    APPROX_TRAFFIC_SIGNAL,
    APPROX_STOP_SIGN,
  }
  
  private static final boolean GENERATE_BASELINE = false;
  private static final boolean SHOW_GUI = (GENERATE_BASELINE || true);

  /**
   * The main function of this experiment
   * 
   * @param args  the command line argument:
   *   args[0] = the type of the simulation
   *   args[1] = traffic pattern/signal pattern data directory
   *   args[2] = static buffer size for FCFS
   *   args[3] = time buffer size for FCFS
   *   args[4] = edge time buffer size for FCFS
   */
  public static void main(String[] args) {
    
    /////////////////////////////////
    // Settings
    /////////////////////////////////

    // the number of (simulated) seconds the simulator should run
    SimConfig.TOTAL_SIMULATION_TIME = 3600.0; // seconds
        
    if (args.length == 0) {
      args = new String[]{"SIGNAL", "data/2phases", ".25", "0.10", "0.25"};
      // args = new String[]{"FCFS", "2phases", ".25", "0.10", "0.25"};
      // args = new String[]{"FCFS", "2phases", ".50", "0.50", "1.00"};
    }
    if (GENERATE_BASELINE) {
      args[0] = "FCFS";
    }

    SIM_TYPE simType;

    if (args[0].equals("FCFS")) {
      simType = SIM_TYPE.FCFS;
    } else if (args[0].equals("SIGNAL")) {
      simType = SIM_TYPE.APPROX_TRAFFIC_SIGNAL;
    } else if (args[0].equals("STOP")) {
      simType = SIM_TYPE.APPROX_STOP_SIGN;
    } else {
      throw new RuntimeException("Incorrect arguments: the sim type should " +
          "be equal to FCFS, LIGHT, or STOP.");
    }

    String trafficSignalPhaseFileName = args[1] + "/AIM4Phases.csv";
    String trafficVolumeFileName = args[1] + "/AIM4Volumes.csv";

//    double staticBufferSize = 0.25;
//    double internalTileTimeBufferSize = 0.1;
//    double edgeTileTimeBufferSize = 0.25;

    double staticBufferSize = Double.parseDouble(args[2]);
    double internalTileTimeBufferSize = Double.parseDouble(args[3]);
    double edgeTileTimeBufferSize = Double.parseDouble(args[4]);


    BasicSimSetup basicSimSetup =
        new BasicSimSetup(2, // columns
                          1, // rows
                          4, // lane width
                          25.0, // speed limit
                          2, // lanes per road
                          1, // median size
                          150, // distance between
                          0.28, // traffic level
                                // (for now, it can be any number)
                          1.0 // stop distance before intersection
        );

    BasicSimSetup basicSimSetup2 = null;
    // ReservationGridManager.Config fcfsPolicyConfig = null;
    
    switch(simType) {
    case FCFS:
      boolean isEdgeTileTimeBufferEnabled = true;
      double granularity = 1.0;

      AutoDriverOnlySimSetup autoDriverOnlySimSetup =
          new AutoDriverOnlySimSetup(basicSimSetup);
      autoDriverOnlySimSetup.setIsBaseLineMode(GENERATE_BASELINE);
      autoDriverOnlySimSetup.setBuffers(staticBufferSize,
                                        internalTileTimeBufferSize,
                                        edgeTileTimeBufferSize,
                                        isEdgeTileTimeBufferEnabled,
                                        granularity);
      autoDriverOnlySimSetup.setTrafficVolume(trafficVolumeFileName);
      basicSimSetup2 = autoDriverOnlySimSetup;
      break;
    case APPROX_TRAFFIC_SIGNAL:
      ApproxNPhasesTrafficSignalSimSetup approxNPhasesTrafficSignalSimSetup =
          new ApproxNPhasesTrafficSignalSimSetup(basicSimSetup,
                                                 trafficSignalPhaseFileName);
      approxNPhasesTrafficSignalSimSetup.setTrafficVolume(trafficVolumeFileName);
      basicSimSetup2 = approxNPhasesTrafficSignalSimSetup;
      
      Debug.SHOW_VEHICLE_COLOR_BY_MSG_STATE = false;
      break;
    case APPROX_STOP_SIGN:
      ApproxStopSignSimSetup approxStopSignSimSetup =
          new ApproxStopSignSimSetup(basicSimSetup);
      approxStopSignSimSetup.setTrafficVolume(trafficVolumeFileName);
      SimConfig.MUST_STOP_BEFORE_INTERSECTION = true;
      Debug.SHOW_VEHICLE_COLOR_BY_MSG_STATE = false;
      basicSimSetup2 = approxStopSignSimSetup;
      break;
    }
    
    V2IPilot.DEFAULT_STOP_DISTANCE_BEFORE_INTERSECTION = 1.0;
    
    
    /////////////////////////////////
    // Instantiate the simulator
    /////////////////////////////////

//    AutoDriverOnlySimSetup autoDriverOnlySimSetup =
//        new AutoDriverOnlySimSetup(basicSimSetup);


    /////////////////////////////////
    // Run the simulator
    /////////////////////////////////

    if (SHOW_GUI) {
      new Viewer(basicSimSetup2, true);
    } else {
      // get the simulatior
      Simulator sim = basicSimSetup2.getSimulator();
      // run the simulator
      double currentTime = 0.0;
      while (currentTime <= SimConfig.TOTAL_SIMULATION_TIME) {
        Debug.clearShortTermDebugPoints();
        sim.step(SimConfig.TIME_STEP);
        currentTime += SimConfig.TIME_STEP;
      }
      
      /////////////////////////////////
      // Generate data files
      /////////////////////////////////

      BasicMap map = sim.getMap();

      // output the collected data of DCL

      String dclOutFileName =
        "ts_dcl_" + Util.concatenate(args, "_").replace('/', '_')
        + Condor.CONDOR_FILE_SUFFIX + ".csv";
      map.printDataCollectionLinesData(dclOutFileName);

      // dump the data collected at the intersection
//      for(IntersectionManager im : map.getIntersectionManagers()) {
//        String fcfsOutFileName =
//          "mre_im" + im.getId() + "_" + Util.concatenate(args, "_") + "_trial"
//          + Condor.CONDOR_ID + ".csv";
//         im.printData(fcfsOutFileName);
//      }

      // done
      System.out.println("Done.");
    }

  }
}

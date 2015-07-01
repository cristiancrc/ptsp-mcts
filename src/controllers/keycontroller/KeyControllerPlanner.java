package controllers.keycontroller;
import java.awt.Color;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;

import planners.Planner;
import planners.Planner2Opt;
import planners.Planner3Opt;
import planners.PlannerBruteForce;
import planners.PlannerGreedy;
import planners.PlannerGreedyCost_w;
import planners.PlannerMC;

import framework.core.Controller;
import framework.core.FuelTank;
import framework.core.Game;
import framework.core.GameObject;
import framework.core.Waypoint;
import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;
import framework.utils.Painter;
import framework.utils.Vector2d;

/**
 * This class is used for the KeyController (human playing).
 * PTSP-Competition
 * Created by Diego Perez, University of Essex.
 * Date: 20/12/11
 * Cristian : added planner call
 */
public class KeyControllerPlanner extends KeyController
{
    /**
     * To manage the keyboard input.
     */
    private KeyInput m_input;

    /**
     * Graph for this controller.
     */
    private Graph m_graph;

    /**
     * Node in the graph, closest to the ship position.
     */
    private Node m_shipNode;

    /**
     * Path to the closest waypoint in the map
     */
    private Path m_pathToClosest;
    
    /**
     * Path to all waypoints in the map
     */
	private Path[] m_pathToWaypoints;
	private Path[] m_pathToFuelTanks;

    /**
     * Hash map that matches waypoints in the map with their closest node in the graph.
     */
    private HashMap<GameObject, Node> m_collectNodes;

    /**
     * Closest waypoint to the ship.
     */
    private GameObject m_closestPickUp;

    /**
     * Distance to fuel tank penalization: as collecting a fuel tank does not reset the timer, it might be dangerous going after a fuel tank
     * instead of a waypoint. Hence, we apply this value to the distance in order to go only after those that are really closer than a waypoint.
     */
    private float FUEL_TANK_PEN = 1.9f;
    
    /**
     * Constructor of the KeyController.
     */
    
    Path pathToClosest;
    private ArrayList<Path> aPlannedPath = new ArrayList<>();// Paths to the waypoints based on the planner
	static double w_lava = 2;//l increase distance when passing over lava 1
	static double w_distance = 1;//b1 travel distance 1
	static double w_directness = 0;//b2 ratio between travel distance and euclidean distance 150
	static double w_angle = 0;//b3 angle change between entering and exiting a waypoint 80
	
	static boolean p_includeFuel = true;//b4 include fuel tanks in planning false
	static double w_fuelTankCost = 200;//b5 cost of picking up a fuel tank 200
	static double w_consecutiveFuelTanksCost = 1000;//b6 cost of picking up consecutive fuel tanks 1000
    
    
    
    public KeyControllerPlanner(Game a_gameCopy, long a_timeDue)
    {
        System.out.println("**** keyboard controller with planner ****");
        m_input = new KeyInput();
        
        m_graph = new Graph(a_gameCopy);
//        Planner planner = new PlannerBruteForce(a_gameCopy);
//        Planner planner = new PlannerGreedy(a_gameCopy);
//        Planner planner = new PlannerMC(a_gameCopy, a_timeDue);
//        Planner planner = new Planner2Opt(a_gameCopy);
        Planner planner = new Planner3Opt(a_gameCopy);
        

        Planner.weightLava = w_lava;
        Planner.weightDistance = w_distance;
        Planner.weightDirectness = w_directness;
        Planner.weightAngle = w_angle;
    	Planner.weightFuelTankCost = w_fuelTankCost;
    	Planner.weightConsecutiveFuelTanksCost = w_consecutiveFuelTanksCost;
    	Planner.includeFuel = p_includeFuel;
        
    	planner.runPlanner();    	
    	
    	//compare flood fill result with node distance result
//    	HashMap<GameObject, Double[][]> distanceMapList = new HashMap<GameObject, Double[][]>();
//    	long totalFillTime = 0;
//    	for (GameObject way : orderedWaypoints)
//    	{
//    		long timeStart = System.currentTimeMillis();
//            Double[][] aMap = planner.computeDistanceMap(a_gameCopy.getMap(), (int)way.s.x, (int)way.s.y);
//            distanceMapList.put(way, aMap);           
//            long timeEnd = System.currentTimeMillis();
//            System.out.println("---- Total time spent for one fill: " + (timeEnd - timeStart) + " ms.");
//            totalFillTime += (timeEnd - timeStart);
//    	}
//    	System.out.println("...total fill time :  " + totalFillTime);
//    	
//    	for (GameObject way : orderedWaypoints)
//    	{
//    		System.out.println("\nfrom ship...");
//    		Double[][] localMap = distanceMapList.get(way);
//    		System.out.println("fill: " + localMap[(int) a_gameCopy.getShip().s.x][(int) a_gameCopy.getShip().s.y]);
//        	Waypoint wpShip = new Waypoint(a_gameCopy, a_gameCopy.getShip().s);        	
//    		System.out.println("distance : " + Planner.distanceMatrix.get(wpShip).get(way));
//    	}
        planner.calculateOrderedWaypointsPaths();//calculate the paths from one waypoint to another
        aPlannedPath = planner.getPlannedPath();//get the path from one waypoint to the next        
        
        m_pathToFuelTanks = new Path[a_gameCopy.getFuelTanks().size()];
        m_pathToWaypoints = new Path[a_gameCopy.getWaypoints().size()];

        //Init the structure that stores the nodes closest to all waypoints and fuel tanks.
        m_collectNodes = new HashMap<GameObject, Node>();
        for(Waypoint way: a_gameCopy.getWaypoints())
        {
            m_collectNodes.put(way, m_graph.getClosestNodeTo(way.s.x, way.s.y,true));
        }

        for(FuelTank ft: a_gameCopy.getFuelTanks())
        {
            m_collectNodes.put(ft, m_graph.getClosestNodeTo(ft.s.x, ft.s.y,true));
        }

        //Calculate the closest waypoint or fuel tank to the ship.
        calculateClosestPickUp(a_gameCopy);
    }

    /**
     * This function is called every execution step to get the action to execute.
     * @param a_gameCopy Copy of the current game state.
     * @param a_timeDue The time the next move is due
     * @return the integer identifier of the action to execute (see interface framework.core.Controller for definitions)
     */
    public int getAction(Game a_gameCopy, long a_timeDue)
    {
        //Get the path to the closest node, if my ship moved.
        Node oldShipId = m_shipNode;
        m_shipNode = m_graph.getClosestNodeTo(a_gameCopy.getShip().s.x, a_gameCopy.getShip().s.y,true);
    	  if(oldShipId != m_shipNode || m_pathToClosest == null)
          {
              //Calculate the closest waypoint/fuel tank to the ship.
              calculateClosestPickUp(a_gameCopy);

              if(m_shipNode == null)
              {
                  //No node close enough and collision free. Just go for the closest.
                  m_shipNode = m_graph.getClosestNodeTo(a_gameCopy.getShip().s.x, a_gameCopy.getShip().s.y,false);
              }

              //And get the path to it from my location.
              m_pathToClosest = m_graph.getPath(m_shipNode.id(), m_collectNodes.get(m_closestPickUp).id());

              //put all (reamining) waypoints paths here
              for(int i = 0; i < a_gameCopy.getWaypointsLeft(); i++)
              {
              	Waypoint way = a_gameCopy.getWaypoints().get(i);
              	if(!way.isCollected())
              		m_pathToWaypoints[i] = m_graph.getPath(m_shipNode.id(), m_collectNodes.get(way).id());
              }
              
              for(int i = 0; i < a_gameCopy.getFuelTanksLeft(); i++)
              {
              	FuelTank tank = a_gameCopy.getFuelTanks().get(i);
              	if(!tank.isCollected())
              		m_pathToFuelTanks[i] = m_graph.getPath(m_shipNode.id(), m_collectNodes.get(tank).id());
              }
          }
    	
    	Vector2d straightup = new Vector2d();
    	straightup.x = 0;
    	straightup.y = 1;
    	straightup.normalise();
    	Vector2d shipHeading;
    	shipHeading = a_gameCopy.getShip().d;
    	shipHeading_x = a_gameCopy.getShip().d.x;
    	shipHeading_y = a_gameCopy.getShip().d.y;
//        System.out.println("heading (deg):" + Math.toDegrees(Math.acos(straightup.dot(shipHeading))));
        return m_input.getAction();
    }
    
    /**
     * Calculates the closest waypoint or fuel tank to the ship.
     * @param a_gameCopy the game copy.
     */
    private void calculateClosestPickUp(Game a_gameCopy)
    {
        double minDistance = Double.MAX_VALUE;
        for(Waypoint way: a_gameCopy.getWaypoints())
        {
            if(!way.isCollected())     //Only consider those not collected yet.
            {
                double fx = way.s.x-a_gameCopy.getShip().s.x, fy = way.s.y-a_gameCopy.getShip().s.y;
                double dist = Math.sqrt(fx*fx+fy*fy);
                if( dist < minDistance )
                {
                    //Keep the minimum distance.
                    minDistance = dist;
                    m_closestPickUp = way;
                }
            }
        }

        //Also check for fuel tanks:
        for(FuelTank ft: a_gameCopy.getFuelTanks())
        {
            if(!ft.isCollected())     //Only consider those not collected yet.
            {
                double fx = ft.s.x-a_gameCopy.getShip().s.x, fy = ft.s.y-a_gameCopy.getShip().s.y;
                double dist = Math.sqrt(fx*fx+fy*fy);
                dist = dist * FUEL_TANK_PEN; //Apply penalization to only chase those fuel tanks really close.
                if( dist < minDistance )
                {
                    //Keep the minimum distance.
                    minDistance = dist;
                    m_closestPickUp = ft;
                }
            }
        }
    }
    
    /**
     * paint additional info
     */
    public void paint(Graphics2D a_gr)
    {
//    	pathToClosest = getPathToClosest();
    	pathToClosest = null;
    	a_gr.setColor(Color.GRAY);
//    	a_gr.drawString("x:" + Double.toString(shipHeading_x), 30,30);
//    	a_gr.drawString("y:" + Double.toString(shipHeading_y), 30,60);
//    	a_gr.drawString("cost:" + Double.toString(pathToClosest.m_cost), 30,90);
    	paintPaths(a_gr);
    }  
    
    public void paintPaths(Graphics2D a_gr)
    {
    	Painter.paintPaths(a_gr, m_graph, aPlannedPath, Color.gray);
    	 //paint all active waypoints
//        a_gr.setColor(Color.blue);
//        Path[] pathToWaypoints = getPathToWaypoints();
//        if ( pathToWaypoints.length > 0) 
//        {
//	        for(int i = 0; i< pathToWaypoints.length; i++)
//	        {	        	
//	        	Path a_path = pathToWaypoints[i];
//	        	if (null != a_path)
//	        	{
//		        	for(int j = 0; j < a_path.m_points.size()-1; ++j)
//		            {
//		                Node thisNode = m_graph.getNode(a_path.m_points.get(j));
//		                Node nextNode = m_graph.getNode(a_path.m_points.get(j+1));
//		                a_gr.drawLine(thisNode.x(), thisNode.y(), nextNode.x(),nextNode.y());
//		            }
//	        	}
//	        }
//        }
        
        //paint closest waypoint
        //m_graph.draw(a_gr);
        a_gr.setColor(Color.red);
        
        if(pathToClosest != null) 
        {
        	for(int i = 0; i < pathToClosest.m_points.size()-1; ++i)
        	
	        {
	            Node thisNode = m_graph.getNode(pathToClosest.m_points.get(i));
	            Node nextNode = m_graph.getNode(pathToClosest.m_points.get(i+1));
	            a_gr.drawLine(thisNode.x(), thisNode.y(), nextNode.x(),nextNode.y());
	        }
        }
        
      //paint paths to fuel tanks
        //m_graph.draw(a_gr);
//        a_gr.setColor(Color.green);
//        Path[] pathToFuelTanks = getPathToFuelTanks();
//        if ( pathToFuelTanks.length > 0) 
//        {
//	        for(int i = 0; i< pathToFuelTanks.length; i++)
//	        {
//	        	Path a_path = pathToFuelTanks[i];
//	        	if (null != a_path)
//	        	{
//		        	for(int j = 0; j < a_path.m_points.size()-1; ++j)
//		            {
//		                Node thisNode = m_graph.getNode(a_path.m_points.get(j));
//		                Node nextNode = m_graph.getNode(a_path.m_points.get(j+1));
//		                a_gr.drawLine(thisNode.x(), thisNode.y(), nextNode.x(),nextNode.y());
//		            }
//	        	}
//	        }
//        }
    }
    
    /**
     * Returns the paths to all waypoints. (for debugging purposes)
     * @return the paths to all waypoints
     */
    //private ArrayList<Path> getPathToWaypoints() {return m_pathToWaypoints;}
    private Path[] getPathToWaypoints() {return m_pathToWaypoints;}

	/**
     * Returns the path to the closest waypoint. (for debugging purposes)
     * @return the path to the closest waypoint
     */
    public Path getPathToClosest() {return m_pathToClosest;}
    
	/**
     * Returns the paths to all fuel tanks. (for debugging purposes)
     * @return the paths to all fuel tanks
     */
    //private ArrayList<Path> getPathToWaypoints() {return m_pathToWaypoints;}
    private Path[] getPathToFuelTanks() {return m_pathToFuelTanks;}

    /**
     * Return the input manager
     * @return the input manager
     */
    public KeyInput getInput() {return m_input;}
    
    /**
     * draws a list of paths on the screen with the same color
     * @param a_gr
     * @param m_graph
     * @param m_pathList
     * @param color
     */
    public static void paintPaths(Graphics2D a_gr, Graph m_graph, ArrayList<Path> m_pathList, Color color)
    {
    	//paint planned paths
        if ( m_pathList.size() > 0) 
        {
	        for(int i = 0; i< m_pathList.size(); i++)
	        {	        	
	        	Path a_path = m_pathList.get(i);
	        	paintPath(a_gr, m_graph, a_path, color);
	        }
        }
    }
    
    /**
     * draws a single path on the screen
     * @param a_gr
     * @param m_graph
     * @param m_path
     * @param color
     */
    public static void paintPath(Graphics2D a_gr, Graph m_graph, Path m_path, Color color)
    {
    	a_gr.setColor(color);
    	if (null != m_path)
    	{
        	for(int j = 0; j < m_path.m_points.size()-1; ++j)
            {
                Node thisNode = m_graph.getNode(m_path.m_points.get(j));
                Node nextNode = m_graph.getNode(m_path.m_points.get(j+1));
                a_gr.drawLine(thisNode.x(), thisNode.y(), nextNode.x(),nextNode.y());
            }
    	}
    }
    
   private double shipHeading_x, shipHeading_y; 
}

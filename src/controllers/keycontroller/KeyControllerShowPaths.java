package controllers.keycontroller;
import java.awt.Color;
import java.awt.Graphics2D;
import java.util.HashMap;

import framework.core.Controller;
import framework.core.FuelTank;
import framework.core.Game;
import framework.core.GameObject;
import framework.core.Waypoint;
import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;
import framework.utils.Vector2d;

/**
 * This class is used for the KeyController (human playing).
 * PTSP-Competition
 * Created by Diego Perez, University of Essex.
 * Date: 20/12/11
 */
public class KeyControllerShowPaths extends KeyController
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
    //TODO: set this values from a_game
	private Path[] m_pathToWaypoints = new Path[10];
	private Path[] m_pathToFuelTanks = new Path[4];

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
    public KeyControllerShowPaths(Game a_gameCopy, long a_timeDue)
    {
        m_input = new KeyInput();
        
      //Init the graph.
        m_graph = new Graph(a_gameCopy);

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
    	a_gr.setColor(Color.GRAY);
    	a_gr.drawString("x:" + Double.toString(shipHeading_x), 30,30);
    	a_gr.drawString("y:" + Double.toString(shipHeading_y), 30,60);
    	paintPaths(a_gr);
    }  
    
    public void paintPaths(Graphics2D a_gr)
    {
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
        Path pathToClosest = getPathToClosest();
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
    
   private double shipHeading_x, shipHeading_y; 
}

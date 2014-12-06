package controllers.greedy;

import framework.core.*;
import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;
import framework.utils.Vector2d;

import java.awt.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;

import planners.Planner;
import planners.Planner2Opt;
import planners.Planner3OptManual;
import planners.PlannerBruteForce;
import planners.PlannerGreedy;
import planners.PlannerMC;
import planners.PlannerPlay;

/**
 * greedy controller
 * 
 * @version 141128
 * @author Cristian
 *
 */

public class DriveGreedy extends Controller
{
    private Graph m_graph;//graph for this controller

    private Node m_shipNode; //closest node to the ship position
    private Node oldShipId;
    private Path m_pathToClosest;//path to closest waypoint
    private ArrayList<Path> m_plannedPath = new ArrayList<>();
    
	private ArrayList<Path> m_pathToWaypoints = new ArrayList<>();//Path to all waypoints in the map
	private ArrayList<Path> m_pathToFuelTanks = new ArrayList<>();//Path to all waypoints in the map
	
	private LinkedList<Waypoint> m_orderedWaypoints = new LinkedList<>();//waypoints in the order they should be visited as computed by the planner
    private GameObject m_closestPickUp;//Closest object (waypoint or fuel tank) to the ship.
	private Waypoint m_nextWaypoint;// Next waypoint in the list resulted by planner

    private HashMap<GameObject, Node> m_collectNodes;//Hash map that matches waypoints in the map with their closest node in the graph.
	
    /**
     * Constructor
     * @param a_gameCopy a copy of the game state
     * @param a_timeDue The time the initialization is due. Finishing this method after a_timeDue will disqualify this controller.
     */
    public DriveGreedy(Game a_gameCopy, long a_timeDue)
    {
    	System.out.println("***greedy controller***");
        
        m_graph = new Graph(a_gameCopy);//Init the graph.

        


//        Planner planner = new PlannerPlay(a_gameCopy);//current working planner
//      Planner planner = new PlannerGreedyEvolved(a_gameCopy);//plan a cost based route through the waypoints   
        Planner planner = new Planner3OptManual(a_gameCopy);//remove three edges and reconnect the graph
        

        
        //done
//        Planner planner = new Planner2Opt(a_gameCopy);//remove two edges and reconnect the graph    
//        Planner planner = new PlannerMC(a_gameCopy);//search through random paths to find a small one // 1512 ... 1680
//        Planner planner = new PlannerBruteForce(a_gameCopy);//brute force search planner  //1512
//        Planner planner = new PlannerGreedy(a_gameCopy);//plan a distance based route through the waypoints //1614

        m_orderedWaypoints = planner.getOrderedWaypoints();//get the planned route
        planner.calculateOrderedWaypointsPaths();//calculate the paths from one waypoint to another
        m_plannedPath = planner.getPlannedPath();//get the path from one waypoint to the next
        
        //Init the structure that stores the nodes closest to all waypoints and fuel tanks.
        m_collectNodes = new HashMap<GameObject, Node>();
        for(Waypoint way: m_orderedWaypoints)
        {
            m_collectNodes.put(way, m_graph.getClosestNodeTo(way.s.x, way.s.y,true));
        }

        for(FuelTank ft: a_gameCopy.getFuelTanks())
        {
            m_collectNodes.put(ft, m_graph.getClosestNodeTo(ft.s.x, ft.s.y,true));
        }
        
    }
    
    /**
     * This function is called every execution step to get the action to execute.
     * @param a_gameCopy Copy of the current game state.
     * @param a_timeDue The time the next move is due
     * @return the integer identifier of the action to execute (see interface framework.core.Controller for definitions)
     */
    public int getAction(Game a_gameCopy, long a_timeDue)
    {
        //Get the closest node to the ship, if my ship moved.
        oldShipId = m_shipNode;
        m_shipNode = m_graph.getClosestNodeTo(a_gameCopy.getShip().s.x, a_gameCopy.getShip().s.y,true);
        if(oldShipId != m_shipNode || m_pathToClosest == null)
        {          
            //get next pickup from planned list
            getNextPlannedWaypoint(a_gameCopy);
            m_closestPickUp = m_nextWaypoint;
           
            //update collected waypoints
        	if(m_nextWaypoint.checkCollected(a_gameCopy.getShip().ps, m_nextWaypoint.radius))
        	{
        		m_nextWaypoint.setCollected(true);
        	}
            
            if(m_shipNode == null)
            {
                m_shipNode = m_graph.getClosestNodeTo(a_gameCopy.getShip().s.x, a_gameCopy.getShip().s.y,false);
            }

            m_pathToClosest = m_graph.getPath(m_shipNode.id(), m_collectNodes.get(m_closestPickUp).id());//get path to closest waypoint
        }
//        calculateObjectPaths(a_gameCopy);
        
        //can we can see the waypoint:
        boolean isThereLineOfSight = a_gameCopy.getMap().LineOfSight(a_gameCopy.getShip().s, m_closestPickUp.s);
        if(isThereLineOfSight)
        {
            return manageStraightTravel(a_gameCopy);
        }

        //The waypoint/ft is behind an obstacle, select which is the best action to take.
        double minDistance = Float.MAX_VALUE;
        int bestAction = -1;
        double bestDot = -2;

        if(m_pathToClosest != null)//We should have a path...
        {
            int startAction = Controller.ACTION_NO_FRONT;
            //For each possible action...
            for(int action = startAction; action < Controller.NUM_ACTIONS; ++action)
            {
                //Simulate that we execute the action and get my potential position and direction
                Game forThisAction = a_gameCopy.getCopy();
                forThisAction.getShip().update(action);
                Vector2d nextPosition = forThisAction.getShip().s;
                Vector2d potentialDirection = forThisAction.getShip().d;

                //Get the next node to go to, from the path to the closest waypoint/ft
                Node nextNode = getNextPathNode();
                Vector2d nextNodeV = new Vector2d(nextNode.x(),nextNode.y());
                nextNodeV.subtract(nextPosition);
                nextNodeV.normalise();   //This is a unit vector from my position pointing towards the next node to go to.
                double dot = potentialDirection.dot(nextNodeV);  //Dot product between this vector and where the ship is facing to.

                //Get the distance to the next node in the tree and update the total distance until the closest waypoint/ft
                double dist = nextNode.euclideanDistanceTo(nextPosition.x, nextPosition.y);
                double totalDistance = m_pathToClosest.m_cost + dist;

                //System.out.format("Action: %d, total distance: %.3f, distance to node: %.3f, dot: %.3f\n",action, totalDistance, dist, dot);

                //Keep the best action so far.
                if(totalDistance < minDistance)
                {
                    minDistance = totalDistance;
                    bestAction = action;
                    bestDot = dot;
                }
                //If the distance is the same, keep the action that faces the ship more towards the next node
                else if((int)totalDistance == (int)minDistance && dot > bestDot)
                {
                    minDistance = totalDistance;
                    bestAction = action;
                    bestDot = dot;
                }
            }
            return bestAction;//This is the best action to take.
        }
        return Controller.ACTION_NO_FRONT;//Default (something went wrong).
    }

    /**
     * Returns the first node in the way to the destination
     * @return the node in the way to destination.
     */
    private Node getNextPathNode()
    {
        Node n0 = m_graph.getNode(m_pathToClosest.m_points.get(0));

        //If only one node in the path, return it.
        if(m_pathToClosest.m_points.size() == 1)
            return n0;

        //Heuristic: Otherwise, take the closest one to the destination
        Node n1 = m_graph.getNode(m_pathToClosest.m_points.get(1));
        Node destination =  m_graph.getNode(m_pathToClosest.m_destinationID);

        if(n0.euclideanDistanceTo(destination) < n1.euclideanDistanceTo(destination))
            return n0;
        else return n1;
    }
   
    /**
     * Manages straight traveling.
     * @param a_gameCopy the game copy
     * @return the id of the best action to execute.
     */
    private int manageStraightTravel(Game a_gameCopy)
    {
        int bestAction = Controller.ACTION_NO_FRONT;
        Vector2d dirToWaypoint = m_closestPickUp.s.copy();
        dirToWaypoint.subtract(a_gameCopy.getShip().s);
        double distance = dirToWaypoint.mag();
        dirToWaypoint.normalise();

        //Check if we are facing the waypoint we are going after.
        Vector2d dir = a_gameCopy.getShip().d;
        boolean notFacingWaypoint = dir.dot(dirToWaypoint) < 0.85;

        //Depending on the time left and the distance to the waypoint/ft, we established the maximum speed.
        //(going full speed could make the ship to overshoot the waypoint/ft... that's the reason of this method!).
        double maxSpeed = 0.4;
        if(distance>100 || a_gameCopy.getStepsLeft() < 50)
            maxSpeed = 0.8;
        else if(distance<30) maxSpeed = 0.25;


        if(notFacingWaypoint || a_gameCopy.getShip().v.mag() > maxSpeed)
        {
            //We should not risk to throttle. Let's rotate in place to face the waypoint/ft better.
            Game forThisAction;
            double bestDot = -2;
            for(int i = Controller.ACTION_NO_FRONT; i <= Controller.ACTION_NO_RIGHT; ++i)
            {
                //Select the action that maximises my dot product with the target (aka. makes the ship face the target better).
                forThisAction = a_gameCopy.getCopy();
                forThisAction.getShip().update(i);
                Vector2d potentialDirection = forThisAction.getShip().d;
                double newDot = potentialDirection.dot(dirToWaypoint);
                if(newDot > bestDot)
                {
                    bestDot = newDot;
                    bestAction = i;
                }
            }
        } else //We can thrust
            return Controller.ACTION_THR_FRONT;

        //There we go!
        return bestAction;
    }
    
    /**
     * set next waypoint to visit from the planned path
     * @param a_gameCopy
     */
    public void getNextPlannedWaypoint(Game a_gameCopy)
    {

    	if (m_nextWaypoint == null || m_nextWaypoint.isCollected())
    	{
    		for(Waypoint way: m_orderedWaypoints)
            {
                if(!way.isCollected())     //Only consider those not collected yet.
                {
               	 System.out.println("next waypoint:" + way.getName());
                       m_nextWaypoint = way;
                       break;
                }
            }
    	}
    }
    
    
    /**
     * calculate paths from ship to each remaining object
     * these can be drawn with paintPathsToAllObjects()
     * @param a_gameCopy
     */
    @SuppressWarnings("unused")
	private void calculateObjectPaths(Game a_gameCopy)
    {            
    	//put all (remaining) waypoints paths here
        m_pathToWaypoints.clear();            
        for(Waypoint way: a_gameCopy.getWaypoints())
        {
            if(!way.isCollected())     //Only consider those not collected yet.
            {
            	m_pathToWaypoints.add(m_graph.getPath(m_shipNode.id(), m_collectNodes.get(way).id()));
            }
        }
        
        m_pathToFuelTanks.clear();
        for(FuelTank tank: a_gameCopy.getFuelTanks())
        {
        	if(!tank.isCollected())
        		m_pathToFuelTanks.add(m_graph.getPath(m_shipNode.id(), m_collectNodes.get(tank).id()));
        }
    }
    

    /**
     * This is a debug function that can be used to paint on the screen.
     * call calculateObjectPaths() to get the paths
     * @param a_gr Graphics device to paint.
     */
    public void paintPathsToAllObjects(Graphics2D a_gr)
    {
    	//paint all nodes
    	//m_graph.draw(a_gr);
    
    	//paint all fuel tanks
        a_gr.setColor(Color.green);
        ArrayList<Path> pathToFuelTanks = getPathToFuelTanks();
        if ( pathToFuelTanks.size() > 0) 
        {
	        for(int i = 0; i< pathToFuelTanks.size(); i++)
	        {
	        	Path a_path = pathToFuelTanks.get(i);
	        	if (null != a_path)
	        	{
		        	for(int j = 0; j < a_path.m_points.size()-1; ++j)
		            {
		                Node thisNode = m_graph.getNode(a_path.m_points.get(j));
		                Node nextNode = m_graph.getNode(a_path.m_points.get(j+1));
		                a_gr.drawLine(thisNode.x(), thisNode.y(), nextNode.x(),nextNode.y());
		            }
	        	}
	        }
        }
        
        //paint all active waypoints
        a_gr.setColor(Color.blue);
        ArrayList<Path> pathToWaypoints = getPathToWaypoints();
        if ( pathToWaypoints.size() > 0) 
        {
	        for(int i = 0; i< pathToWaypoints.size(); i++)
	        {	        	
	        	Path a_path = pathToWaypoints.get(i);
	        	if (null != a_path)
	        	{
		        	for(int j = 0; j < a_path.m_points.size()-1; ++j)
		            {
		                Node thisNode = m_graph.getNode(a_path.m_points.get(j));
		                Node nextNode = m_graph.getNode(a_path.m_points.get(j+1));
		                a_gr.drawLine(thisNode.x(), thisNode.y(), nextNode.x(),nextNode.y());
		            }
	        	}
	        }
        }
        
        //paint closest waypoint
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
    }  
    
    /**
     * paint additional info
     */
    public void paint(Graphics2D a_gr)
    {  
    	paintOrderedWaypointsPaths(a_gr);
//    	paintPathsToAllObjects(a_gr);
    }  
    
    /**
     * draws the path resulted from a planner
     * @param a_gr
     */
    private void paintOrderedWaypointsPaths(Graphics2D a_gr)
    {
    	//paint planned paths
        a_gr.setColor(Color.GRAY);
        if ( m_plannedPath.size() > 0) 
        {
	        for(int i = 0; i< m_plannedPath.size(); i++)
	        {	        	
	        	Path a_path = m_plannedPath.get(i);
	        	if (null != a_path)
	        	{
		        	for(int j = 0; j < a_path.m_points.size()-1; ++j)
		            {
		                Node thisNode = m_graph.getNode(a_path.m_points.get(j));
		                Node nextNode = m_graph.getNode(a_path.m_points.get(j+1));
		                a_gr.drawLine(thisNode.x(), thisNode.y(), nextNode.x(),nextNode.y());
		            }
	        	}
	        }
        }
    	
    }
        
	/**
     * Returns the paths to all waypoints. (for debugging purposes)
     * @return the paths to all waypoints
     */
    private ArrayList<Path> getPathToWaypoints() {return m_pathToWaypoints;}

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
    private ArrayList<Path> getPathToFuelTanks() {return m_pathToFuelTanks;}

    /**
     * Returns the graph. (for debugging purposes)
     * @return the graph.
     */
    public Graph getGraph() {return m_graph;}

}

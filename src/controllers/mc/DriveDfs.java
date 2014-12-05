package controllers.mc;

import framework.core.*;
import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;
import framework.utils.Vector2d;

import java.awt.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Random;

import planners.PlannerGreedyEvolved;

/**
 * deep first search driver with macro actions
 * TODO:
 *  - actually use the time gained from macro actions
 *  - clean up unused code
 *  - paint possible ship positions is not showing anything
 * @version 141128
 * @author Cristian
 *
 */

public class DriveDfs extends Controller
{
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
     * Paths to the waypoints based on the planner
     */
    private ArrayList<Path> m_plannedPath = new ArrayList<>();
    
    /**
     * Path to all waypoints in the map
     */
    //TODO: populate these values from a_game just once
	private ArrayList<Path> m_pathToWaypoints = new ArrayList<>();
	private ArrayList<Path> m_pathToFuelTanks = new ArrayList<>();
	
	/**
	 * waypoints in the order they should be visited as computed by the planner
	 */
	private LinkedList<Waypoint> m_orderedWaypoints = new LinkedList<>();

    /**
     * Hash map that matches waypoints in the map with their closest node in the graph.
     */
    private HashMap<GameObject, Node> m_collectNodes;


	/**
	 * Next waypoint in the list resulted by planner
	 */
	private Waypoint m_nextWaypoint;

    /**
     * Constructor, that receives a copy of the game state
     * @param a_gameCopy a copy of the game state
     * @param a_timeDue The time the initialization is due. Finishing this method after a_timeDue will disqualify this controller.
     */
    public DriveDfs(Game a_gameCopy, long a_timeDue)
    {
    	System.out.println("*** dfs controller***");
        m_graph = new Graph(a_gameCopy);//Init the graph.

        PlannerGreedyEvolved planner = new PlannerGreedyEvolved(a_gameCopy);
        m_orderedWaypoints = planner.getOrderedWaypoints();//get the planned route
        planner.calculateOrderedWaypointsPaths();//calculate the paths from one waypoint to another  
        m_plannedPath = planner.getPlannedPath();//get the path from one waypoint to the next
        
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
    }
    
    /**
     * This function is called every execution step to get the action to execute.
     * @param a_gameCopy Copy of the current game state.
     * @param a_timeDue The time the next move is due
     * @return the integer identifier of the action to execute (see interface framework.core.Controller for definitions)
     */
    public int getAction(Game a_gameCopy, long a_timeDue)
    {
        //done with previous macro action?
    	if(macroMovesLeft > 0) {
    		macroMovesLeft--;
    		return lastAction;
    	}
    	
        //update collected waypoints
        getNextPlannedWaypoint(a_gameCopy);
    	if(m_nextWaypoint.checkCollected(a_gameCopy.getShip().ps, m_nextWaypoint.radius/4*3))
    	{
    		System.out.println("!!!!!marked as completed");
    		m_nextWaypoint.setCollected(true);
    	}
    
    	possiblePosition.clear();//just one level of search
        Game simulatedGame = a_gameCopy.getCopy();
    	int returnedAction = getSimulatedAction(simulatedGame, a_timeDue, 0);
    	return returnedAction;
    }
    

    public int getSimulatedAction(Game a_gameCopy, long a_timeDue, int depth)
    {
//    	System.out.println("entering with depth : " + depth);
    	if(depth > maxDepth) return -1;
    	boolean showInfo;
    	showInfo = ((ticks % 50 == 0) ? true : false);
    	showInfo = false;
        if(showInfo) System.out.println("===tick:" + ticks);
    	
        long timeIn = System.currentTimeMillis();
        if(showInfo) System.out.println(">>>in\t\t" + timeIn);	
        
        if(showInfo) System.out.println("---due in\t" + a_timeDue);
        long due = timeIn+PTSPConstants.ACTION_TIME_MS;
        if(showInfo) System.out.println("---due calc\t" + due); 
       
        //find the most distant node in the path that i have a direct route to
        //turn towards it and accelerate
        //there is a relationship between how close i am to the point i see and the next point
        //	-> check if i can see a new node and turn and aim towards it
        //the closer i am to the target node, the slower i should go
        
        
        //evaluate positions
        //distance - path length
        //directness - path curvature
        
        int nextWaypoint = 1;//skip ship waypoint
        for (int i = 1; i < m_orderedWaypoints.size(); i++)
        {
        	if(!m_orderedWaypoints.get(i).isCollected())
        	{
        		nextWaypoint = i;
        		break;
        	}

        }
    	Path pathToFollow = m_plannedPath.get(nextWaypoint-1);
    	
        double minDistance = Float.MAX_VALUE;
        int bestAction = -1;
        double bestDot = -2;
        int startAction = Controller.ACTION_NO_FRONT;
        int deepAction = -1;
        depth++;
        
        for(int action = startAction; action < Controller.NUM_ACTIONS; ++action)
        {
        	
        	//System.out.println("action " + action);
            //Simulate that we execute the action and get my potential position and direction
            Game forThisAction = a_gameCopy.getCopy();
            for(int i = 0; i < macroCount; i++)
            {
            	forThisAction.getShip().update(action);	
            }

//            deepAction = getSimulatedAction(forThisAction, a_timeDue, depth);
            
            Vector2d nextPosition = forThisAction.getShip().s;
            Vector2d potentialDirection = forThisAction.getShip().d;
            if(!possiblePosition.contains(nextPosition))
            {
            	possiblePosition.add(nextPosition);	
            }
            

            //Get the next node to go to, from the path to the closest waypoint/ft
            Node nextNode = getLastNodeVisible(pathToFollow, a_gameCopy);
            aimedNode = nextNode;
                       
            Vector2d nextNodeV = new Vector2d(nextNode.x(),nextNode.y());
            nextNodeV.subtract(nextPosition);
            nextNodeV.normalise();   //This is a unit vector from my position pointing towards the next node to go to.
            double dot = potentialDirection.dot(nextNodeV);  //Dot product between this vector and where the ship is facing to.

            //Get the distance to the next node in the tree and update the total distance until the closest waypoint/ft
            double dist = nextNode.euclideanDistanceTo(nextPosition.x, nextPosition.y);
            double totalDistance = pathToFollow.m_cost + dist;

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
        
        ticks++;
        if(showInfo) System.out.println("<<<out 1 \t" + System.currentTimeMillis());
        lastAction = bestAction;
        macroMovesLeft = macroCount;
        return bestAction;
        
//        //Default (something went wrong).
//        return Controller.ACTION_NO_FRONT;
//        System.out.println("<<<out 2 \t" + System.currentTimeMillis());
//        return Raction;
    }
    
    /**
     * Returns the last visible node in the way to the destination
     * @return the node in the way to destination.
     */
    private Node getLastNodeVisible(Path a_Path, Game a_gameCopy)
    {
    	Node furthestNodeVisible =  m_graph.getNode(a_Path.m_points.get(0));
    	for(int i = 0; i < a_Path.m_points.size(); i++)
    	{
    		Node a_Node = m_graph.getNode(a_Path.m_points.get(i));
            Vector2d nextNodePos = new Vector2d(a_Node.x(),a_Node.y());
       		boolean isThereLineOfSight = a_gameCopy.getMap().LineOfSight(a_gameCopy.getShip().s, nextNodePos);
       		if(isThereLineOfSight)
       		{
       			furthestNodeVisible = a_Node;
       		}
    	}
    	return furthestNodeVisible;
    }

    	
	private void waitRandom(int a_near)
    {
        Random m_rnd = new Random();
        //TODO: why -30?
        int waitTime = (a_near-30) + m_rnd.nextInt(50);
        long startingTime = System.currentTimeMillis();
        long finalDateMs = startingTime + waitTime;

        while(startingTime < finalDateMs)
            startingTime = System.currentTimeMillis();

        System.out.println("Waited " + waitTime + " ms");
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
               	 System.out.println("next waypoint:" + way.toString());
                       m_nextWaypoint = way;
                       break;
                }
            }
    	}
    }

    /**
     * calculate paths from ship to each remaining object
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
    	paintPossibleShipPositions(a_gr);
    	paintOrderedWaypointsPaths(a_gr);
    	paintAimedNode(a_gr);

    	//paintPathsToAllObjects(a_gr);
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
     * draw the node where the ship is going for
     * @param a_gr
     */
    private void paintAimedNode(Graphics2D a_gr)
    {
        a_gr.setColor(Color.RED);
//    	a_gr.drawOval(aimedNode.x(), aimedNode.y(), 5, 5);
    	
    	a_gr.fillOval(aimedNode.x(), aimedNode.y(), 5, 5);	
    }
    
    /**
     * plot possible ship positions
     */
    private void paintPossibleShipPositions(Graphics2D a_gr)
    {
//    	int drawEvery = 1;
//    	int drawn = 1;
    	a_gr.setColor(Color.blue);
    	ArrayList<Vector2d> positionsToDraw = (ArrayList<Vector2d>) possiblePosition.clone();
    	for(Vector2d position : positionsToDraw) 
    	{
//    		if(drawn % drawEvery == 0)
//    		{
    			a_gr.drawRect((int)position.x, (int)position.y, 1, 1);	
//    		}
//    		drawn++;
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
    
    //how many ticks passed
    private int ticks = 0;
    
    private int macroMovesLeft = 0;
    private int lastAction = ACTION_NO_FRONT;
    private Node aimedNode = null;
    
    //how many times to repeat each action
	//repeat a turn 30 times = 90 degrees
	private int macroCount = 3;
	private int maxDepth = 5;
    
    private ArrayList<Vector2d> possiblePosition = new ArrayList<>();
}

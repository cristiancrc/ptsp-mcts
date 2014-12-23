package controllers.mc;

import framework.core.*;
import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;
import framework.utils.Navigator;
import framework.utils.Painter;
import framework.utils.Value;
import framework.utils.Vector2d;

import java.awt.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import planners.Planner;
import planners.PlannerGreedy;

/**
 * deep first search driver with repeated actions
 * @version 141221
 * @author Cristian
 *
 */

public class DriveDfs extends Controller
{
	private Graph m_graph; //     * Graph for this controller.
	private ArrayList<Path> m_plannedPath = new ArrayList<>();//      * Paths to the waypoints based on the planner
	private LinkedList<Waypoint> m_orderedWaypoints = new LinkedList<>(); //waypoints in the order they should be visited as computed by the planner
	private HashMap<GameObject, Node> m_collectNodes;//Hash map that matches waypoints in the map with their closest node in the graph.
	private Waypoint m_nextWaypoint; //	Next waypoint in the list resulted by planner
	private Integer nextWaypoint = null; // index of next waypoint in m_orderedWaypoints 
	private Value bestValue = new Value();
	private int bestAction;
	private Path pathToFollow;
    private int ticks = 0;//game ticks counter 
    private Node aimedNode = null;
    
    //how many times to repeat each action
	//repeat a turn 30 times = 90 degrees
	private int macroCount = 1;
	private int maxDepth = 4;
	private boolean verbose = false;
    
    private ArrayList<Vector2d> possiblePosition = new ArrayList<>();	

    /**
     * Constructor, that receives a copy of the game state
     * @param a_gameCopy a copy of the game state
     * @param a_timeDue The time the initialization is due. Finishing this method after a_timeDue will disqualify this controller.
     */
    public DriveDfs(Game a_gameCopy, long a_timeDue)
    {
    	System.out.println("**** dfs controller ****");
        m_graph = new Graph(a_gameCopy);//Init the graph.

		Planner planner = new PlannerGreedy(a_gameCopy);//plan a distance based route through the waypoints //1614
		
		m_orderedWaypoints = planner.getOrderedWaypoints();//get the planned route
		m_nextWaypoint = m_orderedWaypoints.get(0);//set immediate goal        
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
    	//target reached?
        if(m_nextWaypoint.checkCollected(a_gameCopy.getShip().ps, m_nextWaypoint.radius/4*3))
    	{
    		if (verbose) System.out.println("!!!!!marked as completed " + m_nextWaypoint);
    		m_nextWaypoint.setCollected(true);
    		m_orderedWaypoints.get(m_orderedWaypoints.indexOf(m_nextWaypoint)).setCollected(true);    		
    		
    		//set next waypoint
    		nextWaypoint = m_orderedWaypoints.indexOf(m_nextWaypoint)+1;
    		m_nextWaypoint = m_orderedWaypoints.get(m_orderedWaypoints.indexOf(m_nextWaypoint)+1);
    	}   
        //get path to it from the previous waypoint    	    
        pathToFollow = m_plannedPath.get(nextWaypoint-1);      
    	
        //Get the next node to go to, from the path to the closest waypoint/ fuel tank
    	aimedNode = Navigator.getLastNodeVisible(pathToFollow, a_gameCopy); 
    	
    	possiblePosition.clear();//just one level of search
        Game simulatedGame = a_gameCopy.getCopy();
        bestValue.reset();
        bestAction = -1;
        if (verbose) System.out.println("----------------------------------------------------" + ticks);
    	getSimulatedAction(simulatedGame, a_timeDue, 0);
    	if (verbose) System.out.println("<<< selected " + bestAction);
    	ticks++;
//    	if(ticks == 2) System.exit(0);
    	return bestAction;
    }
    
    /**
     * run search
     * @param a_gameCopy
     * @param a_timeDue
     * @param depth
     * @return
     */
    public Value getSimulatedAction(Game a_gameCopy, long a_timeDue, int depth)
    {    	
    	if(depth == maxDepth)
    	{
    		 Vector2d nextPosition = a_gameCopy.getShip().s;
        	 if(!possiblePosition.contains(nextPosition))
             {
        		 possiblePosition.add(nextPosition);	
             }        	
        	 return Navigator.evaluateShipPosition(a_gameCopy, aimedNode);
    	}
    	else if (depth < maxDepth)
        {
    		Value localValue = new Value();
	        for(int action = 1; action <= Controller.NUM_ACTIONS; action++)
	        {
	            Game forThisAction = a_gameCopy.getCopy();
	            for(int i = 0; i < macroCount; i++)
	            {
	            	forThisAction.getShip().update(action);  	            	
	            }
            	Vector2d nextPosition = forThisAction.getShip().s;
           	 	if(!possiblePosition.contains(nextPosition))
                {
           	 		possiblePosition.add(nextPosition);	
                } 	            
           	 	int newDepth = depth + 1;
	            localValue = getSimulatedAction(forThisAction, a_timeDue, newDepth);
	            if (verbose) for(int i = 0; i < depth; i++)
	        	{
	        		System.out.print(" ");
	        	}	            
	            if (verbose) System.out.println(action + " -> " + localValue.direction + " " + localValue.distance);	            
	            if (localValue.distance < bestValue.distance)
		        {
	            	if (verbose) System.out.println("distance updated " + bestValue.distance + " > " + localValue.distance);
		        	bestValue.distance = localValue.distance;
		        	bestValue.direction = localValue.direction;
		        	bestAction = action;
		        }
	            else if((int)localValue.distance == (int)bestValue.distance && localValue.direction > bestValue.direction)
	            {
	            	if (verbose) System.out.println("direction updated " + bestValue.direction + " > " + localValue.direction);
                	bestValue.distance = localValue.distance;
		        	bestValue.direction = localValue.direction;
		        	bestAction = action;

	            }
	        }	        
        }
		return bestValue;   	
    }    
        
    /**
     * paint additional info
     */
    public void paint(Graphics2D a_gr)
    {  
    	Painter.paintPossibleShipPositions(a_gr, possiblePosition);
    	Painter.paintPaths(a_gr, m_graph, m_plannedPath, Color.gray);
    	Painter.paintAimedNode(a_gr, aimedNode);
    }  
}

package controllers.greedy;

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
	private Graph aGraph; //     * Graph for this controller.
	private ArrayList<Path> plannedPath = new ArrayList<>();//      * Paths to the waypoints based on the planner
	private LinkedList<GameObject> orderedWaypoints = new LinkedList<>(); //waypoints in the order they should be visited as computed by the planner
	private HashMap<GameObject, Node> collectNodes;//Hash map that matches waypoints in the map with their closest node in the graph.
	private GameObject nextWaypoint; //	Next waypoint in the list resulted by planner
	private Integer nextWaypointIndex = null; // index of next waypoint in orderedWaypoints 
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
     * @param aGameCopy a copy of the game state
     * @param timeDue The time the initialization is due. Finishing this method after a_timeDue will disqualify this controller.
     */
    public DriveDfs(Game aGameCopy, long timeDue)
    {
    	System.out.println("**** dfs controller ****");
        aGraph = new Graph(aGameCopy);//Init the graph.

		Planner planner = new PlannerGreedy(aGameCopy);//plan a distance based route through the waypoints //1614
		planner.runPlanner();
		
		orderedWaypoints = planner.getOrderedWaypoints();//get the planned route
		nextWaypoint = orderedWaypoints.get(0);//set immediate goal        
		planner.calculateOrderedWaypointsPaths();//calculate the paths from one waypoint to another
		plannedPath = planner.getPlannedPath();//get the path from one waypoint to the next
		 
		  
		//Init the structure that stores the nodes closest to all waypoints and fuel tanks.
		collectNodes = new HashMap<GameObject, Node>();
		for(GameObject way: orderedWaypoints)
		{
		    collectNodes.put(way, aGraph.getClosestNodeTo(way.s.x, way.s.y,true));
		}
		
		for(FuelTank ft: aGameCopy.getFuelTanks())
		{
		    collectNodes.put(ft, aGraph.getClosestNodeTo(ft.s.x, ft.s.y,true));
		}
    }
    
    /**
     * This function is called every execution step to get the action to execute.
     * @param aGameCopy Copy of the current game state.
     * @param a_timeDue The time the next move is due
     * @return the integer identifier of the action to execute (see interface framework.core.Controller for definitions)
     */
    public int getAction(Game aGameCopy, long a_timeDue)
    {   	
    	//target reached?
    	boolean targetReached = false;
    	if(nextWaypoint instanceof Waypoint)
    	{
    		if(((Waypoint) nextWaypoint).checkCollected(aGameCopy.getShip().ps, nextWaypoint.radius/4*3))
    		{
    			((Waypoint) nextWaypoint).setCollected(true);
    			((Waypoint) orderedWaypoints.get(orderedWaypoints.indexOf(nextWaypoint))).setCollected(true);    			
    			targetReached = true;
    		}
    	}
    	else if(nextWaypoint instanceof FuelTank)
    	{
    		if(((FuelTank) nextWaypoint).checkCollected(aGameCopy.getShip().ps, nextWaypoint.radius/4*3))
    		{
    			((FuelTank) nextWaypoint).setCollected(true);
    			((FuelTank) orderedWaypoints.get(orderedWaypoints.indexOf(nextWaypoint))).setCollected(true);
    			targetReached = true;
    		}
    	}
    	if(targetReached)
    	{
    		if (verbose) System.out.println("!!!!!marked as completed " + nextWaypoint);
    		//set next waypoint
    		nextWaypointIndex = orderedWaypoints.indexOf(nextWaypoint)+1;
    		nextWaypoint = orderedWaypoints.get(orderedWaypoints.indexOf(nextWaypoint)+1);
    	}   
        //get path to it from the previous waypoint    	    
        pathToFollow = plannedPath.get(nextWaypointIndex-1);      
    	
        //Get the next node to go to, from the path to the closest waypoint/ fuel tank
    	aimedNode = Navigator.getLastNodeVisible(pathToFollow, aGameCopy, aGraph); 
    	
    	possiblePosition.clear();//just one level of search
        Game simulatedGame = aGameCopy.getCopy();
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
        	 return Navigator.evaluateShipPositionVisibleNode(a_gameCopy, aimedNode);
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
    	Painter.paintPaths(a_gr, aGraph, plannedPath, Color.gray);
    	Painter.paintAimedNode(a_gr, aimedNode);
    }  
    
    /**
     * is collected for both waypoint and fueltank
     * @param aGameObject
     * @return
     */
    public boolean isCollected(GameObject aGameObject)
    {
    	if(aGameObject instanceof Waypoint)
    	{
    		if(((Waypoint) aGameObject).isCollected())
    		{
    			return true;
    		}
    	}
    	if(aGameObject instanceof FuelTank)
    	{
    		if( ((FuelTank) aGameObject).isCollected())
    		{
    			return true;
    		}
    	}
    	return false;
    }
}

package controllers.mc;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Random;

import planners.Planner;
import planners.Planner3Opt;
import framework.core.Controller;
import framework.core.FuelTank;
import framework.core.Game;
import framework.core.GameObject;
import framework.core.Waypoint;
import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;
import framework.utils.Navigator;
import framework.utils.Painter;
import framework.utils.Value;
import framework.utils.Vector2d;

/**
 * monte carlo simulation driver
 * @version 141128
 * @author Cristian
 * TODO 9 this should be like the macro rs controller but worse, without macro; consider adding ucb
 */

public class DriveMC extends Controller
{
    private Graph m_graph; //     * Graph for this controller.
    private ArrayList<Path> plannedPath = new ArrayList<>();//      * Paths to the waypoints based on the planner
	private LinkedList<GameObject> orderedWaypoints = new LinkedList<>(); //waypoints in the order they should be visited as computed by the planner
    private HashMap<GameObject, Node> m_collectNodes;//     * Hash map that matches waypoints in the map with their closest node in the graph.
	private GameObject nextWaypoint; //	 * Next waypoint in the list resulted by planner
    private Integer nextWaypointIndex = null; // index of next waypoint in orderedWaypoints 
    private int ticks = 0;    //how many ticks passed
    private Node aimedNode = null;
    private Path pathToFollow;
    private ArrayList<Vector2d> possiblePosition = new ArrayList<>();
    public Random rnd = new Random();    

    /**
     * Constructor, that receives a copy of the game state
     * @param a_gameCopy a copy of the game state
     * @param a_timeDue The time the initialization is due. Finishing this method after a_timeDue will disqualify this controller.
     */
    public DriveMC(Game a_gameCopy, long a_timeDue)
    {
    	System.out.println("**** monte carlo search controller ****");
        m_graph = new Graph(a_gameCopy);//Init the graph.

        Planner planner = new Planner3Opt(a_gameCopy);//remove three edges and reconnect the graph        
        orderedWaypoints = planner.getOrderedWaypoints();//get the planned route
        nextWaypoint = orderedWaypoints.get(0);//set immediate goal        
        planner.calculateOrderedWaypointsPaths();//calculate the paths from one waypoint to another
        plannedPath = planner.getPlannedPath();//get the path from one waypoint to the next
        
        
        //Init the structure that stores the nodes closest to all waypoints and fuel tanks.
        m_collectNodes = new HashMap<GameObject, Node>();
        for(GameObject way: orderedWaypoints)
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
     * @param aGameCopy Copy of the current game state.
     * @param timeDue The time the next move is due
     * @return the integer identifier of the action to execute (see interface framework.core.Controller for definitions)
     */
    public int getAction(Game aGameCopy, long timeDue)
    {   	
    	boolean verbose = true;
//    	verbose = ((ticks % 100 == 0) ? true : false);
    	
        long timeIn = System.currentTimeMillis();
        if(verbose) System.out.println("\n>>>in\t\t" + timeIn);
        if(verbose) System.out.println("---due in\t" + timeDue);
              
        long remainingTime;
        remainingTime = timeDue - System.currentTimeMillis();
                       
        possiblePosition.clear();//display just one level of search
        
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
    	
        //Get the next node to go to, from the path to the closest waypoint/ fueltank
    	aimedNode = Navigator.getLastNodeVisible(pathToFollow, aGameCopy, m_graph);  

    	// for each basic action
    	int playouts = 1;
    	int depth = 100;
    	ActionData[] actionData = new ActionData[6];
    	for (int baseAction = 1; baseAction < Controller.NUM_ACTIONS; baseAction++)
    	{   
    		Value baseActionScore = new Value();
    		actionData[baseAction] = new ActionData();			
			actionData[baseAction].action = baseAction;					
    		// do several playouts    		
    		for (int i = 0; i < playouts; i++)
    		{
    			actionData[baseAction].visited = actionData[baseAction].visited + 1;
    			Value playoutScore = new Value();
               	// make a copy of the game
            	Game forThisAction = aGameCopy.getCopy(); 

            	// in each, make some random moves
            	for (int j = 0; j < depth; j++)
            	{
            		// select a random action
                	int action = rnd.nextInt(Controller.NUM_ACTIONS-1)+1;//avoid action 0        	                   	            	
                	forThisAction.getShip().update(action);
                	
                	// stop if target is reached
//                	if(nextWaypoint.checkCollected(forThisAction.getShip().ps, nextWaypoint.radius/4*3)) break;                  	
            	}
            	
              	// get new ship position
                Vector2d nextPosition = forThisAction.getShip().s;
                // draw dots on screen
                if(!possiblePosition.contains(nextPosition)) possiblePosition.add(nextPosition);	            	
            	
                // check position where we end up
            	playoutScore = Navigator.evaluateShipPositionVisibleNode(forThisAction, aimedNode);
                if (playoutScore.distance < baseActionScore.distance)
                {
                	baseActionScore.distance = playoutScore.distance;
                }
	            else if((int)playoutScore.distance == (int)baseActionScore.distance && playoutScore.direction > baseActionScore.direction)
	            {
	            	if (verbose) System.out.println("direction updated " + baseActionScore.direction + " > " + playoutScore.direction);
	            	baseActionScore.distance = playoutScore.distance;
	            	baseActionScore.direction = playoutScore.direction;
	            }                                
                actionData[baseAction].score = actionData[baseAction].score + playoutScore.distance;                
    		}
    		    		   	
        }
    	//find the best action
    	int bestAction = -1;
    	double bestScore = Double.MAX_VALUE;
    	
    	for (int i =1 ; i < actionData.length; i++)
    	{
    		System.out.println("-----at action " + i);
    		System.out.println("act " + actionData[i].action);
    		System.out.println("vis " + actionData[i].visited);
    		System.out.println("scr " + actionData[i].score);
    		//skip unused actions
    		if (-1 == actionData[i].action) 
    		{
    			System.out.println("skipping " + i);
    			continue;
    		}
    		//calculate value
    		actionData[i].value = actionData[i].score / actionData[i].visited;
    		if (actionData[i].value < bestScore)
    		{
    			bestScore = actionData[i].value;
    			bestAction = actionData[i].action;
    		}
    	}
    	
    	System.out.println("returning " + bestAction);    	
        if(verbose) System.out.println("\n>>>out\t\t" + System.currentTimeMillis());
    	return bestAction;
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
       
    /**
     * paint additional info, called automatically every frame
     */
    public void paint(Graphics2D a_gr)
    {  
    	Painter.paintPossibleShipPositions(a_gr, possiblePosition);
    	Painter.paintPaths(a_gr, m_graph, plannedPath, Color.gray);
    	Painter.paintAimedNode(a_gr, aimedNode);
    }  
    
    

}



class ActionData {
	int action = -1;//the action for which we store data
	int visited = 0;
	double score = 0;
	double value;
}

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
 * TODO:
 *  - check implementation, this should be a naive mcts
 *  - takes pauses after a  while, something is not cleaned up
 *  !!! stop wasting time on this, use gvgai and make mcts !!!
 * @version 141128
 * @author Cristian
 *
 */

public class DriveMC extends Controller
{
    private Graph m_graph; //     * Graph for this controller.
    private ArrayList<Path> m_plannedPath = new ArrayList<>();//      * Paths to the waypoints based on the planner
	private LinkedList<Waypoint> m_orderedWaypoints = new LinkedList<>(); //waypoints in the order they should be visited as computed by the planner
    private HashMap<GameObject, Node> m_collectNodes;//     * Hash map that matches waypoints in the map with their closest node in the graph.
	private Waypoint m_nextWaypoint; //	 * Next waypoint in the list resulted by planner
    private Integer nextWaypoint = null; // index of next waypoint in m_orderedWaypoints 
    private int ticks = 0;    //how many ticks passed
    private Node aimedNode = null;
    private Path pathToFollow;
    private ArrayList<Vector2d> possiblePosition = new ArrayList<>();
    public Random m_rnd = new Random();    

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
    	boolean verbose = true;
//    	verbose = ((ticks % 100 == 0) ? true : false);
    	
        long timeIn = System.currentTimeMillis();
        if(verbose) System.out.println("\n>>>in\t\t" + timeIn);
        if(verbose) System.out.println("---due in\t" + a_timeDue);
              
        long remainingTime;
        remainingTime = a_timeDue - System.currentTimeMillis();
                       
        possiblePosition.clear();//display just one level of search
        
    	//target reached?
        if(m_nextWaypoint.checkCollected(a_gameCopy.getShip().ps, m_nextWaypoint.radius/4*3))
    	{
    		System.out.println("!!!!!marked as completed " + m_nextWaypoint);
    		m_nextWaypoint.setCollected(true);
    		m_orderedWaypoints.get(m_orderedWaypoints.indexOf(m_nextWaypoint)).setCollected(true);    		
    		
    		//set next waypoint
    		nextWaypoint = m_orderedWaypoints.indexOf(m_nextWaypoint)+1;
    		m_nextWaypoint = m_orderedWaypoints.get(m_orderedWaypoints.indexOf(m_nextWaypoint)+1);
    	}
            	
        //get path to it from the previous waypoint    	    
        pathToFollow = m_plannedPath.get(nextWaypoint-1);      
    	
        //Get the next node to go to, from the path to the closest waypoint/ fueltank
    	aimedNode = Navigator.getLastNodeVisible(pathToFollow, a_gameCopy, m_graph);  

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
            	Game forThisAction = a_gameCopy.getCopy(); 

            	// in each, make some random moves
            	for (int j = 0; j < depth; j++)
            	{
            		// select a random action
                	int action = m_rnd.nextInt(Controller.NUM_ACTIONS-1)+1;//avoid action 0        	                   	            	
                	forThisAction.getShip().update(action);
                	
                	// stop if target is reached
//                	if(m_nextWaypoint.checkCollected(forThisAction.getShip().ps, m_nextWaypoint.radius/4*3)) break;                  	
            	}
            	
              	// get new ship position
                Vector2d nextPosition = forThisAction.getShip().s;
                // draw dots on screen
                if(!possiblePosition.contains(nextPosition)) possiblePosition.add(nextPosition);	            	
            	
                // check position where we end up
            	playoutScore = Navigator.evaluateShipPosition(forThisAction, aimedNode);
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
     * paint additional info, called automatically every frame
     */
    public void paint(Graphics2D a_gr)
    {  
    	Painter.paintPossibleShipPositions(a_gr, possiblePosition);
    	Painter.paintPaths(a_gr, m_graph, m_plannedPath, Color.gray);
    	Painter.paintAimedNode(a_gr, aimedNode);
    }  
    

}



class ActionData {
	int action = -1;//the action for which we store data
	int visited = 0;
	double score = 0;
	double value;
}

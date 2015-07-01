package controllers.mcts;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Random;

import planners.Planner;
import planners.Planner2Opt;
import planners.Planner3Opt;
import planners.PlannerGreedy;
import planners.PlannerMC;
import framework.ExecSync;
import framework.core.Controller;
import framework.core.FuelTank;
import framework.core.Game;
import framework.core.GameObject;
import framework.core.PTSPView;
import framework.core.Waypoint;
import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;
import framework.utils.JEasyFrame;
import framework.utils.Navigator;
import framework.utils.Painter;
import framework.utils.Value;
import framework.utils.Vector2d;

/**
 * Monte-Carlo tree search driver
 * @version 150621
 * @author Cristian
 * TODO 7 implement transposition tables for states (see 2013 detail paper)
 */

public class DriveMCTS extends Controller
{
	public static boolean verbose = false;
	static double sqrt2 = Math.sqrt(2);
	public static double ucb1Exploration = 1;//C ucb1 exploration constant
	
	//controller weights
	public static double scorePanicMode = 0.1;//a0 score threshold for panic mode 0.1
	public static double search_wpCollectedRoute = 1;//a1 waypoints collected on the planned route 1
	public static double search_wpCollectedOutOfRoute = -1;//a2 waypoints collected out of the planned route -1
	public static double search_wpDistance = 0.75;//a3 distance to next waypoint 0.75
	public static double search_fuelConsumed = 0.001;//a4 penalty fuel consumed 0.001
	public static double search_wpFuelOutOfRoute = 0.2;//a5 bonus for fuel tank 0.2 (without planned fuel)
	public static double search_damageIncurred = 0.002;//a6 damage taken 0.002
	public static double search_damageCollisions = 0.3;//a7 damage taken only from collisions 0.3
	public static int search_searchDepth = 10;//d playout depth
	//TODO 3 what's wrong with setting this to 1? even without macro time	
	public static int macroActionsCount = 5;//T macro action length	
			
	//panic mode weights	
	static double panic_searchDepth = 3;
	static double panic_fuelConsumed = 0;
	static double panic_wpFuelOutOfRoute = 0;
	static double panic_damageIncurred = 0;
	static double panic_damageCollisions = 0;
	static double panic_wpCollectedRoute = 10;
	static double panic_wpDistance = 0.75;
	
	//variables actually used	
	static double w_wpCollectedRoute = search_wpCollectedRoute;	
	static double w_fuelConsumed = search_fuelConsumed;
	static double w_wpFuelOutOfRoute = search_wpFuelOutOfRoute;
	static double w_damageIncurred = search_damageIncurred;
	static double w_damageCollisions = search_damageCollisions;
	static double w_wpDistance = search_wpDistance;
	static double searchDepth = search_searchDepth;
	
	
	//planner weights	
	public static double w_lava = 2;//l increase distance when passing over lava 2
	public static double w_distance = 1;//b1 travel distance 1 
	public static double w_directness = 150;//b2 ratio between travel distance and euclidean distance 150
	public static double w_angle = 80;//b3 angle change between entering and exiting a waypoint 80
	public static boolean p_includeFuel = false;//b4 include fuel tanks in planning true
	public static double w_fuelTankCost = 200;//b5 cost of picking up a fuel tank 200
	public static double w_consecutiveFuelTanksCost = 1000;//b6 cost of picking up consecutive fuel tanks 1000
    
	boolean inPanicMode = false;
		
		
	private LinkedList<GameObject> orderedWaypoints;	

	private Graph aGraph;
	private Node aimedNode;
    private ArrayList<Path> aPlannedPath = new ArrayList<>();// Paths to the waypoints based on the planner
    static ArrayList<Vector2d> possiblePosition = new ArrayList<>();    
    static ArrayList<Vector2d> panicPosition = new ArrayList<>();
    
    private int ticks = 0;
    
    private SearchTreeNode searchTree;
    int macroActionsRemaining = 0;
    int macroAction;
    int ongoingsearchresult = -1;

    public Random rnd = new Random();	

    /**
     * Constructor, that receives a copy of the game state
     * @param aGameCopy a copy of the game state
     * @param a_timeDue The time the initialization is due. Finishing this method after a_timeDue will disqualify this controller.
     */
    public DriveMCTS(Game aGameCopy, long a_timeDue)
    {
    	System.out.println("**** mcts controller ****");
        aGraph = new Graph(aGameCopy);
       
        Planner planner = new Planner3Opt(aGameCopy);//remove three edges and reconnect the graph
//        Planner planner = new PlannerGreedy(aGameCopy);//distance only
//        Planner planner = new PlannerMC(aGameCopy, a_timeDue);//use greedy for testing
        
        Planner.weightLava = w_lava;
        Planner.weightDistance = w_distance;
        Planner.weightDirectness = w_directness;
        Planner.weightAngle = w_angle;
    	Planner.weightFuelTankCost = w_fuelTankCost;
    	Planner.weightConsecutiveFuelTanksCost = w_consecutiveFuelTanksCost;
    	Planner.includeFuel = p_includeFuel;
    	
    	planner.runPlanner();
    	orderedWaypoints = planner.getOrderedWaypoints();
        
        planner.calculateOrderedWaypointsPaths();//calculate the paths from one waypoint to another
        aPlannedPath = planner.getPlannedPath();//get the path from one waypoint to the next
    }
    

    
    /**
     * This function is called every execution step to get the action to execute.
     * @param aGameCopy Copy of the current game state.
     * @param a_timeDue The time the next move is due
     * @return the integer identifier of the action to execute (see interface framework.core.Controller for definitions)
     */
    public int getAction(Game aGameCopy, long a_timeDue)
    {
    	ticks++;
    	if(false == ExecSync.m_visibility) if(0 == ticks % 100) System.out.print(".");
    	//verbose = ((ticks % 10 == 0) ? true : false);    	
        long timeIn = System.currentTimeMillis();
        if(verbose) System.out.println("\n>>>in\t\t" + timeIn);
        if(verbose) System.out.println("   due on\t" + a_timeDue);
        
		if(macroActionsRemaining-- > 0)
		{				
			if(true == ExecSync.m_visibility) System.out.print('.');
			if(verbose) System.out.println("---continuing");
			possiblePosition.clear();//display just one level of search
			if(macroActionsRemaining+1 <= DriveMCTS.macroActionsCount/4)
			{
				if(!inPanicMode)
				{
					//check root node
					double rootNodeFitness = searchTree.evaluateShipPosition(searchTree.worldSate, orderedWaypoints, aGraph);								
					if ( rootNodeFitness + scorePanicMode < searchTree.value) 
					{
						usePanicWeights();
						if(true == ExecSync.m_visibility) System.out.print("panic");
					} 									
				}
			}
			//mctsSearch call			
			ongoingsearchresult = mctsSearch(aGameCopy, a_timeDue);
			if(verbose) System.out.print("macro actions remaining " + macroActionsRemaining + "[" + macroAction + "]\n");			
			return macroAction;			
		}		
		//done with a macro action, clean up
        searchTree = null;
        useSearchWeights();
    	int bestAction = -1;
    	if(macroActionsRemaining <= 0)
		{    		
			//mctsSearch call
    		if(verbose) System.out.println("\n---new search");
			ongoingsearchresult = mctsSearch(aGameCopy, a_timeDue);
			if(true == ExecSync.m_visibility) System.out.print("\n" + ongoingsearchresult);			
		} 		
        macroAction = ongoingsearchresult;
        macroActionsRemaining = macroActionsCount;
        bestAction = macroAction;
        
        //display next target
        if(true == ExecSync.m_visibility)
        {
	        //get target waypoint
	        GameObject target = null;
	        for (GameObject way : orderedWaypoints)
	        {        	
	        	if(!isCollected(way))
	        	{
	        		target = way;
	        		break;
	        	}
	        }
	        isWaypointReached(aGameCopy, target);        	       
	        aimedNode = aGraph.getClosestNodeTo(target.s.x, target.s.y);    	
        }
        
        if(verbose) System.out.println("\n>>>out\t\t" + System.currentTimeMillis());              
        return bestAction;    	
    }

    
	public int mctsSearch(Game a_gameCopy, long timeDue)
    {	
    	long remainingTime;
    	long now = System.currentTimeMillis();
        remainingTime = timeDue - now;
//        System.out.print("\n now:" + now );
//        System.out.print("\n due:" + timeDue);
//    	System.out.print("\n remaining:" + remainingTime );
        
        SearchTreeNode rootNode;
        if(searchTree == null)
        {
//        	System.out.println("new search");        
//        	create root node for initial state
        	rootNode = new SearchTreeNode(a_gameCopy, null);
        } else 
        {
//        	System.out.println("continuing search");
        	rootNode = SearchTreeNode.copyTree(searchTree);        	
        }
        //make sure tree increases
//        System.out.println(SearchTreeNode.getTotalChildren(rootNode));
        
    	//create root node for initial state
//    	SearchTreeNode rootNode = new SearchTreeNode(a_gameCopy, null);
    	if (verbose) System.out.println(" root data: " + SearchTreeNode.getTotalChildren(rootNode));
    	
    	int bestAction = -1;    	    	
    	int playouts = 0;
        while(remainingTime > 5)
        {
        	playouts++;
        	if (verbose) System.out.print("\n\n" +ticks + " : " + playouts);        	        

        	// apply tree policy to select the urgent node
        	SearchTreeNode urgentNode = treePolicy(rootNode);
        	
        	// simulate
        	if (verbose) System.out.println(" simulating from " + SearchTreeNode.getFullIdentifier(urgentNode));
        	double matchValue = urgentNode.simulate(orderedWaypoints, aGraph);
        	
        	// back propagate
        	urgentNode.backPropagate(urgentNode, matchValue);
        	
        	remainingTime = timeDue - System.currentTimeMillis();
        	if (verbose) System.out.print("\n" + ticks + " : "  + playouts + " value:" + matchValue + " remaining: " + remainingTime );        	
        	if (verbose) System.out.println("\ntotal children nodes : " + SearchTreeNode.getTotalChildren(rootNode));
        }
        //select child node
		bestAction = rootNode.getActionRobustChild();//most visited child		
//		bestAction = rootNode.getActionSecureChild();//lowest average score child
//        bestAction = rootNode.getActionMinValue();//lowest average score child                 
       
//        System.out.println("\n=====full tree");
//        rootNode.present();
        //TODO 9 erratic when copying the child, due to different depth end
        //store the selected tree
//        searchTree = SearchTreeNodeLive.copyTree(rootNode.getChild(bestAction));
        searchTree = SearchTreeNode.copyTree(rootNode);
//        System.out.println("\nselected : " + bestAction);
//        System.out.println("\n=====leaf");
//        searchTree.present();               
    	return bestAction;
    }
	
    /**
     * returns the next node in the tree to simulate
     * @param incomingNode
     * @return
     */
	private SearchTreeNode treePolicy(SearchTreeNode incomingNode) {
    	SearchTreeNode currentNode = incomingNode;
        while (currentNode.depth < searchDepth)
        {
	        if (!currentNode.isFullyExpanded()) 
	        {
	        	if (verbose) System.out.println("\n " + currentNode.getIdentifier() + " expanding");
	            return currentNode.expand(false);
	        } else 
	        {        	
	        	SearchTreeNode nextNode = currentNode.uct();
//	        	SearchTreeNodeLive nextNode = currentNode.egreedy();	        	
	        	if (verbose) System.out.println("\n" + currentNode.getIdentifier() + " uct decided: " + nextNode.getIdentifier() );	               
	            currentNode = nextNode;
	        }
        }
        return currentNode;
    }
	
	private void useSearchWeights() 
	{
    	inPanicMode = false;
		//switch to panic mode weights
		w_wpCollectedRoute = search_wpCollectedRoute;	
		w_fuelConsumed = search_fuelConsumed;
		w_wpFuelOutOfRoute = search_wpFuelOutOfRoute;
		w_damageIncurred = search_damageIncurred;
		w_damageCollisions = search_damageCollisions;
		searchDepth = search_searchDepth;
	}

	private void usePanicWeights() 
	{
		inPanicMode = true;
		//switch to panic mode weights
		w_wpCollectedRoute = panic_wpCollectedRoute;	
		w_fuelConsumed = panic_fuelConsumed;
		w_wpFuelOutOfRoute = panic_wpFuelOutOfRoute;
		w_damageIncurred = panic_damageIncurred;
		w_damageCollisions = panic_damageCollisions;
		searchDepth = panic_searchDepth;
	}

	
    /**
     * is collected for both waypoint and fuel tank
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
     * checks if the ship reached the next waypoint
     * @param a_gameCopy
     * @return
     */
    public boolean isWaypointReached(Game aGameCopy, GameObject nextWaypoint)
    {
    	//target reached?
    	if(nextWaypoint instanceof Waypoint)
    	{
    		if(((Waypoint) nextWaypoint).checkCollected(aGameCopy.getShip().ps, nextWaypoint.radius/4*3))
    		{
    			((Waypoint) nextWaypoint).setCollected(true);
    			((Waypoint) orderedWaypoints.get(orderedWaypoints.indexOf(nextWaypoint))).setCollected(true);    			
    			return true;
    		}
    	}
    	else if(nextWaypoint instanceof FuelTank)
    	{
    		if(((FuelTank) nextWaypoint).checkCollected(aGameCopy.getShip().ps, nextWaypoint.radius/4*3))
    		{
    			((FuelTank) nextWaypoint).setCollected(true);
    			((FuelTank) orderedWaypoints.get(orderedWaypoints.indexOf(nextWaypoint))).setCollected(true);
    			return true;
    		}
    	}
    	return false;   
    }
    	        
  	/**
     * paint additional info
     */
    public void paint(Graphics2D a_gr)
    {  
    	Painter.paintAimedNode(a_gr, aimedNode);
    	Painter.paintPossibleShipPositions(a_gr, possiblePosition);
    	Painter.paintPaths(a_gr, aGraph, aPlannedPath, Color.gray);
    }
}
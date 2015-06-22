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
 * TODO 0 debug planner path, weights one by one
 * TODO 0 debug mcts search, weights one by one
 * TODO 0 link the parameters to CMA-ES library
 * TODO 7 implement transposition tables for states (see 2013 detail paper)
 */

public class DriveMCTS extends Controller
{
	public static boolean verbose = false;
	static double sqrt2 = Math.sqrt(2);
	
	//controller weights
	//TODO 1 implement panic mode
	static double scorePanicMode = 0.1;//a0 score threshold for panic mode
	static double w_wpCollectedRoute = 1;//a1 waypoints collected on the planned route
	static double w_wpCollectedOutOfRoute = -1;//a2 waypoints collected out of the planned route
	static double w_wpDistance = 0.75;//a3 distance to next waypoint
	static double w_fuelConsumed = 0.001;//a4 total fuel consumed
	static double w_wpFuelOutOfRoute = 1;//a5 fuel tanks picked up opportunistically	
	static double w_damageIncurred = 0.002;//a6 damage taken
	static double w_damageCollisions = 0.3;//a7 damage taken only from collisions
	
	//planner weights	
	static double w_lava = 1.5;//l increase distance when passing over lava
	static double w_distance = 1.5;//b1 travel distance
	static double w_directness = 150;//b2 ratio between travel distance and euclidean distance
	static double w_angle = 80;//b3 angle change between entering and exiting a waypoint
	static boolean p_includeFuel = true;//b4 include fuel tanks in planning	
	static double w_fuelTankCost = 200;//b5 cost of picking up a fuel tank 
	static double w_consecutiveFuelTanksCost = 1000;//b6 cost of picking up consecutive fuel tanks
    //TODO 3 what's wrong with setting this to 1? even without macro time.	
	static int macroActionsCount = 5;//T macro action length
	static int searchDepth = 10;//d playout depth
	static double ucb1Exploration = 1;//C ucb1 exploration constant	
		
	private LinkedList<GameObject> orderedWaypoints;	

	private Graph aGraph; // Graph for this controller.
	private Node aimedNode;
    private ArrayList<Path> aPlannedPath = new ArrayList<>();// Paths to the waypoints based on the planner
    static ArrayList<Vector2d> possiblePosition = new ArrayList<>();    
    static ArrayList<Vector2d> panicPosition = new ArrayList<>();
    
    //TODO 0 consider using a list of nodes instead of gameobject (waypoint / fuel tank)
        
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
        aGraph = new Graph(aGameCopy);//Init the graph.
       
        Planner planner = new Planner3Opt(aGameCopy);//remove three edges and reconnect the graph
//        Planner planner = new PlannerGreedy(a_gameCopy);//use greedy for testing
//        Planner planner = new PlannerMC(a_gameCopy, a_timeDue);//use greedy for testing
        
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
        
//        System.out.println("planned path " + aPlannedPath.get(0).m_cost);
                
        //TODO - this stops the execution 0
//        System.exit(1);////////////////////////////////////////////////////////////////////////////       
    }
    
    /**
     * This function is called every execution step to get the action to execute.
     * @param a_gameCopy Copy of the current game state.
     * @param a_timeDue The time the next move is due
     * @return the integer identifier of the action to execute (see interface framework.core.Controller for definitions)
     */
    public int getAction(Game a_gameCopy, long a_timeDue)
    {
    	ticks++;
    	//verbose = ((ticks % 10 == 0) ? true : false);    	
        long timeIn = System.currentTimeMillis();
        if(verbose) System.out.println("\n>>>in\t\t" + timeIn);
        if(verbose) System.out.println("---due on\t" + a_timeDue);
        
		//TODO 2 macro action are slower than normal actions. better evaluation might fix? macro in simulation?
		if(macroActionsRemaining-- > 0)
		{ 
			//mctsSearch call
			System.out.println("---continuing");
			ongoingsearchresult = mctsSearch(a_gameCopy, a_timeDue);
			System.out.print("macro actions remaining " + macroActionsRemaining + "[" + macroAction + "]\n");
			return macroAction;			
		}
		//done with a macro action, clean up
        searchTree = null;      
        possiblePosition.clear();//display just one level of search
    	int bestAction = -1;
    	if(macroActionsRemaining <= 0)
		{ 
			//mctsSearch call
			System.out.println("\n---new search");
			ongoingsearchresult = mctsSearch(a_gameCopy, a_timeDue);
		} 		
        macroAction = ongoingsearchresult;
        macroActionsRemaining = macroActionsCount;
        bestAction = macroAction;
        
        //get target waypoint
        GameObject target = null;
        for (GameObject way : orderedWaypoints)
        {
        	if(way instanceof Waypoint && !((Waypoint)way).isCollected())
        	{
        		target = way;
        		break;
        	}
        	if(way instanceof FuelTank && !((FuelTank)way).isCollected())
        	{
        		target = way;
        		break;
        	}
        }
        aimedNode = aGraph.getClosestNodeTo(target.s.x, target.s.y);
    	
        if(verbose) System.out.println("\n>>>out\t\t" + System.currentTimeMillis());      
        //TODO - this stops the execution 0
//        System.exit(1);////////////////////////////////////////////////////////////////////////////
        
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
        
//        SearchTreeNodeLive rootNode;
        if(searchTree == null)
        {
        	System.out.println("+++++++++new search");        
        	//create root node for initial state
//        	rootNode = new SearchTreeNodeLive(a_gameCopy, null);
        } else 
        {
        	System.out.println("+++++++++continuing search...");
//        	rootNode = SearchTreeNodeLive.copyTree(searchTree);        	
        }
        
    	//create root node for initial state
    	SearchTreeNode rootNode = new SearchTreeNode(a_gameCopy, null);
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
        	if (verbose) System.out.println(" simulating from " + SearchTreeNode.getFullIdentifier(urgentNode));//TODO 1 basic debug mcts
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
//      System.out.println("selected "+ bestAction);               
       
//        System.out.println("\n=====full tree");
//        rootNode.present();
        //TODO 9 supposedly erratic when copying the child, due to different depth end
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
    	//TODO 1 basic debug mcts
        while (currentNode.depth < searchDepth)
        {
	        if (!currentNode.isFullyExpanded()) 
	        {
	        	if (verbose) System.out.println("\n" + currentNode.getIdentifier() + " expanding");
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
     * paint additional info
     */
    public void paint(Graphics2D a_gr)
    {  
    	Painter.paintAimedNode(a_gr, aimedNode);
    	Painter.paintPossibleShipPositions(a_gr, possiblePosition);
    	Painter.paintPaths(a_gr, aGraph, aPlannedPath, Color.gray);
    }
}
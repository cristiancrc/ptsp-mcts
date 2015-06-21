package controllers.mctsLive;

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
 * monte carlo tree search driver using live view - next visible point on path and the next from that point on
 * @version 150621
 * @author Cristian
 */

public class DriveMCTSLive extends Controller
{
	public static boolean verbose = false;
	
	//controller weights
	static int scorePanicMode = 0;//a0 score threshold for panic mode
	public static int w_wpCollectedRoute = 1;//a1 waypoints collected on the planned route
	public static int w_wpCollectedOutOfRoute = 1;//a2 waypoints collected out of the planned route
	public static int w_wpDistance = 1;//a3 distance to next waypoint
	public static int w_fuelConsumed = 1;//a4 total fuel consumed
	public static int w_damageIncurred = 1;//a5 damage taken
	public static int w_wpFuelOutOfRoute = 1;//a6 fuel tanks picked up opportunistically
	public static int w_damageCollisions = 1;//a7 damage taken only from collisions
	
	//planner weights	
	public static double w_lava = 1;//l increase distance when passing over lava
	public static double w_distance = 1;//b1 travel distance
	public static double w_directness = 1;//b2 ratio between travel distance and euclidean distance
	public static double w_angle = 1;//b3 angle change between entering and exiting a waypoint
	public static boolean p_includeFuel = true;//b4 include fuel tanks in planning	
	public static double w_fuelTankCost = 1;//b5 cost of picking up a fuel tank 
	public static double w_consecutiveFuelTanksCost = 1;//b6 cost of picking up consecutive fuel tanks
    //TO DO 3 what's wrong with setting this to 1? even without macro time.	
	public static int macroActionsCount = 5;//T macro action length
	public static int searchDepth = 10;//d playout depth
	public static double ucb1Exploration = 0;//C ucb1 exploration constant	
	
    private Graph aGraph; // Graph for this controller.
    private ArrayList<Path> aPlannedPath = new ArrayList<>();//      * Paths to the waypoints based on the planner
	private LinkedList<GameObject> orderedWaypoints = new LinkedList<>(); //waypoints in the order they should be visited as computed by the planner
	private GameObject nextWaypoint;
	private Integer nextWaypointIndex = null; // index of next waypoint in orderedWaypoints
    private int ticks = 0;
    static Node aimedNode = null;//the farthest node in the path that can be seen
    static Node aimedNodeNext = null;//the farthest node in the path that can be seen from aimedNode
    private Path pathToFollow;
    static ArrayList<Vector2d> possiblePosition = new ArrayList<>();    
    static HashMap<Vector2d, Double> possiblePositionScore = new HashMap<Vector2d, Double>();
//    static ArrayList<Vector2d> possiblePositionEnd = new ArrayList<Vector2d>();
    static ArrayList<Vector2d> panicPosition = new ArrayList<>();
    
    private SearchTreeNodeLive searchTree;
        
    boolean inPanicMode = false;
    long panicModeStart;
    long panicModeLength = 1000;//in miliseconds
    int panicModeCheckInterval = 5;//how many ticks to wait between checks
    int panicModeNextCheck = panicModeCheckInterval;    
    int panicModeAction = 1;//action to take while in panic mode
    Vector2d previousShipPosition;

    
    int macroActionsRemaining = 0;
    int macroAction;
    int ongoingsearchresult = -1;

    

	
    /*
     * searchDepth * macroActionsCount should be around 100
     * 
     * deeper than that and the paths are too chaotic
     * count = 1, searchDepth = 100  : no macro actions 
     * count = 5, searchDepth = 20
     * 
     * paper: 5 x 10
     * 
     */
    public Random rnd = new Random();

    /**
     * Constructor, that receives a copy of the game state
     * @param aGameCopy a copy of the game state
     * @param a_timeDue The time the initialization is due. Finishing this method after a_timeDue will disqualify this controller.
     */
    public DriveMCTSLive(Game aGameCopy, long a_timeDue)
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
    	
    	planner.runPlanner();
        
        orderedWaypoints = planner.getOrderedWaypoints();//get the planned route
        nextWaypoint = orderedWaypoints.get(0);//set immediate goal        
        planner.calculateOrderedWaypointsPaths();//calculate the paths from one waypoint to another
        aPlannedPath = planner.getPlannedPath();//get the path from one waypoint to the next
        
//        System.out.println("planned path " + aPlannedPath.get(0).m_cost);
                
        //TO DO - this stops the execution 0
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
        
		//TO DO 2 macro action are slower than normal actions. better evaluation might fix? macro in simulation?
		if(macroActionsRemaining-- > 0)
		{ 
			//mctsSearch call
			System.out.println("---continuing");
			ongoingsearchresult = mctsSearch(a_gameCopy, a_timeDue);
			System.out.print("macro actions remaining " + macroActionsRemaining + "[" + macroAction + "]\n");
			return macroAction;			
		}
        searchTree = null;      
        possiblePosition.clear();//display just one level of search
    	int bestAction = -1;
//        if(inPanicMode)
//    	{        
//        	if (System.currentTimeMillis() > panicModeStart + panicModeLength)
//        	{
//        		if (verbose) System.out.println("\t<<<out of panic mode");
//        		/*
//        		 * the panic mode action can take longer than the normal next interval
//        		 * so push next check further so a normal action can be applied
//        		 */
//        		panicModeNextCheck += 100 *panicModeCheckInterval;
//        		inPanicMode = false;
//        	}
//        	else
//        	{
//        		if (verbose) System.out.println("panic mode!!!");
//        		return panicModeAction;
//        	}        		
//    	}
    	if(isWaypointReached(a_gameCopy)) advanceWaypointTarget();
        
        //get path the next waypoint    	    
        pathToFollow = aPlannedPath.get(nextWaypointIndex-1);
        Path pathToFollowFarther = aPlannedPath.get(nextWaypointIndex-1);
        
        //Get the next node to go to, from the path to the closest waypoint/ fueltank
    	aimedNode = Navigator.getLastNodeVisible(pathToFollow, a_gameCopy, aGraph);
    	//create a copy of the game
    	Game a_gameCopyForAimedNode = a_gameCopy.getCopy();
    	//set put the ship in the aimed node position
    	a_gameCopyForAimedNode.getShip().s.x = aimedNode.x();
    	a_gameCopyForAimedNode.getShip().s.y = aimedNode.y();
    	aimedNodeNext = Navigator.getLastNodeVisible(pathToFollowFarther, a_gameCopyForAimedNode, aGraph);
    	
    	if(//both nodes are on the same position (next waypoint)
    			aimedNodeNext.x() == aimedNode.x() &&
    			aimedNodeNext.y() == aimedNode.y() &&
    			nextWaypointIndex < aPlannedPath.size()
    		)
    	{
    		pathToFollowFarther = aPlannedPath.get(nextWaypointIndex);
    		aimedNodeNext = Navigator.getLastNodeVisible(pathToFollowFarther, a_gameCopyForAimedNode, aGraph);    		
    	}
		if(macroActionsRemaining <= 0)
		{ 
			//mctsSearch call
			System.out.println("\n---new search");
			ongoingsearchresult = mctsSearch(a_gameCopy, a_timeDue);
		} 		
        macroAction = ongoingsearchresult;
        macroActionsRemaining = macroActionsCount;
        bestAction = macroAction;
    	
//    	//check if ship is stuck in the same position
//    	if(ticks >= panicModeNextCheck)
//    	{
//        	Vector2d currentShipPosition = a_gameCopy.getShip().s;
//        	if(currentShipPosition.equals(previousShipPosition))
//        	{
//        		if (verbose) System.out.println(">>>entering panic mode");
//        		inPanicMode = true;
//        		panicModeStart = System.currentTimeMillis();
//           	 	if(!panicPosition.contains(currentShipPosition))
//                {
//           	 		panicPosition.add(currentShipPosition);	
//                }        		
//        		return panicModeAction;         		
//        	}
//        	previousShipPosition = a_gameCopy.getShip().s;
//        	panicModeNextCheck = ticks + panicModeCheckInterval;
//    	}    	
        if(verbose) System.out.println("\n>>>out\t\t" + System.currentTimeMillis());

        //TO DO - this stops the execution 0
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
    	SearchTreeNodeLive rootNode = new SearchTreeNodeLive(a_gameCopy, null);
    	if (verbose) System.out.println(" root data: " + SearchTreeNodeLive.getTotalChildren(rootNode));
    	
    	int bestAction = -1;    	    	
    	int playouts = 0;
        while(remainingTime > 5)
        {
        	playouts++;
        	if (verbose) System.out.print("\n\n" +ticks + " : " + playouts);        	        

        	// apply tree policy to select the urgent node
        	SearchTreeNodeLive urgentNode = treePolicy(rootNode);
        	
        	// simulate
        	if (verbose) System.out.println(" simulating from " + SearchTreeNodeLive.getFullIdentifier(urgentNode));//TO DO 1 basic debug mcts
        	double matchValue = urgentNode.simulateTarget(aimedNode);
        	
        	// back propagate
        	urgentNode.backPropagate(urgentNode, matchValue);
        	
        	remainingTime = timeDue - System.currentTimeMillis();
        	if (verbose) System.out.print("\n" + ticks + " : "  + playouts + " value:" + matchValue + " remaining: " + remainingTime );        	
        	if (verbose) System.out.println("\ntotal children nodes : " + SearchTreeNodeLive.getTotalChildren(rootNode));
        }
        //select child node
		bestAction = rootNode.getActionRobustChild();//most visited child		
//		bestAction = rootNode.getActionSecureChild();//lowest average score child
//        bestAction = rootNode.getActionMinValue();//lowest average score child                 
//      System.out.println("selected "+ bestAction);               
       
//        System.out.println("\n=====full tree");
//        rootNode.present();
        //TO DO 9 supposedly erratic when copying the child, due to different depth end
        //store the selected tree
//        searchTree = SearchTreeNodeLive.copyTree(rootNode.getChild(bestAction));
        searchTree = SearchTreeNodeLive.copyTree(rootNode);
//        System.out.println("\nselected : " + bestAction);
//        System.out.println("\n=====leaf");
//        searchTree.present();               
    	return bestAction;
    }      

	private SearchTreeNodeLive treePolicy(SearchTreeNodeLive incomingNode) {
    	SearchTreeNodeLive currentNode = incomingNode;
    	//TO DO 1 basic debug mcts
        while (currentNode.depth < searchDepth)
        {
	        if (!currentNode.isFullyExpanded()) {
	        	if (verbose) System.out.println("\n" + currentNode.getIdentifier() + " expanding");
	            return currentNode.expand();
	        } else {        	
	        	SearchTreeNodeLive nextNode = currentNode.uct();
	        	if (verbose) System.out.println("\n" + currentNode.getIdentifier() + " uct decided: " + nextNode.getIdentifier() );
	//                SearchTreeNodeLive nextNode = currentNode.egreedy();
	//                SearchTreeNodeLive nextNode = currentNode.random();//MC search
	            currentNode = nextNode;
	        }
        }
        return currentNode;
    }
	    
    /**
     * checks if the ship reached the next waypoint
     * @param a_gameCopy
     * @return
     */
    public boolean isWaypointReached(Game aGameCopy)
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
    
    public void advanceWaypointTarget()
    {
    	// (verbose) System.out.println("!!!!!marked as completed " + nextWaypoint);
//		(nextWaypoint).setCollected(true);
//		(orderedWaypoints.get(orderedWaypoints.indexOf(nextWaypoint))).setCollected(true);    		
		
		//set next waypoint
		nextWaypointIndex = orderedWaypoints.indexOf(nextWaypoint)+1;
		nextWaypoint = orderedWaypoints.get(orderedWaypoints.indexOf(nextWaypoint)+1);
    }

	/**
     * paint additional info
     */
    public void paint(Graphics2D a_gr)
    {  
    	Painter.paintPossibleShipPositions(a_gr, possiblePosition);
//    	Painter.paintPossibleShipPositions(a_gr, (HashMap<Vector2d, Double>) possiblePositionScore.clone());
    	Painter.paintPanicPositions(a_gr, panicPosition);
    	Painter.paintPaths(a_gr, aGraph, aPlannedPath, Color.gray);
    	Painter.paintAimedNode(a_gr, aimedNode);
    	Painter.paintAimedNode(a_gr, aimedNodeNext, Color.GREEN);
    }
    
	/**
     * computes a score / cost for getting from the current position to the aimedNode
     * @param a_gameCopy
     * @param aimedNode
     * @return
     * //TO DO 8 implement this as a double view a* live alternative to flood fills
     */
    public static Value evaluateShipPositionVisibleNode(Game a_gameCopy, Node aimedNode) 
    {
    	Vector2d nextPosition = a_gameCopy.getShip().s;
		Vector2d potentialDirection = a_gameCopy.getShip().d;
		Vector2d aimedNodeV = new Vector2d(aimedNode.x(),aimedNode.y());
        
		aimedNodeV.subtract(nextPosition);
		aimedNodeV.normalise();   //This is a unit vector from my position pointing towards the next node to go to.
		double dot = potentialDirection.dot(aimedNodeV);  //Dot product between this vector and where the ship is facing to.
		//		Get the distance to the next node in the tree and update the total distance until the closest waypoint/fueltank
		double dist = aimedNode.euclideanDistanceTo(nextPosition.x, nextPosition.y);
		
		Value value = new Value();
		value.distance = dist;
		value.direction = dot;
		value.value = dist + dot;
		return value;
	}

	public LinkedList<GameObject> get_m_orderedWaypoints() {
		return orderedWaypoints;
	}  
}
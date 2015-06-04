package controllers.mcts;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Random;

import planners.Planner;
import planners.Planner3Opt;
import planners.PlannerGreedy;
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
import framework.utils.Vector2d;

/**
 * monte carlo tree search driver
 * @version 150515
 * @author Cristian
 *
 */

public class DriveMCTS extends Controller
{
	public static boolean verbose = true;
	
    private Graph m_graph; //     * Graph for this controller.
    private Node m_shipNode; //     * Node in the graph, closest to the ship position.
    private ArrayList<Path> m_plannedPath = new ArrayList<>();//      * Paths to the waypoints based on the planner
	private LinkedList<Waypoint> m_orderedWaypoints = new LinkedList<>(); //waypoints in the order they should be visited as computed by the planner
    private HashMap<GameObject, Node> m_collectNodes;//     * Hash map that matches waypoints in the map with their closest node in the graph.
	private Waypoint m_nextWaypoint;
	private Integer nextWaypoint = null; // index of next waypoint in m_orderedWaypoints
    private int ticks = 0;
    static Node aimedNode = null;//the farthest node in the path that can be seen
    static Node aimedNodeNext = null;//the farthest node in the path that can be seen from aimedNode
    private Path pathToFollow;
    static ArrayList<Vector2d> possiblePosition = new ArrayList<>();    
    static HashMap<Vector2d, Double> possiblePositionScore = new HashMap<Vector2d, Double>();
//    static ArrayList<Vector2d> possiblePositionEnd = new ArrayList<Vector2d>();
    static ArrayList<Vector2d> panicPosition = new ArrayList<>();
    
    private SearchTreeNode searchTree;
        
    boolean inPanicMode = false;
    long panicModeStart;
    long panicModeLength = 1000;//in miliseconds
    int panicModeCheckInterval = 5;//how many ticks to wait between checks
    int panicModeNextCheck = panicModeCheckInterval;    
    int panicModeAction = 1;//action to take while in panic mode
    Vector2d previousShipPosition;
    //TODO 5 what's wrong with setting this to 1?
    static int macroActionsCount = 5;
    int macroActionsRemaining = 0;
    int macroAction;

    public static int searchDepth = 10;

	
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
    public Random m_rnd = new Random();

    /**
     * Constructor, that receives a copy of the game state
     * @param a_gameCopy a copy of the game state
     * @param a_timeDue The time the initialization is due. Finishing this method after a_timeDue will disqualify this controller.
     */
    public DriveMCTS(Game a_gameCopy, long a_timeDue)
    {
//    	if (true) throw new NotImplementedException();
    	System.out.println("**** mcts controller ****");
        m_graph = new Graph(a_gameCopy);//Init the graph.

        Planner planner = new Planner3Opt(a_gameCopy);//remove three edges and reconnect the graph
//        Planner planner = new PlannerGreedy(a_gameCopy);//use greedy for testing
        m_orderedWaypoints = planner.getOrderedWaypoints();//get the planned route
        m_nextWaypoint = m_orderedWaypoints.get(0);//set immediate goal        
        planner.calculateOrderedWaypointsPaths();//calculate the paths from one waypoint to another
        m_plannedPath = planner.getPlannedPath();//get the path from one waypoint to the next
        
//        System.out.println("planned path " + m_plannedPath.get(0).m_cost);
                
        //TODO - this stops the execution
//        System.exit(1);////////////////////////////////////////////////////////////////////////////
        
        //TODO 1 is this still needed? maybe for nice shows. yea.
        //Init the structure that stores the nodes closest to all waypoints and fuel tanks.
//        m_collectNodes = new HashMap<GameObject, Node>();
//        for(Waypoint way: m_orderedWaypXXaoints)
//        {
//            m_collectNodes.put(way, m_graph.getClosestNodeTo(way.s.x, way.s.y,true));
//        }
//
//        for(FuelTank ft: a_gameCopy.getFuelTanks())
//        {
//            m_collectNodes.put(ft, m_graph.getClosestNodeTo(ft.s.x, ft.s.y,true));
//        }
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
        
		//TODO 6 do something useful with this time macroRS is doing that
		if(macroActionsRemaining-- > 0)
		{
			System.out.print("macro actions remaining " + macroActionsRemaining + "[" + macroAction + "]");
			return macroAction;			
		}        
              
        possiblePosition.clear();//display just one level of search
        possiblePositionScore.clear();
    	
    	int bestAction = -1;
        if(inPanicMode)
    	{        
        	if (System.currentTimeMillis() > panicModeStart + panicModeLength)
        	{
        		if (verbose) System.out.println("\t<<<out of panic mode");
        		/*
        		 * the panic mode action can take longer than the normal next interval
        		 * so push next check further so a normal action can be applied
        		 */
        		panicModeNextCheck += 100 *panicModeCheckInterval;
        		inPanicMode = false;
        	}
        	else
        	{
        		if (verbose) System.out.println("panic mode!!!");
        		return panicModeAction;
        	}
        		
    	}
    	if(isWaypointReached(a_gameCopy)) advanceWaypointTarget();
        
        //get path the next waypoint    	    
        pathToFollow = m_plannedPath.get(nextWaypoint-1);
        Path pathToFollowFarther = m_plannedPath.get(nextWaypoint-1);
        
        //Get the next node to go to, from the path to the closest waypoint/ fueltank
    	aimedNode = Navigator.getLastNodeVisible(pathToFollow, a_gameCopy, m_graph);
    	//create a copy of the game
    	Game a_gameCopyForAimedNode = a_gameCopy.getCopy();
    	//set put the ship in the aimed node position
    	a_gameCopyForAimedNode.getShip().s.x = aimedNode.x();
    	a_gameCopyForAimedNode.getShip().s.y = aimedNode.y();
    	aimedNodeNext = Navigator.getLastNodeVisible(pathToFollowFarther, a_gameCopyForAimedNode, m_graph);
    	
    	if(//both nodes are on the same position (next waypoint)
    			aimedNodeNext.x() == aimedNode.x() &&
    			aimedNodeNext.y() == aimedNode.y() &&
    			nextWaypoint < m_plannedPath.size()
    		)
    	{
    		pathToFollowFarther = m_plannedPath.get(nextWaypoint);
    		aimedNodeNext = Navigator.getLastNodeVisible(pathToFollowFarther, a_gameCopyForAimedNode, m_graph);    		
    	}
    	bestAction = mctsSearch(a_gameCopy, a_timeDue);      
        macroAction = bestAction;
        macroActionsRemaining = macroActionsCount;    	               
    	
    	//check if ship is stuck in the same position
    	if(ticks >= panicModeNextCheck)
    	{
        	Vector2d currentShipPosition = a_gameCopy.getShip().s;
        	if(currentShipPosition.equals(previousShipPosition))
        	{
        		if (verbose) System.out.println(">>>entering panic mode");
        		inPanicMode = true;
        		panicModeStart = System.currentTimeMillis();
           	 	if(!panicPosition.contains(currentShipPosition))
                {
           	 		panicPosition.add(currentShipPosition);	
                }        		
        		return panicModeAction;         		
        	}
        	previousShipPosition = a_gameCopy.getShip().s;
        	panicModeNextCheck = ticks + panicModeCheckInterval;
    	}    	
        if(verbose) System.out.println("\n>>>out\t\t" + System.currentTimeMillis());

//        //View of the game, if applicable.
//        PTSPView aView = new PTSPView(a_gameCopy, a_gameCopy.getMapSize(), a_gameCopy.getMap(), a_gameCopy.getShip(), null);
//        JEasyFrame frame = new JEasyFrame(aView, "imagine");
        //TODO - this stops the execution
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
        
    	//create root node for initial state
    	SearchTreeNode rootNode = new SearchTreeNode(a_gameCopy, null);
    	System.out.println(" root data: " + SearchTreeNode.getTotalChildren(rootNode));
    	
    	int bestAction = -1;    	    	
    	int playouts = 0;
        while(remainingTime > 5)
        {
        	playouts++;
        	if (verbose) System.out.print("\n\n" +ticks + " : " + playouts);        	        

        	// apply tree policy to select the urgent node
        	SearchTreeNode urgentNode = treePolicy(rootNode);
        	
        	// simulate
        	System.out.println(" simulating from " + SearchTreeNode.getFullIdentifier(urgentNode));
        	double matchValue = urgentNode.simulate(aimedNode);
        	
        	// back propagate
        	urgentNode.backPropagate(urgentNode, matchValue);
        	
        	remainingTime = timeDue - System.currentTimeMillis();
        	if (verbose) System.out.print("\n" + ticks + " : "  + playouts + " value:" + matchValue + " remaining: " + remainingTime );        	
        	System.out.println("\ntotal children nodes : " + SearchTreeNode.getTotalChildren(rootNode));
        }
        //TODO 0 time this and set search alloted time accordingly
        //select child node
//		bestAction = rootNode.getActionRobustChild();//most visited child		
//		bestAction = rootNode.getActionSecureChild();//lowest average score child
        bestAction = rootNode.getActionMinValue();//lowest average score child                 
//      System.out.println("selected "+ bestAction);  
        
        System.out.println("\n=====full tree");
        rootNode.present(0);
        
        System.out.println("\n=====leaf");
        searchTree = SearchTreeNode.copyTree(rootNode.getChild(bestAction));        
        searchTree.present(0);
               
        //TODO - this stops the execution
//      System.exit(1);////////////////////////////////////////////////////////////////////////////
    	return bestAction;
    }      

	private SearchTreeNode treePolicy(SearchTreeNode incomingNode) {
    	SearchTreeNode currentNode = incomingNode;

        while (currentNode.depth < searchDepth)
        {
	        if (currentNode.notFullyExpanded()) {
	        	System.out.println(currentNode.getIdentifier() + " expanding");
	            return currentNode.expand();
	        } else {        	
	        	SearchTreeNode nextNode = currentNode.uct();
	        	System.out.println(currentNode.getIdentifier() + " uct decided: " + nextNode.getIdentifier() );
	//                SearchTreeNode nextNode = currentNode.egreedy();
	//                SearchTreeNode nextNode = currentNode.random();//MC search
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
    public boolean isWaypointReached(Game a_gameCopy)
    {
    	//target reached?
        if(m_nextWaypoint.checkCollected(a_gameCopy.getShip().ps, m_nextWaypoint.radius/4*3))
    	{    		
    		return true;
    	}
        else return false;
    }
    
    public void advanceWaypointTarget()
    {
    	System.out.println("!!!!!marked as completed " + m_nextWaypoint);
		m_nextWaypoint.setCollected(true);
		m_orderedWaypoints.get(m_orderedWaypoints.indexOf(m_nextWaypoint)).setCollected(true);    		
		
		//set next waypoint
		nextWaypoint = m_orderedWaypoints.indexOf(m_nextWaypoint)+1;
		m_nextWaypoint = m_orderedWaypoints.get(m_orderedWaypoints.indexOf(m_nextWaypoint)+1);
    }

	/**
     * paint additional info
     */
    public void paint(Graphics2D a_gr)
    {  
    	Painter.paintPossibleShipPositions(a_gr, possiblePosition);
    	Painter.paintPossibleShipPositions(a_gr, possiblePositionScore);
    	Painter.paintPanicPositions(a_gr, panicPosition);
    	Painter.paintPaths(a_gr, m_graph, m_plannedPath, Color.gray);
    	Painter.paintAimedNode(a_gr, aimedNode);
    	Painter.paintAimedNode(a_gr, aimedNodeNext, Color.GREEN);
    }  
}
package controllers.mc;

import framework.core.*;
import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;
import framework.utils.Vector2d;

import java.awt.*;
import java.awt.RenderingHints.Key;
import java.awt.font.FontRenderContext;
import java.awt.font.GlyphVector;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.awt.image.BufferedImageOp;
import java.awt.image.ImageObserver;
import java.awt.image.RenderedImage;
import java.awt.image.renderable.RenderableImage;
import java.text.AttributedCharacterIterator;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Random;
import java.util.Vector;

import javax.swing.text.html.HTMLDocument.HTMLReader.PreAction;

import planners.Planner;
import planners.Planner3Opt;
import planners.PlannerGreedyEvolved;

import controllers.mctsnavi.SearchTreeNode;

/**
 * monte carlo simulation driver
 * TODO:
 *  - why doesn't search fan out? it should be a triangle
 *  - move paint functions to controller or a brand new painter class
 *  - clean up code
 *  - check implementation, this should be a naive mcts
 *  - takes pauses after a  while, something is not cleaned up
 * @version 141128
 * @author Cristian
 *
 */

public class DriveMC extends Controller
{
    private Graph m_graph; //     * Graph for this controller.
    private Node m_shipNode; //     * Node in the graph, closest to the ship position.
    private Path m_pathToClosest; //     * Path to the closest waypoint in the map
    private ArrayList<Path> m_plannedPath = new ArrayList<>();//      * Paths to the waypoints based on the planner
	private LinkedList<Waypoint> m_orderedWaypoints = new LinkedList<>(); //waypoints in the order they should be visited as computed by the planner
    // Path from ship to all waypoints in the map
	private ArrayList<Path> m_pathToWaypoints = new ArrayList<>();
	private ArrayList<Path> m_pathToFuelTanks = new ArrayList<>();

    private HashMap<GameObject, Node> m_collectNodes;//     * Hash map that matches waypoints in the map with their closest node in the graph.

	private Waypoint m_nextWaypoint; //	 * Next waypoint in the list resulted by planner
    private Integer nextWaypoint = null; // index of next waypoint in m_orderedWaypoints 

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
    		m_nextWaypoint = m_orderedWaypoints.get(m_orderedWaypoints.indexOf(m_nextWaypoint)+1);//TODO: end game error?
    	}
            	
        //get path to it from the previous waypoint    	    
        pathToFollow = m_plannedPath.get(nextWaypoint-1);      
    	
        //Get the next node to go to, from the path to the closest waypoint/ fueltank
    	aimedNode = getLastNodeVisible(pathToFollow, a_gameCopy);  

    	//search for the next action
        int bestAction = 0;
        double bestScore = Double.MAX_VALUE;
        int playouts = 0;
        while(remainingTime > 5)
        {
        	playouts++;
        	if (verbose) System.out.print("\n" +ticks + ":" + playouts + ">");

    		// select a random action
        	int action = m_rnd.nextInt(Controller.NUM_ACTIONS-1)+1;//avoid action 0
        	
        	// apply the action            	
        	Game forThisAction = a_gameCopy.getCopy();
        	forThisAction.getShip().update(action);
        	
        	if(m_nextWaypoint.checkCollected(forThisAction.getShip().ps, m_nextWaypoint.radius/4*3)) break;
          	
          	// get new ship position
            Vector2d nextPosition = forThisAction.getShip().s;
            Vector2d potentialDirection = forThisAction.getShip().d;                                
            
            // draw dots on screen
            if(!possiblePosition.contains(nextPosition))
            {
//                	System.out.println(" +");
            	possiblePosition.add(nextPosition);	
            } else
            {
//                	System.out.println(" -");
            }
        	        	
        	remainingTime = a_timeDue - System.currentTimeMillis();
//        	System.out.println("\n" + playouts + " --- remaining:" + remainingTime );
        }
        if(verbose) System.out.println("\n" + ticks + " tick ran: " + playouts + " playouts");
        if(verbose) System.out.println("\n>>>out\t\t" + System.currentTimeMillis());
        ticks++;
    	return bestAction;
    }
       
    private double evaluateShipPosition(Game a_gameCopy) {
		
    	Vector2d nextPosition = a_gameCopy.getShip().s;
		Vector2d potentialDirection = a_gameCopy.getShip().d;
		Vector2d aimedNodeV = new Vector2d(aimedNode.x(),aimedNode.y());
        
		aimedNodeV.subtract(nextPosition);
		aimedNodeV.normalise();   //This is a unit vector from my position pointing towards the next node to go to.
		double dot = potentialDirection.dot(aimedNodeV);  //Dot product between this vector and where the ship is facing to.
		//Get the distance to the next node in the tree and update the total distance until the closest waypoint/ft
		double dist = aimedNode.euclideanDistanceTo(nextPosition.x, nextPosition.y);
		
//		double actionScore = -(pathToFollow.m_cost+dist);
		
		//hack
		double actionScore = dist;
		
		System.out.println(actionScore);
		return actionScore;
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
        int waitTime = (a_near-30) + m_rnd.nextInt(50);
        long startingTime = System.currentTimeMillis();
        long finalDateMs = startingTime + waitTime;

        while(startingTime < finalDateMs)
            startingTime = System.currentTimeMillis();

        System.out.println("Waited " + waitTime + " ms");
    }
	
	private void wait(int waitTime)
    {
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
     * paint additional info, called automatically every frame
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
    	a_gr.setColor(Color.red);
    	@SuppressWarnings("unchecked")
		ArrayList<Vector2d> positionsToDraw = (ArrayList<Vector2d>) possiblePosition.clone();
    	for(Vector2d position : positionsToDraw) 
    	{
			a_gr.fillRect((int)position.x, (int)position.y, 1, 1);
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
    
    private Node aimedNode = null;
    
    private Path pathToFollow;
    
    //how many times to repeat each action
	//repeat a turn 30 times = 90 degrees
	private int macroCount = 3;
	private int maxDepth = 1;
    
    private ArrayList<Vector2d> possiblePosition = new ArrayList<>();
    
    public Random m_rnd = new Random();
}

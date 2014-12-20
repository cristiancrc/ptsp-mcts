package controllers.mctsnavi;

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

import com.sun.org.apache.xalan.internal.xsltc.compiler.util.ErrorMsg;

import planners.Planner;
import planners.Planner3Opt;
import planners.PlannerGreedyEvolved;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;

import controllers.mctsnavi.SearchTreeNode;

/**
 * monte carlo simulation driver
 * TODO:
 * !!!this is a mess
 *  - clean up code
 *  - check implementation, this should be a naive mcts
 *  - takes pauses after a  while, something is not cleaned up
 * @version 141128
 * @author Cristian
 *
 */

public class DriveMCTS extends Controller
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
    private GameObject m_closestPickUp; //Closest object (waypoint or fuel tank) to the ship.

    /**
     * Distance to fuel tank penalization: as collecting a fuel tank does not reset the timer, it might be dangerous going after a fuel tank
     * instead of a waypoint. Hence, we apply this value to the distance in order to go only after those that are really closer than a waypoint.
     */
    private float FUEL_TANK_PEN = 1.9f;

    /**
     * Closest waypoint to the ship.
     */
	private Waypoint m_closestWaypoint;

	/**
	 * Next waypoint in the list resulted by planner
	 */
	private Waypoint m_nextWaypoint;

    /**
     * Constructor, that receives a copy of the game state
     * @param a_gameCopy a copy of the game state
     * @param a_timeDue The time the initialization is due. Finishing this method after a_timeDue will disqualify this controller.
     */
    public DriveMCTS(Game a_gameCopy, long a_timeDue)
    {
    	if (true) throw new NotImplementedException();
    	System.out.print("**** mc driver ****");
        m_graph = new Graph(a_gameCopy);//Init the graph.

      //completed planners
        Planner planner = new Planner3Opt(a_gameCopy);//remove three edges and reconnect the graph        
//        Planner planner = new Planner2Opt(a_gameCopy);//remove two edges and reconnect the graph    
//        Planner planner = new PlannerMC(a_gameCopy);//search through random paths to find a small one // 1512 ... 1680
//        Planner planner = new PlannerBruteForce(a_gameCopy);//brute force search planner  //1512
//        Planner planner = new PlannerGreedy(a_gameCopy);//plan a distance based route through the waypoints //1614

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
        if(--macroMovesLeft > 0)
     	{
     		return macroAction;
     		//TODO: do some more computing!!!
     	}
        else ticks++;
    	
    	boolean verbose = false;
    	//verbose = ((ticks % 10 == 0) ? true : false);
    	
        long timeIn = System.currentTimeMillis();
        if(verbose) System.out.println("\n>>>in\t\t" + timeIn);
        
        if(verbose) System.out.println("---due in\t" + a_timeDue);
              
        long remainingTime;
        remainingTime = a_timeDue - System.currentTimeMillis();
                
        SearchTreeNode rootNode = new SearchTreeNode();
		rootNode.setAction(-1);
		rootNode.setScore(0);
		rootNode.setVisited(0);
        
        possiblePosition.clear();//display just one level of search
    	
    	//target reached?
        if(m_nextWaypoint.checkCollected(a_gameCopy.getShip().ps, m_nextWaypoint.radius/4*3))
    	{
    		System.out.println("!!!!!marked as completed " + m_nextWaypoint);
    		m_nextWaypoint.setCollected(true);
    		m_orderedWaypoints.get(m_orderedWaypoints.indexOf(m_nextWaypoint)).setCollected(true);
    	}
    	
        //find the next waypoint and the path to it from the previous waypoint
    	Integer nextWaypoint = null;
    	if (nextWaypoint == null) nextWaypoint = 1;//skip ship waypoint
        for (int i = nextWaypoint; i < m_orderedWaypoints.size(); i++)
        {
        	if(!m_orderedWaypoints.get(i).isCollected())
        	{
        		nextWaypoint = i;
        		m_nextWaypoint = m_orderedWaypoints.get(i);
        		break;
        	}
        }
        pathToFollow = m_plannedPath.get(nextWaypoint-1);      
    	
        //Get the next node to go to, from the path to the closest waypoint/ft
    	aimedNode = getLastNodeVisible(pathToFollow, a_gameCopy);  

    	//TODO: pause stuff
//        if(ticks % 100 != 0) return 0;
        
        int playouts = 0;
        while(remainingTime > 5)
//        while(playouts < 1)
        {
        	playouts++;
        	System.out.print("\n" +ticks + ":" + playouts + ">");
        	getActionPlayouts(a_gameCopy, a_timeDue, rootNode, -1, -1);
        	remainingTime = a_timeDue - System.currentTimeMillis();
//        	System.out.println("\n" + playouts + " --- remaining:" + remainingTime );
        }
        if(verbose) System.out.println("\n" + ticks + " tick ran: " + playouts + " playouts");

//        rootNode.present(0);
        
        int bestAction = getUrgentNodeAction(rootNode, true);
        
        macroMovesLeft = macroCount;
        macroAction = bestAction;
        
        if(verbose) System.out.println("\n>>>out\t\t" + System.currentTimeMillis());
        
    	return bestAction;
    }
    
	/**
	 * searches the root node children for the best value and returns the respective action
	 * @param rootNode
	 * @param min boolean denoting if looking to minimize score
	 * @return
	 */
	private int getUrgentNodeAction(SearchTreeNode rootNode, boolean min) {
		double bestValue;
		int bestAction = -1;
		
		//aiming to get minimum score
		if(min)
		{
			bestValue = Double.POSITIVE_INFINITY;
			for(SearchTreeNode node : rootNode.getChildren())
			{
				node.present(10);
				if(node.getValue() < bestValue)
				{
					bestValue = node.getValue();
					bestAction = node.getAction();
				}
			}
		}
		//aiming to get maximum score
		else
		{
			bestValue = Double.NEGATIVE_INFINITY;
			for(SearchTreeNode node : rootNode.getChildren())
			{
				node.present(10);
				if(node.getValue() > bestValue)
				{
					bestValue = node.getValue();
					bestAction = node.getAction();
				}
			}
		}
		System.out.println("\nselected " + bestAction + " ("+ bestValue +") ");
		return bestAction;
	}

	public void getActionPlayouts(Game a_gameCopy, long a_timeDue, SearchTreeNode parentNode, int actionTaken, int depth)
    {
//    	System.out.println("entering with depth : " + depth); 
        depth++;
        double actionScore = 0;
        
    	//target reached?
        if(m_nextWaypoint.checkCollected(a_gameCopy.getShip().ps, m_nextWaypoint.radius/4*3))
    	{
    		System.out.println("simulation marked as completed [" + depth + "] " + m_nextWaypoint + "___" + aimedNode);
    		depth = maxDepth; //force parsing the out code and go out
    		actionScore -= 100;

//    		m_nextWaypoint.setCollected(true);
//    		m_orderedWaypoints.get(m_orderedWaypoints.indexOf(m_nextWaypoint)).setCollected(true);
    	}
        //
        
        if(depth==maxDepth)
    	{
        	System.out.print("^");
			actionScore += evaluateShipPosition(a_gameCopy);
			
//			System.out.print("in v:" + parentNode.getVisited() + " s:" + parentNode.getScore());
			SearchTreeNode leafNode = new SearchTreeNode();
			leafNode.setParent(parentNode);
			leafNode.setScore(actionScore);
			leafNode.setAction(actionTaken);
		
//			parentNode.trySetScore(actionScore);
        	
//			System.out.println("\n this node action: " + actionTaken);
//        	System.out.println(" parent node info : " + parentNode.hasChild(leafNode));
//        	System.out.println(" children count : " + parentNode.getChildren().size());
//        	System.out.println(" parent node id : " + parentNode.hashCode());
        	
        	if(!parentNode.hasChild(leafNode)) 
        	{
        		parentNode.addChild(leafNode);
            	System.out.print(" [+] ");
//            	System.out.println(" children count : " + parentNode.getChildren().size());
        	}
        	else
        	{
//        		parentNode.getChild(actionTaken).addScore(actionScore);
        		System.out.print(" [-] ");
        		
        		parentNode.getChild(actionTaken).trySetScore(actionScore);
        	}
//        	System.out.print(":: out v:" + parentNode.getVisited() + " s:" + parentNode.getScore());
//        	System.out.println("");
    	}
        if(5 >= a_timeDue - System.currentTimeMillis())
        {
        	System.out.println("T");
        }
        if(depth < maxDepth && 5 < (a_timeDue - System.currentTimeMillis()))
        {
        	
        	int action = m_rnd.nextInt(Controller.NUM_ACTIONS);//-1)+1;//avoid action 0
            Game forThisAction = a_gameCopy.getCopy();
            for(int i = 0; i < macroCount; i++)
            {
            	forThisAction.getShip().update(action);	
            }
            Vector2d nextPosition = forThisAction.getShip().s;
            Vector2d potentialDirection = forThisAction.getShip().d;
            if(!possiblePosition.contains(nextPosition))
            {
            	System.out.print(".");
            	possiblePosition.add(nextPosition);	
            } else
            {
            	System.out.print(",");
            }
            getActionPlayouts(forThisAction, a_timeDue, parentNode, action, depth);
        }
        
//        //Default (something went wrong).
//        return Controller.ACTION_NO_FRONT;
//        System.out.println("<<<out 2 \t" + System.currentTimeMillis());
//        return Raction;
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
     * Returns the first node in the way to the destination
     * @return the node in the way to destination.
     */
    private Node getNextNode()
    {
        Node n0 = m_graph.getNode(m_pathToClosest.m_points.get(0));

        //If only one node in the path, return it.
        if(m_pathToClosest.m_points.size() == 1)
            return n0;

        //Heuristic: Otherwise, take the closest one to the destination
        Node n1 = m_graph.getNode(m_pathToClosest.m_points.get(1));
        Node destination =  m_graph.getNode(m_pathToClosest.m_destinationID);

        if(n0.euclideanDistanceTo(destination) < n1.euclideanDistanceTo(destination))
            return n0;
        else return n1;
    }

    /**
     * Manages straight traveling.
     * @param a_gameCopy the game copy
     * @return the id of the best action to execute.
     */
    private int manageStraightTravel(Game a_gameCopy)
    {
        int bestAction = Controller.ACTION_NO_FRONT;
        Vector2d dirToWaypoint = m_closestPickUp.s.copy();
        dirToWaypoint.subtract(a_gameCopy.getShip().s);
        double distance = dirToWaypoint.mag();
        dirToWaypoint.normalise();

        //Check if we are facing the waypoint we are going after.
        Vector2d dir = a_gameCopy.getShip().d;
        boolean notFacingWaypoint = dir.dot(dirToWaypoint) < 0.85;

        //Depending on the time left and the distance to the waypoint/ft, we established the maximum speed.
        //(going full speed could make the ship to overshoot the waypoint/ft... that's the reason of this method!).
        double maxSpeed = 0.4;
        if(distance>100 || a_gameCopy.getStepsLeft() < 50)
            maxSpeed = 0.8;
        else if(distance<30) maxSpeed = 0.25;


        if(notFacingWaypoint || a_gameCopy.getShip().v.mag() > maxSpeed)
        {
            //We should not risk to throttle. Let's rotate in place to face the waypoint/ft better.
            Game forThisAction;
            double bestDot = -2;
            for(int i = Controller.ACTION_NO_FRONT; i <= Controller.ACTION_NO_RIGHT; ++i)
            {
                //Select the action that maximises my dot product with the target (aka. makes the ship face the target better).
                forThisAction = a_gameCopy.getCopy();
                forThisAction.getShip().update(i);
                Vector2d potentialDirection = forThisAction.getShip().d;
                double newDot = potentialDirection.dot(dirToWaypoint);
                if(newDot > bestDot)
                {
                    bestDot = newDot;
                    bestAction = i;
                }
            }
        } else //We can thrust
            return Controller.ACTION_THR_FRONT;

        //There we go!
        return bestAction;
    }

    /**
     * Calculates the closest waypoint or fuel tank to the ship.
     * @param a_gameCopy the game copy.
     */
    private void calculateClosestPickUp(Game a_gameCopy)
    {
        double minDistance = Double.MAX_VALUE;
        calculateClosestWaypoint(a_gameCopy);

        //Also check for fuel tanks:
        for(FuelTank ft: a_gameCopy.getFuelTanks())
        {
            if(!ft.isCollected())     //Only consider those not collected yet.
            {
                double fx = ft.s.x-a_gameCopy.getShip().s.x, fy = ft.s.y-a_gameCopy.getShip().s.y;
                double dist = Math.sqrt(fx*fx+fy*fy);
                dist = dist * FUEL_TANK_PEN; //Apply penalization to only chase those fuel tanks really close.
                if( dist < minDistance )
                {
                    //Keep the minimum distance.
                    minDistance = dist;
                    m_closestPickUp = ft;
                }
            }
        }
    }
 
    /**
     * find closest waypoint (disregard other objects)
     * @param a_gameCopy
     */
    public void calculateClosestWaypoint(Game a_gameCopy)
    {
    	//get closest waypoint to ship position
        double minDistance = Double.MAX_VALUE;
        for(Waypoint way: a_gameCopy.getWaypoints())
        {
            if(!way.isCollected())     //Only consider those not collected yet.
            {
                double fx = way.s.x-a_gameCopy.getShip().s.x, fy = way.s.y-a_gameCopy.getShip().s.y;
                //TODO: consider avoiding sqrt
                double dist = Math.sqrt(fx*fx+fy*fy);
                if( dist < minDistance )
                {
                    //Keep the minimum distance.
                    minDistance = dist;
                    m_closestWaypoint = way;
                }
            }
        }
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
     * nearest neighbor solver, distance, lava and directness used
     * @param a_gameCopy
     */
    //TODO: add heading difference
	public void plannerGreedyHeading(Game a_gameCopy)
    {
    	System.out.println("***greedy evolved planner***");
    	boolean verbose = false;
    	m_orderedWaypoints.clear();
    	//add all waypoints that must be touched
    	LinkedList<Waypoint> waypointList = a_gameCopy.getWaypoints();    
        
    	//add ship position as waypoint
        Waypoint wpFrom = new Waypoint(a_gameCopy, a_gameCopy.getShip().s);        
        m_collectNodes.put(wpFrom, m_graph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y,true));
    	m_orderedWaypoints.add(wpFrom);
		
		//heading
    	Vector2d shipHeading = a_gameCopy.getShip().d;
    	shipHeading.normalise();
    	Vector2d entryHeading = shipHeading;
    	Vector2d lastHeadingForMinimum = null;
    	
    	//distance
    	double distanceTravelled = 0;
		
		//iterate through the other waypoints to find the closest one to each next one
		int check = 0;
		while(waypointList.size() > 0 )
		{
			if(verbose) System.out.println("\n" + check + ": wpFrom: " + wpFrom.toString()  + ", wpList size " + waypointList.size());// + ", ordered " + orderedWaypoints.size());
			if(check++ > a_gameCopy.getNumWaypoints()+1) break;//one more waypoint for the initial ship position
			Waypoint closestWaypoint = null;
	    	
			Vector2d pathHeadingInit =  new Vector2d();
			Vector2d pathHeading = new Vector2d();
			
			double minPathCost = Double.MAX_VALUE;
	    	
			for(Waypoint wpEnd : waypointList)
			{
				if(wpEnd.equals(wpFrom)) continue;
				if(verbose) System.out.println("\n...checking " + wpEnd.toString());
				double pathCost = 0;
				double headingDiffCost = 0;
				double distanceCost = 0;
				double degreesToTurn = 0;
				
				pathHeadingInit = entryHeading.copy();
				
	    		Path a_path = m_graph.getPath(m_collectNodes.get(wpFrom).id(), m_collectNodes.get(wpEnd).id());
	    		if (null != a_path)
	        	{
		        	for(int k = 0; k < a_path.m_points.size()-1; k++)
		            {
		                Node thisNode = m_graph.getNode(a_path.m_points.get(k));
		                Node nextNode = m_graph.getNode(a_path.m_points.get(k+1));
		                
		                distanceCost += thisNode.euclideanDistanceTo(nextNode);
//		                if(verbose) System.out.println("\t" + k + " this node: " + thisNode.x() + " , " + thisNode.y());
//		                if(verbose) System.out.println("\t" + k + " next node: " + nextNode.x() + " , " + nextNode.y());
						pathHeading.x = nextNode.x() - thisNode.x();
		                pathHeading.y = nextNode.y() - thisNode.y();
		                pathHeading.normalise();
//		                if(verbose) System.out.println("\t" + k + " resulting vector: " + pathHeading.x + " , " + pathHeading.y);
		                
		                Game tempGame = a_gameCopy.getCopy();		                
		                Ship tempShip = tempGame.getShip();
		                tempShip.s.x = nextNode.x();
		                tempShip.s.y = nextNode.y();
		                tempGame.setShip(tempShip);
		                //TODO: use actual heading resulted from planned path
//		                System.out.println("ship heading: " + tempGame.getShip().d);
		                tempGame.tick(ACTION_NO_LEFT);//this tick is needed to register if the ship is above lava, using ACTION_NO_FRONT will not work for tempShip.isOnLava()
		                if(tempShip.isOnLava()) {
//		                	if(verbose) System.out.println("++++++++++++at " + nextNode.x() + " , " + nextNode.y() + " on lava");
		                	//if ship is on lava add more distance cost (one distance is already added!)
		                	distanceCost += 0.5*thisNode.euclideanDistanceTo(nextNode);
		                }
		                
		                //degreesToTurn = Math.toDegrees(Math.acos(pathHeadingInit.dot(pathHeading)));
		                degreesToTurn = (Math.acos(pathHeadingInit.dot(pathHeading)));
		                
//		                if(verbose) System.out.println("\t" + k + " heading (deg):" + degreesToTurn);
		                headingDiffCost += degreesToTurn;
//		                pathHeadingInit.x = pathHeading.x;
//		                pathHeadingInit.y = pathHeading.y;
		                pathHeadingInit = pathHeading.copy();//set new init heading as the last heading used for previous node    
		            }

		        	//TODO: use constants as multipliers for heading, lava, distance
		        	pathCost = distanceCost + 5*headingDiffCost;
		        	if(verbose) System.out.print(" this path distance cost " + distanceCost + " , heading cost " + headingDiffCost + ", total cost " + pathCost + ", minPath " + minPathCost );
		           
		        	if(pathCost < minPathCost) {
		        		minPathCost = pathCost;
		        		closestWaypoint = wpEnd.getCopy(a_gameCopy);
		        		lastHeadingForMinimum = pathHeading.copy();
		        		if(verbose) System.out.print( " ... new minimum");
		        	}
	        	}
			}			
			if(verbose) System.out.println("\nclosest is " + closestWaypoint.toString());			
			m_orderedWaypoints.add(closestWaypoint);
			
			//store total distance
			distanceTravelled += minPathCost;
			if(verbose) System.out.println("\ndistance travelled so far: " + distanceTravelled);
			
			//store last heading for minimum
			entryHeading = lastHeadingForMinimum.copy();
			
			//next search starting from this node
			wpFrom = closestWaypoint.getCopy(a_gameCopy);
			
			//remove node from list
			waypointList.remove(closestWaypoint);
		}
    }
    
    /**
     * calculate paths from ship to each remaining object
     * @param a_gameCopy
     */
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
     * store paths from one point to another, in the order resulted by a planner
     * @param a_gameCopy
     */
    private void calculateOrderedWaypointsPaths(Game a_gameCopy) 
    {
    	Node m_fromNode = null;
    	Node m_toNode = null;
    	Path pathToNext = null;

        //calculate paths from one waypoint to the next
        for (int i = 0; i < m_orderedWaypoints.size()-1; i++)
        {
        	if(!m_orderedWaypoints.get(i).isCollected())//TODO: newly added
        	{
		        	//get node for the current waypoint
		        	m_fromNode = m_graph.getClosestNodeTo(m_orderedWaypoints.get(i).s.x, m_orderedWaypoints.get(i).s.y, false);
		        	
		        	//get node for the next waypoint
		        	m_toNode = m_graph.getClosestNodeTo(m_orderedWaypoints.get(i+1).s.x, m_orderedWaypoints.get(i+1).s.y, false);
		        	
		        	//And get the path from one to the other
		        	pathToNext = m_graph.getPath(m_fromNode.id(), m_toNode.id());
		        	
		        	//store the path
		        	m_plannedPath.add(pathToNext);
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
    private int macroAction = ACTION_NO_FRONT;
    private Node aimedNode = null;
    
    private Path pathToFollow;
    
    //how many times to repeat each action
	//repeat a turn 30 times = 90 degrees
	private int macroCount = 3;
	private int maxDepth = 1;
    
    private ArrayList<Vector2d> possiblePosition = new ArrayList<>();
    
    public Random m_rnd = new Random();
}

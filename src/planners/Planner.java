package planners;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import com.sun.xml.internal.ws.util.StringUtils;

import controllers.mcts.DriveMCTS;
import framework.core.Controller;
import framework.core.FuelTank;
import framework.core.Game;
import framework.core.GameObject;
import framework.core.Map;
import framework.core.Ship;
import framework.core.Waypoint;
import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;
import framework.utils.Navigator;
import framework.utils.Vector2d;

/**
 *  abstract planner class
 *  @author Cristian
 *  @version 141128
 */	
public abstract class Planner {
	
	boolean verbose = false;
	static HashMap<GameObject, HashMap<GameObject, Double>> distanceMatrix = new HashMap<>();//distance matrix from each waypoint to each other
	static HashMap<GameObject, HashMap<GameObject, Double>> matrixCostLava = new HashMap<>();//cost (distance and lava) matrix from each waypoint to each other
	static HashMap<GameObject, HashMap<GameObject, Double>> matrixCostDirectness = new HashMap<>();//directness matrix from each waypoint to each other
	static HashMap<GameObject, HashMap<GameObject, HashMap<GameObject, Double>>> matrixCostAngle = new HashMap<>();//directness matrix from each waypoint, through each other, to each other
	LinkedList<GameObject> orderedWaypoints = new LinkedList<>();//the route resulted from the planner
	ArrayList<Path> plannedPath = new ArrayList<>();//path between waypoints ready to be displayed on-screen
	Graph aGraph;
	Game aGameCopy;
	//TODO 7 check variable visibility and static properties
	//TODO 7 check variable names
	public static double weightLava = 1;
	public static double weightDistance = 1;
	public static double weightDirectness = 1;
	public static double weightAngle = 1;
	public static double weightFuelTankCost = 1;
	public static double weightConsecutiveFuelTanksCost = 2;
	//TODO 8 if a waypoint is taken out of order, what do we do? should we replan? remove it from the path? with 1 0 0 1 this happens
	//fuel true
	//bonus fuel 200
	//penalty for consecutive tanks 1000
	
	/**
	 * perform the planning
	 * @param aGameCopy
	 */
	public abstract void runPlanner();
		
    /**
     * store paths from one point to another, in the order resulted by a planner
     * @param a_gameCopy
     */
    public void calculateOrderedWaypointsPaths() 
    {
        //calculate paths from one waypoint to the next
        for (int i = 0; i < orderedWaypoints.size()-1; i++)
        {
        	Node nodeFrom = aGraph.getClosestNodeTo(orderedWaypoints.get(i).s.x, orderedWaypoints.get(i).s.y, false);//get node for the current waypoint
        	Node nodeTo = aGraph.getClosestNodeTo(orderedWaypoints.get(i+1).s.x, orderedWaypoints.get(i+1).s.y, false);//get node for the next waypoint
        	Path pathToNext = aGraph.getPath(nodeFrom.id(), nodeTo.id());//And get the path from one to the other
        	plannedPath.add(pathToNext);//store the path
        }
        
    }
    
    /**
     * create the 3d matrix containing angle changes between waypoints, counting the last and first steps of the path
     * @param waypointList
     * @return 3d hash map, double
     */    
    protected HashMap<GameObject , HashMap<GameObject, HashMap<GameObject , Double>>> createAngleMatrix(LinkedList<GameObject> waypointList)
    {
		long timeIn = System.currentTimeMillis();
    	HashMap<GameObject, HashMap<GameObject, HashMap<GameObject, Double>>> matrixCostAngle = new HashMap<>();//directness matrix from each waypoint, through each other, to each other
    	HashMap<GameObject, Double> wpTo_Value = new HashMap<GameObject, Double>();
    	HashMap<GameObject, HashMap<GameObject, Double>> wpThrough_wpTo = new HashMap<GameObject, HashMap<GameObject, Double>>();
		for(int i = 0; i < waypointList.size(); i++)
   		{
			GameObject wpFrom = waypointList.get(i);
			Node nodeFrom = aGraph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
			wpThrough_wpTo.clear();
			for (int j = 1; j < waypointList.size(); j++)
			{
				if(j == i) continue;
				GameObject wpThrough = waypointList.get(j);
				Node nodeThrough = aGraph.getClosestNodeTo(wpThrough.s.x, wpThrough.s.y, false);				
				Path pathIncoming = aGraph.getPath(nodeFrom.id(), nodeThrough.id());
				Node nodeIncoming =  aGraph.getNode(pathIncoming.m_points.get(0));
				for(int n = 0; n < pathIncoming.m_points.size()-1; n++)//<size-1 because last node in path is the same as wpThrough
				{                           
					Node itNode = aGraph.getNode(pathIncoming.m_points.get(n));
					Vector2d itNodePos = new Vector2d(itNode.x(),itNode.y());
				      
				    boolean isThereLineOfSight = aGameCopy.getMap().LineOfSight(wpThrough.s, itNodePos);
				    if(isThereLineOfSight)
				    {
				        nodeIncoming = itNode;
				        break;
				    }
				}
				Vector2d entryVector = new Vector2d(nodeThrough.x() - nodeIncoming.x(), nodeThrough.y() - nodeIncoming.y() );
				entryVector.normalise();				

				wpTo_Value.clear();
				for(int k =1; k < waypointList.size(); k++)
				{					
					if(k == j) continue;
					if(k == i) continue;
					GameObject wpTo = waypointList.get(k);				
					double dotVector = 0;
//					System.out.println(wpFrom.getName() + " > " + wpThrough.getName() + " > " + wpTo.getName());
//					System.out.print("i:" + i + ",j:" + j + ",k:" + k);
					if(k < i)
					{
						//retrieve
						dotVector = matrixCostAngle.get(wpTo).get(wpThrough).get(wpFrom);
					}
					else
					{
						//calculate
						Node nodeTo = aGraph.getClosestNodeTo(wpTo.s.x, wpTo.s.y, false);						
                        Path pathOutgoing = aGraph.getPath(nodeThrough.id(), nodeTo.id());
                        Node nodeOutgoing =  aGraph.getNode(pathOutgoing.m_points.get(0));
        				for(int n = pathOutgoing.m_points.size()-1; n > 0 ; n--)//>0 because first node is wpTrough
        				{                           
        					Node itNode = aGraph.getNode(pathOutgoing.m_points.get(n));
        					Vector2d itNodePos = new Vector2d(itNode.x(),itNode.y());
        				      
        				    boolean isThereLineOfSight = aGameCopy.getMap().LineOfSight(wpThrough.s, itNodePos);
        				    if(isThereLineOfSight)
        				    {
        				        nodeOutgoing = itNode;
        				        break;
        				    }
        				}

						Vector2d exitVector = new Vector2d(nodeThrough.x() - nodeOutgoing.x(), nodeThrough.y() - nodeOutgoing.y());
						exitVector.normalise();
						
						dotVector = entryVector.dot(exitVector);
//						System.out.printf("\nw[%d][%s] -> w[%d][%s] -> w[%d][%s] : entry(%f,%f) . exit(%f,%f) = dot(%f)\n", i, wpFrom.getName(), j, wpThrough.getName(), k, wpTo.getName(), entryVector.x, entryVector.y, exitVector.x, exitVector.y, dotVector);																	
					}
					wpTo_Value.put(wpTo, dotVector);
				}
				wpThrough_wpTo.put(wpThrough, (HashMap<GameObject, Double>) wpTo_Value.clone());
			}
			matrixCostAngle.put(wpFrom, (HashMap<GameObject, HashMap<GameObject, Double>>) wpThrough_wpTo.clone());
   		}
		long timeOut = System.currentTimeMillis();
		System.out.println(" Time spent inside create angle matrix: " + (timeOut - timeIn) + " ms.");
//		present3dMatrix(matrixCostAngle);
		return matrixCostAngle;		
		
	}
    
	/**
	 * creates a matrix of directness between waypoints
	 * directness is a measure of how much a path deviates from a straight line
	 * so is the ratio of the path distance (not cost!) and the Euclidean distance
	 * needs distanceMatrix
	 * @param waypointList
	 * @return directness matrix
	 */
	protected HashMap<GameObject, HashMap<GameObject, Double>> createDirectnessMatrix(LinkedList<GameObject> waypointList) {
		long timeIn = System.currentTimeMillis();
		double distPath = 0;
		double distEuclidean = 0;
		HashMap<GameObject, HashMap<GameObject, Double>> matrixCostDirectness = new HashMap<>();//directness matrix from each waypoint to each other
		HashMap<GameObject, Double> wpEndValue = new HashMap<>();//distance from -one- waypoint to each of the others
		for(int i = 0; i < waypointList.size(); i++)
   		{
			GameObject wpFrom = waypointList.get(i);
			Node nodeFrom = aGraph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
			wpEndValue.clear();		
			for (int j = 0; j < waypointList.size(); j++)
			{
				double ratio = 0;					
				GameObject wpTo = waypointList.get(j);				
				Node nodeTo = aGraph.getClosestNodeTo(wpTo.s.x, wpTo.s.y, false);						
				
				if ( j == i){
					continue;
				}
				if ( j < i )
				{//retrieve
					ratio = matrixCostDirectness.get(wpTo).get(wpFrom);
				} else
				{//calculate
					distPath = distanceMatrix.get(wpFrom).get(wpTo);
					distEuclidean = nodeFrom.euclideanDistanceTo(nodeTo);					
					ratio = distPath/distEuclidean;									
				}
				wpEndValue.put(wpTo, ratio);
				matrixCostDirectness.put(wpFrom, (HashMap<GameObject, Double>) wpEndValue.clone());
			}			
   		}
		long timeOut = System.currentTimeMillis();
		System.out.println(" Time spent inside create directness matrix: " + (timeOut - timeIn) + " ms.");
		return matrixCostDirectness;
	}
    
    /**
     * creates a distance matrix for a list of waypoints
	 * @param a list of waypoints
	 * @return a distance matrix
	 */
	protected HashMap<GameObject, HashMap<GameObject, Double>> createDistanceMatrix(LinkedList<GameObject> waypointList) {
		long timeIn = System.currentTimeMillis();
		//compute distance matrix for waypoints
		HashMap<GameObject, HashMap<GameObject, Double>> distanceMatrix = new HashMap<>();//distance from -each- waypoint to each of the others
       	HashMap<GameObject, Double> distanceList = new HashMap<>();//distance from -one- waypoint to each of the others
       		
    	for (int i = 0; i < waypointList.size(); i++)    		
    	{
    		GameObject wpFrom = waypointList.get(i);
    		distanceList.clear();
    		
    		for (int j = 0; j < waypointList.size(); j++) 
			//j = i for symmetric weights , j = 0 for asymmetric (uphill downhill)
    		//we need full matrix for easy lookup, so j = 0, but we will avoid computing cost by retrieving previous result
    		{
    			GameObject wpTo = waypointList.get(j);    			
    			if (i == j) continue; //distance to self is 0
    			double pathCost = 0;
    			//avoid computing the path cost for symmetric distances, and retrieve previous result
    			if (j < i )
    			{
					pathCost = distanceMatrix.get(wpTo).get(wpFrom);	
//    				System.out.println(" cost retrieved:" + pathCost);
    			} 
    			else
    			{
    				Node nodeFrom = aGraph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
    				Node nodeTo = aGraph.getClosestNodeTo(wpTo.s.x, wpTo.s.y, false);					
    				Path aPath = aGraph.getPath(nodeFrom.id(), nodeTo.id());
    	    		pathCost = aPath.m_cost;
//    	    		System.out.println(" calculated new path cost:" + pathCost);
    			}
	        	distanceList.put(wpTo, pathCost);	        	
    		}
    		distanceMatrix.put(wpFrom, (HashMap<GameObject, Double>) distanceList.clone());
    	}
    	long timeOut = System.currentTimeMillis();
		System.out.println(" Time spent inside create distance matrix: " + (timeOut - timeIn) + " ms.");
    	return distanceMatrix;
	}
	
	/**
	 * creates the distance matrix and the distance matrix with lava cost, both at the same time
	 * is usually faster than creating both of them separately
	 * @param waypointList
	 * @return array with two distance matrices
	 */
	protected HashMap<GameObject, HashMap<GameObject, Double>>[] createDistanceMatrices(LinkedList<GameObject> waypointList) {		
		long timeIn = System.currentTimeMillis();
		HashMap<GameObject, HashMap<GameObject, Double>>[] result = new HashMap[2];
		
		//compute distance matrix for waypoints
		HashMap<GameObject, HashMap<GameObject, Double>> distanceMatrix = new HashMap<>();//distance from -each- waypoint to each of the others
		HashMap<GameObject, HashMap<GameObject, Double>> distanceMatrixLava = new HashMap<>();//distance from -each- waypoint to each of the others
       	HashMap<GameObject, Double> distanceList = new HashMap<>();//distance from -one- waypoint to each of the others
       	HashMap<GameObject, Double> distanceListLava = new HashMap<>();//distance from -one- waypoint to each of the others
       	Map a_map = aGameCopy.getMap();       
       	
    	for (int i = 0; i < waypointList.size(); i++)    		
    	{
    		GameObject wpFrom = waypointList.get(i);
    		distanceList.clear();
    		distanceListLava.clear();
    		
    		for (int j = 1; j < waypointList.size(); j++) 
			//j = i for symmetric weights , j = 0 for asymmetric (uphill downhill)
    		//we need full matrix for easy lookup, so j = 0, but we will avoid computing cost by retrieving previous result
    		{    	       	
    			GameObject wpTo = waypointList.get(j);    			
    			if (i == j) continue; //distance to self is 0
    			double pathCost = 0;
    			double lavaCost = 0;
    			//avoid computing the path cost for symmetric distances, and retrieve previous result
    			if (j < i )
    			{
					pathCost = distanceMatrix.get(wpTo).get(wpFrom);
					lavaCost = distanceMatrixLava.get(wpTo).get(wpFrom);
//    				System.out.println(" cost retrieved:" + pathCost);
    			} 
    			else
    			{    				
    				Node nodeFrom = aGraph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
    				Node nodeTo = aGraph.getClosestNodeTo(wpTo.s.x, wpTo.s.y, false);					
    				Path aPath = aGraph.getPath(nodeFrom.id(), nodeTo.id());
    	    		pathCost = aPath.m_cost;
    	    		double distanceCost = 0;
    	    		for(int k = 0; k < aPath.m_points.size()-1; k++)
		            {    	    	    	    			
		                Node thisNode = aGraph.getNode(aPath.m_points.get(k));		                
		                Node nextNode = aGraph.getNode(aPath.m_points.get(k+1));
		                double dist = thisNode.euclideanDistanceTo(nextNode);
		                if(  i == 0 ||
		                	 (wpFrom instanceof FuelTank && wpTo instanceof FuelTank) )
		                {
		                	dist *= weightConsecutiveFuelTanksCost;
		                }
		                if(a_map.isLava(nextNode.x(), nextNode.y()))
		                {
		                	dist *= weightLava;
		                }
	                	distanceCost += dist;
		            }    	    		
    	    		lavaCost = distanceCost;    				
    			}    			
	        	distanceList.put(wpTo, pathCost);
	        	distanceListLava.put(wpTo, lavaCost);
    		}
    		distanceMatrix.put(wpFrom, (HashMap<GameObject, Double>) distanceList.clone());
    		distanceMatrixLava.put(wpFrom, (HashMap<GameObject, Double>) distanceListLava.clone());
    	}
    	result[0] = distanceMatrix;
    	result[1] = distanceMatrixLava;
		long timeOut = System.currentTimeMillis();
		System.out.println(" Time spent inside create distance matrices: " + (timeOut - timeIn) + " ms.");    	
    	return result;
	}
	
	/**
	 * creates the distance and cost matrices
	 */
	protected void createMatrices()
	{
		LinkedList<GameObject> waypointList = (LinkedList<GameObject>) aGameCopy.getWaypoints().clone();//start with all the waypoints
		waypointList.addAll((Collection<? extends GameObject>) aGameCopy.getFuelTanks().clone());
		Waypoint wpShip = new Waypoint(aGameCopy, aGameCopy.getShip().s);
    	waypointList.addFirst(wpShip);//add the ship to start from
    	HashMap<GameObject, HashMap<GameObject, Double>>[] distanceMatrices = createDistanceMatrices(waypointList);
    	distanceMatrix = distanceMatrices[0];
    	matrixCostLava = distanceMatrices[1];  
    	matrixCostDirectness = createDirectnessMatrix(waypointList);
    	matrixCostAngle = createAngleMatrix(waypointList);
	}
	
	/**
	 * flood fill the map and create the distance matrices
	 * TODO 1 flood fill map
	 * @param aMap
	 */
	public static void floodFill(Map aMap)
	{
		char[][] charMap = aMap.getMapChar();
		System.out.println("size x " + charMap.length);
		System.out.println("size y " + charMap[0].length);
		for(int i = 0; i < charMap.length; i++)
		{
			System.out.println("");
			for(int j =0; j < charMap[i].length; j++)
			{
				System.out.print(charMap[i][j]);				
			}
		}
	}

	/**
	 * displays a matrix in a humanly readable form
	 * @param matrix
	 */
	protected void present2dMatrix(HashMap<GameObject, HashMap<GameObject, Double>> matrix)
	{
		for (GameObject wpFrom : matrix.keySet())    		
    	{
    		for (GameObject wpTo : matrix.keySet()) //j = i for symmetric weights    			
    		{
    			if(wpTo instanceof Waypoint)
    			{
    				if(wpTo.equals(wpFrom)) continue; //is 0	
    			}
    			
    			System.out.print("from:" + wpFrom.getName());
    			System.out.print(" to:" + wpTo.getName());
    			System.out.println(" =" + matrix.get(wpFrom).get(wpTo));
    		}
    	}
	}
	
	/**
	 * displays a matrix in a humanly readable form
	 * @param matrix
	 */
	protected void present3dMatrix(HashMap<GameObject, HashMap<GameObject, HashMap<GameObject, Double>>> matrix)
	{
		for (GameObject wpFrom : matrix.keySet())    		
    	{
			for (GameObject wpThrough : matrix.keySet())
			{
				if(wpThrough.equals(wpFrom)) continue;
				for (GameObject wpTo : matrix.keySet()) //j = i for symmetric weights    			
	    		{
	    			if(wpTo.equals(wpFrom)) continue; //is 0
	    			if(wpTo.equals(wpThrough)) continue; //is 0
	    			System.out.print("from:" + wpFrom.getName());
	    			System.out.print(" through:" + wpThrough.getName());
	    			System.out.print(" to:" + wpTo.getName());
	    			System.out.println(" = " + matrix.get(wpFrom).get(wpThrough).get(wpTo));
	    		}	
			}    		
    	}
	} 
	
	
	/**
	 * show a list as consecutive waypoints
	 * @param waypointList
	 */
    protected void presentList(LinkedList<GameObject> waypointList)
    {
    	System.out.println("");
    	for (GameObject way : waypointList)
    	{
    		System.out.print(way.getName() + " ");
    	}
    }
    
    /**
     * calculate the total path distance of a list of waypoints
     * look-up in distanceMatrix 
     * @param waypointList
     * @return
     */
	protected double getPathDistance(LinkedList<GameObject> waypointList) {

		double pathCost = 0;		
		for(int i = 1; i < waypointList.size(); i++)
   		{
			pathCost += distanceMatrix.get(waypointList.get(i-1)).get(waypointList.get(i));
   		}		
		return pathCost;
	}
	
    /**
     * calculate the total path distance of a list of waypoints
     * look-up in matrixCostLava 
     * @param waypointList
     * @return
     */
	protected double getPathDistanceLava(LinkedList<GameObject> waypointList) {
		double pathCost = 0;		
		for(int i = 1; i < waypointList.size(); i++)
   		{			
			pathCost += matrixCostLava.get(waypointList.get(i-1)).get(waypointList.get(i));
   		}
		return pathCost;
	}
	
	/**
	 * directness is a measure of how much a path deviates from a straight line
	 * so is the ratio of the path distance and the euclidean distance
	 * look-up in matrixCostDirectness
	 * @param waypointList
	 * @return distance ratio
	 */
	private double getPathDirectness(LinkedList<GameObject> waypointList) {
		double pathRatio = 0;
		for(int i = 1; i < waypointList.size(); i++)
   		{
			GameObject wpFrom = waypointList.get(i-1);
			GameObject wpTo = waypointList.get(i);			
			pathRatio += matrixCostDirectness.get(wpFrom).get(wpTo); 			
   		}
		return pathRatio;
	}
	
	/**
	 * returns total angle cost for a path
	 * look-up in matrixCostAngle
	 * @param waypointList
	 * @return
	 */
	private double getPathCostAngleChange(LinkedList<GameObject> waypointList) {
		double angleCost = 0;
		for(int i = 1; i < waypointList.size()-1; i++)
   		{
			GameObject wpFrom = waypointList.get(i-1);
			GameObject wpThrough = waypointList.get(i);
			GameObject wpTo = waypointList.get(i+1);
			angleCost += matrixCostAngle.get(wpFrom).get(wpThrough).get(wpTo); 			
   		}
		return angleCost;
	}

	/**
	 * cost for a route is composed of:
	 *  distance cost, including lava
	 *  indirectness, how much the shortest path deviates from a straight line
	 *  change in angle between entering and leaving a waypoint
	 * @param waypointList
	 * @return total cost
	 */
	public double getPathAlternateCost(LinkedList<GameObject> waypointList, double weightDistance, double weightDirectness, double weightAngle, double weightFuelTankCost)
	{			
		//get the latest waypoint
		//get the fuel tank count before the latest waypoint
		int latestWaypoint = 0;
		int countFuelTanks = 0;		
		for (int i = waypointList.size()-1; i >=0 ; i--)			
		{
			if(i > latestWaypoint && waypointList.get(i) instanceof Waypoint)
			{
				//only get the highest waypoint
				latestWaypoint = i;
				
				//list ends with a waypoint
				if(i == waypointList.size()-1)
				{
					countFuelTanks = aGameCopy.getNumFuelTanks();					
					break;
				}
			}
			if(i < latestWaypoint && waypointList.get(i) instanceof FuelTank)
			{
				countFuelTanks++;
			}
		}
		LinkedList<GameObject> shortList = new LinkedList<GameObject>();
		for(int i = 0; i <= latestWaypoint; i++)
		{
			shortList.add(waypointList.get(i));
		}
		
		//present list
		if (verbose) presentList((LinkedList<GameObject>) shortList);

////		//TODO - this stops the execution 0
//		System.exit(1);
		
		double costDistanceLava = 0;
		costDistanceLava = getPathDistanceLava(shortList);
//		System.out.println("distance matrix lava: " + costDistanceLava);		
		
		double costDirectness = 0;
		costDirectness = getPathDirectness(shortList);
//		System.out.println("directness: " + costDirectness);
		
		double costAngleChange = 0;
		costAngleChange = getPathCostAngleChange(shortList);		
//		System.out.println("angle change: " + costAngleChange);


		//TODO 0 how come this doesn't always return a path without any fuel tanks? such a shortlist should almost always have a better score
		double totalCost = Math.pow(costDistanceLava, weightDistance) + weightDirectness *costDirectness + weightAngle*costAngleChange - weightFuelTankCost*countFuelTanks;
		if (verbose) System.out.println(totalCost);
//		System.out.printf("\ncost: distance [%f] directness [%f]  angle [%f] = total[%f]", Math.pow(costDistanceLava, weightDistance), weightDirectness *costDirectness, weightAngle*costAngleChange, totalCost);
		return totalCost;
	}
	public double getPathAlternateCost(LinkedList<GameObject> waypointList)
	{	
		return getPathAlternateCost(waypointList, weightDistance, weightDirectness, weightAngle, weightFuelTankCost);		
	}   
	
	/**
	 * cost for a route is composed of:
	 *  distance cost, including lava
	 *  indirectness, how much the shortest path deviates from a straight line
	 *  change in angle between entering and leaving a waypoint
	 * @param waypointList
	 * @return total cost
	 */
	public double getPathCost(LinkedList<GameObject> waypointList, double weightDistance, double weightDirectness, double weightAngle)
	{			
		double costDistanceLava = 0;
		costDistanceLava = getPathDistanceLava(waypointList);
//		System.out.println("distance matrix lava: " + costDistanceLava);		
		
		double costDirectness = 0;
		costDirectness = getPathDirectness(waypointList);
//		System.out.println("directness: " + costDirectness);
		
		double costAngleChange = 0;
		costAngleChange = getPathCostAngleChange(waypointList);		
//		System.out.println("angle change: " + costAngleChange);
			
		double totalCost = Math.pow(costDistanceLava, weightDistance) + weightDirectness *costDirectness + weightAngle*costAngleChange;
//		System.out.printf("\ncost: distance [%f] directness [%f]  angle [%f] = total[%f]", Math.pow(costDistanceLava, weightDistance), weightDirectness *costDirectness, weightAngle*costAngleChange, totalCost);
		return totalCost;
	}	
	
	/**
	 * compute and return weighted cost for a given list
	 * @param waypointList
	 * @return
	 */
	public double getPathCost(LinkedList<GameObject> waypointList)
	{
		return getPathCost(waypointList, weightDistance, weightDirectness, weightAngle);		
	}        
	
	/**
	 * return the planned path as a graph path
	 * @return
	 */
	public ArrayList<Path> getPlannedPath() 
	{
		return plannedPath;
	}
	
	/**
	 * return the planned path as a list of waypoints
	 * @return
	 */
	public LinkedList<GameObject> getOrderedWaypoints() 
	{
		return orderedWaypoints;
	}
}

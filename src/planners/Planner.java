package planners;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;

import com.sun.xml.internal.ws.util.StringUtils;

import controllers.mcts.DriveMCTS;
import framework.core.Controller;
import framework.core.Game;
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
	static HashMap<Waypoint, HashMap<Waypoint, Double>> distanceMatrix = new HashMap<>();//distance matrix from each waypoint to each other
	static HashMap<Waypoint, HashMap<Waypoint, Double>> matrixCostLava = new HashMap<>();//cost (distance and lava) matrix from each waypoint to each other
	static HashMap<Waypoint, HashMap<Waypoint, Double>> matrixCostDirectness = new HashMap<>();//directness matrix from each waypoint to each other
	static HashMap<Waypoint, HashMap<Waypoint, HashMap<Waypoint, Double>>> matrixCostAngle = new HashMap<>();//directness matrix from each waypoint, through each other, to each other
	LinkedList<Waypoint> m_orderedWaypoints = new LinkedList<>();//the route resulted from the planner
	ArrayList<Path> m_plannedPath = new ArrayList<>();//path between waypoints ready to be displayed on-screen
	Graph m_graph;
	Game aGameCopy;
	static double weightLava = 1;
	static double weightDistance = 1;
	static double weightDirectness = 0;
	static double weightAngle = 0;
	//TODO 8 if a waypoint is taken out of order, what do we do? should we replan? remove it from the path? with 1 0 0 1 this happens
	//fuel true
	//bonus fuel 200
	//penalty for consecutive tanks 1000
		
    /**
     * store paths from one point to another, in the order resulted by a planner
     * @param a_gameCopy
     */
    public void calculateOrderedWaypointsPaths() 
    {
        //calculate paths from one waypoint to the next
        for (int i = 0; i < m_orderedWaypoints.size()-1; i++)
        {
        	Node nodeFrom = m_graph.getClosestNodeTo(m_orderedWaypoints.get(i).s.x, m_orderedWaypoints.get(i).s.y, false);//get node for the current waypoint
        	Node nodeTo = m_graph.getClosestNodeTo(m_orderedWaypoints.get(i+1).s.x, m_orderedWaypoints.get(i+1).s.y, false);//get node for the next waypoint
        	Path pathToNext = m_graph.getPath(nodeFrom.id(), nodeTo.id());//And get the path from one to the other
        	m_plannedPath.add(pathToNext);//store the path
        }
        
    }
    
    /**
     * create the 3d matrix containing angle changes between wapyoints, counting the last and first steps of the path
     * @param waypointList
     * @return 3d hash map, double
     */    
    protected HashMap<Waypoint , HashMap<Waypoint, HashMap<Waypoint , Double>>> createAngleMatrix(LinkedList<Waypoint> waypointList)
    {
		long timeIn = System.currentTimeMillis();
    	HashMap<Waypoint, HashMap<Waypoint, HashMap<Waypoint, Double>>> matrixCostAngle = new HashMap<>();//directness matrix from each waypoint, through each other, to each other
    	HashMap<Waypoint, Double> wpTo_Value = new HashMap<Waypoint, Double>();
    	HashMap<Waypoint, HashMap<Waypoint, Double>> wpThrough_wpTo = new HashMap<Waypoint, HashMap<Waypoint, Double>>();
		for(int i = 0; i < waypointList.size(); i++)
   		{
			Waypoint wpFrom = waypointList.get(i);
			Node nodeFrom = m_graph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
			wpThrough_wpTo.clear();
			for (int j = 0; j < waypointList.size(); j++)
			{
				if(j == i) continue;
				Waypoint wpThrough = waypointList.get(j);
				Node nodeThrough = m_graph.getClosestNodeTo(wpThrough.s.x, wpThrough.s.y, false);				
				Path pathIncoming = m_graph.getPath(nodeFrom.id(), nodeThrough.id());
				Node nodeIncoming =  m_graph.getNode(pathIncoming.m_points.get(0));
				for(int n = 0; n < pathIncoming.m_points.size()-1; n++)//<size-1 because last node in path is the same as wpThrough
				{                           
					Node itNode = m_graph.getNode(pathIncoming.m_points.get(n));
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
				for(int k =0; k < waypointList.size(); k++)
				{					
					if(k == j) continue;
					if(k == i) continue;
					Waypoint wpTo = waypointList.get(k);				
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
						Node nodeTo = m_graph.getClosestNodeTo(wpTo.s.x, wpTo.s.y, false);						
                        Path pathOutgoing = m_graph.getPath(nodeThrough.id(), nodeTo.id());
                        Node nodeOutgoing =  m_graph.getNode(pathOutgoing.m_points.get(0));
        				for(int n = pathOutgoing.m_points.size()-1; n > 0 ; n--)//>0 because first node is wpTrough
        				{                           
        					Node itNode = m_graph.getNode(pathOutgoing.m_points.get(n));
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
				wpThrough_wpTo.put(wpThrough, (HashMap<Waypoint, Double>) wpTo_Value.clone());
			}
			matrixCostAngle.put(wpFrom, (HashMap<Waypoint, HashMap<Waypoint, Double>>) wpThrough_wpTo.clone());
   		}
		long timeOut = System.currentTimeMillis();
		System.out.println(" Time spent inside create angle matrix: " + (timeOut - timeIn) + " ms.");
//		present3dMatrix(matrixCostAngle);
		return matrixCostAngle;		
		
	}
    
	/**
	 * creates a matrix of directness between waypoints
	 * directness is a measure of how much a path deviates from a straight line
	 * so is the ratio of the path distance (not cost!) and the euclidean distance
	 * needs distanceMatrix
	 * @param waypointList
	 * @return directness matrix
	 */
	protected HashMap<Waypoint, HashMap<Waypoint, Double>> createDirectnessMatrix(LinkedList<Waypoint> waypointList) {
		long timeIn = System.currentTimeMillis();
		double distPath = 0;
		double distEuclidean = 0;
		HashMap<Waypoint, HashMap<Waypoint, Double>> matrixCostDirectness = new HashMap<>();//directness matrix from each waypoint to each other
		HashMap<Waypoint, Double> wpEndValue = new HashMap<>();//distance from -one- waypoint to each of the others
		for(int i = 0; i < waypointList.size(); i++)
   		{
			Waypoint wpFrom = waypointList.get(i);
			Node nodeFrom = m_graph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
			wpEndValue.clear();		
			for (int j = 0; j < waypointList.size(); j++)
			{
				double ratio = 0;					
				Waypoint wpTo = waypointList.get(j);				
				Node nodeTo = m_graph.getClosestNodeTo(wpTo.s.x, wpTo.s.y, false);						
				
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
				matrixCostDirectness.put(wpFrom, (HashMap<Waypoint, Double>) wpEndValue.clone());
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
	protected HashMap<Waypoint, HashMap<Waypoint, Double>> createDistanceMatrix(LinkedList<Waypoint> waypointList) {
		long timeIn = System.currentTimeMillis();
		//compute distance matrix for waypoints
		HashMap<Waypoint, HashMap<Waypoint, Double>> distanceMatrix = new HashMap<>();//distance from -each- waypoint to each of the others
       	HashMap<Waypoint, Double> distanceList = new HashMap<>();//distance from -one- waypoint to each of the others
       		
    	for (int i = 0; i < waypointList.size(); i++)    		
    	{
    		Waypoint wpFrom = waypointList.get(i);
    		distanceList.clear();
    		
    		for (int j = 0; j < waypointList.size(); j++) 
			//j = i for symmetric weights , j = 0 for asymmetric (uphill downhill)
    		//we need full matrix for easy lookup, so j = 0, but we will avoid computing cost by retrieving previous result
    		{
    			Waypoint wpTo = waypointList.get(j);    			
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
    				Node nodeFrom = m_graph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
    				Node nodeTo = m_graph.getClosestNodeTo(wpTo.s.x, wpTo.s.y, false);					
    				Path aPath = m_graph.getPath(nodeFrom.id(), nodeTo.id());
    	    		pathCost = aPath.m_cost;
//    	    		System.out.println(" calculated new path cost:" + pathCost);
    			}
	        	distanceList.put(wpTo, pathCost);	        	
    		}
    		distanceMatrix.put(wpFrom, (HashMap<Waypoint, Double>) distanceList.clone());
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
	protected HashMap<Waypoint, HashMap<Waypoint, Double>>[] createDistanceMatrices(LinkedList<Waypoint> waypointList) {		
		long timeIn = System.currentTimeMillis();
		HashMap<Waypoint, HashMap<Waypoint, Double>>[] result = new HashMap[2];
		
		//compute distance matrix for waypoints
		HashMap<Waypoint, HashMap<Waypoint, Double>> distanceMatrix = new HashMap<>();//distance from -each- waypoint to each of the others
		HashMap<Waypoint, HashMap<Waypoint, Double>> distanceMatrixLava = new HashMap<>();//distance from -each- waypoint to each of the others
       	HashMap<Waypoint, Double> distanceList = new HashMap<>();//distance from -one- waypoint to each of the others
       	HashMap<Waypoint, Double> distanceListLava = new HashMap<>();//distance from -one- waypoint to each of the others
       		
    	for (int i = 0; i < waypointList.size(); i++)    		
    	{
    		Waypoint wpFrom = waypointList.get(i);
    		distanceList.clear();
    		distanceListLava.clear();
    		
    		for (int j = 0; j < waypointList.size(); j++) 
			//j = i for symmetric weights , j = 0 for asymmetric (uphill downhill)
    		//we need full matrix for easy lookup, so j = 0, but we will avoid computing cost by retrieving previous result
    		{    	       	
    			Waypoint wpTo = waypointList.get(j);    			
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
    				Node nodeFrom = m_graph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
    				Node nodeTo = m_graph.getClosestNodeTo(wpTo.s.x, wpTo.s.y, false);					
    				Path aPath = m_graph.getPath(nodeFrom.id(), nodeTo.id());
    	    		pathCost = aPath.m_cost;
    	    		double distanceCost = 0;
    	    		for(int k = 0; k < aPath.m_points.size()-1; k++)
		            {    	    			
		                Node thisNode = m_graph.getNode(aPath.m_points.get(k));
		                Node nextNode = m_graph.getNode(aPath.m_points.get(k+1));
		                Game tempGame = aGameCopy.getCopy();		                
		                Ship tempShip = tempGame.getShip();
		                tempShip.s.x = nextNode.x();
		                tempShip.s.y = nextNode.y();
		                
		                //this action is needed to register if the ship is above lava, using ACTION_NO_FRONT will not work for tempShip.isOnLava()
		                tempGame.getShip().update(Controller.ACTION_NO_LEFT);
//		                tempGame.tick(Controller.ACTION_NO_LEFT);//ship.update only updates ship, tick updates the whole world
		                if(tempShip.isOnLava()) {		                	
		                	distanceCost += weightLava*thisNode.euclideanDistanceTo(nextNode);
		                }
		                else {
		                	distanceCost += thisNode.euclideanDistanceTo(nextNode);
		                }		               		                  
		            }    	    		
    	    		lavaCost = distanceCost;    				
    			}    			
	        	distanceList.put(wpTo, pathCost);
	        	distanceListLava.put(wpTo, lavaCost);
    		}
    		distanceMatrix.put(wpFrom, (HashMap<Waypoint, Double>) distanceList.clone());
    		distanceMatrixLava.put(wpFrom, (HashMap<Waypoint, Double>) distanceListLava.clone());
    	}
    	result[0] = distanceMatrix;
    	result[1] = distanceMatrixLava;
		long timeOut = System.currentTimeMillis();
		System.out.println(" Time spent inside create distance matrices: " + (timeOut - timeIn) + " ms.");    	
    	return result;
	}
	
	protected void createMatrices()
	{
		LinkedList<Waypoint> waypointList = (LinkedList<Waypoint>) aGameCopy.getWaypoints().clone();//start with all the waypoints
		Waypoint wpShip = new Waypoint(aGameCopy, aGameCopy.getShip().s);
    	waypointList.addFirst(wpShip);//add the ship to start from
    	HashMap<Waypoint, HashMap<Waypoint, Double>>[] distanceMatrices = createDistanceMatrices(waypointList);
    	distanceMatrix = distanceMatrices[0];
    	matrixCostLava = distanceMatrices[1];  
    	matrixCostDirectness = createDirectnessMatrix(waypointList);
    	matrixCostAngle = createAngleMatrix(waypointList);
	}

	/**
	 * displays a matrix in a humanly readable form
	 * @param matrix
	 */
	protected void present2dMatrix(HashMap<Waypoint, HashMap<Waypoint, Double>> matrix)
	{
		for (Waypoint wpFrom : matrix.keySet())    		
    	{
    		for (Waypoint wpTo : matrix.keySet()) //j = i for symmetric weights    			
    		{
    			if(wpTo.equals(wpFrom)) continue; //is 0
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
	protected void present3dMatrix(HashMap<Waypoint, HashMap<Waypoint, HashMap<Waypoint, Double>>> matrix)
	{
		for (Waypoint wpFrom : matrix.keySet())    		
    	{
			for (Waypoint wpThrough : matrix.keySet())
			{
				if(wpThrough.equals(wpFrom)) continue;
				for (Waypoint wpTo : matrix.keySet()) //j = i for symmetric weights    			
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
    protected void showList(LinkedList<Waypoint> waypointList)
    {
    	System.out.println("");
    	for (Waypoint way : waypointList)
    	{
    		System.out.print(way.getName() + " ");
    	}
    	System.out.println("");
    }
    
    /**
     * calculate the total path distance of a list of waypoints
     * look-up in distanceMatrix 
     * @param waypointList
     * @return
     */
	protected double getPathDistance(LinkedList<Waypoint> waypointList) {

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
	protected double getPathDistanceLava(LinkedList<Waypoint> waypointList) {
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
	private double getPathDirectness(LinkedList<Waypoint> waypointList) {
		double pathRatio = 0;
		for(int i = 1; i < waypointList.size(); i++)
   		{
			Waypoint wpFrom = waypointList.get(i-1);
			Waypoint wpTo = waypointList.get(i);			
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
	private double getPathCostAngleChange(LinkedList<Waypoint> waypointList) {
		double angleCost = 0;
		for(int i = 1; i < waypointList.size()-1; i++)
   		{
			Waypoint wpFrom = waypointList.get(i-1);
			Waypoint wpThrough = waypointList.get(i);
			Waypoint wpTo = waypointList.get(i+1);
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
	public double getPathCost(LinkedList<Waypoint> waypointList, double weightDistance, double weightDirectness, double weightAngle)
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


	public double getPathCost(LinkedList<Waypoint> waypointList)
	{
		return getPathCost(waypointList, weightDistance, weightDirectness, weightAngle);		
	}        
	public ArrayList<Path> getPlannedPath() 
	{
		return m_plannedPath;
	}
	public LinkedList<Waypoint> getOrderedWaypoints() 
	{
		return m_orderedWaypoints;
	}
}

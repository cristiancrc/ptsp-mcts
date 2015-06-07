package planners;

import java.util.ArrayList;
import java.util.LinkedList;

import framework.core.Controller;
import framework.core.Game;
import framework.core.Ship;
import framework.core.Waypoint;
import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;
import framework.utils.Vector2d;

/**
 *  cost greedy planner
 *  goes through the list waypoints and gets the closest one
 *  evaluation function is my own, does not follow paper
 *  TODO: intuitive implementation, useful for snippets. remove.
 *  @author Cristian
 *  @version 141128
 */	
public class PlannerGreedyEvolved extends Planner {
	
	public PlannerGreedyEvolved(Game a_gameCopy)
	{
    	System.out.println("***greedy evolved planner***");
        long timeStart = System.currentTimeMillis();
		verbose = true;        
        m_graph = new Graph(a_gameCopy);

    	//weights
    	double weightDistance = 1.0;
    	double weightAngles = 1.0;
    	double weightLava = 1.0;//multiplier for distances over lava
    	
    	//add ship position as starting waypoint
        Waypoint wpShip = new Waypoint(a_gameCopy, a_gameCopy.getShip().s);        
    	m_orderedWaypoints.add(wpShip);
		
    	Vector2d shipHeading = a_gameCopy.getShip().d;//ship heading
    	shipHeading.normalise();
    	System.out.println("ship heading " + shipHeading);    	
    	Vector2d entryHeading = shipHeading;
    	Vector2d lastHeadingForMinimum = null;
		
		//playground
//    	Node anode = new Node(900, 136, 352);    
//    	Node bnode = new Node(901, 128, 344);
//    	Node anode = new Node(900, 0, 0);
//    	Node bnode = new Node(901, -1, -1);  
//    	Vector2d baseVector = new Vector2d(0, -1);
//    	Vector2d nodeVector = new Vector2d();
//    	nodeVector.x = bnode.x() - anode.x();
//    	nodeVector.y = bnode.y() - anode.y();
//		nodeVector.normalise();
//		double signedAngle = (Math.acos(nodeVector.dot(baseVector)));
//		if (bnode.x() < anode.x()) {
//			signedAngle = -signedAngle;
//		}
//		System.out.println("vector " + nodeVector);
//		System.out.println("entry " + entryHeading);
//		System.out.println("signed angle " + Math.toDegrees(signedAngle));
//		double degreesToTurn1 = (Math.acos(nodeVector.dot(entryHeading)));		
//        System.out.println("\t  rads to turn:" + degreesToTurn1);
//        System.out.println("\t deg to turn:" + Math.toDegrees(degreesToTurn1));
//		
//		System.exit(0);
		
    	
		//iterate through the other waypoints to find the closest one to each next one
    	double distanceTravelled = 0;//not using distance matrix
    	@SuppressWarnings("unchecked")
		LinkedList<Waypoint> waypointList = (LinkedList<Waypoint>) a_gameCopy.getWaypoints().clone();//a list of all waypoints    	
		int checkedWaypoints = 0;
		Waypoint wpFrom = wpShip;
		while(waypointList.size() > 0 )
		{
			if(verbose) System.out.println("\n" + checkedWaypoints + ": wpFrom: " + wpFrom.getName()  + ", wpList size " + waypointList.size());// + ", ordered " + orderedWaypoints.size());
//			if(check++ > a_gameCopy.getNumWaypoints()+1) break;//one more waypoint for the initial ship position
			Waypoint closestWaypoint = null;
	    	
			Vector2d pathHeadingInit = entryHeading.copy();		
			Vector2d pathHeading = new Vector2d();
			double minPathCost = Double.MAX_VALUE;
	    	
			for(Waypoint wpEnd : waypointList)
			{
				if(wpEnd.equals(wpFrom)) continue;
				if(verbose) System.out.println("...checking " + wpEnd.getName());
				double pathCost = 0;
				double headingDiffCost = 0;
				double distanceCost = 0;
				double degreesToTurn = 0;
				
				Node nodeFrom = m_graph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
				Node nodeTo = m_graph.getClosestNodeTo(wpEnd.s.x, wpEnd.s.y, false);					
	    		Path aPath = m_graph.getPath(nodeFrom.id(), nodeTo.id());
				
	    		if (null != aPath)
	        	{
		        	for(int k = 0; k < aPath.m_points.size()-1; k++)
		            {
		                Node thisNode = m_graph.getNode(aPath.m_points.get(k));
		                Node nextNode = m_graph.getNode(aPath.m_points.get(k+1));
		                
		                //add up all the turns needed to the next waypoint (directness)
		                distanceCost += thisNode.euclideanDistanceTo(nextNode);
		                if(verbose) System.out.println("\t" + k + " this node: " + thisNode.x() + " , " + thisNode.y());
		                if(verbose) System.out.println("\t" + k + " next node: " + nextNode.x() + " , " + nextNode.y());
						pathHeading.x = nextNode.x() - thisNode.x();
		                pathHeading.y = nextNode.y() - thisNode.y();
		                pathHeading.normalise();
		                if(verbose) System.out.println("\t" + k + " resulting vector: " + pathHeading.x + " , " + pathHeading.y);
		                if(verbose) System.out.println("\t" + k + " base vector: " + pathHeadingInit.x + " , " + pathHeadingInit.y);
		                
		                //increase cost for going over lava
		                Game tempGame = a_gameCopy.getCopy();		                
		                Ship tempShip = tempGame.getShip();
		                tempShip.s.x = nextNode.x();
		                tempShip.s.y = nextNode.y();
		                tempGame.setShip(tempShip);
		                System.out.println("ship heading: " + tempGame.getShip().d);
		                tempGame.tick(Controller.ACTION_NO_LEFT);//this tick is needed to register if the ship is above lava, using ACTION_NO_FRONT will not work for tempShip.isOnLava()
		                if(tempShip.isOnLava()) {
//		                	if(verbose) System.out.println("++++++++++++at " + nextNode.x() + " , " + nextNode.y() + " on lava");
		                	//if ship is on lava add more distance cost (one distance is already added!)
		                	distanceCost += 0.5*thisNode.euclideanDistanceTo(nextNode);
		                }
		                
		                //degreesToTurn = Math.toDegrees(Math.acos(pathHeadingInit.dot(pathHeading)));
		                degreesToTurn = (Math.acos(pathHeadingInit.dot(pathHeading)));		                
		                if(verbose) System.out.println("\t" + k + " rads to turn:" + degreesToTurn);
		                if(verbose) System.out.println("\t" + k + " deg to turn:" + Math.toDegrees(degreesToTurn));
		                
		                headingDiffCost += degreesToTurn;
		                pathHeadingInit = pathHeading.copy();//set new init heading as the last heading used for previous node    
		            }

		        	pathCost = weightDistance * distanceCost + weightAngles * headingDiffCost;
		        	if(verbose) System.out.println(" this path distance cost " + distanceCost + " , heading cost " + headingDiffCost + ", total cost " + pathCost + ", minPath " + minPathCost );
		           
		        	if(pathCost < minPathCost) 
		        	{
		        		minPathCost = pathCost;
		        		closestWaypoint = wpEnd;
		        		lastHeadingForMinimum = pathHeading.copy();
		        		if(verbose) System.out.println( " ... new minimum");
		        	}
	        	}
			}			
			if(verbose) System.out.println("\nclosest is " + closestWaypoint.getName());			
			m_orderedWaypoints.add(closestWaypoint);
			
			showList(m_orderedWaypoints);
			distanceMatrix = createDistanceMatrix(m_orderedWaypoints);
			

			
			//store total distance
			distanceTravelled += minPathCost;
			if(verbose) System.out.println("\ncost so far: " + distanceTravelled);
			
			//store last heading for minimum
			entryHeading = lastHeadingForMinimum.copy();
			
			//next search starting from this node
			wpFrom = closestWaypoint.getCopy(a_gameCopy);
			
			//remove node from list
			waypointList.remove(closestWaypoint);
		}
		long timeAfter = System.currentTimeMillis();
		System.out.println("Path cost:" + distanceTravelled);
		System.out.println("Path distance " + getPathDistance(m_orderedWaypoints));
		System.out.println("Greedy Evolved Planner time: " + (timeAfter - timeStart) + " ms.");
    }	
}

package planners;

import java.util.Collection;
import java.util.LinkedList;

import framework.core.Game;
import framework.core.GameObject;
import framework.core.Waypoint;
import framework.core.Waypoint;
import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;

/**
 *  greedy planner
 *  goes through the list waypoints and gets the closest one
 *  creates just the distance matrix if it isn't present
 *  @author Cristian
 *  @version 141128
 */
public class PlannerGreedyCost_w extends Planner {
		
	public PlannerGreedyCost_w(Game aGameCopy)
	{
    	System.out.println("***cost greedy planner***");
    	this.aGameCopy = aGameCopy;
//    	verbose = true;
	}
	
	public void runPlanner()
	{
        long timeStart = System.currentTimeMillis();
    	aGraph = new Graph(aGameCopy);
    	@SuppressWarnings("unchecked")
		LinkedList<Waypoint> waypointList = (LinkedList<Waypoint>) aGameCopy.getWaypoints().clone();//a list of all waypoints
//		if(includeFuel && false)    			
//		{
//			System.out.println("planner greedy cost : including fuel");
//			waypointList.addAll((Collection<? extends Waypoint>) aGameCopy.getFuelTanks().clone());//add all the fuel tanks
//		}
    	//add ship position as waypoint
        Waypoint wpShip = new Waypoint(aGameCopy, aGameCopy.getShip().s);    	
    	if(distanceMatrix.size() == 0)
    	{
    		createMatrices();
    	}    	    	
//    	orderedWaypoints.addFirst(wpShip);
    	
		//iterate through the other waypoints to find the closest one to each next one
		int checkedWaypoints = 0;
		Waypoint wpFrom = wpShip;	
		orderedWaypoints.add(wpFrom);
    	double distanceTravelled = 0;//total distance, no distance matrix needed	
		while(waypointList.size() > 0 )
		{
			LinkedList<Waypoint> shortList = new LinkedList<>();
			shortList.add(wpFrom);
			if(verbose) System.out.println("\n" + checkedWaypoints++ + ": wpFrom: " + wpFrom.getName() + ", wpList size " + waypointList.size());// + ", ordered " + orderedWaypoints.size());
			
			Waypoint wpThroughSelected = null;
			Waypoint wpEndSelected = null;
			double minPathCost = Double.MAX_VALUE;//path between the two waypoints we are checking
			for(Waypoint wpThrough : waypointList)
			{
				shortList.clear();
				shortList.add(wpFrom);
				shortList.add(wpThrough);
				if(verbose) System.out.print("\n\n---through " + wpThrough.getName());
				for(Waypoint wpEnd : waypointList)
				{
					shortList.clear();
					shortList.add(wpFrom);
					shortList.add(wpThrough);
					if(wpEnd == wpThrough) continue;
					shortList.add(wpEnd);
					if(verbose) System.out.print("\n...checking " + wpEnd.getName());
					
					LinkedList<GameObject> tempList = new LinkedList<>();
					for(GameObject way : shortList)
					{
						tempList.add(way);
					}
					double pathCost = getPathCost(tempList);
					if(pathCost < minPathCost)
					{
						System.out.println("...");
						wpThroughSelected = wpThrough;
						wpEndSelected = wpEnd;
						minPathCost = pathCost;
					}
//					shortList.remove(wpEnd);
				}
//				shortList.remove(wpThrough);
			}
			if(verbose) System.out.println("smallest cost is " + wpThroughSelected.getName() + " > " + wpEndSelected.getName());
			distanceTravelled += minPathCost;
			orderedWaypoints.add(wpThroughSelected);
			orderedWaypoints.add(wpEndSelected);
			wpFrom = wpEndSelected;//next search starting from this node
			System.out.println(wpFrom.getName());
			System.out.println(wpThroughSelected.getName());
			System.out.println(wpEndSelected.getName());
			waypointList.remove(wpThroughSelected);//remove node from list
			waypointList.remove(wpEndSelected);//remove node from list
		}		
		long timeAfter = System.currentTimeMillis();
		System.out.println("Path cost:" + distanceTravelled);
		System.out.println("Cost Greedy Planner running time: " + (timeAfter - timeStart) + " ms.");
    }	
}

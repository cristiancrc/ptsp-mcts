package fr.inria.optimization.cmaes.examples;
import java.util.HashMap;
import java.util.Random;

import controllers.mcts.DriveMCTS;

import fr.inria.optimization.cmaes.CMAEvolutionStrategy;
import fr.inria.optimization.cmaes.CMASolution;
import fr.inria.optimization.cmaes.PrintfFormat;
import fr.inria.optimization.cmaes.fitness.IObjectiveFunction;
import framework.ExecSync;
import framework.core.Exec;

class OptimizeMe implements IObjectiveFunction { // meaning implements methods valueOf and isFeasible
	
	int trials = 2;//how many times to run on each map
//	String[] mapList = new String[]{"maps/ptsp_map01.map","maps/ptsp_map02.map","maps/ptsp_map08.map",
//            "maps/ptsp_map19.map","maps/ptsp_map24.map","maps/ptsp_map35.map","maps/ptsp_map40.map",
//            "maps/ptsp_map45.map","maps/ptsp_map56.map","maps/ptsp_map61.map"}; 
	String[] mapList = new String[]{"maps/ptsp_map99.map","maps/ptsp_map01.map"}; 	
	public double valueOf (double[] parameters)
	{		
		double avg_score = 0;
		double avg_waypointsVisited = 0;
		double avg_time = 0;
		double avg_fuelSpent = 0;
		double avg_damageTaken = 0;
		System.out.println("=================================== starting valueOf");
		for(String runThisMap : mapList)
		{			
			System.out.println("*********************************** map:" + runThisMap);
			for( int i = 0; i < trials; i++)
			{
				System.out.println("----------------------------------- map :" + runThisMap + ", trial : " + i);
			
				//start moptsp game
				ExecSync aGame = new ExecSync();
				Exec.m_visibility = false;
				Exec.m_verbose = true;
				Exec.m_mapNames = new String[]{runThisMap};
				Exec.m_controllerName = "controllers.mcts.DriveMCTS";	
				
				//set-up parameters
				boolean includeFuelParam = (parameters[4] > 0 ? true : false);
		
				//update each parameter
				DriveMCTS.w_lava = parameters[0];		
				DriveMCTS.w_distance = parameters[1]; 
				DriveMCTS.w_directness = parameters[2];
				DriveMCTS.w_angle = parameters[3];
				DriveMCTS.p_includeFuel = includeFuelParam;
				DriveMCTS.w_fuelTankCost = parameters[5];
				DriveMCTS.w_consecutiveFuelTanksCost =  parameters[6];	
				DriveMCTS.macroActionsCount =  (int) parameters[7];
				DriveMCTS.search_searchDepth =  (int) parameters[8];
				DriveMCTS.ucb1Exploration =  parameters[9];	
				DriveMCTS.scorePanicMode = parameters[10];								
				DriveMCTS.search_wpCollectedRoute = parameters[11];		
				DriveMCTS.search_wpCollectedOutOfRoute = parameters[12];
				DriveMCTS.search_wpDistance = parameters[13];
				DriveMCTS.search_fuelConsumed = parameters[14];
				DriveMCTS.search_wpFuelOutOfRoute = parameters[15];	
				DriveMCTS.search_damageIncurred = parameters[16];
				DriveMCTS.search_damageCollisions = parameters[17];
								
				//run one game
				ExecSync.runGame(false, 0);				
				//while a game is running with the mcts controller a dot is printed to the console each 100 ticks
				
				//results
				HashMap<String, Double> results = ExecSync.m_game.getResults();		
				CMA_MOPTSP.presentMap(results);	
				avg_score += results.get("finalScore");
				avg_waypointsVisited += results.get("waypointsVisited");
				avg_time += results.get("time");
				avg_fuelSpent += results.get("fuelSpent");
				avg_damageTaken += results.get("damageTaken");
			}
		}	
		System.out.println("\n valueOf finished");
		return avg_score/(trials * mapList.length);
	}
	public boolean isFeasible(double[] x) 
	{
		return true;
	}
}

/*
 * 
 * @see CMAEvolutionStrategy
 * 
 * @author Nikolaus Hansen, released into public domain. 
 */
public class CMA_MOPTSP {
	public static void main(String[] args) {
		IObjectiveFunction fitfun = new OptimizeMe();
		
		//parameters from 2013 moptsp paper
		double[] paperParameters = new double[18];
		paperParameters[0] = 0.5;
		paperParameters[1] = 1.5;
		paperParameters[2] = 150;
		paperParameters[3] = 80;
		paperParameters[4] = 1;
		paperParameters[5] = 200;
		paperParameters[6] = 1000;
		paperParameters[7] = 15;
		paperParameters[8] = 8;
		paperParameters[9] = 1.0;
		paperParameters[10] = 0.1;
		paperParameters[11] = 1.0;
		paperParameters[12] = -1.0;
		paperParameters[13] = 0.75;
		paperParameters[14] = 0.001;
		paperParameters[15] = 0.2;
		paperParameters[16] = 0.002;
		paperParameters[17] = 0.3;

		// new a CMA-ES and set some initial values
		CMAEvolutionStrategy cma = new CMAEvolutionStrategy();
//		cma.readProperties(); // read options, see file CMAEvolutionStrategy.properties
		cma.setDimension(18); // overwrite some loaded properties	
		cma.setTypicalX(paperParameters);// start from paper attributes
		cma.setInitialStandardDeviation(0.2); // also a mandatory setting
		cma.setSeed(0);
		cma.options.stopFitness = 1e-14;       // optional setting

		// initialize cma and get fitness array to fill in later
		double[] fitness = cma.init();  // new double[cma.parameters.getPopulationSize()];
				
		// initial output to files
		cma.writeToDefaultFilesHeaders(0); // 0 == overwrites old files

		// iteration loop
		while(cma.stopConditions.getNumber() == 0) {
			System.out.println(":"+cma.getCountIter());
            // --- core iteration step ---
			double[][] pop = cma.samplePopulation(); // get a new population of solutions			
				
			present2dArray(pop);
			for(int i = 0; i < pop.length; ++i) {    // for each candidate solution i
            	// a simple way to handle constraints that define a convex feasible domain  
            	// (like box constraints, i.e. variable boundaries) via "blind re-sampling" 
            	                                       // assumes that the feasible domain is convex, the optimum is  
				while (!fitfun.isFeasible(pop[i]))     //   not located on (or very close to) the domain boundary,  
					pop[i] = cma.resampleSingle(i);    //   initialX is feasible and initialStandardDeviations are  
                                                       //   sufficiently small to prevent quasi-infinite looping here
                // compute fitness/objective value	
				fitness[i] = fitfun.valueOf(pop[i]); // fitfun.valueOf() is to be minimized
			}
			cma.updateDistribution(fitness);         // pass fitness array to update search distribution
            // --- end core iteration step ---

			// output to files and console 
			cma.writeToDefaultFiles();
			int outmod = 150;
			if (cma.getCountIter() % (15*outmod) == 1)
				cma.printlnAnnotation(); // might write file as well
			if (cma.getCountIter() % outmod == 1)
				cma.println(); 
		}
		// evaluate mean value as it is the best estimator for the optimum
		cma.setFitnessOfMeanX(fitfun.valueOf(cma.getMeanX())); // updates the best ever solution 

		// final output
		cma.writeToDefaultFiles(1);
		cma.println();
		cma.println("Terminated due to");
		for (String s : cma.stopConditions.getMessages())
			cma.println("  " + s);
		cma.println("best function value " + cma.getBestFunctionValue() + " at evaluation " + cma.getBestEvaluationNumber());
		
		System.out.println("\n---");
		CMASolution bestSolution = cma.getBestSolution();
		double[] bestX = bestSolution.getX();
//		cma.getBestRecentSolution()
		System.out.println("best solution: " + bestSolution.getFitness() + ": " +  bestSolution.getEvaluationNumber());
		System.out.print("best x: "); present1dArray(bestX);


		
	} // main
	
	public static void present1dArray(double[] anArray)
	{
		System.out.println("---");
		for (int i =0 ; i < anArray.length; i++)
		{
//			System.out.print("\t " + anArray[i]);
			 System.out.printf("\t%.3f", anArray[i]);
		}
		System.out.println("");
	}
	
	public static void present2dArray(double[][] anArray)
	{
		System.out.print("---");
		for (int i =0 ; i < anArray.length; i++)
		{
			System.out.println("");
			for(int j =0; j < anArray[i].length; j++)
			{
//				System.out.print("\t " + anArray[i][j]);
				System.out.printf("\t\t%.3f", anArray[i][j]);
			}			
		}
		System.out.println("");
	}
	public static void presentMap(HashMap<String, Double> aMap)
	{
		for(String aKey : aMap.keySet())
		{
			System.out.println(aKey + " : " + aMap.get(aKey));
		}
//		System.out.println(aMap);
	}
} // class

package framework;

import framework.core.*;
import framework.utils.JEasyFrame;

import java.awt.*;

import controllers.keycontroller.KeyController;
import controllers.*;

/**
 * This class may be used to execute the game in timed or un-timed modes, with or without
 * visuals. Competitors should implement his controller in a subpackage of 'controllers'.
 * The skeleton classes are already provided. The package
 * structure should not be changed (although you may create sub-packages in these packages).
 */
@SuppressWarnings("unused")
public class ExecSync extends Exec
{
    /**
     * Run a game in ONE map. In order to slow thing down in case
     * the controllers return very quickly, a ti me limit can be used.
     * Use this mode to play the game with the KeyController.
     *
     * @param delay The delay between time-steps
     */
    public static void playGame(boolean visual, int delay)
    {

    	//override controller set in main()
    	//m_controllerName = "controllers.keycontroller.KeyController";//human    

        //Get the game ready.
        if(!prepareGame())
            return;


        //Indicate what are we running
        if(m_verbose) System.out.println("Running " + m_controllerName + " in map " + m_game.getMap().getFilename() + "...");

        JEasyFrame frame;
        
        if(visual)
        {
	        //View of the game, if applicable.
	        m_view = new PTSPView(m_game, m_game.getMapSize(), m_game.getMap(), m_game.getShip(), m_controller);
	        frame = new JEasyFrame(m_view, "PTSP-Game: " + m_controllerName);
	        //If we are going to play the game with the cursor keys, add the listener for that.
	        if(m_controller instanceof KeyController)
	        {
	        	System.out.println("adding listener");
	            frame.addKeyListener(((KeyController)m_controller).getInput());
	        }
        }



        while(!m_game.isEnded())
        {
        	
        	//TODO: remove time limit between waypoints
        	m_game.setStepsLeft(1000);

            //When the result is expected:
            long then = System.currentTimeMillis();
            long due = then+PTSPConstants.ACTION_TIME_MS;

            //Advance the game.
            m_game.tick(m_controller.getAction(m_game.getCopy(), due));

            long now = System.currentTimeMillis();
            int remaining = (int) Math.max(0, delay - (now-then));     //To adjust to the proper framerate.

            //Wait until the next cycle.
            waitStep(remaining);

            if(visual)
            {
	            //And paint everything.
	            m_view.repaint();
            }
        }

        if(m_verbose)
            m_game.printResults();

        //And save the route, if requested:
        if(m_writeOutput)
            m_game.saveRoute();

    }

    /**
     * Runs a game in ONE map.
     *
     * @param visual Indicates whether or not to use visuals
     * @param delay Includes delay between game steps.
     */
    public static void runGame(boolean visual, int delay)
    {
        //Get the game ready.
        if(!prepareGame())
            return;


        //Indicate what are we running
        if(m_verbose) System.out.println("Running " + m_controllerName + " in map " + m_game.getMap().getFilename() + "...");

        JEasyFrame frame;
        if(visual)
        {
            //View of the game, if applicable.
            m_view = new PTSPView(m_game, m_game.getMapSize(), m_game.getMap(), m_game.getShip(), m_controller);
            frame = new JEasyFrame(m_view, "PTSP-Game: " + m_controllerName);
        }


        while(!m_game.isEnded())
        {
            //When the result is expected:
            long then = System.currentTimeMillis();
            long due = then + PTSPConstants.ACTION_TIME_MS;

            //Advance the game.
            int actionToExecute = m_controller.getAction(m_game.getCopy(), due);

            //Exceeded time
            long now = System.currentTimeMillis();
            long spent = now - then;

            if(spent > PTSPConstants.TIME_ACTION_DISQ)
            {
                actionToExecute = 0;
                System.out.println("Controller disqualified. Time exceeded: " + (spent - PTSPConstants.TIME_ACTION_DISQ));
                m_game.abort();

            }else{

                if(spent > PTSPConstants.ACTION_TIME_MS)
                    actionToExecute = 0;
                m_game.tick(actionToExecute);
            }

            int remaining = (int) Math.max(0, delay - (now-then));//To adjust to the proper framerate.
            //Wait until the next cycle.
            waitStep(remaining);

            //And paint everything.
            if(m_visibility)
            {
                m_view.repaint();
                if(m_game.getTotalTime() == 1)
                    waitStep(m_warmUpTime);
            }
        }

        if(m_verbose)
            m_game.printResults();

        //And save the route, if requested:
        if(m_writeOutput)
            m_game.saveRoute();

    }

    /**
     * For running multiple games without visuals, in several maps (m_mapNames).
     *
     * @param trials The number of trials to be executed
     */
    public static void runGames(int trials)
    {
        //Prepare the average results.
        double avgTotalWaypoints=0;
        double avgTotalTimeSpent=0;
        double avgTotalDamageTaken=0;
        double avgTotalFuelSpent=0;
        int totalDisqualifications=0;
        int totalNumGamesPlayed=0;
        boolean moreMaps = true;

        for(int m = 0; moreMaps && m < m_mapNames.length; ++m)
        {
            String mapName = m_mapNames[m];
            double avgWaypoints=0;
            double avgTimeSpent=0;
            double avgDamage=0;
            double avgFuel=0;
            int numGamesPlayed = 0;

            if(m_verbose)
            {
                System.out.println("--------");
                System.out.println("Running " + m_controllerName + " in map " + mapName + "...");
            }

            //For each trial...
            for(int i=0;i<trials;i++)
            {
                // ... create a new game.
                if(!prepareGame())
                    continue;

                numGamesPlayed++; //another game

                //PLay the game until the end.
                while(!m_game.isEnded())
                {
                    //When the result is expected:
                    long due = System.currentTimeMillis()+PTSPConstants.ACTION_TIME_MS;

                    //Advance the game.
                    int actionToExecute = m_controller.getAction(m_game.getCopy(), due);

                    //Exceeded time
                    long exceeded = System.currentTimeMillis() - due;
                    if(exceeded > PTSPConstants.TIME_ACTION_DISQ)
                    {
                        actionToExecute = 0;
                        numGamesPlayed--;
                        m_game.abort();

                    }else{

                        if(exceeded > PTSPConstants.ACTION_TIME_MS)
                            actionToExecute = 0;

                        m_game.tick(actionToExecute);
                    }

                }

                //Update the averages with the results of this trial.
                avgWaypoints += m_game.getWaypointsVisited();
                avgTimeSpent += m_game.getTotalTime();
                avgDamage += m_game.getShip().getDamage();
                avgFuel += (PTSPConstants.INITIAL_FUEL-m_game.getShip().getRemainingFuel());

                //Print the results.
                if(m_verbose)
                {
                    System.out.print(i+"\t");
                    m_game.printResults();
                }

                //And save the route, if requested:
                if(m_writeOutput)
                    m_game.saveRoute();
            }

            moreMaps = m_game.advanceMap();

            avgTotalWaypoints += (avgWaypoints / numGamesPlayed);
            avgTotalTimeSpent += (avgTimeSpent / numGamesPlayed);
            avgTotalDamageTaken += (avgDamage / numGamesPlayed);
            avgTotalFuelSpent += (avgFuel / numGamesPlayed);
            totalDisqualifications += (trials - numGamesPlayed);
            totalNumGamesPlayed += numGamesPlayed;

            //Print the average score.
            if(m_verbose)
            {
                System.out.println("--------");
                System.out.format("Average waypoints: %.3f, average time spent: %.3f, average damage taken: %.3f, average fuel spent: %.3f\n",
                        (avgWaypoints / numGamesPlayed), (avgTimeSpent / numGamesPlayed),
                        (avgDamage / numGamesPlayed), (avgFuel / numGamesPlayed));
                System.out.println("Disqualifications: " + (trials - numGamesPlayed) + "/" + trials);
            }
        }

        //Print the average score.
        if(m_verbose)
        {
            System.out.println("\n-------- Final score --------");
            System.out.format("Average waypoints: %.3f, average time spent: %.3f, average damage taken: %.3f, average fuel spent: %.3f\n",
                    (avgTotalWaypoints / m_mapNames.length), (avgTotalTimeSpent / m_mapNames.length),
                    (avgTotalDamageTaken / m_mapNames.length), (avgTotalFuelSpent / m_mapNames.length));
            System.out.println("Disqualifications: " + (trials*m_mapNames.length - totalNumGamesPlayed) + "/" + trials*m_mapNames.length);
        }
    }


    /**
     * The main method. Several options are listed - simply remove comments to use the option you want.
     *
     * @param args the command line arguments. Not needed in this class.
     */
    public static void main(String[] args)
    {
        m_mapNames = new String[]{"maps/ptsp_map02.map"}; //Set here the name of the map to play in.
        //m_mapNames = new String[]{"maps/ptsp_map01.map","maps/ptsp_map02.map","maps/ptsp_map08.map",
        //        "maps/ptsp_map19.map","maps/ptsp_map24.map","maps/ptsp_map35.map","maps/ptsp_map40.map",
        //        "maps/ptsp_map45.map","maps/ptsp_map56.map","maps/ptsp_map61.map"}; //In an array, to play in multiple maps with runGames().
        //01 - open symmetrical
        //02 - walled inside -> heading test
        //08 - clear inside -> use of walls
        //19 - half maze half open
        //24 - walled inside 2
        //35 - tight
        //40 - tight walled inside
        //45 - tight walled inside 2
        //56 - labyrinth //going off path getting stuck in corners
        //61 - wide open
        
        
        //TODO: worthy of notice
        //exec 153 controller init time = 1000ms (10 waypoints * 10)
        //exec sync 61 time between checkpoints 
        
        
        m_controllerName = "controllers.mc.DriveDfs"; //dfs driver
//        m_controllerName = "controllers.mc.DriveMC"; //mc driver
//        m_controllerName = "controllers.mctsnavi.DriveMCTS"; //mcts driver, not implemented
        
        //completed controller
//      m_controllerName = "controllers.greedy.DriveGreedy"; //greedy controller with planner selection

        //defaults
//        m_controllerName = "controllers.greedy.GreedyController"; // default greedy
//        m_controllerName = "controllers.MacroRandomSearch.MacroRSController"; //default macro actions (TODO: check if time is wasted when macroing)
//        m_controllerName = "controllers.lineofsight.LineOfSight";//default line of sight greedy
//        m_controllerName = "controllers.random.RandomController";//default random walker
//        m_controllerName = "controllers.WoxController.WoxController"; //default
//        m_controllerName = "controllers.keycontroller.KeyControllerShowPaths"; //key controller
        
        //parameters
        m_visibility = true; //Set here if the graphics must be displayed or not (for those modes where graphics are allowed).
        m_writeOutput = false; //Indicate if the actions must be saved to a file after the end of the game (the file name will be the current date and time)..
        m_verbose = true;
//        m_warmUpTime = 150; //Change this to modify the wait time (in milliseconds) before starting the game in a visual mode


        /////// 1. To play the game with the key controller.
        int delay = PTSPConstants.DELAY;  //PTSPConstants.DELAY: best human play speed
        playGame(m_visibility, delay); //show graphics

        /////// 2. Executes one game.
        //int delay = 1;  //1: quickest; PTSPConstants.DELAY: human play speed, PTSPConstants.ACTION_TIME_MS: max. controller delay
//        runGame(m_visibility, delay);

        ////// 3. Executes N games (numMaps x numTrials), graphics disabled.
        //int numTrials=10;
        //runGames(numTrials);

    }





}
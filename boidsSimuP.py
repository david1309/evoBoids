"""
********************************************************************************
 Evolution of Boid behavior parameters: 

    * Evolution Parameters: 
        - Genotype: [neighRadii,maxVel,cohW,repW,alignW,crowdingThr] 
        - Fitness f(x): [No. interAgent collisions, averageOfMedianOfGroupRadii, meanDeltaTofArrival,t2Goal]

    * Behaviors [applied to (Casual Agents -> rainbow color, self.type == 0)]
        - Cohesion -> Moving closer towards center of Mass of boids neightborhood
        - Repulsion -> Move further, away of center of Mass of boids neightborhood
                   to avoid crowding local flockmates
        - Alignement -> " Move to Orientation: steer towards average heading 
                    " of local flockmates "

    * Special Agents:
        - Leader Following -> Boids have a strong atraction to a particular agent (Leader -> Blue, self.type>=1)
        %%%- Predator run away -> Boids run away from a particular agent (Predator -> Red, -99<=self.type<=-1)

    * Others:
        %%% Obstacle avoidance -> Boids evade and split up to avoid runing into bostacle (Obstacle-> Green, self.type<=-100)
********************************************************************************
    Author: David C. Alvarez - Charris
    Contact: david13.ing@gmail.com

    Created on Tu Jan 19 13:58:35 2016
    Author: David C. Alvarez - Charris
    Contact: david13.ing@gmail.com

    Created on Mon Jan 18 19:09:45 2016
********************************************************************************
"""

import sys, pygame, random, math
import numpy as np
from datetime import datetime # for keeping time of algo runtime
import time # time.sleep([s]) OR raw_input("Press Enter to continue...")
import copy # used to copy list of objects content and NOT's its pointer

"GLOBAL VARIABLES"
pygame.init()
scenarioSize = width,height = 1350,700 #scenario dimensions
scenario = 0 # Only used to stablish it as a global variable

velBoostORI = 0 # Only used to stablish it as a global variable
casualAgentR = 10
huntPersistOri = 15

" Class containing all the Boid behaviors "
class Boid: 

    def __init__ (self, type, strip, maxVel):
        # Casual Agent : set pos/vel randonmly ; Special Agents: pos == center of screen and vel == maxSpeed %
        self.x = random.randint(0,width)#*(type==0) + (width)*(type!=0)
        self.y = random.randint(0,height)#*(type==0)  + (height/2)*(type!=0)
        self.velX = float(random.uniform(-maxVel,maxVel)) 
        self.velY = float(random.uniform(-maxVel,maxVel))      
        self.type = type

        # This attributes are ONLY used by the PREDATOR agent (correct  programming 
        # practice would be to create a child class inheriting'Boid' ... but :| :|) 
        self.velBoost = velBoostORI 
        self.neighPredator = [] # list which stores predators preys  
        # Max Val -> Predator hunts ; ZeroVal -> abandons hunting and re-starts random exploration
        self.huntPersist = huntPersistOri
        

    " Compute (Euclidian) distance from self - to - 'boid' "
    def distance (self,boid):
        distX = self.x - boid.x
        distY = self.y - boid.y
        return math.sqrt(distX*distX + distY*distY)


    " Move closer: towards center of Mass of boids neightborhood"
    def cohesion(self,boids,cohW,leaderW,predatorW):
        # Average distnace variables
        avgX = 0
        avgY = 0

        # comment at: C1!? ->When to many agents follow the leader, the agents start
        # having more "cohesion force" than the leader. They end up prefering being 
        # together over following the leader. This command forces the increment of the leaders 
        # "cohesion force" so that he can continue leading, despite the number of agents on the group

        for boid in boids:
            fact = 1.0 # used to modify Special Agents "cohesion force"
            if boid.type > 0: fact = leaderW #  leader positive reinforcement
            # if len(boids)>10 and type>0: fact = leaderW*leaderW # <-- C1!?                               
            elif boid.type < 0: fact = 1.0/predatorW # predator negative reinforcement

            # Tab this 2 lines under the first 'if' and increase cohFact >=2 0 to have agents only follow LEADER
            # Calculate average distances to other boids
            avgX += (self.x - boid.x)*fact
            avgY += (self.y - boid.y)*fact            
                       
        # As an analogy, this calculation is  like obtaining the center of mass (x,y) 
        # among the neighborhood members, BUT w.r.t. to the boid of interest (self)
        # current position (not the CoM with respect to the abosolute zero) ...        
        avgX /= float(len(boids)) 
        avgY /= float(len(boids))  

        # ... and then pushing the boid of interest (self) towards the Center of mass 
        # (CoM) to produce the cohesion behavior. The direction (sign of dispalcement)
        # in which the boid has to be moved to approach the CoM is computed ->
        # currentPos = (currentPos - CoM) , for its X and Y position
        self.velX -= (avgX*cohW) 
        self.velY -= (avgY*cohW) 


    " Move further: away of center of Mass of boids neightborhood "
    " to avoid crowding local flockmates"
    def repulsion(self,boids,repW,crowdingThr,leaderW,predatorW):
        # Vector which added to currentPosition moves the boid away from those near it
        escapeTX = 0 # X - escape Trajectory
        escapeTY = 0 # Y - escape Trajectory
        isCrowed = 0 # bool: 0-> No agent extremely close ; 1-> agents to close
        
        # comment at: C2!? -> When there are to many agents in the leaers cluster,
        # make agents repude the Leader from a greater radii

        # Check for crowded agents
        for boid in boids:
            distance = self.distance(boid)

            fact = 1.0 # used to modify Special Agents "repulsion force"
            if boid.type > 0: fact = 1 #leaderW # decreases radii at which leader is repulsed
            # if len(boids)>10 and type>0: fact /=100 # <-- C2!?
            elif boid.type < 0 and boid.type>=-99 : fact = 1.0/predatorW  # increases radii at which predator is repulsed
            elif boid.type<-99 : fact = 1.0/avoidW  # increases radii at which obstacle is avoided

            if distance < crowdingThr/fact:
                isCrowed = 1          
                # Accumulating directions (and mag.) of Escape trajectory vector
                escapeTX += (self.x - boid.x)
                escapeTY += (self.y - boid.y)

        # Compute new velocity   
        if isCrowed:
            self.velX += (escapeTX*repW) 
            self.velY += (escapeTY*repW) 


    " Move to Orientation: steer towards average heading of local "
    " flockmates "
    def alignment(self,boids,alignW):
        #Calculate average speed of the neighboring boids
        avgX = 0
        avgY = 0

        for boid in boids:  
            fact = 1.0 # used to modify Special Agents "ralignement force"
            
            avgX += boid.velX*fact
            avgY += boid.velY*fact  

        avgX /= float(len(boids)) 
        avgY /= float(len(boids))  

        self.velX += (avgX*alignW) 
        self.velY += (avgY*alignW) 


    "Perform movement: based on boids velocity (determined by the behavioral rules)"
    def move(self,maxVel,maxDeltaVel,visualON):
        
        if self.type>1 : # increase special autonomous agent speed            
            self.velX += random.uniform(-maxDeltaVel,maxDeltaVel)
            self.velY += random.uniform(-maxDeltaVel,maxDeltaVel)

        # for each timeStep of evaluating the rules, theres a maximum (saturation) displacement
        # value (boids cant move instantly from one place to the other, its got to me smooth)
        if  abs(self.velX) > maxVel or abs(self.velY) > maxVel:
            if self.type<-1: maxVel = maxVel*self.velBoost
            scaleFactor = maxVel / max(abs(self.velX), abs(self.velY))
            self.velX *= scaleFactor
            self.velY *= scaleFactor

        # Update Agents Position
        # Check on visualON, because for mouse position capture the video system (visual simulation) must be ON
        if ( (self.type == 1 or self.type == -1) and visualON): # Controlled Special Agents
            self.x,self.y = pygame.mouse.get_pos()            
        else:  # Autonomous casual agents  && Autonomous special agents
            self.x += self.velX
            self.y += self.velY

        # Ensure boids  stay within the screen space -> if they are to close to 
        # the border, then change the Direction of the velocity vector, and change
        # its magnitud by a random factor between [0.0,1.0]

        # IMPORTANT: if the the 'border' is to big, casuaal agents might take cover on 
        # the borders and when the predator attemps to hunt them, the border repulsion 
        # wont let him reach its prey. Causing the predator to never hunt the casual 
        # agent and stay "infinetely" bumping against the border

        border = casualAgentR # how close from the borders does one wants the boids to get
        bordFact = 10000.0
        if self.x < border and self.velX< 0: self.velX = -self.velX*random.random()*bordFact #generates val in [0.0,1.0]
        if self.x > (width - border*2) and self.velX> 0: self.velX= -self.velX* random.random()*bordFact
        if self.y < border and self.velY < 0: self.velY = -self.velY * random.random()*bordFact
        if self.y > (height - border*2) and self.velY > 0: self.velY = -self.velY * random.random()*bordFact 


    "Exclusive predatorAgent method: based on the preys he's got on sight, compute predators displacement"
    def predatorHunt(self,maxDeltaVel,huntCoh,deltaBoost,velBoostORI):          
        # if NO prey in sight --> random exploration behavior
        # If huntPersist reaches minumum level (0), Predator surrenders and activates --> random exploration behavior
        if not self.neighPredator or not self.huntPersist:
            self.velX += random.uniform(-maxDeltaVel,maxDeltaVel)
            self.velY += random.uniform(-maxDeltaVel,maxDeltaVel)

        # if  prey in sight --> "hunt centroid"
        else:
            # Compute preys centroid
            avgPos = np.zeros((1,2)) # groups average position, useful to compute centroid
            # Obtain Number of Agents inside a percentage of the groups position dispersion
            avgPos[0,0] = sum([prey.x for prey in self.neighPredator ])
            avgPos[0,1] = sum([prey.y for prey in self.neighPredator ])

            GCentroid = avgPos/len(self.neighPredator) # preyGroup centroid 
            GCentroid = GCentroid.astype(int).tolist()# cast from float to int, and then to python list
            preysCentroid = Boid(0,0,0) # trick used to be able to recycle cohesion method of Boid Class
            preysCentroid.x = GCentroid[0][0]
            preysCentroid.y = GCentroid[0][1]
            preysCentroid = [preysCentroid] # transform into iterable object (list)

            # Move predators to preys centroid
            self.cohesion(preysCentroid,huntCoh,0,0)

        # Update predators speed boost based on number of preys
        # if numOfPreys different from zero, the linear equation works (if not, force it to original value)
        if self.neighPredator: self.velBoost = -deltaBoost*len(self.neighPredator) + (velBoostORI+deltaBoost)
        else: self.velBoost = velBoostORI  
        self.velBoost = self.velBoost*(self.velBoost>=0) + 0*(self.velBoost<0)

        # When there are few preys, predator tends to "bounce arround" these preys and 
        # never hunt them --> Reduce velBoost propotionally to the time (persistance level)
        # spent hunting them. Providing the predator a more fine/precise attack movement.
        # Increasing its chances to hunt the preys and get out of the "bounce arround" loop
        if len(self.neighPredator) <= 2:
            persistBoost = (self.huntPersist*1.15/huntPersistOri)
            if persistBoost > 1.0: persistBoost = 1.0
            self.velBoost *= persistBoost

" Main Code "       
def main(argv): 
    
    "Simulation  Parameters: neighRadii, cohW, repW, alignW, crowdingThr, leaderW, predatorW "    
    if len(argv)>1:
        neighRadii = float(argv[1]) # [150.0]Vecinity which each agents checks to apply behavioral rules
        crowdingThr = float(argv[5]) # [10.0]agents proximity before activation of "repulsion behavior"

        # Behavioral weights      
        cohW = float(argv[2]) #  [4.0/100] for Cohesion 
        repW = float(argv[3]) # [23.0/100] for Repulsion 
        alignW = float(argv[4]) # [20.0/100] for alignment

        # Special Agents weights
        leaderW = float(argv[6]) # 10
        predatorW = float(argv[7])  # 12
        # Auxilliary Var's
        visualON = float(argv[-2]) # [1] 0: dont show visual environment ; 1: show visual environment
        timeOutT = float(argv[-1]) # [15] time in [s] after which simlation will abort 
    else: 
        print " ERROR: Please provide the simulations parameters"
        sys.exit()

    "Scenario Visualization Parameters"
    if(visualON): 
        global scenario
        scenario = pygame.display.set_mode(scenarioSize) # Configure pygame window  
        pygame.display.set_caption('Boids Simulation')
        sceneColorOri = (0,100,160) # Initial background color
        sceneColor = [sceneColorOri[0],sceneColorOri[1],sceneColorOri[2]] # stores current BG color
        deltC = 15 # how much does the BG color change on each interation
        varC = [1,0,0] # variable used to increase/decrase each [R,G,B] color

        # Time after casual agents extinction that hte game will shut down
        endGameEllapsedT = 0 # DONT change THIS
        endGameTimeOutT = 5
        stableC = 0 # if BG color is at desired value, STOP changing it w/ f(x)'updateBG()'
    
    " General Parameters"
    noBoids = 15 # Number of casualAgents
    noAutoPred = 2 # Number of Autonomous Predators
    noAutoLead = 3 # Number of Autonomous Leaders
    # Create a controlled 0: Nothing ; 1: leaderAgent ; 2: predatorAgent
    contSpecial = 1

    specialRFact = 1.0 # = max(scenarioSize) #= 8 Factor which increases the Radii at which Special Agents are considerder neighbors 
    maxVel = 12.0  # Max. delta speed that can be performed on each time step

    " Predator Parameters "
    predSight = 150
    neightPredatorTemp = []
    predKillZone = casualAgentR*3
    huntCoh = 4.0/100 # How attracted is the predator towards the preysCentroid

    # Special Agent speed boost is proportional to num of preys in predSight
    # y = m.X + b -> velBoost = -deltaBoost*numPreyINSight + (velBoostORI+deltaBoost)
    ## velBoost
    velBoostORI = 1.8
    deltaBoost = 0.2
    # when Special Agents are automous (controlled by code), determines the maximum change in speed at each time step
    maxDeltaVel = 7.0
    endGame = 0 # 0: Game continues; 1: End of game
    endGameT = 0

    " Leader Parameters "
    reviveZone = predKillZone# Radii in which leader agent will revive dead casual agents
    
    " Auxilliary Variables "
    strip = width # Strip among which the init.pos of casual agent is going to be randomlly selected
    ellapsedT = 0 # simulation run time
    numIter = 0 # Number of positions updates   
    casualAgentR # Artificial "Size" of casual agents and Radii among which a collision is determined
    
    " Other params (previously used for evolution "
    goalRadii = int(math.ceil(neighRadii))
    boidsNotATgoal = []
    # avgPos = np.zeros((1,2)) # groups average position, useful to compute centroid


    "Special Agents and obstacles params. [Corresponds to self.type != 0]"
    # predatorW = 12# = 12.0 # How much more repulsion will predator have
    # avoidW = 16.0 # How much more repulsion (how far away) will agents evade obstacles
    # 0: Created both, leader/predator Auto.Agents ; 1/-1: Creadte Auto.leader/Auto predator agents
    # autoAgents = 0 
    # lists that store controled and special agent object instances (needed in order to delete them when needed) 
    # specialAgent = []
    # avoidPos = [] # list storing the position obstables

    " Create Agents"
    # Initialize casualAgents boids, with a random X,Y position  
    boids = [] # list containing all the agents
    for i in range(noBoids):
        boids.append(Boid(0,strip,maxVel)) # casual agent 

    # Initialize predatorAgents (AUTONOMOUS) 
    predatorAgents = []  
    for i in range(noAutoPred):
        predatorAgents.append(Boid(-2,0,maxVel))
        boids.append(predatorAgents[-1]) # predator agent  

    # # Initialize leaderAgents (AUTONOMOUS) 
    leaderAgents = []
    for i in range(noAutoLead):
        leaderAgents.append(Boid(2,0,maxVel))
        boids.append(leaderAgents[-1]) # predator agent

    # Initialize Special Agents (CONTROLLED) 
    if contSpecial== 1: # Controlled Leader
        leadContAgent = Boid(1,0,maxVel)
        leaderAgents.append(leadContAgent)
        boids.append(leadContAgent) # Add to list saving all Boid instances 
    
    elif contSpecial== 2:# Controlled Predator
        predContAgent = Boid(-1,0,maxVel)
        predatorAgents.append(predContAgent)
        boids.append(predContAgent) # Add to list saving all Boid instances 

    # stores casualAgents that are (still) alive --> copy objects not pointers, to be able 
    # to remove elements from the copy wo/affecting the original list of objects)
    livingBoids = copy.copy(boids) 
    huntedBoids = [] # stores casualAgents that have been hunted by 
    numSpecial = sum([1 for boid in livingBoids if(boid.type != 0)]) # Number of SpecialAgents (leader || predator) 

    " MAIN LOOP "
    while endGameEllapsedT <= endGameTimeOutT:
        numIter += 1
        if numIter == 2:
            # First time pause for user to visualize simulation parameters
            # if(visualON): 
                # if(numIter == 2): raw_input("... Press enter to start simulation ...")
            # Capture Initial start time of simulation
            initT = datetime.now().time()
        elif numIter > 1:
            currentT = datetime.now().time()
            ellapsedT = (currentT.hour - initT.hour)*3600.0 + (currentT.minute-initT.minute)*60.0 + (currentT.second-initT.second)
    
        # Reset " per iteration " Evo. params
        # interCollV_PerIter = 0
        # avgPos[:] = 0
        # collidedAgents = []

        # Check for any event which exits the simulation
        if(visualON):
            for event in pygame.event.get():
                if event.type == pygame.QUIT: 
                    pygame.display.quit()
                    sys.exit()
         
        " Compute neighbooring boids and apply ALL behaviors " 
        for boid in livingBoids :# for each boid

            # RESET Neighborhood variables
            neighBoids = []# list which stores boid neighboors
            neighPredatorTemp = []# list which temporarelly stores predators preys

            # CHECK: boid's distance with the other boids
            for otherBoid in boids:
                # Dont check the boids distance with its self                
                if otherBoid == boid: continue                
                dist2neigh = boid.distance(otherBoid)

                fact = 1.0
                # if the otherBoid is a SpecialAgent, change the radii at which he will
                # be considered a neighbor 
                if otherBoid.type: fact = specialRFact

                " Evaluate casualAgents neighborhood and Collisions (only w/those alive otherBoid) "
                if (dist2neigh < neighRadii*fact) and not boid.type and (otherBoid in livingBoids):                   
                    neighBoids.append(otherBoid) # add to neighbors list

                    # "EVO.FITNESS -> Inter Agent Colission"
                    # # if the otherAgent is in collision w/ the the boid                                         
                    # if (dist2neigh < casualAgentR*2 and not otherBoid.type):
                        
                    #     if( (otherBoid,boid) not in collidedAgents):                     
                    #         interCollV += 1
                    #         interCollV_PerIter += 1
                    #         collidedAgents.append((boid,otherBoid)) 

                " Evaluate predatorAgent close preys, ONLY those 'otherBoids' that are casual agents and alive (obviously) "
                if (boid.type<0 and boid.type>=-99) and (dist2neigh < predSight) and (not otherBoid.type) and (otherBoid in livingBoids) and (not endGame):
                    if(dist2neigh < predKillZone): 
                        livingBoids.remove(otherBoid) # update survivors
                        huntedBoids.append(otherBoid) # update casualties                        
                    else: neighPredatorTemp.append(otherBoid)   

                " Evaluate AUXILLIARY leaderAgent closeness toReviveAgents, ONLY those 'otherBoids' that are dead (obviously) "
                if (boid.type>=2) and (dist2neigh < reviveZone) and (otherBoid in huntedBoids) and (not endGame):
                    livingBoids.append(otherBoid) # update revived casualAgents
                    huntedBoids.remove(otherBoid) # update casualties
             
            
            if (boid.type<0 and boid.type>=-99):

                # If predators is  trying to hunt the same casualBoids gradually reduce its persistance level 
                #(ignore the definition of "same" when no preys are being hunted i.e. []) 
                deltaPersit = 0.1
                if neighPredatorTemp == boid.neighPredator and neighPredatorTemp:
                    if boid.huntPersist - deltaPersit >= 0.0: boid.huntPersist -= deltaPersit # if it isnt already zero, reduce it         
                    boid.huntPersist  = round(boid.huntPersist,2)
                # if predator finds a different group of preys, re-activate hunting behavior
                else:
                    boid.huntPersist = huntPersistOri 

                # Update predators list of preys
                boid.neighPredator = neighPredatorTemp


            # # CHECK: neighboring obstacles
            # neighAvoid = [] # list which stores obstacle neighboors
            # for avoid in avoidPos:                
            #     dist2obs = boid.distance(avoid)

            #     if dist2obs < neighRadii:
            #         neighAvoid.append(otherBoid) # add to list

            " Apply the three behaviors based on current neighborhood (if existant) "
            if neighBoids and not boid.type : # Only apply rule to Casual Agents
                boid.cohesion(neighBoids,cohW,leaderW,predatorW)
                boid.repulsion(neighBoids,repW,crowdingThr,leaderW,predatorW)
                boid.alignment(neighBoids,alignW)

                # if neighAvoid: # if current boid has some near obstacle, apply repulsion behavior to it
                #     boid.repulsion(avoidPos,repW)  

            " Apply predatorHunting behavior based on predators current preys "
            if boid.type<-1 and boid.type>=-99: # Only apply rule to Predator Agents
                boid.predatorHunt(maxDeltaVel,huntCoh,deltaBoost,velBoostORI)

        # avgPos[0,0] = sum([boid.x for boid in boids if (boid.type == 0)])
        # avgPos[0,1] = sum([boid.y for boid in boids if (boid.type == 0)])

        # ## Dont use len(boids) as the averaging number since it will mistakenly include the Special Agents
        # GCentroid = avgPos/noBoids # Groups centroid
        # GCentroid = GCentroid.astype(int).tolist()# cast from float to int, and then to python list
        # centroidAgent = Boid(0,0,maxVel) # trick used to be able to recycle Euclidian distance to boids method of Boid Class
        # centroidAgent.x = GCentroid[0][0]
        # centroidAgent.y = GCentroid[0][1]  #   # Obtain Number of Agents inside a percentage of the groups position dispersion
        

        # " EVO.FITNESS ->  Measurement of Group Compactness:"
        # ## Obtain groups dispersion  
        # dist2Centroid = [centroidAgent.distance(boid) for boid in boids if (boid.type == 0)]

        # medGRadiiV_PerIter = np.median(np.array(dist2Centroid))
        # medGRadiiV += medGRadiiV_PerIter


        # " EVO.FITNESS ->  Measurement of groups arrival time:"
        if numIter == 1: 
            boidsNotATgoal = [boid for boid in boids if(boid.type==0)]
            # check if the leaderControlledAgents ¿ EXIST's ?
            leadContExist = ('leadContAgent' in locals() or 'leadContAgent' in globals())
        elif leadContExist: 
            boidsNotATgoal = [boid for boid in boids if(leadContAgent.distance(boid) >= goalRadii)]
            # boidsAtgoal = [boid for boid in boids if(boid not in boidsNotATgoal)]

        # numATgoal = noBoids - len(boidsNotATgoal) 
        # if (numIter-iterINgoal) > iterWindow and iterINgoal: 
        #     iterINgoal = numIter
        #     countAvg += 1
        #     avgNumATgoal += numATgoal

        # if (numATgoal-numATgoalPast) > 0:  # >0 to only count deltas when agents are going into the goal, not leaving it
        #     numATgoalPast = numATgoal
        #     pastTGoal = currentTGoal
        #     currentTGoal = datetime.now().time()

        #     if firstGoal: 
        #         iterINgoal = numIter
        #         firstGoal = 0
        #     else:     
        #         deltaTV_PerIter = (currentTGoal.hour-pastTGoal.hour)*3600.0*factUS + \
        #                           (currentTGoal.minute-pastTGoal.minute)*60.0*factUS + \
        #                           (currentTGoal.second-pastTGoal.second)*1.0*factUS + \
        #                           (currentTGoal.microsecond-pastTGoal.microsecond)
        #         deltaTV += deltaTV_PerIter

        # if numATgoal == noBoids and not evoSuccess:
        #     currentT= datetime.now().time()
        #     t2GoalV = (currentT.hour-initT.hour)*3600.0 + (currentT.minute-initT.minute)*60.0 + (currentT.second-initT.second)*1.0
        #     evoSuccess = 1 

        " End of game desicion"
        # If ALL casual agents have been hunted == GAME OVER
        if not (len(livingBoids) - numSpecial) and not endGame:
            endGameT = currentT
            winner = 0 # Predator wins            

        # If the TimeOut time is reached == GAME OVER
        elif ellapsedT>timeOutT and not endGame:
            endGameT = currentT       
            winner = (len(livingBoids) - numSpecial)>len(huntedBoids) # 0: Predator wins ; 1: Leader wins
            
        if endGameT :   
            if not endGame:          
                sceneColor = [255,255,255]#*(winner) + [255,255,255]*not(winner)
                sceneColorOri = (30,245,170)*(winner) + (173,15,15)*( not winner)
            endGame = 1
            endGameEllapsedT = (currentT.hour - endGameT.hour)*3600.0 + (currentT.minute-endGameT.minute)*60.0 + (currentT.second-endGameT.second)
                   
        " Simu. Visualization: Update virtual boids display "
        # visualON determines if the simulation will show the visual environment
        if (visualON):       
            # Draw obstacles
            # if avoidPos: 
            #     for pos in avoidPos:
            #         pygame.draw.circle(scenario,(255,255,255),(pos.x,pos.y),9,0)  
            #         pygame.draw.circle(scenario,(0,255,0),(pos.x,pos.y),7,0)        # Update Dynamic Color Background visualization    
            
            # paint BG to erase past objects
            
            if endGame: 
                sceneColor = updateBG(sceneColor,sceneColorOri,deltC,varC)[0]
                if sceneColor[0] == sceneColorOri[0] or stableC:
                    stableC = 1
                    sceneColor[0] = sceneColorOri[0]
            scenario.fill(sceneColor) # fill scenario with RGB color

            "Insert Simulation parameters on screen scenario"  
            # PARAMETERS: txts,pos,scenario,fontSize = 15,color = (255,255,255),font = 'freesansbold.ttf' 

            # Living VS Hunted TEXT
            fontSize = 20
            color = (255,128,0)
            pos = (80,50)
            scoreP = "Hunted : " + str(len(huntedBoids))               
            drawTXT ([scoreP],pos,fontSize,color)

            pos = (80,80)
            color = (0,200,255)
            scoreL = "Living : " + str((len(livingBoids) - numSpecial))               
            drawTXT ([scoreL],pos,fontSize,color)

            # Num of Special Agents
            color = (255,128,0)
            pos = (400,50)
            numPredTXT = "Num.Predators : " + str(len(predatorAgents))            
            drawTXT ([numPredTXT],pos,fontSize,color)
            
            color = (0,200,255)
            pos = (400,80)
            numLeadTXT = "Num.Leaders : " + str(len(leaderAgents))
            drawTXT ([numLeadTXT],pos,fontSize,color)
            
            # Simulation StopWatch TEXT
            fontSize = 65      
            pos = (240,60)
            font_path = 'digital-7.ttf'
            if (not endGame): stopWatch = (timeOutT-ellapsedT)
            stopWatchTXT = str(int(stopWatch))
            color = (128,255,0)
            if stopWatch < 10: color = (255,0,0)
            elif stopWatch < 0.5*timeOutT: color = (255,200,51)
            drawTXT ([stopWatchTXT],pos,fontSize,color,font = font_path)


            # Simulation Parameters text
            if not endGame:                             
                neighRadiiTXT = "neighRadii : " + str(int(neighRadii))
                cohWTXT = "cohW : " + str(int(cohW*100)) + "%"
                repWTXT = "repW : " + str(int(repW*100)) + "%"
                alignWTXT = "alignW : " + str(int(alignW*100)) + "%"
                crowdingThrTXT = "crowdingThr : " + str(int(crowdingThr))
                leaderWTXT = "leaderW : " + str(int(leaderW))
                predatorWTXT = "predatorW : " + str(int(predatorW))
                geneRangeTXT = [neighRadiiTXT,cohWTXT,repWTXT,alignWTXT,crowdingThrTXT,leaderWTXT,predatorWTXT] 
                
                # Concatenate velboost and huntPersist for EACH of the predatorAgents
                predatorTXT = ["velBoost : " + str(float(round(pred.velBoost,2))) + "  << >>  huntPersist : " + str(float(pred.huntPersist)) for pred in predatorAgents]            
                geneRangeTXT = geneRangeTXT + predatorTXT
                
                pos = (int(width/2),int(height/2))
                drawTXT (geneRangeTXT,pos)
                                
            # Display end of Game TEXT
            else:
                if winner: endGameTXT = ":D Game Over: Leader Wins :D"
                else: endGameTXT = ":@ Game Over: Predator Wins :@"

                fontSize = 30
                pos = (int(width/2),int(height/2))                
                drawTXT ([endGameTXT],pos,fontSize)
                
            " Draw Groups Centroid,  Compactness Radii (Median Radii) "
            # widthCirc = 3  
            # groupC = (GCentroid[0][0],GCentroid[0][1])
            # if(medGRadiiV_PerIter>widthCirc):        
            #     pygame.draw.circle(scenario, (255,255,255), groupC, 6, 0)
            #     pygame.draw.circle(scenario, (255,255,255), groupC, int(medGRadiiV_PerIter), widthCirc)

            " Draw interval where casual voids are born "
            # pygame.draw.line(scenario,(255,0,255),(strip,0),(strip,height),4)

            " Update Boids visualization "
            for boid in boids:
                if boid.type > 0 : # Leaders              
                    if boid.type == 1:pygame.draw.circle(scenario, (0,255,0),(int(boid.x), int(boid.y)), 14, 0)
                    if boid.type > 1: pygame.draw.circle(scenario, (0,255,255),(int(boid.x), int(boid.y)), 14, 0)
                    pygame.draw.circle(scenario, (0,0,255),  (int(boid.x), int(boid.y)), 10, 0)

                    # Draw Goal Radii
                    # pygame.draw.circle(scenario, (0,255,0), (int(boid.x), int(boid.y)), goalRadii,8)
                
                elif boid.type < 0 :  # Predators         
                    pygame.draw.circle(scenario, (255,128,0),(int(boid.x), int(boid.y)), 14, 0)
                    pygame.draw.circle(scenario,(255,0,0), (int(boid.x), int(boid.y)), 10, 0)

                    # Draw neighPredator radii
                    # pygame.draw.circle(scenario, (255,0,0), (int(boid.x), int(boid.y)), predSight,8)

                else: # Casual Agents 

                    if boid in huntedBoids:
                        agentColor = (0,0,0)                     
                        pygame.draw.circle(scenario, agentColor, (int(boid.x), int(boid.y)), int(casualAgentR),0)
                        colorRing = (225,0,0) 
                        pygame.draw.circle(scenario, colorRing, (int(boid.x), int(boid.y)), int(casualAgentR),int(casualAgentR/2))
                    
                    elif boid in livingBoids:
                        # Make casual agents blink w/ rainbow like colors + Draw its body + Draw its outer layer
                        agentColor = (random.randint(0,240),random.randint(0,240),random.randint(0,240))                     
                        pygame.draw.circle(scenario, agentColor, (int(boid.x), int(boid.y)), int(casualAgentR),0)
                        
                        # Check if current casualAgent ('boid') is prey of ANY of the predatorAgents
                        isHunted = []                         
                        isHunted = [1 for pred in predatorAgents if boid in pred.neighPredator]
                        if isHunted : colorRing = (255,128,0) # Boid in hunting zone

                        elif boid not in boidsNotATgoal: colorRing = (0,180,255) # Boids in leaderAgents protected zone
                        else:  colorRing = (255,255,255) # Any where in the arena
                        pygame.draw.circle(scenario, colorRing, (int(boid.x), int(boid.y)), int(casualAgentR),int(casualAgentR/2))
                    
                    else: print "fuck!!!, theres a Limbo type error"

                    # # Draw distance to groups centroid line
                    # pygame.draw.line(scenario, (255,255,255), groupC, (int(boid.x), int(boid.y)), 2)
                    
                    # # Draw neighboord radii
                    # pygame.draw.circle(scenario, (0,255,0), (int(boid.x), int(boid.y)), int(neighRadii),2)

                    # # Draw collision radii
                    # pygame.draw.circle(scenario, (255,0,0), (int(boid.x), int(boid.y)), int(crowdingThr),2)

                    # # Draw special agent detection radii
                    # pygame.draw.circle(scenario, (255,255,0), (int(boid.x), int(boid.y)), int(neighRadii*specialRFact),2)

            pygame.display.flip()
            pygame.time.delay(10)

        # print " ***************** Colissions ****************"
        # print interCollV_PerIter
        # print " ****************** Med Rad ******************"
        # print medGRadiiV_PerIter # --> insideRadV_PerIter
        # print " ******* numAgentsAtGoal and deltaTime *******"
        # if numATgoal:
        #     print numATgoal
        #     print deltaTV_PerIter/factUS
        #     print "\n"*2

        # userCommand = raw_input("Press Enter to continue...")
        # if(userCommand):
        #     pygame.display.quit()
        #     sys.exit() 

        " RUN/PAUSA  SIMULATION WITH MOUSE CLICK'S "

        # mouse = pygame.mouse.get_pressed() 

        # if mouse[0]: pygame.time.delay(150) # pass
        # else:
        #     print "PAUSED"
        #     while not(mouse[0]):
        #         for event in pygame.event.get():
        #             if event.type == pygame.QUIT: 
        #                 pygame.display.quit()
        #                 sys.exit()
        #         mouse = pygame.mouse.get_pressed()
        #         if mouse[2]:
        #             print "EXIT"
        #             pygame.display.quit()
        #             sys.exit()
                     
        " Update boids position vector "
        [boid.move(maxVel,maxDeltaVel,visualON) for boid in livingBoids] # if (abs(boid.type) == 0 )]  

    " Simulation Exit "    
    # if (evoSuccess):
    #     print "\nEnd of Simulation: Success, CLEAN EXIT\n"
    # else:
    #     print "\nEnd of Simulation: TIME OUT\n"
    # pygame.time.delay(3500)

    pygame.display.quit()
    # print "Before Normalizing : \n "
    # print "interCollV: %d" %interCollV
    # print "medGRadiiV: %f" %medGRadiiV
    # print "deltaTV: %f" %deltaTV
    # print "t2GoalV: %f" %t2GoalV
    # print "numATgoal: %f" %numATgoal
    # print "evoSuccess: %f" %evoSuccess
    # print "INTER : %d" %numIter
    # raw_input()

    " Transform Fitness Parameters "
    # The selected fitness parameters are suited for a minimization algo. (i.e. lower value => higher fitness)
    # ... compute cocient param = 1/(x + 1)(adding to parameter value + 1, to prevent n/0 = NaN) to make them maximization parameters    
    # interCollV = 1.0/((interCollV*1.0/numIter)+1) # Average No. compactAgents over  No. of position updates (iterations)
   
    # # Average median raddi ver  No. of position updates (iterations)
    # avgGRadii = (medGRadiiV/numIter)
    # idealGRadii = 100
    # std = 40
    # medGRadiiV = 1*math.exp(- ((avgGRadii-idealGRadii)**2) / (2*std**2) )    
    # if math.isnan(medGRadiiV) : medGRadiiV = 0.0

    # if deltaTV == 0.0: deltaTV = 0.0
    # else: deltaTV = 1.0/( ((deltaTV/factUS)/(noBoids)) + 1)  # Average deltaTimes over  No. of arrivals to goal --> Precisely, it should ve over(numATgoal-1) , 
    #                                                             # since if N-agents are at goal there exist (N-1)-delta time intervals ... but its done '+1' to prevent NaN
   
    # # numATgoalV = numATgoal*1.0/noBoids    
    # if countAvg != 0: avgNumATgoalV = (avgNumATgoal*1.0/countAvg)*1.0/noBoids
    # # if iterations at the end of the simu. arent sufficiente for a complete iterWindow
    # # to occur. Simply compute the avg, as the current number of agets at goal
    # else: avgNumATgoalV = numATgoal*1.0/noBoids 
    # # print avgNumATgoalV*noBoids
    
    # # evoSuccessV = evoSuccess
    # # print "After Normalizing : \n "
    # # print "interCollV: %f" %interCollV
    # # print "medGRadiiV: %f" %medGRadiiV
    # # print "deltaTV: %f" %deltaTV
    # # print "t2GoalV: %f" %t2GoalV
    # # print "numATgoalV: %f" %numATgoalV
    # # print "evoSuccessV: %f" %evoSuccessV
    # # raw_input()

    # fitVals = np.array([[interCollV, medGRadiiV, deltaTV,avgNumATgoalV]])
    # return fitVals


# " Class which initializes obstacles objects, in a similar way to boid objects."
# " In order to be able to reuse class Boid functions, and present the obstacles"
# " as a variation of a predator"
# class Avoid:
#     def __init__ (self,pos):
#         self.x = pos[0]
#         self.y = pos[1]
#         self.type = -100 # obstacles -> self.type <=-100

" Function which draws a list of text in the pygame window "

def drawTXT(txts,pos,fontSize = 15,color = (255,255,255),font = 'freesansbold.ttf' ):
    ## text characteristics
    txtColor = color
    stdFont = pygame.font.Font(font,fontSize)
    txtPos = pos

    ## texts rederings
    offSet = 0
    for txt in txts:
        txtSurf = stdFont.render(txt, True, txtColor)
        txtRect = txtSurf.get_rect()
        txtRect.center = (txtPos[0],txtPos[1]+offSet)
        offSet += 20
        scenario.blit(txtSurf,txtRect)

" Generates the sequence of step that enable the background to change color"
" Increases R -> then G -> then B all until 255 ; then decreases them in the same"
" order to the stablished original (base) color"
def updateBG(sceneColor,sceneColorOri,deltC,varC):
    sceneColor[0] = sceneColor[0] + random.randint(-deltC*(varC[0]==-1),deltC*(varC[0]==1))*(varC[0]==1 or varC[0]==-1)
    sceneColor[1] = sceneColor[1] + random.randint(-deltC*(varC[1]==-1),deltC*(varC[1]==1))*(varC[1]==1 or varC[1]==-1)
    sceneColor[2] = sceneColor[2] + random.randint(-deltC*(varC[2]==-1),deltC*(varC[2]==1))*(varC[2]==1 or varC[2]==-1)
    
    if(sceneColor[0]>255):
        sceneColor[0] = 255 
        varC[0] = 0
        varC[1] = 1
    if(sceneColor[1]>255): 
        sceneColor[1] = 255
        varC[1] = 0
        varC[2] = 1
    if(sceneColor[2]>255): 
        sceneColor[2] = 255
        varC[0] = -1
        varC[2] = 0
    if(sceneColor[0]<sceneColorOri[0] or sceneColor[0]<0 ): 
        sceneColor[0] = sceneColorOri[0]            
        varC[1] = -1
        varC[0] = 0
    if(sceneColor[1]<sceneColorOri[1] or sceneColor[1]<0): 
        sceneColor[1] = sceneColorOri[1]
        varC[2] = -1
        varC[1] = 0            
    if(sceneColor[2]<sceneColorOri[2] or sceneColor[2]<0): 
        sceneColor[2] = sceneColorOri[2]
        varC[2] = 0
        varC[0] = 1

    return sceneColor, varC
            
if __name__ == "__main__":
     main(sys.argv)



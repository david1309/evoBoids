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
import socket # TCP/IP python library
import getPose

"GLOBAL VARIABLES"
pygame.init()
scenarioSize = width,height = 1280,700 #1350,700 # Virtual scenario dimensions
realWidth, realHeight = 340,280 # Physical scenario dimensions
scenario = 0 # Only used to stablish it as a global variable
casualAgentR = 8

velBoostORI = 0 # Only used to stablish it as a global variable
persistORI = 0 # Only used to stablish it as a global variable

" Class containing all the Boid behaviors "
class Boid: 

    def __init__ (self, type, strip, maxVel):

        # Position and Velocity Attributes
        if(gamePlay == 0):
            self.x = random.randint(0,width)
            self.y = random.randint(0,height)
        elif(gamePlay == 1):
            self.x = (width)*(type==1 or type == -2) + (0)*(type>2 or type==0)
            self.y = (height/2)*(type==1) + random.randint(0,height)*(type!=1)

        self.velX = float(random.uniform(-maxVel,maxVel)) 
        self.velY = float(random.uniform(-maxVel,maxVel))      

        # Type Attribute: 
        # 0: casualAgent ; 1: controlledLeaderAgent ; >1 : autonomousLeaderAgent (assistant)
        # -1: controlledPredatorAgent ; <-1 && >-99: autonomousPredatorrAgent
        # <-99: obstacle
        self.type = type

        " This attributes are ONLY used by the PREDATOR agent"
        # (correct  programming practice would create a child class inheriting'Boid' ... but :| :|) 
        
        # Agents Neighborhood Attribute:
        self.neighAgent = [] # list which stores the otherAgents that the currentAgents sees 
        
        # Velocity Boost Attribute:
        # Predators Agent speed boost is proportional to num of preys in predSight
        # y = m.X + b -> velBoost = -deltaBoost*numPreyINSight + (velBoostORI+deltaBoost)
        self.velBoost = velBoostORI 
                
        # Persistances Level Attribute:
        # Max Val -> Predator/assitant hunts/revives ; ZeroVal -> abandons hunting/revival and re-starts random exploration
        self.persistLevel = persistORI

        # Random Exploration count Attribute:
        # counts for how much "time" is predator in Random Exploration behavior
        # when its value reaches randMAX, triggers agents instant "GlobalVision" of the scenario
        self.randCount = 0 
        

    " Compute (Euclidian) distance from self - to - 'boid' "
    def distance (self,boid):
        distX = self.x - boid.x
        distY = self.y - boid.y
        return math.sqrt(distX*distX + distY*distY)


    " Move closer: towards center of Mass of boids neightborhood"
    def cohesion(self,boids,cohW,leaderW,predatorW,guiderW):
        # Average distnace variables
        avgX = 0
        avgY = 0
        factORI = 1.0

        # when self == guiderAgent, gives extra attraction to casualBoids
        if self.type == 3:factORI = guiderW
        for boid in boids:
            fact = factORI # used to modify Special Agents "cohesion force"
            if boid.type == 1: fact = leaderW # controlledLeader extra-attraction
            elif boid.type == 2 : fact = leaderW/2.0 #  assistAgent extra-attraction
            elif boid.type == 3 : fact = leaderW*guiderW # guiderAgent extra-attraction
                        
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
            if boid.type > 0: fact = 1.0 #leaderW # decreases radii at which leader is repulsed
            # if len(boids)>10 and type>0: fact /=100 # <-- C2!?
            elif boid.type < 0 and boid.type>=-99 : fact = 1.0/predatorW  # increases radii at which predator is repulsed
            elif boid.type<-99 : fact = 1.0/(predatorW/2)  # increases radii at which obstacle is avoided

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
    def move(self,maxVel,maxDeltaVel,visualON,gravity):
        global pastPose # varirable that keeps track of the last position (capture by the tracking script)

        if self.type>1 : # increase special autonomous agent speed            
            self.velX += random.uniform(-maxDeltaVel,maxDeltaVel)
            self.velY += random.uniform(-maxDeltaVel,maxDeltaVel)

        # for each timeStep of evaluating the rules, theres a maximum (saturation) displacement
        # value (boids cant move instantly from one place to the other, its got to me smooth)
        if  abs(self.velX) > maxVel or abs(self.velY) > maxVel:
            if self.type==-2: maxVel = maxVel*self.velBoost
            scaleFactor = maxVel / max(abs(self.velX), abs(self.velY))
            self.velX *= scaleFactor
            self.velY *= scaleFactor

        # Update Agents Position
        # Check on visualON, because for mouse position capture the video system (visual simulation) must be ON
        if ( (self.type == 1 or self.type == -1) and visualON): # Controlled Special Agents
            # Capture Object pose -> tcpCapture (tcpobject, timeout [us]). Returns [-1,-1] for invalid pose
            pose = getPose.tcpCapture(s,500) 

            # Calibrate  Cam so that min/max coordVals are consistant with the min/max coordValss of the simu.l
            # Simply obtain min/max vals in X and Y  and introduce such values in the 'camX' and 'camY' vectors
            camX = [20.0,306.0] # X axis [minVal , maxVal] returned by Swisstrack (in camera IFR)
            mX = (-width*1.0)/(camX[1]-camX[0])
            bX = -mX*camX[1]

            camY = [112.0,276.0] # Y axis [minVal , maxVal] returned by Swisstrack (in camera IFR)
            mY = (height*1.0)/(camY[1]-camY[0])
            bY = -mY*camY[0]
       
            pose = [mX*pose[0]+bX , mY*pose[1]+bY] # Physical -> Virtual Dimension transformation
            
            if pose[1] >= 0 : pastPose = pose # update past pose if Valid pose
            else: pose = pastPose # Invalid Pose -> Dont alterate current boids position

            if gravity >=20:  # Make controlledAgents directly go to tracked position
                self.x,self.y = pose

            else: # Make controlledAgents move towards tracked position, in a 'gravity' medium                        
                movTowards = Boid(1,0,0) # trick used to be able to recycle cohesion method of Boid Class
                movTowards.x,movTowards.y = pose
                movTowardsW = 5.0/100
                self.cohesion([movTowards],movTowardsW,gravity,0,0)                    
                self.x += self.velX
                self.y += self.velY

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
    def move2centroid(self,maxDeltaVel,centerCoh,deltaBoost,velBoostORI, deltaRandCount):          
        # if NO prey in sight --> Random Exploration Behavior
        # If persistLevel reaches minimum level (0), Predator surrenders and activates --> random exploration behavior
        if not self.neighAgent or not self.persistLevel:
            self.velX += random.uniform(-maxDeltaVel,maxDeltaVel)
            self.velY += random.uniform(-maxDeltaVel,maxDeltaVel)

            self.randCount += deltaRandCount # Increase Random Exploration behavior count

        # if  prey in sight --> Move To Centroid Behavior
        else:    
            self.randCount = 0 # Once huntig is triggered , reset random behavior count 

            # Compute preys centroid
            avgPos = np.zeros((1,2)) # groups average position, useful to compute centroid
            # Obtain Number of Agents inside a percentage of the groups position dispersion
            avgPos[0,0] = sum([prey.x for prey in self.neighAgent ])
            avgPos[0,1] = sum([prey.y for prey in self.neighAgent ])

            GCentroid = avgPos/len(self.neighAgent) # preyGroup centroid 
            GCentroid = GCentroid.astype(int).tolist()# cast from float to int, and then to python list
            preysCentroid = Boid(0,0,0) # trick used to be able to recycle cohesion method of Boid Class
            preysCentroid.x = GCentroid[0][0]
            preysCentroid.y = GCentroid[0][1]
            preysCentroid = [preysCentroid] # transform into iterable object (list)

            # Move predators to preys centroid
            self.cohesion(preysCentroid,centerCoh,0,0,0)

        if self.type <= -1 and not G: # Only for predators
            # Update predators speed boost based on number of preys
            # if numOfPreys different from zero, the linear equation works (if not, force it to original value)
            if self.neighAgent: self.velBoost = -deltaBoost*len(self.neighAgent) + (velBoostORI+deltaBoost)
            else: self.velBoost = velBoostORI  
            self.velBoost = self.velBoost*(self.velBoost>=0) + 0*(self.velBoost<0)

            # When there are few preys, predator tends to "bounce arround" these preys and 
            # never hunt them --> Reduce velBoost propotionally to the time (persistance level)
            # spent hunting them. Providing the predator a more fine/precise attack movement.
            # Increasing its chances to hunt the preys and get out of the "bounce arround" loop
            if len(self.neighAgent) <= 2:
                # persistBoost might decrease to rapidlly. therefore a '*Factor' is used
                # to decrease the rate of change
                persistBoost = (self.persistLevel*1.15/persistORI) # 
                if persistBoost > 1.0: persistBoost = 1.0 # Upper saturation Limit
                elif persistBoost <0.5: persistBoost = 0.5 # Lower saturation Limit
                self.velBoost *= persistBoost

" Main Code "       
def main(argv): 
    
    "Simulation  Parameters: neighRadii, cohW, repW, alignW, crowdingThr, leaderW, predatorW "   
    if len(argv)>1:
        # neighRadii  cohW  repW  alignW  crowdingThr  leaderW  preadatorW  guiderW  visualON  timeOutT
        neighRadii = float(argv[1]) # [150.0]Vecinity which each agents checks to apply behavioral rules
        crowdingThr = float(argv[5]) # [10.0]agents proximity before activation of "repulsion behavior"

        # Behavioral weights      
        cohW = float(argv[2]) #  [4.0/100] for Cohesion 
        repW = float(argv[3]) # [23.0/100] for Repulsion 
        alignW = float(argv[4]) # [20.0/100] for alignment

        # Special Agents weights
        leaderW = float(argv[6]) # 
        predatorW = float(argv[7])  # 
        guiderW =  float(argv[8])  # 12
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
        deltC = 18 # how much does the BG color change on each interation
        varC = [1,0,0] # variable used to increase/decrase each [R,G,B] color

        # Time after casual agents extinction that hte game will shut down
        endGameEllapsedT = 0 # DONT change THIS
        endGameTimeOutT = 5
        stableC = 0 # if BG color is at desired value, STOP changing it w/ f(x)'updateBG()'
    
    " General Parameters"

    # Pose Tracking Variables
    global s
    s = socket.socket()# Create Socket

    # Binding (WirelleIP,PORT)  
    myIP = 'localhost'# PC's Local IP
    port = 3000 # SwissTrack default TCP/IP port is 3000
    s.connect((myIP,port)) # Establish connection with server

    global gamePlay
    gamePlay = 0
    # Determines how direct do the controlled agents moves towards mouse position
    gravity = 15

    " Obstacle Parameters "
    resources = 10
    pastNumObs = 0

    " Leader VS Predator. Lead the Swarm + Slight Defense + Assisted Revival "
    if(gamePlay == 0): 
        noBoids = 25 # Number of casualAgents
        noAutoPred = 0 # Number of Autonomous Predators
        noAutoGuide = 0 # Number of Autonomous (guider) Leaders
        noAutoAssist = 0 # Number of Autonomous (assistant) Leaders
        # Create a controlled 0: Nothing ; 1: leaderAgent ; 2: predatorAgent
        contSpecial = 2
        # add some extra attraction towards leader
        leaderW *= 10

    # "Combat Guider Agent "
    elif(gamePlay==1):
        noBoids = 20 # Number of casualAgents
        noAutoPred = 0 # Number of Autonomous Predators
        noAutoGuide = 1 # Number of Autonomous (guider) Leaders
        noAutoAssist = 0 # Number of Autonomous (assistant) Leaders
        # Create a controlled 0: Nothing ; 1: leaderAgent ; 2: predatorAgent
        contSpecial = 2

    maxVel = 11.0  # Max. delta speed that can be performed on each time step

    " Predator Parameters "    
    predSight = 150 # radius of vision at which predator will be able to localize its preys
    randMAX= 10 # Random Exploration Count Attribute: see constructor at class 'Boid'
    deltaRandCount = 0.1 # how fast does random behavior count increase
    globalFact = 1.0 # factor by which predSight is *, enabling predator to have a globalVision for a short time
    global G
    G = 0 # Counts for how much time should global vision be active

    predKillZone = casualAgentR*4 # distance at which predator will effectively kill its prey
    factP2P = 2.1 # Factor by which collisionRadii is scaled, determining how close can 2 predators stand
    centerCoh = 4.0/100 # How attracted is the predator towards the preysCentroid
    global velBoostORI
    velBoostORI = 1.9 # Velocity Boost Attribute: see constructor at class 'Boid'
    deltaBoost = 0.055 # Slope determining impact on the predatorsVelocity when group of agents is encountered
    maxDeltaVel = 7 # Determines Autonomous Special Agents  maximum change in speed at each time step
    
    global persistORI
    persistORI = 15 # Persistance Level Attribute: see constructor at class 'Boid'
    deltaPersist = 0.25 # How fast does persistant level reduce when attempting to hunt/revive the same preys


    " Leader Parameters "
    factP2L = 1.0# Factor by which collisionRadii is scaled, determining frighten would a predator be from a leader
    reviveZone = predKillZone# Radii in which autoLeader (ASSISTANT) agent will revive dead casual agents
    reviveSigth = 100 # radius of vision at which autoleader will be able to localize a dead casualAgent 

    " Guider Parameters "
    guideFact = 1.3 # Factor which increases the Radii at which guiderAgent are considerder neighbors 
   

    " Auxilliary Variables "
    strip = width/3 # Strip among which the init.pos of casual agent is going to be randomlly selected
    ellapsedT = 0 # simulation run time
    numIter = 0 # Number of positions updates   
    casualAgentR # Artificial "Size" of casual agents and Radii among which a collision is determined
    endGame = 0 # 0: Game continues; 1: End of game
    endGameT = 0
   
    " Create Agents"
    # Initialize casualAgents boids
    boids = [] # list containing all the agents
    for i in range(noBoids):
        boids.append(Boid(0,strip,maxVel)) # casual agent 

    # Initialize predatorAgents (AUTONOMOUS) 
    predatorAgents = []  
    for i in range(noAutoPred):
        predatorAgents.append(Boid(-2,0,maxVel))
        boids.append(predatorAgents[-1]) # predator agent  

    # Initialize assistantAgent (AUTONOMOUS) 
    leaderAgents = []   
    for i in range(noAutoAssist):
        leaderAgents.append(Boid(2,0,maxVel))
        boids.append(leaderAgents[-1]) # predator agent

    # Initialize guiderAgents (AUTONOMOUS) 
    boidsATguide = []  # Initialize list containing boids that are near the guiderLeader
    for i in range(noAutoGuide):
        leaderAgents.append(Boid(3,0,maxVel))
        boids.append(leaderAgents[-1]) # predator agent

    # Initialize Controlled Special Agents (CONTROLLED) 
    boidsATgoal = [] # Initialize list containing boids that are near the controlledLeader

    if contSpecial== 1: # Controlled Leader
        leadContAgent = Boid(1,0,maxVel)
        leaderAgents.append(leadContAgent)
        boids.append(leadContAgent) # Add to list saving all Boid instances  

    elif contSpecial== 2:# Controlled Predator
        predContAgent = Boid(-1,0,maxVel)
        predatorAgents.append(predContAgent)
        boids.append(predContAgent) # Add to list saving all Boid instances 

    # Initialize Artificial ControlledLeader which is used as the GOAL to reach
    if (gamePlay == 1):
        leadContAgent = Boid(1,0,maxVel)
        leaderAgents.append(leadContAgent)
        boids.append(leadContAgent) # Add to list saving all Boid instances  
   
    # Auxiliary Agents Variables

    ## stores casualAgents that are (still) alive --> copy objects not pointers, to be able 
    ## to remove elements from the copy wo/affecting the original list of objects)
    livingBoids = copy.copy(boids) 
    huntedBoids = [] # stores casualAgents that have been hunted by 
    numSpecial = sum([1 for boid in livingBoids if(boid.type != 0)]) # Number of SpecialAgents (leader || predator) 
    avoidPos = [] # list storing the position obstables
    global pastPose
    pastPose = [0,0] # Stores the last pose capture by the object traking script (module 'getPose.py')
    
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

        # Check for any event which exits the simulation
        if(visualON):
            for event in pygame.event.get():
                if event.type == pygame.QUIT: 
                    pygame.display.quit()
                    sys.exit()
         
        " Compute neighbooring boids and apply ALL behaviors "

        # RESET variables
        boidsATgoal = [] 
        boidsATguide = []
       
        " Create obstacles "
        if resources > 0:
            pastNumObs = len(avoidPos)
            mouseButtons = pygame.mouse.get_pressed()
            if(mouseButtons[2]): # Initialize new "Avoid"  object instance and save its positionn
                # avoidPos.append(Avoid(pygame.mouse.get_pos()))
                if gamePlay == 0:avoidPos.append(Avoid( (leadContAgent.x,leadContAgent.y) ))
                elif gamePlay == 1:avoidPos.append(Avoid( (predContAgent.x,predContAgent.y) ))
                                            

        for boid in livingBoids :# for each boid

            # RESET Neighborhood variables
            neighBoids = []# list which stores boid neighboors
            neighPredatorTemp = []# list which temporarelly stores predators preys
            
            # Global Vision Enabling/Disabling
            if (boid.type<0 and boid.type>=-99) and (boid.randCount >= randMAX or G): 
                G += 1
                if G < 20: globalFact = 20.0 # ammount of iterations global vision is going to be active
                else : # disable global vision and reset factor and counter
                    globalFact = 1.0
                    G = 0   

            # CHECK: boid's distance with the other boids
            for otherBoid in boids:
                # Dont check the boids distance with its self                
                if otherBoid == boid: continue                
                dist2neigh = boid.distance(otherBoid)

                fact = 1.0
                # if the otherBoid is a SpecialAgent, change the radii at which he will
                # be considered a neighbor 
                if otherBoid.type == 3: fact = guideFact

                " Evaluate casualAgents neighborhood and Collisions (only w/those alive casualAgent otherBoid) "
                if (dist2neigh < neighRadii*fact) and not boid.type and (otherBoid in livingBoids):                   
                    neighBoids.append(otherBoid) # add to neighbors list
                
                " Evaluate predatorAgent close preys (casualAgents) OR  close predatorAgents OR close controlledLeaderAgents. Theyve all got to bealive (obviously) "
                if (boid.type<0 and boid.type>=-99) and ( ( (dist2neigh < predSight*globalFact and not otherBoid.type) or \
                   (dist2neigh < crowdingThr*factP2P and (otherBoid.type<0 and otherBoid.type>=-99)) ) or \
                   (dist2neigh < crowdingThr*factP2L and otherBoid.type == 1 ) )and (otherBoid in livingBoids) and (not endGame):
                    
                    # Hunted casualAgent
                    if(dist2neigh < predKillZone and not otherBoid.type ): 
                        dav = 1
                        # livingBoids.remove(otherBoid) # update survivors
                        # huntedBoids.append(otherBoid) # update casualties 
                    # Casual agent in predators Sight Zone
                    elif not otherBoid.type: 
                        neighPredatorTemp.append(otherBoid)  
                        # raw_input()

                    # otherBoid predator agents that are to close to the current Predator boid, move away to avoid colission (overlapping)                                             
                    elif(otherBoid.type<0 and otherBoid.type>=-99): boid.repulsion([otherBoid],repW,crowdingThr*factP2P,leaderW,predatorW)
                    
                    # otherBoid predator agents that are to close to the Controlled Leader, get away from it   
                    # NOTE: check persistance level to avoid bug that produces predator getting stuck to  controlledLeader                                         
                    elif otherBoid.type == 1 and boid.persistLevel != 0: boid.repulsion([otherBoid],repW,crowdingThr*factP2L,leaderW,predatorW)

                " Evaluate assistantAgent closeness, ONLY to those 'otherBoids' that are dead (obviously) "
                if (boid.type>=2) and (dist2neigh < reviveSigth) and (otherBoid in huntedBoids) and (not endGame):
                    if dist2neigh < reviveZone: # Revival !!!
                        livingBoids.append(otherBoid) # update revived casualAgents
                        huntedBoids.remove(otherBoid) # update casualties

                    # Dead prey inside Revival Radius
                    else: neighPredatorTemp.append(otherBoid) 

                " Evaluate guiderAgent "
                if (boid.type==3) and (otherBoid in livingBoids) and (otherBoid.type == 1 or not otherBoid.type):
                    neighBoids.append(otherBoid) # add to neighbors list

                " Evaluate if the controlledLeaderAgent does exist && check which otherBoids aren't close to him"
                if boid.type == 1 and not otherBoid.type and boid.distance(otherBoid)<= neighRadii :                    
                     boidsATgoal.append(otherBoid)

                " Evaluate if the guiderAgent does exist && check which otherBoids aren't close to him"
                if boid.type == 3 and not otherBoid.type and boid.distance(otherBoid)<= neighRadii*guideFact :                    
                     boidsATguide.append(otherBoid)


            " Apply predatorAgent/assitantAgent persistance level and Update neighAgent attribute "
            if (boid.type<0 and boid.type>=-99) or boid.type >=2 :

                # If predators/assitant is  trying to hunt/revive the same casualBoids gradually reduce its persistance level 
                # (ignore the definition of "same" when no preys are being hunted/revived i.e. []) 
                if neighPredatorTemp == boid.neighAgent and neighPredatorTemp:
                    if boid.persistLevel - deltaPersist >= 0.0: boid.persistLevel -= deltaPersist # if it isnt already zero, reduce it         
                    boid.persistLevel  = round(boid.persistLevel,2)
                # if predator/assistant finds a different group of preys, re-activate hunting/random behavior
                else:
                    boid.persistLevel = persistORI 

                # Update predators/assistnat list of preys
                boid.neighAgent = neighPredatorTemp

            " CHECK: neighboring obstacles "
            neighAvoid = [] # list which stores obstacle neighboors
            for avoid in avoidPos:                
                dist2obs = boid.distance(avoid)

                if dist2obs < neighRadii:
                    neighAvoid.append(otherBoid) # add to list

            " Apply the three behaviors based on current neighborhood (if existant) "
            if not boid.type and neighBoids: # Only apply rule to Casual Agents
                boid.cohesion(neighBoids,cohW,leaderW,predatorW,guiderW)
                boid.repulsion(neighBoids,repW,crowdingThr,leaderW,predatorW)
                boid.alignment(neighBoids,alignW)

                if neighAvoid: # if current boid has some near obstacle, apply repulsion behavior to it
                    boid.repulsion(avoidPos,repW,crowdingThr,leaderW,predatorW)
                

            " Apply predatorHunting/assistantRevival behavior based on predators/assistants current preys "
            if (boid.type<-1 and boid.type>=-99) or boid.type >=2  : # Only apply rule to Predator Agents or Assistant Agents
                boid.move2centroid(maxDeltaVel,centerCoh,deltaBoost,velBoostORI,deltaRandCount)
                if neighAvoid: # if current predator has some near obstacle, apply repulsion behavior to it
                    boid.repulsion(avoidPos,repW,crowdingThr,leaderW,predatorW)
            " Apply guiderAgent behavior "
            if (boid.type==3) and neighBoids: # Only apply rule to Guider Agents
                 boid.cohesion(neighBoids,cohW,leaderW,predatorW,guiderW)

                
        " End of game desicion"
        # If ALL casual agents have been hunted == GAME OVER
        if not (len(livingBoids) - numSpecial) and not endGame and noBoids !=0:
            endGameT = currentT
            winner = 0 # Predator wins    

        # If guiderAgent manages to take ANY casual agents to Goal == GAME OVER
        # From the boids that arrive to the GOAl zone, only thoses that have been guided by the guiderAgent are Valid
        boidsATgoalATguide = [1 for boid in boidsATgoal if boid in boidsATguide]
        if gamePlay == 1 and boidsATgoalATguide and noAutoGuide !=0 and not endGame and noBoids !=0:
            endGameT = currentT
            winner = 2 # Guider wins         

        # If the TimeOut time is reached == GAME OVER
        elif ellapsedT>timeOutT and not endGame:
            endGameT = currentT       
            if gamePlay == 0: winner = (len(livingBoids) - numSpecial)>len(huntedBoids) # 0: Predator wins ; 1: Leader wins
            # winner = (len(livingBoids) - numSpecial) >0 # 0: Predator wins ; 1: Leader wins
            elif gamePlay == 1: winner = 0
            
        if endGameT :   
            if not endGame:          
                sceneColor = [255,255,255]
                sceneColorOri = (173,15,15)*(winner==0) + (30,245,190)*(winner==1) + (235,219,25)*(winner==2)
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
            pos = (390,80)
            numLeadTXT = "Num.Leaders : " + str(len(leaderAgents))
            drawTXT ([numLeadTXT],pos,fontSize,color)

            # No. of resources left
            color = (110,225,0)
            pos = (375,110)
            numResTXT = "Resources : " + str(resources)
            drawTXT ([numResTXT],pos,fontSize,color)
            
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
                
                # Concatenate velboost and persistLevel for EACH of the predatorAgents
                predatorTXT = ["velBoost : " + str(float(round(pred.velBoost,2))) + "  << >>  persistLevel : " + str(float(pred.persistLevel)) for pred in predatorAgents]            
                geneRangeTXT = geneRangeTXT + predatorTXT
                
                pos = (int(width/2),int(height/2))
                drawTXT (geneRangeTXT,pos)
                                
            # Display end of Game TEXT
            else:
                if winner == 0: endGameTXT = ":@ Game Over: Predator Wins :@"
                elif winner == 1: endGameTXT = ":D Game Over: Leader Wins :D"
                elif winner == 2: endGameTXT = ":) Game Over: Guider Wins :)"

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

            " Draw obstacles "
            if avoidPos : 
                if pastNumObs - len(avoidPos) and resources >0: resources -=1 # decrease available obstacle resources
                for pos in avoidPos:
                    pygame.draw.circle(scenario,(255,255,255),(pos.x,pos.y),9,0)  
                    pygame.draw.circle(scenario,(0,255,0),(pos.x,pos.y),7,0)
                

            " Update Boids visualization "
            for boid in boids:
                if boid.type > 0 : # controlledLeader
                    if boid.type == 1 and gamePlay == 0:
                        pygame.draw.circle(scenario, (0,255,0),(int(boid.x), int(boid.y)), 14, 0)
                        pygame.draw.circle(scenario, (0,0,255),  (int(boid.x), int(boid.y)), 10, 0)

                    elif boid.type == 1 and gamePlay == 1:
                        pygame.draw.circle(scenario, (0,255,0),(int(boid.x), int(boid.y)), 14, 0)
                        pygame.draw.circle(scenario, (0,255,0),  (int(boid.x), int(boid.y)), 10, 0)            
                                                            
                    else: # assitant and guider leaders
                        if boid.type == 2: pygame.draw.circle(scenario, (0,255,255),(int(boid.x), int(boid.y)), 14, 0)
                        elif boid.type == 3:pygame.draw.circle(scenario, (255,255,0),(int(boid.x), int(boid.y)), 14, 0)
                        pygame.draw.circle(scenario, (0,0,255),  (int(boid.x), int(boid.y)), 10, 0)

                    # Draw Goal Radii
                    # pygame.draw.circle(scenario, (0,255,0), (int(boid.x), int(boid.y)), neighRadii,8)
                
                elif boid.type < 0 :  # Predators         
                    pygame.draw.circle(scenario, (255,128,0),(int(boid.x), int(boid.y)), 14, 0)
                    pygame.draw.circle(scenario,(255,0,0), (int(boid.x), int(boid.y)), 10, 0)

                    # Draw neighAgent radii
                    # pygame.draw.circle(scenario, (255,0,0), (int(boid.x), int(boid.y)), predSight,8)

                else: # Casual Agents 

                    if boid in huntedBoids: # Dead casualAgents
                        agentColor = (0,0,0)                     
                        pygame.draw.circle(scenario, agentColor, (int(boid.x), int(boid.y)), int(casualAgentR),0)
                        colorRing = (225,0,0) 
                        pygame.draw.circle(scenario, colorRing, (int(boid.x), int(boid.y)), int(casualAgentR),int(casualAgentR/2))
                    
                    elif boid in livingBoids: # Alive casualAgents
                        # Make casual agents blink w/ rainbow like colors + Draw its body + Draw its outer layer
                        agentColor = (random.randint(0,240),random.randint(0,240),random.randint(0,240))                     
                        pygame.draw.circle(scenario, agentColor, (int(boid.x), int(boid.y)), int(casualAgentR),0)
                        
                        # Check if current casualAgent ('boid') is prey of ANY of the predatorAgents
                        isHunted = []                         
                        isHunted = [1 for pred in predatorAgents if boid in pred.neighAgent]
                        
                        if isHunted : colorRing = (255,128,0) # Boid in hunting zone
                        elif boid in boidsATgoal and gamePlay == 0: colorRing = (0,180,255) # Boids in leaderAgents protected zone
                        elif boid in boidsATgoal and gamePlay == 1: colorRing = (0,255,0) # Boids in leaderAgents protected zone
                        elif boid in boidsATguide: colorRing = (255,255,0) # Boids in leaderAgents protected zone
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
        if gamePlay == 0:[boid.move(maxVel,maxDeltaVel,visualON,gravity) for boid in livingBoids]
        elif gamePlay == 1: [boid.move(maxVel,maxDeltaVel,visualON,gravity) for boid in livingBoids if (boid.type != 1 )]  
    " Simulation Exit "    
    pygame.display.quit()



" Class which initializes obstacles objects, in a similar way to boid objects."
" In order to be able to reuse class Boid functions, and present the obstacles"
" as a variation of a predator"
class Avoid:
    def __init__ (self,pos):
        self.x = int(pos[0])
        self.y = int(pos[1])
        self.type = -100 # obstacles -> self.type <=-100

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



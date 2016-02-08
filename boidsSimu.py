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
from datetime import datetime # for keeping time of algo run.
import time # time.sleep([s]) OR raw_input("Press Enter to continue...")

pygame.init()
scenarioSize = width,height = 800,600 #scenario dimensions
casualAgentR = 10

" Class containing all the Boid behaviors "
class Boid: 

    def __init__ (self, type, strip, maxVel):
        # Casual Agent : set pos/vel randonmly ; Special Agents: pos == center of screen and vel == maxSpeed %
        self.x = random.randint(0,strip)*(type==0) + (width)*(type!=0)
        self.y = random.randint(0,height)*(type==0)  + (height/2)*(type!=0)
        self.velX = float(random.uniform(-maxVel,maxVel)) 
        self.velY = float(random.uniform(-maxVel,maxVel))      
        self.type = type          


    " Compute (Euclidian) distance from self - to - 'boid' "
    def distance (self,boid):
        distX = self.x - boid.x
        distY = self.y - boid.y
        return math.sqrt(distX*distX + distY*distY)


    " Move closer: towards center of Mass of boids neightborhood"
    def cohesion(self,boids,cohW,leaderW):
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
            elif boid.type < 0: fact = 0 # predator negative reinforcement

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
        # print avgX
        # print type(avgX)
        # print cohW
        # print type(cohW)
        # print type(self.velX)
        # time.sleep(4)
        self.velX -= (avgX*cohW) 
        self.velY -= (avgY*cohW) 


    " Move further: away of center of Mass of boids neightborhood "
    " to avoid crowding local flockmates"
    def repulsion(self,boids,repW,crowdingThr,leaderW):
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
            if boid.type > 0: fact = leaderW # decreases radii at which leader is repulsed
            # if len(boids)>10 and type>0: fact /=100 # <-- C2!?
            elif boid.type < 0 and boid.type>=-99 : fact = 1/predatorW  # increases radii at which predator is repulsed
            elif boid.type<=-99 : fact = 1/(avoidW/3)  # increases radii at which obstacle is avoided

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
            if boid.type > 0: fact = 0 # dont follow leaders speed
            elif boid.type < 0 and boid.type>=-99 : fact = 0 # dont follow predator speed  

            avgX += boid.velX*fact
            avgY += boid.velY*fact  

        avgX /= float(len(boids)) 
        avgY /= float(len(boids))  

        self.velX += (avgX*alignW) 
        self.velY += (avgY*alignW) 


    "Perform movement: based on boids velocity (determined by the behavioral rules)"
    def move(self,maxVel):
        # if self.type : # increase special autonomous agent speed            
        #     self.velX += random.uniform(-maxDeltaVel,maxDeltaVel)
        #     self.velY += random.uniform(-maxDeltaVel,maxDeltaVel)

        # for each timeStep of evaluating the rules, theres a maximum (saturation) displacement
        # value (boids cant move instantly from one place to the other, its got to me smooth)
        if  abs(self.velX) > maxVel or abs(self.velY) > maxVel:
            scaleFactor = maxVel / max(abs(self.velX), abs(self.velY))
            self.velX *= scaleFactor
            self.velY *= scaleFactor

        # Update Agents Position
        # if (self.type == 1 or self.type == -1): # Controlled Special Agents
            # self.x,self.y = pygame.mouse.get_pos()            
        # else:  # Autonomous casual agents                  
        self.x += self.velX
        self.y += self.velY

        # Ensure boids  stay within the screen space -> if they are to close to 
        # the border, then change the Direction of the velocity vector, and change
        # its magnitud by a random factor between [0.0,1.0]
        border = 10 # how close from the borders does one wants the boids to get
        bordFact = 3.0
        if self.x < border and self.velX< 0: self.velX = -self.velX*random.random()*bordFact #generates val in [0.0,1.0]
        if self.x > (width - border*2) and self.velX> 0: self.velX= -self.velX* random.random()*bordFact
        if self.y < border and self.velY < 0: self.velY = -self.velY * random.random()*bordFact
        if self.y > (height - border*2) and self.velY > 0: self.velY = -self.velY * random.random()*bordFact 

        # # In case the change in velocity isnt enoguth, force agents position to be
        # # Insider the simulation scenario
        # if self.x<=0: self.x = border*2
        # if self.x>=width-int(border/3): self.x = width-border*2
        # if self.y<=0: self.y = border*2
        # if self.y>=height-int(border/3): self.y = height-border*2


" Main Code "       
def main(argv): 
    
    "Simulation Evolutionary Parameters: neighRadii, cohW, repW', alignW, crowdingThr "
    if len(argv)>1:
        neighRadii = float(argv[1]) # [150.0]Vecinity which each agents checks to apply behavioral rules
        crowdingThr = float(argv[5]) # [10.0]agents proximity before activation of "repulsion behavior"

        # Behavioral weights      
        cohW = float(argv[2]) #  [4.0/100] for Cohesion 
        repW = float(argv[3]) # [23.0/100] for Repulsion 
        alignW = float(argv[4]) # [20.0/100] for alignment

        # Special Agents weights
        leaderW = float(argv[6]) # 10 for alignment
        # Auxilliary Var's
        visualON = argv[-2] # [1] 0: dont show visual environment ; 1: show visual environment
        timeOutT = float(argv[-1]) # [15] time in [s] after which simlation will abort 
    else: 
        print " ERROR: Please provide the simulations parameters"
        sys.exit()

    "Scenario Visualization Parameters"
    if(visualON): 
        scenario = pygame.display.set_mode(scenarioSize) # Configure pygame window  
        pygame.display.set_caption('Boids Simulation')

    sceneColorOri = (0,0,50) # Initial background color
    sceneColor = [sceneColorOri[0],sceneColorOri[1],sceneColorOri[2]] # stores current BG color
    deltC = 10 # how much does the BG color change on each interation
    varC = [1,0,0] # variable used to increase/decrase each [R,G,B] color   
    strip = width/3 # Strip among which the init.pos of casual agent is going to be randomlly selected

    "Static Parameters"
    noBoids = 15
    boids = [] # list containing all the agents
    # isControlAgent = 0 # 0: No controlled agents created ; 1: controlled agent already created
    # simulation might runs faster/slower depending on amount of agents. Changing damping factor easily
    # adjust speed and other important factor for a nice simulation 
    damp = 1.0
    # spread = 1.0 # Determines the y-position range, among which initial position are going to be randonmly stablished

    "Dynamic parameters"   
    maxVel = 6.0*damp  # Max. delta speed that can be performed on each time step
 
    " Evolutionary Variables"
    # Auxilliary Var's       
    ellapsedT = 0 # simulation run time
    numIter = 0 # Number of positions updates

    # Main Evo. Var's
    interCollV = 0 # Measure Number of Inter Agent Collision
    interCollV_PerIter = 0 # Not cummulative, only per iteration
    casualAgentR # Artificial "Size" of casual agents and Radii among which a collision is determined
    collidedAgents = [] # Closed list that stores pairs of agents who have collided on the current iteration

    medGRadiiV = 0.0# --> insideRadV = 0  # Measures the group level of compactness 
    medGRadiiV_PerIter = 0.0 # --> medGRadiiV_PerIter = 0 # Not cummulative, only per iteration
    avgPos = np.zeros((1,2)) # groups average position, useful to compute centroid
    percRIn = 50.0/100 # % of groups radii, among which agents inside it will count as being compact

    deltaTV = 0.0 
    deltaTV_PerIter = 0.0
    goalRadii = casualAgentR*noBoids
    factUS = 1E6

    t2GoalV = timeOutT*1.0
    numATgoalPast = 0
    numATgoal = 0
    boidsNotATgoal = []
    boidsAtgoal = []
    currentTGoal = 0
    firstGoal = 1
    evoSuccess = 0

    "Special Agents and obstacles params. [Corresponds to self.type != 0]"
    # when Special Agents are automous (controlled by code), determines the maximum change in speed at each time step
    # maxDeltaVel = 4.0*damp
    # leaderW = 1.0 #10 How much more cohesion and less repulsion will Leader have
    # predatorW = 12.0 # How much more repulsion will predator have
    # avoidW = 16.0 # How much more repulsion (how far away) will agents evade obstacles
    specialRFact = 1.75 #max(scenarioSize) #8 Factor which increases the Radii at which Special Agents are considerder neighbors
    # 0: Created both, leader/predator Auto.Agents ; 1/-1: Creadte Auto.leader/Auto predator agents
    # autoAgents = 0 
    # lists that store controled and special agent object instances (needed in order to delete them when needed)
    controledAgent = []
    # specialAgent = []
    # avoidPos = [] # list storing the position obstables

    # neighRadii = 150.0
    # crowdingThr = casualAgentR*3
    # cohW = 0.04
    # repW = 0.23
    # alignW = 0.20
    # leaderW = 20
    # timeOutT = 100

    # initialize Casual boids, with a random X,Y position
    for i in range(noBoids):
        boids.append(Boid(0,strip,maxVel)) # casual agent 

    controledAgent = Boid(1,0,maxVel)
    boids.append(controledAgent) # Add to list saving all Boid instances     

    " MAIN LOOP "

    while ellapsedT<=timeOutT and not(evoSuccess):
        numIter += 1
        if numIter == 2:
            # First time pause for user to visualize simulation parameters
            if(numIter == 1): raw_input("... Press enter to start simulation ...")
            # Capture Initial start time of simulation
            initT = datetime.now().time()
        elif numIter > 1:
            currentT = datetime.now().time()
            ellapsedT = (currentT.hour - initT.hour)*3600.0 + (currentT.minute-initT.minute)*60.0 + (currentT.second-initT.second)
    
        # Reset " per iteration " Evo. params
        interCollV_PerIter = 0
        avgPos[:] = 0
        collidedAgents = []

        # Check for any event which exits the simulation
        if(visualON):
            for event in pygame.event.get():
                if event.type == pygame.QUIT: 
                    pygame.display.quit()
                    sys.exit()

        # " User Controls "
        # mouseButtons = pygame.mouse.get_pressed()
         
        # if mouseButtons[0] and mouseButtons[2]: # Delete all created agents and obstacles
            # isControlAgent = 0 
            # draw = 0
            # if controledAgent : # Destroy Controled Agents
            #     boids.remove(controledAgent)   
            #     controledAgent = []       
            # if specialAgent : # Destroy Autonomous special agents
            #     for i in range(len(specialAgent)):
            #         boids.remove(specialAgent[i])  
            #     specialAgent = [] 
            # if avoidPos : # Destroy drawed obstacles
            #     for i in range(len(avoidPos)):
            #         avoidPos.remove(avoidPos[0])  
            #     avoidPos = [] 

            # time.sleep(0.25) 

        # elif mouseButtons[0]: # Create Controlled special agent 
        # if isControlAgent == 0:               
            # controledAgent = Boid(1,0)
            # boids.append(controledAgent) # Add to list saving all Boid instances 

            # Conmmute between: Leader and Depredator on each new left click
            # controledAgent.type = controledAgent.type-controledAgent.type*isControlAgent*2 
            # isControlAgent = 1
            # time.sleep(0.15) 

        # elif mouseButtons[1]: # Create Autonomous special agents 
        #     if (autoAgents): specialAgent.append(Boid(2*autoAgents)) # Create Auto. Leader/Predator
        #     elif(not autoAgents):# Created both special agents
        #         specialAgent.append(Boid(2))
        #         specialAgent.append(Boid(-2))
        #     for i in range(abs(autoAgents)+ 2*(not autoAgents)):
        #             boids.append(specialAgent[-1-i])            
        #     time.sleep(0.15)   

        # elif(mouseButtons[2]): # Initialize new "Avoid"  object instance and save its positionn
        #     avoidPos.append(Avoid(pygame.mouse.get_pos()))
         
        " Compute neighbooring boids and apply ALL behaviors " 
        for boid in boids:# for each boid

            neighBoids = []# list which stores boid neighboors
            
            # CHECK: boid's distance with the other boids
            for otherBoid in boids:
                # Dont check the boids distance with its self                
                if otherBoid == boid: continue                
                dist2neigh = boid.distance(otherBoid)

                fact = 1.0
                # if the otherBoid is a SpecialAgent, change the radii at which he will
                # be considered a neighbor 
                if otherBoid.type: fact = specialRFact

                if dist2neigh < neighRadii*fact:                    
                    neighBoids.append(otherBoid) # add to neighbors list

                    "EVO.FITNESS -> Inter Agent Colission"
                    # if the otherAgent is in collision w/ the the boid                                         
                    if (dist2neigh < casualAgentR*2 and otherBoid.type == 0 ):
                        
                        if( (otherBoid,boid) not in collidedAgents):                     
                            interCollV += 1
                            interCollV_PerIter += 1
                            collidedAgents.append((boid,otherBoid)) 
                        

            # # CHECK: neighboring obstacles
            # neighAvoid = [] # list which stores obstacle neighboors
            # for avoid in avoidPos:                
            #     dist2obs = boid.distance(avoid)

            #     if dist2obs < neighRadii:
            #         neighAvoid.append(otherBoid) # add to list

            " Apply the three behaviors based on current neighborhood (if existant) "
            if neighBoids and boid.type == 0 : # Only apply rule to Casual Agents
                boid.cohesion(neighBoids,cohW,leaderW)
                boid.repulsion(neighBoids,repW,crowdingThr,leaderW)
                boid.alignment(neighBoids,alignW)
                
                # if neighAvoid: # if current boid has some near obstacle, apply repulsion behavior to it
                #     boid.repulsion(avoidPos,repW)      
 

        " EVO.FITNESS ->  Measurement of Group Compactness:"
        # Obtain Number of Agents inside a percentage of the groups position dispersion
        avgPos[0,0] = sum([boid.x for boid in boids if (boid.type == 0)])
        avgPos[0,1] = sum([boid.y for boid in boids if (boid.type == 0)])

        ## Dont use len(boids) as the averaging number since it will mistakenly include the Special Agents
        GCentroid = avgPos/noBoids # Groups centroid
        GCentroid = GCentroid.astype(int).tolist()# cast from float to int, and then to python list
        centroidAgent = Boid(0,0,maxVel) # trick used to be able to recycle Euclidian distance to boids method of Boid Class
        centroidAgent.x = GCentroid[0][0]
        centroidAgent.y = GCentroid[0][1]
        ## Obtain groups dispersion (GRadii) 
        dist2Centroid = [centroidAgent.distance(boid) for boid in boids if (boid.type == 0)]
        GRadii = max(dist2Centroid)
        # --> insideRadV_PerIter = len([distBoids for distBoids in dist2Centroid if(distBoids<=GRadii*percRIn)])
        # --> insideRadV +=insideRadV_PerIter
        medGRadiiV_PerIter = np.median(np.array(dist2Centroid))
        medGRadiiV += medGRadiiV_PerIter


        " EVO.FITNESS ->  Measurement of groups arrival time:"
        if numIter == 1: boidsNotATgoal = [boid for boid in boids if(boid.type==0)]
        else:  
            boidsNotATgoal = [boid for boid in boids if(controledAgent.distance(boid) >= goalRadii)]
            boidsAtgoal = [boid for boid in boids if(boid not in boidsNotATgoal)]

        numATgoal = noBoids - len(boidsNotATgoal) 
        if (numATgoal-numATgoalPast) > 0:
            numATgoalPast = numATgoal
            pastTGoal = currentTGoal
            currentTGoal = datetime.now().time()

            if firstGoal: 
                firstGoal = 0
            else:     
                deltaTV_PerIter = (currentTGoal.hour-pastTGoal.hour)*3600.0*factUS + \
                                  (currentTGoal.minute-pastTGoal.minute)*60.0*factUS + \
                                  (currentTGoal.second-pastTGoal.second)*1.0*factUS + \
                                  (currentTGoal.microsecond-pastTGoal.microsecond)
                deltaTV += deltaTV_PerIter

        if numATgoal == noBoids:
            currentT= datetime.now().time()
            t2GoalV = (currentT.hour-initT.hour)*3600.0 + (currentT.minute-initT.minute)*60.0 + (currentT.second-initT.second)*1.0
            evoSuccess = 1          


        " Simu. Visualization: Update virtual boids display "
        # visualON determines if the simulation will show the visual environment
        if (visualON):       
            # Draw obstacles
            # if avoidPos: 
            #     for pos in avoidPos:
            #         pygame.draw.circle(scenario,(255,255,255),(pos.x,pos.y),9,0)  
            #         pygame.draw.circle(scenario,(0,255,0),(pos.x,pos.y),7,0)        # Update Dynamic Color Background visualization    
            
            # paint BG to erase past objects
            # sceneColor = updateBG(sceneColor)        
            scenario.fill(sceneColor) # fill scenario with RGB color

            # Insert Simulation parameters on screen scenario
            ## text characteristics
            txtColor = (255,255,255)
            stdFont = pygame.font.Font('freesansbold.ttf',15)
            txtPos = (width-100,30)

            ## texts
            neighRadiiTXT = "neighRadii : " + str(int(neighRadii))
            cohWTXT = "cohW : " + str(int(cohW*100)) + "%"
            repWTXT = "repW : " + str(int(repW*100)) + "%"
            alignWTXT = "alignW : " + str(int(alignW*100)) + "%"
            crowdingThrTXT = "crowdingThr : " + str(int(crowdingThr))
            leaderWTXT = "leaderW : " + str(int(leaderW))
            geneRangeTXT = [neighRadiiTXT,cohWTXT,repWTXT,alignWTXT,crowdingThrTXT,leaderWTXT] 

            ## texts rederings
            offSet = 0
            for txt in geneRangeTXT:
                txtSurf = stdFont.render(txt, True, txtColor)
                txtRect = txtSurf.get_rect()
                txtRect.center = (txtPos[0],txtPos[1]+offSet)
                offSet += 20
                scenario.blit(txtSurf,txtRect)

            # Draw Groups Centroid,  Compactness Radii (Median Radii) 
            widthCirc = 3  
            groupC = (GCentroid[0][0],GCentroid[0][1])
            if(medGRadiiV_PerIter>widthCirc):        
                pygame.draw.circle(scenario, (255,255,255), groupC, 6, 0)
                pygame.draw.circle(scenario, (255,255,255), groupC, int(medGRadiiV_PerIter), widthCirc)
            # --> pygame.draw.circle(scenario, (255,255,255), groupC, int(GRadii), 3)
            # --> pygame.draw.circle(scenario, (0,255,0), groupC, int(GRadii*percRIn), 3)

            # Draw interval where casual voids are born
            pygame.draw.line(scenario,(255,0,255),(strip,0),(strip,height),4)

            # Update Boids visualization
            for boid in boids:
                if boid.type > 0 : # Leaders              
                    # pygame.draw.circle(scenario, (0,0,200), (int(boid.x), int(boid.y)), 11, 0)
                    # pygame.draw.circle(scenario, (51,128,255), (int(boid.x), int(boid.y)), 7, 0)
                    pygame.draw.circle(scenario, (0,255,0), (int(boid.x), int(boid.y)), goalRadii,8)
                # elif boid.type < 0 :  # Predators              
                #     pygame.draw.circle(scenario, (0,0,0), (int(boid.x), int(boid.y)), 11, 0)
                #     pygame.draw.circle(scenario, (255,0,0), (int(boid.x), int(boid.y)), 7, 0)
                else: # Casual Agents      

                    # Make casual agents blink w/ rainbow like colors + Draw its body + Draw its outer layer
                    agentColor = (255,255,255)#(random.randint(0,240),random.randint(0,240),random.randint(0,240))                     
                    pygame.draw.circle(scenario, agentColor, (int(boid.x), int(boid.y)), int(casualAgentR),0)
                    if boid not in boidsNotATgoal: colorRing = (255,255,255) 
                    else:  colorRing = (0,0,0)
                    pygame.draw.circle(scenario, colorRing, (int(boid.x), int(boid.y)), int(casualAgentR),int(casualAgentR/2))
                    
                    # Draw distance to groups centroid line
                    pygame.draw.line(scenario, (255,255,255), groupC, (int(boid.x), int(boid.y)), 2)
                    
                    # Draw neighboord radii
                    pygame.draw.circle(scenario, (0,255,0), (int(boid.x), int(boid.y)), int(neighRadii),2)

                    # Draw collision radii
                    pygame.draw.circle(scenario, (255,0,0), (int(boid.x), int(boid.y)), int(crowdingThr),2)

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
        [ boid.move(maxVel) for boid in boids if (boid.type == 0 )]  

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
    interCollV = 1.0/((interCollV*1.0/numIter)+1) # Average No. compactAgents over  No. of position updates (iterations)
   
    # Average median raddi ver  No. of position updates (iterations)
    avgGRadii = (medGRadiiV/numIter)
    idealGRadii = 100
    std = 40
    medGRadiiV = 1*math.exp(- ((avgGRadii-idealGRadii)**2) / (2*std**2) )    
    if math.isnan(medGRadiiV) : medGRadiiV = 0.0

    if deltaTV == 0.0: deltaTV = 0.0
    else: deltaTV = 1.0/( ((deltaTV/factUS)/(noBoids)) + 1)  # Average deltaTimes over  No. of arrivals to goal --> Precisely, it should ve over(numATgoal-1) , 
                                                                # since if N-agents are at goal there exist (N-1)-delta time intervals ... but its done '+1' to prevent NaN
    # if t2GoalV == timeOutT: t2GoalV = 0.0
    # else: t2GoalV = 1.0/ ( (t2GoalV*1.0/timeOutT) + 1)    

    numATgoalV = numATgoal*1.0/noBoids
    # evoSuccessV = evoSuccess
    # print "After Normalizing : \n "
    # print "interCollV: %f" %interCollV
    # print "medGRadiiV: %f" %medGRadiiV
    # print "deltaTV: %f" %deltaTV
    # print "t2GoalV: %f" %t2GoalV
    # print "numATgoalV: %f" %numATgoalV
    # print "evoSuccessV: %f" %evoSuccessV
    # raw_input()

    fitVals = np.array([[interCollV, medGRadiiV, deltaTV,numATgoalV]])
    return fitVals


# " Class which initializes obstacles objects, in a similar way to boid objects."
# " In order to be able to reuse class Boid functions, and present the obstacles"
# " as a variation of a predator"
# class Avoid:
#     def __init__ (self,pos):
#         self.x = pos[0]
#         self.y = pos[1]
#         self.type = -100 # obstacles -> self.type <=-100


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



"""
********************************************************************************
    * Evolution Parameters: 
        - Genotype: [neighRadii,cohW,repW,alignW,crowdingThr] 
        - Fitness f(x): [No. interAgent collisions, averageOfMedianOfGroupRadii, meanDeltaTofArrival,t2Goal]

********************************************************************************
    Author: David C. Alvarez - Charris
    Contact: david13.ing@gmail.com

    Created on Mon Jan 18 17:25:11 2016
********************************************************************************
"""
"""
evoLog_try1 --> specialRFact = 2.5 ; std = 30 ; deltaTV average obtained by numATgoalW  
                popSize = 5# Population Size
                generations = 4 # Number of Evolution generations
                selectMethod = ['roulette',0.5] #['selectioMethod',probOfRandomParentSelection (i.e. dont use roulette)]
                elitism =[1,int(math.ceil(0.20*popSize))]
                crossProb = 0.85
                mutProb = [0.08,0.7] # [prob.OfMutation , %OfMutation]
                numRept = 3 # No. of repetions each ind. is simulated, to obatain avg. fitness
                timeOutT = 7
                
                # Fitness Weights
                interCollW = 25.0/100
                medGRadiiW = 35.0/100
                deltaTW = 25.0/100
                # t2GoalW = 1.0/6
                numATgoalW = 15.0/100

evoLog_try2 --> specialRFact = 1.75 ; Gaussian medGRadiiV w/ ideal = 100, std = 40 ; 
                deltaTV average obtained by numBoids

                popSize = 10# Population Size
                generations = 10 # Number of Evolution generations
                selectMethod = ['roulette',0.5] #['selectioMethod',probOfRandomParentSelection (i.e. dont use roulette)]
                elitism =[1,int(math.ceil(0.20*popSize))]
                crossProb = 0.85
                mutProb = [0.08,0.7] # [prob.OfMutation , %OfMutation]
                numRept = 3 # No. of repetions each ind. is simulated, to obatain avg. fitness
                timeOutT = 10
               
                # Fitness Weights
                interCollW = 30.0/100
                medGRadiiW = 40.0/100
                deltaTW = 20.0/100
                # t2GoalW = 1.0/6
                numATgoalW = 10.0/100

evoLog_try3 --> Used evoLog_try2 as populationSeed
evoLog_try4 --> Used evoLog_try3 as populationSeed + below changes:
                generations = 20
                selectMethod = ['roulette',0.2] #['selectioMethod',probOfRandomParentSelection (i.e. dont use roulette)]
                elitism =[1,int(math.ceil(0.3*popSize))]
                crossProb = 0.95
                mutProb = [0.2,0.8] # [prob.OfMutation , %OfMutation]

evoLog_try5 --> NO PREVIOUS SEEDING
                # Simu Params: noBoids = 15 ; maxVel = 6.0 ; specialRFact = 1.75
                # MedGRadii Calculation:  idealGRadii = 100 ; std = 40
                # Evo Params:

                popSize = 15# Population Size
                generations = 25 # Number of Evolution generations
                selectMethod = ['roulette',0.35] #['selectioMethod',probOfRandomParentSelection (i.e. dont use roulette)]
                elitism =[1,int(math.ceil(0.20*popSize))]
                crossProb = 0.85
                mutProb = [0.05,0.60] # [prob.OfMutation , %OfMutation]
                numRept = 3 # No. of repetions each ind. is simulated, to obatain avg. fitness
                timeOutT = 10
                
                # Fitness Weights
                interCollW = 27.0/100
                medGRadiiW = 38.0/100
                deltaTW = 20.0/100
                numATgoalW = 15.0/100

                # IMPORTANT CHANGE OF INTERVALS:
                neighRadii = [0,casualAgentR*10]
                cohW = [0.0,1.0]
                repW = [0.0,1.0]
                alignW = [0.0,1.0]
                crowdingThr = [0.0,casualAgentR*10]
                leaderW = [0.001,10.0]
  
"""
import random as rand
import numpy as np # array and matrix manipulation
import matplotlib.pyplot as plt # plotting graphs
import pickle # Saving data to log file
import boidsSimu # Boid simulator library
import sys,math
from datetime import datetime 

# Simulation Scenario size (used for evolutionary genes ranges)
scenarioSize = width,height = 800,600 #scenario dimensions
casualAgentR = 10.0

" Create Individuals -> each posible solution"
def individual (geneRange):
    # Concatenate in a list each gene value of the individual and then return it as a numPy array
    return np.array(  [rand.uniform(gene[0],gene[1]) for gene in geneRange]  )

" Create Population -> collection of all individuals"
def population (popSize, geneRange):
    return np.hstack( (individual(geneRange) for pop in range(popSize)) ).reshape(popSize,len(geneRange))

" Evolutionary Process -> 0. Living | 1. Selection | 2. CrossOver | 3. Mutation "
def evolution (pop,fitWeights,geneRange,numRept,selectMethod,elitism,crossProb,mutProb): # elitism-rank method-SUS method-torunament method
    popSize = pop.shape[0]
    fitHistAux = np.empty((popSize,fitWeights.shape[1]+1)) 

    " 0. Living " 
    for ind in range(popSize):
        simuParams = np.array([0,visualON,timeOutT], dtype= 'float64') 
        simuParams = np.insert(simuParams,1,pop[ind,:]).tolist() # insert in index=1 of simuParams
        
        # Individuals Fitnes = avg. gitness along 'numRept' simulations
        fitness = 0.0
        fitValsAvg = np.zeros((1,fitWeights.shape[1]))
        for rept in range(numRept):
            fitVals = boidsSimu.main(simuParams) # Simulate Individual 
            fitness += fitWeights.dot(fitVals.transpose()) # Linear Combination Fitness Function
            fitValsAvg += fitVals
        fitHistAux[ind,:] = np.concatenate((fitValsAvg/numRept,fitness/numRept),axis=1)

    " 1. Selection "
    newPop = np.zeros((popSize,pop.shape[1]))
    newPopIdx = 0
    numCouples = popSize# number of couples to generate

    # Elitism or Truncation Selection (fittest ind. are selected)
    if elitism[0]:
        elitIdx = np.argsort(fitHistAux[:,-1])[-elitism[1]::]
        newPop[0:elitism[1],:] = pop[elitIdx,:]

        newPopIdx = elitism[1]    
        numCouples -= elitism[1]

    if(selectMethod[0] == 'roulette' and numCouples > 0):
        # Roulette-Wheel Selection
        probRand = selectMethod[1]
        selected = roulette(probRand,numCouples,fitHistAux[:,-1]) 
    else:
        print "ERROR: No selection method specified"
        sys.exit()

    " 2. CrossOver "    
    ind = 0
    # Each couple either: Produces ONE child OR one of the parents passes directly
    for couple in selected: 
        crossDart = rand.uniform(0,1)

        if crossDart < crossProb:
            AParent = rand.randint(0,1)
            parentA = np.array([pop[couple[AParent],:]]) # Select AParent
            parentB = np.array([pop[couple[1-AParent],:]]) # Select BParent (complement idx)
            crossI = rand.randint(0,math.floor((pop.shape[1])/2)-1 ) 
            crossF = rand.randint(math.floor((pop.shape[1])/2),pop.shape[1]-1)

            newPop[newPopIdx+ind,:] = np.hstack((parentA[0,0:crossI+1], parentB[0,crossI+1:crossF+1], parentA[0,crossF+1:] ))
        else:
            newPop[newPopIdx+ind,:] = pop[couple[rand.randint(0,1)],:]
        ind += 1

    " 3. Mutation "
    for ind in range(newPop.shape[0]):
        for gene in range(newPop.shape[1]):
            dice = rand.uniform(0,1)

            if(mutProb[0]>=dice):
                newPop[ind,gene] = newPop[ind,gene] + \
                                   rand.uniform(geneRange[gene][0],geneRange[gene][1]*mutProb[1])*rand.random()

    return newPop,fitHistAux

" Selection Operator: Fitness Propotionate Selection (FPS) "
def roulette (probRand,numCouples,fitHistAux): 
    popSize = fitHistAux.shape[0]
    selectProb = np.zeros((2,popSize+1))    
    selectProb[0,1:] = np.divide(fitHistAux[:] , np.sum(fitHistAux[:]))# : indicates end, therefore ::-1 is equivalente to end:-1
    selectProb[1,1:] = range(popSize)    
    selectProb = np.roll(selectProb[:,selectProb[0,:].argsort()][:,::-1],1)    

    # Create Roulette (Probabilities percentage for each individual)
    for i in range(popSize):
        selectProb[0,i+1] = selectProb[0,i]+selectProb[0,i+1]
    selectProb = selectProb[:,1:]

    couple = []
    selected = []
    while(len(selected)<numCouples): 
        while(len(couple)<2):
            randORroul = rand.uniform(0,1)    

            if randORroul > probRand: # Roulette parent selection
                while(1):
                    indDart = rand.uniform(0,1)   
                    try: 
                        ind = selectProb[1,np.where(selectProb[0,:]>=indDart)[0][0]] # [0] -> select index list , [0] -> select first occurance
                    except:
                        print "NaN in probability of selection (`selectProb`)"
                        sys.exit()

                    if(ind not in couple):
                        couple.append(ind)
                        break        

            else: # Random parent selection
                while(1): # do - while loop
                    ind = rand.randint(0,popSize-1)
                    if(ind not in couple):
                        couple.append(ind)
                        break

        selected.append(couple)
        couple = []
    
    selected = [[int(i[0]),int(i[1])] for i in selected]
    return selected

def main():

    ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Configuration <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"   
    " Auxiliary Parameters "
    global visualON # 0: dont show visual environment ; 1: show visual environment
    visualON = 0
    percGenDisplay = 0.2
    interruptEvo = 0
    interruptperc =  0.20
    logPath = 'C:\Users\IVAN\Desktop\evoBoids\evoBoids Code\evoLog\evoLog_try6.pckl'

    " Evolutionary Configuration"
    popSeeding = 0
    popSeeding = int(raw_input("Do you want to load a SeedPopuation: [1:Yes ; 0:No]"))
    
    # Load evolutionary parameters from previous log evolution (seed population)
    # User could also modify/config. any parameter as he wants to
    global timeOutT # time in [s] after which simulation will abort 
    if popSeeding: 

        popSeedFile = raw_input("Press ENTER to Load %s logFile or input the number of the desired logFile: " % logPath[-len("evoLog_tryN.pckl"):-1]) 

        if popSeedFile: 
            logPath = list(logPath)
            logPath[-6] = popSeedFile
            logPath = ''.join(logPath)

        f = open(logPath)
        # [lastPop,bestIndPerAll,evoConfig,fitHist,fitPlotData,bestIndPerGen,ellapsedT]
        lastPop,bestIndPerAll,evoConfig,_,_,_,_= pickle.load(f)
        f.close()        
        popSize, generations,selectMethod,elitism,crossProb,mutProb,numRept,timeOutT,fitWeights = evoConfig  
        
        # Copy and Paste Evolution parameters from below, to @Overide the loaded ones
        # _ _ _ _
        # _ _ _ _

    # User configuration of evolutionary parameters      
    else: 
        # Evolution Parameters
        popSize = 20# Population Size
        generations = 35 # Number of Evolution generations
        selectMethod = ['roulette',0.4] #['selectioMethod',probOfRandomParentSelection (i.e. dont use roulette)]
        elitism =[1,int(math.ceil(0.20*popSize))]
        crossProb = 0.85
        mutProb = [0.065,0.70] # [prob.OfMutation , %OfMutation]
        numRept = 3 # No. of repetions each ind. is simulated, to obatain avg. fitness
        timeOutT = 10
       
        # Fitness Weights
        interCollW = 33.0/100
        medGRadiiW = 33.0/100
        deltaTW = 17.0/100
        avgNumATgoal = 17.0/100
        fitWeights = interCollW, medGRadiiW, deltaTW,avgNumATgoal    
        fitWeights = np.array([fitWeights])       

    # Store Evolutionary Configuration (for Log)
    evoConfig = [popSize, generations,selectMethod,elitism,crossProb,mutProb,numRept,timeOutT,fitWeights]    
    estimatetEvoT = popSize*timeOutT*numRept*generations
    # estimatetEvoT += estimatetEvoT*0.05
    raw_input("\nEstimated Evolution time: %f s  ... Press Enter to continue ...\n" %estimatetEvoT)
   
    # Ranges of evolutionary genes (used for initial random population AND for mutation operation)
    # Example param values: neighRadii = 150.0; cohW = 4.0/100 ; repW = 23.0/100; alignW = 20.0/100; crowdingThr = 10.0; leaderW = 17
    neighRadii = [0.0,casualAgentR*15]
    cohW = [0.0,1.0]
    repW = [0.0,1.0]
    alignW = [0.0,1.0]
    crowdingThr = [0.0,casualAgentR*15]
    leaderW = [0.001,10.0]
    geneRange = [neighRadii,cohW,repW,alignW,crowdingThr,leaderW] 
    
    # Population History: layer-> generation; Row-> indiviual; Col -> gene
    popHist = np.zeros((popSize,len(geneRange),generations)) 
    fitHist = np.zeros((popSize,fitWeights.shape[1]+1,generations)) 
    maxFitHist = np.zeros((1,generations)) 
    avgFitHist = np.zeros((1,generations)) 
    minFitHist = np.zeros((1,generations)) 

    " Fitness plotting config. "
    plt.close('all')  
    plt.figure()
    plt.xlabel('Generations')
    plt.ylabel('Fitness')
    plt.title('Average Fitness per Evolved Generation')
    plt.grid(True)
    plt.pause(0.05)
    plt.gca().set_xlim(0,generations-1)
    plt.ion()
    plotGen = np.empty((0))


    ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Evolution <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"   
    initT = datetime.now().time()

    # Initial population (Either from the lastGenerationpopulation OR bestIndOfAllGenerations
    if popSeeding: popHist[:,:,0] = lastPop # bestIndPerAll # Seed Init. Population
    else : popHist[:,:,0]  = population(popSize,geneRange) # Random Init. population
    # "My" ideal poputaion
    # popHist[:,:,0] = kron( np.ones((popSize,len(geneRange))) , np.array([[150.0, 4.0/100, 23.0/100, 20.0/100, casualAgentR*3, 17.0]]) )

    for generation in range(generations-1):

        # Evolve population
        popHist[...,generation+1],fitHist[...,generation] = \
        evolution(popHist[...,generation],fitWeights,geneRange,numRept,selectMethod,elitism,crossProb,mutProb)
        maxFitHist[0,generation]  = fitHist[:,-1,generation].max()
        avgFitHist[0,generation] = np.divide(np.sum(fitHist[:,-1,generation]),fitHist.shape[0])
        minFitHist[0,generation]  = fitHist[:,-1,generation].min()

        print "Generation No. : %d " %generation

        # Plot max,average and min Fitness (live plotting)
        plotGen = np.concatenate((plotGen,[generation]))
        plt.plot(plotGen,maxFitHist[0,0:generation+1],marker = 'o',markerfacecolor = 'k',ms = 6.0, \
                                                      color= 'm', linewidth = 2.5, label = "Max.Fitness")  
        plt.plot(plotGen,avgFitHist[0,0:generation+1],marker = 'o',markerfacecolor = 'k',ms = 6.0, \
                                                      color= 'g', linewidth = 2.5, label = "Avg.Fitness")  
        plt.plot(plotGen,minFitHist[0,0:generation+1],marker = 'o',markerfacecolor = 'k',ms = 6.0, \
                                                      color= 'r', linewidth = 2.5, label = "Min.Fitness")  
        if not generation: 
            plt.legend(bbox_to_anchor=(0,1.02,1,0.15),ncol=3, mode="expand", borderaxespad=0.)
            plt.gcf().tight_layout(rect=[0,0,1,0.925])
        plt.pause(0.05)

        # User-> Cancel Evolution on certain percentage of the generations
        if (not (generation%math.ceil(interruptperc*generations)) and generation and interruptEvo):
            userInterrupt = raw_input("\n   Generation No. %d . Do you wish to continue: [Yes:1 ; No: 0]" %generation)
            if(userInterrupt == '0'):
                print ("   End of Evolution. Evaluating Fitness of last evolved generation (No. %d)\n" %(generation+1))
                break
            print("\n")

    # Get Fitness of last evolved population
    _ , fitHist[...,generation+1] = evolution(popHist[...,generation+1],fitWeights,geneRange,numRept,selectMethod,elitism,crossProb,mutProb)
    maxFitHist[0,generation+1]  = fitHist[:,-1,generation+1].max()
    avgFitHist[0,generation+1] = np.divide(np.sum(fitHist[:,-1,generation+1]),fitHist.shape[0])
    minFitHist[0,generation+1]  = fitHist[:,-1,generation+1].min()

    print "Generation No. : %d " %(generation+1)

    currentT = datetime.now().time()
    ellapsedT = (currentT.hour - initT.hour)*3600.0 + (currentT.minute-initT.minute)*60.0 + (currentT.second-initT.second)
    print ("\nEvolution time: %f s" % ellapsedT)

    # Plot average Fitness of last evolved population
    plotGen = np.concatenate((plotGen,[generation+1]))
    plt.plot(plotGen,maxFitHist[0,0:generation+2],marker = 'o',markerfacecolor = 'k',\
                                                  ms = 6.0,color= 'm', linewidth = 2.5)  
    plt.plot(plotGen,avgFitHist[0,0:generation+2],marker = 'o',markerfacecolor = 'k',\
                                                  ms = 6.0,color= 'g', linewidth = 2.5)
    plt.plot(plotGen,minFitHist[0,0:generation+2],marker = 'o',markerfacecolor = 'k',\
                                                  ms = 6.0,color= 'r', linewidth = 2.5)
    plt.pause(0.05)
    print "\n .......... End of Evolution :D .......... \n" 


    ">>>>>>>>>>>>>>>>>>>>>>>>>>>> Save Genetic Information <<<<<<<<<<<<<<<<<<<<<<<<<<<<"   
    genomeSave = 0 # Default
    genomeSave = int(raw_input("Save populations genomes and evolution configuration ? : [1:Yes; 0: No]"))    
    
    # Obtain Best individual PER generation
    allFit = fitHist[:,-1,0:generation+2] 
    _,rowIxBests = allFit.max(0),allFit.argmax(0) # Find best individual per generation
    # _,genIxBest= BestFitPerGen.max(0),BestFitPerGen.argmax(0) # Find the best among all generations

    bestIndPerGen = np.zeros(( (generation+2),len(geneRange) ))
    for gen in range(generation+2): # NOTE: THIS CAN BE ELIMINATED
        bestIndPerGen[gen,:] = popHist[rowIxBests[gen],:,gen]

    if genomeSave:
        popSeedFile = raw_input("Press ENTER to save in %s file or input the number of the desired saving file:\
                                   " % logPath[-len("evoLog_tryN.pckl"):-1])

        if popSeedFile: 
            logPath = list(logPath)
            logPath[-6] = popSeedFile
            logPath = ''.join(logPath)

        print("...Saving evolutionary data... ")
        # Obtain Best - N individuals in ALL generations
        allFit = allFit.reshape(1,popSize*generations)
        Nbest = popSize
        bestIdx = allFit.argsort()[0][-Nbest::][::-1] 
        ind,gen = [],[]

        for i in range(len(bestIdx)): # Transform lienar index -> to -> Matrix Index (row,col)
            ind.append(int(math.ceil(bestIdx[i]/generations)))
            gen.append(np.fmod(bestIdx[i],generations))
        bestIndPerAll = popHist[ind,:,gen]

        # Obtain last evolved generation
        lastPop = popHist[...,-1]

        # Store Fitness Plotting Data
        fitPlotData = [maxFitHist , avgFitHist , minFitHist]

        f = open(logPath, 'w')
        pickle.dump([lastPop,bestIndPerAll,evoConfig,fitHist,fitPlotData,bestIndPerGen,ellapsedT], f)
        f.close()


    ">>>>>>>>>>>>>>>>>>>>>>>>>>>>> Generations Simulation <<<<<<<<<<<<<<<<<<<<<<<<<<<<<"   
    simuEvolved = 0 # Default
    simuEvolved = int(raw_input("\nDo you want to : [1:Simulate evolved agents ; 0: QUIT]"))    
    
    if simuEvolved:
        visualON = 1
        if int((generation+2)*percGenDisplay):gens2show = np.arange(0,generation+2,int((generation+2)*percGenDisplay))
        else: gens2show = np.arange(0,(generation+2),1)
       
        if (generation+1) not in gens2show.tolist():
            gens2show = np.concatenate((gens2show,[generation+1]))

        
        for index,val in np.ndenumerate(gens2show):
            print "Simulating generation No. %d" %val
            simuParams = np.array([0,visualON,timeOutT], dtype= 'float64') 
            simuParams = np.insert(simuParams,1,bestIndPerGen[val,:]).tolist() # insert in index=1 of simuParams
            _ = boidsSimu.main(simuParams)
          
    closeGraph = 0
    closeGraph = raw_input("\nDo you want to keep the Fitness Evolution Graph: [1:Yes ; 0:No]")
    if (closeGraph =='0'): plt.close()  

if __name__ == "__main__":
    main()

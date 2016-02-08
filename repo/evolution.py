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
import random as rand
import numpy as np 
import matplotlib.pyplot as plt
import pickle
import boidsSimu
import sys,math
from datetime import datetime 


" Simulation Variables "
scenarioSize = width,height = 800,600 #scenario dimensions

" Create Individuals -> each posible solution"
def individual (geneRange):
    # Concatenate in a list each gene value of the individual and then return it as a numPy array
    return np.array(  [rand.uniform(gene[0],gene[1]) for gene in geneRange]  )

" Create Population -> collection of all individuals"
def population (popSize, geneRange):
    return np.hstack( (individual(geneRange) for pop in range(popSize)) ).reshape(popSize,len(geneRange))

" Fitness Function -> determine how effective is each individual solution"
def fitness (individual,fitWeights,fitVals):
    return fitWeights.dot(fitVals)

" Evolutionary Process -> 0.Simulation | 1. Selection | 2.CrossOver | 3.Mutation "
def evolution (pop,fitWeights,geneRange,selectMethod,mutProb): # elitism-rank method-SUS method-torunament method
    popSize = pop.shape[0]
    fitHistAux = np.empty((popSize,fitWeights.shape[1]+1)) 

    " 0.Simulation " # NOTE: MULTIPLE ROUNDS FOR EACH INDIVIDUAL AND AVERGAE FITNESS
    for ind in range(popSize):
        simuParams = np.array([0,visualON,timeOutT], dtype= 'float64') 
        simuParams = np.insert(simuParams,1,pop[ind,:]).tolist() # insert in index=1 of simuParams
        fitVals = boidsSimu.main(simuParams)
        fitness = fitWeights.dot(fitVals.transpose()) # Linear Combination Fitness Function
        fitHistAux[ind,:] = np.concatenate((fitVals,fitness),axis=1)

    " 1.Selection "
    # if elitism:

    numCouples = popSize # number of couples to generate
    if(selectMethod[0] == 'roulette'):
        # Roulette-Wheel Selection
        probRand = selectMethod[1]
        selected = roulette(probRand,numCouples,fitHistAux[:,-1])   
    else:
        print "ERROR: No selection method specified"
        sys.exit()

    " 2. CrossOver "
    newPop = np.zeros((popSize,pop.shape[1]))
    ind = 0
    for couple in selected:
        parentA = np.array([pop[couple[0],:]])
        parentB = np.array([pop[couple[1],:]])
        crossI = rand.randint(0,math.floor((pop.shape[1])/2)-1 ) 
        crossF = rand.randint(math.floor((pop.shape[1])/2),pop.shape[1]-1)

        newPop[ind,:] = np.hstack((parentA[0,0:crossI+1], parentB[0,crossI+1:crossF+1], parentA[0,crossF+1:] ))
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
            randORroul = rand.uniform(0.0,1.0)    

            if randORroul > probRand: # Roulette parent selection
                while(1):
                    dart = rand.uniform(0,1)   
                    try: 
                        ind = selectProb[1,np.where(selectProb[0,:]>=dart)[0][0]] # [0] -> select index list , [0] -> select first occurance
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
    " Evolutionary Configuration"
    popSize = 10# Population Size
    generations = 10 # Number of Evolution generations
    selectMethod = ['roulette',0.0]
    mutProb = [0.0,0.1] # [prob.OfMutation , %OfMutation]
    evoConfig = [popSize, generations,selectMethod,mutProb]
    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    global visualON # 0: dont show visual environment ; 1: show visual environment
    visualON = 0
    global timeOutT # time in [s] after which simulation will abort 
    timeOutT = 8
    percGenDisplay = 0.2
    interruptEvo = 0
    interruptperc =  0.25

    " Evolutionary Variables "
    # Ranges of parameters to evolve
    # Example param values: neighRadii = 150.0; cohW = 4.0/100 ; repW = 23.0/100; alignW = 20.0/100; crowdingThr = 10.0; leaderW = 17
    neighRadii = [0,min(scenarioSize)/2]
    cohW = [0.0,1.0]
    repW = [0.0,1.0]
    alignW = [0.0,1.0]
    crowdingThr = [0.0,min(scenarioSize)/2]
    leaderW = [0.0,30.0]
    geneRange = [neighRadii,cohW,repW,alignW,crowdingThr,leaderW] 
    
    # Fitness Weights
    interCollW = 1.0/6
    medGRadiiW = 1.0/6
    deltaTW = 1.0/6
    t2GoalW = 1.0/6
    numATgoalW = 1.0/6
    evoSuccessW = 1.0/6
    fitWeights = interCollW, medGRadiiW, deltaTW,t2GoalW,numATgoalW,evoSuccessW    
    fitWeights = np.array([fitWeights])
    
    # Population History: layer-> generation; Row-> indiviual; Col -> gene
    popHist = np.zeros((popSize,len(geneRange),generations)) 
    fitHist = np.zeros((popSize,fitWeights.shape[1]+1,generations)) 
    avgFitHist = np.zeros((1,generations)) 

    # Fitness plotting config.
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

    " Evolution "   
    initT = datetime.now().time()

    # # Initial population
    popHist[:,:,0]  = population(popSize,geneRange) 
    # popHist[:,:,0] = np.array([[150.0, 4.0/100, 23.0/100, 20.0/100, 10.0, 17.0]])
    for generation in range(generations-1):

        # Evolve population
        popHist[...,generation+1],fitHist[...,generation] = \
        evolution(popHist[...,generation],fitWeights,geneRange,selectMethod,mutProb)
        avgFitHist[0,generation] = np.divide(np.sum(fitHist[:,-1,generation]),fitHist.shape[0])
        
        print "Generation No. : %d " %generation

        # Plot average Fitness (live)
        plotGen = np.concatenate((plotGen,[generation]))
        plt.plot(plotGen,avgFitHist[0,0:generation+1],marker = 'o',markerfacecolor = 'b',\
                                                      ms = 6.0,color= 'r', linewidth = 2.5)   
        yVal = max(plt.gca().get_lines()[0].get_ydata())   
        plt.text(generations-3,yVal-0.1*yVal,r'$\mu=%f$' %mutProb[0], bbox=dict(facecolor='red', alpha=0.5))
        plt.pause(0.05)

        # User-> Cancel Evolution on certain percentage of the generations
        if (not (generation%math.ceil(interruptperc*generations)) and generation and interruptEvo):
            userInterrupt = raw_input("\n   Generation No. %d . Do you wish to continue: [Yes:1 ; No: 0]" %generation)
            if(userInterrupt == '0'):
                print ("   End of Evolution. Evaluating Fitness of last evolved generation (No. %d)\n" %(generation+1))
                break
            print("\n")

    # Get Fitness of last evolved population
    _ , fitHist[...,generation+1] = evolution(popHist[...,generation+1],fitWeights,geneRange,selectMethod,mutProb)
    avgFitHist[0,generation+1] = np.divide(np.sum(fitHist[:,-1,generation+1]),fitHist.shape[0])
    print "Generation No. : %d " %(generation+1)

    currentT = datetime.now().time()
    ellapsedT = (currentT.hour - initT.hour)*3600.0 + (currentT.minute-initT.minute)*60.0 + (currentT.second-initT.second)
    
    # Plot average Fitness of last evolved population
    plotGen = np.concatenate((plotGen,[generation+1]))
    plt.plot(plotGen,avgFitHist[0,0:generation+2],marker = 'o',markerfacecolor = 'b',\
                                                      ms = 6.0,color= 'r', linewidth = 2.5)  
    yVal = max(plt.gca().get_lines()[0].get_ydata())   
    plt.text(generations-3,yVal-0.1*yVal,r'$\mu=%f$' %mutProb[0], bbox=dict(facecolor='red', alpha=0.5))
    plt.pause(0.05)

    print "\n\n>>>>>>>>>>>>>>>>>>>>> End of Evolution :D <<<<<<<<<<<<<<<<<<<<<<<\n\n" 

    " Save Genetic Information" 
    print("\n   ...Saving evolutionary data... ")
    allFit = fitHist[:,-1,0:generation+2] 
    BestFitPerGen,rowIxBests = allFit.max(0),allFit.argmax(0) # Find best individual per generation
    _,genIxBest= BestFitPerGen.max(0),BestFitPerGen.argmax(0) # Find the best among all generations

    bestIndPerGen = np.zeros(( ( (generation+2)+3 ),len(geneRange) ))
    for gen in range(generation+2):
        bestIndPerGen[gen,:] = popHist[rowIxBests[gen],:,gen]
    bestIndPerGen[-3,:] = popHist[rowIxBests[genIxBest],:,genIxBest]
    bestIndPerGen[-2,0] = genIxBest
    bestIndPerGen[-1,0] = rowIxBests[genIxBest]

    f = open('C:\Users\IVAN\Desktop\evoPy\evoLog\evoLog_try3.pckl', 'w')
    pickle.dump([bestIndPerGen, fitHist[:,:,0:generation+2], avgFitHist[0,0:generation+2], evoConfig,ellapsedT], f)
    f.close()

    simuEvolved = 0 # Default
    simuEvolved = int(raw_input("Do you want to : [1:Simulate evolved agents ; 0: QUIT]"))    
    print("\n")

    " Generations Simulation"
    if simuEvolved:
        visualON = 1
        if int((generation+2)*percGenDisplay):gens2show = np.arange(0,generation+2,int((generation+2)*percGenDisplay))
        else: gens2show = np.arange(0,(generation+2),1)
       
        if (generation+1) not in gens2show.tolist():
            gens2show = np.concatenate((gens2show,[generation+1]))

        for index,val in np.ndenumerate(gens2show):
            print "Simulating generation No. %d" %val
            simuParams = np.array([0,visualON,timeOutT], dtype= 'float64') 
            choosen = rand.randint(0,popSize-1)
            simuParams = np.insert(simuParams,1,popHist[choosen,:,val]).tolist() # insert in index=1 of simuParams
            _ = boidsSimu.main(simuParams)

    closeGraph = 0
    closeGraph = raw_input("\nDo you want to close the Fitness Evolution Graph: [Yes:1 ; No: 0]")
    if (closeGraph =='1'): plt.close()  

if __name__ == "__main__":
    main()

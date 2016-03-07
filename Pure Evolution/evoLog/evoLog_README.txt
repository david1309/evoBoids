-------------------------------------------------------------------
-------------------------------------------------------------------

GENERAL DESCRIPTION : 
    
* try 1 - to try 5 where experiments (not the first one, those disappeared. But they
the first experiments after some major modifications)
* try 1,2 = FAIl and try 2,3 and 5 area bit ok, but their parameters surpass limits or
are ilegal or ilogical
* try 6.XXX = best evolved individuals
* try 7-8 = exploration of different paramenters, some failed other "kind" of succeeded 

* BestOFBest: evoLog_try64 && evoLog_try642 && evoLog_try644 && evoLog_try646 && evoLog_try7

-------------------------------------------------------------------
-------------------------------------------------------------------

EVOLOGS DESCRIPTION :

evoLog_try1 --> 
> Evo Description: specialRFact = 2.5 ; std = 30 ; deltaTV average obtained by numATgoalW  
           
evoLog_try2 --> 
> Evo Description:specialRFact = 1.75 ; Gaussian medGRadiiV w/ ideal = 100, std = 40 
deltaTV average obtained by numBoids

evoLog_try3 --> 
> Evo Description: Used evoLog_try2 as populationSeed

evoLog_try4 --> 
> Evo Description: Used evoLog_try3 as populationSeed + weightChanges                

evoLog_try5 --> 
> Evo Description: NO PREVIOUS SEEDING + Simu Params: noBoids = 15 ; maxVel = 6.0 ; specialRFact = 1.75
MedGRadii Calculation:  idealGRadii = 100 ; std = 40 ...
IMPORTANT CHANGE OF INTERVALS:
neighRadii = [0,casualAgentR*10]
crowdingThr = [0.0,casualAgentR*10]
leaderW = [0.001,10.0]

evoLog_try6--> 
> Evo Description: Intermediate point between exploration and exploitation ... not hight penalty for collisions
> Performance: [interCollW,medGRadiiW]:[ 0.38785626  0.9255998 ] 
> evoConfig: [10, 35, ['roulette', 0.4], [1, 2], 0.9, [0.065, 0.7], 3, 10, array([[ 0.35,  0.31,  0.17,  0.17]])]
> Analisis: OK , Good leader following BUT repW values exceed max range  
{:) -> high compactness ; :( -> OUT of RANGE values (not saturated) and just a bit to many collisionson}
                            
evoLog_try61--> 
> Evo Description: fastEvo seeded ONLY with try6 evoConfig + mutation saturation + high interColl weight
> Performance: [interCollW,medGRadiiW]:[0.20481075  0.98786906]
> Analisis: DOESNT EVEN MOVE, great for not colliding, but it DOESNT EVEN MOVE, swarm gets stuck 
{ :| :( ;( }    

evoLog_try62--> 
> Evo Description: fastEvo seeded ONLY with try6 evoConfig + mutation saturation + NOT that high interColl weight
> Performance: [interCollW,medGRadiiW]:[0.08929192  0.24758025]
> Analisis: UNACCEPTABLE collisions AND Good LEADER following { :| :|}    

evoLog_try63--> 
> Evo Description: SlowEvo seeded ONLY with try6 evoConfig+ mutation saturation + NOT that high interColl weight
> Performance: [interCollW,medGRadiiW]:[0.67528894  0.99926705] 
> evoConfig: [10, 20, ['roulette', 0.4], [1, 2], 0.9, [0.065, 0.7], 3, 10, array([[ 0.35,  0.25,  0.15,  0.25]])]]
> Analisis: OK,{:D -> low collisions ; :( -> not natural movement, bumpy and not smooth}
                                         
evoLog_try64--> 
> Evo Description: SlowEvo seeded ONLY with try6 evoConfig+ mutation saturation + NOT that high interColl weight
+ a bit less pressure to being a compact swarm (lower medRadii & deltaT weights)
> Performance: [interCollW,medGRadiiW]:[0.94067797  0.71296587] 
> evoConfig: [10, 20, ['roulette', 0.4], [1, 2], 0.9, [0.065, 0.7], 3, 10, array([[ 0.3 ,  0.2 ,  0.15,  0.35]])]
> Fitness Ploting: MIN: 0.15-0.4; AVG: 0.4-0.8; MAX: 0.9-0.98
> Analisis: VOILA,{:D -> More compact visualization and smoother movement ; :( -> low leader attention}
                                                                              
evoLog_try642--> 
> Evo Description: Try 6 Params + No Pop.Seeding + Try 64 Weights
> Performance: [interCollW,medGRadiiW]:[0.95421245  0.999215] 
> evoConfig: [10, 20, ['roulette', 0.4], [1, 2], 0.9, [0.065, 0.7], 3, 10, array([[ 0.3 ,  0.2 ,  0.15,  0.35]])]
> Fitness Ploting: MIN: 0.18-0.8; AVG: 0.38-0.9; MAX: 0.5-0.98
> Analisis: VOILA !!!,{:D -> High leader attention ; :( -> less compact visualization and just a bit bumpy movement}

evoLog_try643--> 
> Evo Description: Try 64 Params + No Pop.Seeding + bigger pop (15) + generations (30)
> Performance: [interCollW,medGRadiiW]:[0.99125874  0.50112171] 
> evoConfig: [15, 30, ['roulette', 0.4], [1, 2], 0.9, [0.065, 0.7], 3, 10, array([[ 0.3 ,  0.2 ,  0.15,  0.35]])]
> Fitness Ploting: MIN: 0.25-0.50; AVG: 0.45-0.8; MAX: 0.6-0.95
> Analisis: OK,{:D -> High leader attention, really nice and smooth movement ; 
:( -> no compactness, they are just loosely together because of alignment}

evoLog_try644--> 
> Evo Description: Try 64 Params + No Pop.Seeding + bigger pop (15) + generations (30)
+ Modified weights towards a greater EXPLORATION 
> Performance: [interCollW,medGRadiiW]:[0.95246479  0.91367358] 
> evoConfig: [15, 30, ['roulette', 0.4], [1, 2], 0.7, [0.07, 0.5], 3, 10, array([[ 0.3 ,  0.2 ,  0.15,  0.35]])]
> Fitness Ploting: MIN: 0.2-0.38; AVG: 0.32-0.72; MAX: 0.5-0.1
> Analisis: VOILA !!!,{:D -> High leader attention (even do they can get disconnected really easily), high compactess,
quite nice movement ; :( -> really rigid group, easy leader deteachment, might be difficult for a predator to brake}

evoLog_try645--> 
> Evo Description: Try 64 Params + No Pop.Seeding + bigger pop (15) + generations (30)
Higher mutation rate, lower crossover rate
> Analisis: IDK { :| -> NOT useful, but INTERESTING. Since through the leaders attraction one can
"pull the swarm" towards a especific place }
        
evoLog_try646--> 
> Evo Description: Try 64 Params + No Pop.Seeding + bigger pop (15) + generations (20)
Higher mutation rate, lower crossover rate
> Analisis: VOILA
                                            
evoLog_try7--> 
> Evo Description: More exploitation
> Performance: [interCollW,medGRadiiW]:[0.87372014  0.64273792] 
> Analisis: OK, nice movement {:D -> Smoth displacement ; :( -> swarm splits to easilly and low leader attention}    
                                                        
evolog_try8--> 
> Evo Description: More exploration + Not seeded
> Analisis: IDK { :| -> NOT useful, but INTERESTING. Since through the leaders attraction one can
"pull the swarm" towards a especific place }
                                                            
-------------------------------------------------------------------
-------------------------------------------------------------------
                                                            
EVOBOIDS PRESENTATION VIDEO:
                                                            
FAILURE:
    2 -> groups collapses && huge neigh radii (show 2 gen.)
    61 -> to much coh and rep , therefore, dont even move and get stuck in a oscillatory in/out motion (show 1 gen.)
EVOLUTION GENERATIONS:
    6 -> good progressive evolution, really bumpy at the end (show all gen. with n-steps)
SUCESS: (show 1 (best) gen.)
    643 --> low cohesion - loosely together group && high leader attention
    64 --> Compact group && high momentum compared to leader following
    642 --> Compact group and high leader attention and "chaotic" bumby grouping
                                                            
-------------------------------------------------------------------
-------------------------------------------------------------------
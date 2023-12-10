# -*- coding: utf-8 -*-
"""
Created on Mon Sep 25 06:43:58 2023

@author: barib
"""
from pyMaze import maze, agent, COLOR
import numpy as np
from aStarModifie import aStar

goal_10 = (5, 5)
goal_20 = (1, 1)
start_10 = (10, 10)
start_20 = (20, 1)

m = maze ()
m.CreateMaze(loadMaze='mazeCustom_20.csv', theme=COLOR.light)
a = agent( m, start_20[0], start_20[1], goal=goal_20, filled=True, footprints=True)
x0 = a.x
y0= a.y
#path_2 = [(10,1)]
av = 0
ar = 0
gauche = 0
droite = 0
point_critique = 0
dico_critique = {}
critic_point_reached = False
list_obstacle = []
liste_points_critiques = []
is_obstacle = False
x = 0
y = 0
avant = False 
arriere = False
gauche = False
droite = False
nbre_critique = 0
print(f" x: {a.x} et y : {a.y}")

path_2 = [(a.x, a.y)]
path_2_initial = []
list_obstacle = [(a.x, a.y)]
voisins = []
points_retour = []
cles = []
les_backtracking = {}
    
def ajouter_voisin(a,b):
    global av, ar
    i = 0
    x = 0
    y = 0
    while i < 3:
        
        if (i==0):
            x = a
            y = b-1
            if (x,y) in list(m.maze_map.keys()):
                if ((x,y) not in path_2 and ((x,y) not in voisins)):
                    if ((x, y) not in list_obstacle):
                        voisins.append((x, y))
                    print(f"Ajout de {(x, y)}")
        if (i==1):
            if (avant):
                x = a-1
            else:
                x = a+1
            y = b
            if (x,y) in list(m.maze_map.keys()):
                if ((x,y) not in path_2 and ((x,y) not in voisins)):
                    if ((x, y) not in list_obstacle):
                        voisins.append((x, y))
                    print(f"Ajout de {(x, y)}")
        if (i==2):
            x = a
            y = b+1
            if (x,y) in list(m.maze_map.keys()):
                if (((x,y) not in path_2) and ((x,y) not in voisins)):
                    if ((x, y) not in list_obstacle):
                        voisins.append((x, y))
                    print(f"Ajout de {(x, y)}")
        i += 1

def euclidienne (pc):
    distances = {}
    pc = np.array(pc)
    print(f"pc : {pc}")
    for point in points_retour:
        #point = np.array(point)
        print(point)
        dist = np.sqrt(((pc - point)**2).sum()) ##distances[point]
        distances[point] = dist
    distances = sorted(distances.items(), key=lambda x: x[1], reverse=False)
    back_point = distances[0][0]
    return back_point
    
        
    

def chercher_obstacle():
    for cle in m.maze_map.keys():
        murs = m.maze_map[cle]
        for w in murs.values():
            if w != 0:
                is_obstacle = False
                break
            is_obstacle = True
        if is_obstacle == True:
            list_obstacle.append(cle)
                      
def trouver_voisins(a, b):
    neighbors = []
    neighbors.append((a, b+1)) # s1
    neighbors.append((a-1, b+1)) # s2
    neighbors.append((a-1, b)) # s3
    neighbors.append((a-1, b-1) )# s4
    neighbors.append((a, b-1)) # s5
    neighbors.append((a+1, b-1) )# s6
    neighbors.append((a+1, b)) # s7
    neighbors.append((a+1, b+1)) # s8
    return neighbors

# candidates to be backtracking point
def sigma (v1, v2):
    if (v1 not in list_obstacle and v1 in list(m.maze_map.keys())) :
        # print("Premier pas ")
        if (v2 in list_obstacle or v2 not in list(m.maze_map.keys())):
            # print("2 e pas ")
            # print(f"V1 {v1} ET V2 : {v2}")
            retour = 1
            return retour
        else:
            retour = 0
            return retour
    else:
        retour = 0
        return retour
    
def back_tracking_points():
    for point in path_2:
        cout = 0
        jirani = trouver_voisins(point[0], point[1])
        
        cout = sigma(jirani[0], jirani[7]) + sigma(jirani[0], jirani[1]) + sigma(jirani[4], jirani[3]) + sigma(jirani[4], jirani[5]) + sigma(jirani[6], jirani[5]) + sigma(jirani[6], jirani[7])
        #print(sigma((10,2), (11,2)) + 0)
        if cout > 0:
            points_retour.append(point)


chercher_obstacle()  
# Recherche des voisins du point de depart 
if ((a.x, a.y-1) in list(m.maze_map.keys()) and (a.x, a.y-1) not in list_obstacle):
    voisins.append((a.x, a.y-1))
    print(f"Ajout de {(a.x, a.y-1)}")
if ((a.x-1, a.y) in list(m.maze_map.keys()) and (a.x-1, a.y) not in list_obstacle):
    voisins.append((a.x-1, a.y))
    print(f"Ajout de {(a.x-1, a.y)}")
if ((a.x, a.y+1) in list(m.maze_map.keys()) and (a.x, a.y+1) not in list_obstacle):
    voisins.append((a.x, a.y+1))
    print(f"Ajout de {(a.x, a.y+1)}")

 
#print("Initiales obs :", list_obstacle)
compteur = 0
running = True

# algotithme du Coverage path planning

while running : # running
    x = path_2[-1][0]
    y = path_2[-1][1]
    # check du nord
    if (m.maze_map[x, y]['N'] and (x-1, y) not in list_obstacle):
        avant = True
        path_2.append((x-1, y))
        if nbre_critique > 0:
            dico_critique[liste_points_critiques[-1]].append((x-1, y))
        list_obstacle.append((x-1, y))
        if ((x-1, y) in voisins):
            voisins.remove((x-1, y))
            print(f"Suppression de {(x-1, y)} et taille= {len(voisins)}")
        ajouter_voisin(x-1, y)
        avant = False

    # Check Sud 
    elif(m.maze_map[x, y]['S'] and (x+1, y) not in list_obstacle):
        path_2.append((x+1, y))
        if nbre_critique > 0:
            dico_critique[liste_points_critiques[-1]].append((x+1, y))
        list_obstacle.append((x+1, y))
        if ((x+1, y) in voisins):
            voisins.remove((x+1, y))
            print(f"Suppression de {(x+1, y)} et taille= {len(voisins)}")
        ajouter_voisin(x+1, y)
            
    # Check Est
    elif (m.maze_map[x, y]['E'] and (x, y+1) not in list_obstacle):
        path_2.append((x, y+1))
        if nbre_critique > 0:
            dico_critique[liste_points_critiques[-1]].append((x, y+1))
        list_obstacle.append((x, y+1))
        if ((x, y+1) in voisins):
            voisins.remove((x, y+1))
            print(f"Suppression de {(x, y+1)} et taille= {len(voisins)}")
        ajouter_voisin(x, y+1)
            
    # check Ouest
    elif (m.maze_map[x, y]['W'] and (x, y-1) not in list_obstacle): 
        path_2.append((x, y-1))
        if nbre_critique > 0:
            dico_critique[liste_points_critiques[-1]].append((x, y-1))
        list_obstacle.append((x, y-1))
        if ((x, y-1) in voisins):
            voisins.remove((x, y-1))
            print(f"Suppression de {(x, y-1)} et taille= {len(voisins)}")
        ajouter_voisin(x, y-1)
    else:
        print(f"!!!!Point critique au {compteur}e tour")
        if nbre_critique==0:
            path_2_initial = path_2.copy()
        nbre_critique += 1
        point_critique = (path_2[-1][0], path_2[-1][1])
    
        critic_point_reached = True
        #break
    if (critic_point_reached):
        back_tracking_points()
        print(f"Point critique {nbre_critique}")
        if (len(points_retour)==0):
            print("Fin balayage")
            break
        liste_points_critiques.append(point_critique)
        retour = euclidienne(point_critique)
        path_critique = aStar(m, point_critique, retour)
        
        cles = list(path_critique.keys())
        cles.reverse()
        cles.append(path_critique[cles[-1]])
        les_backtracking[point_critique] = cles.copy()
        dico_critique[point_critique] = []#cles.copy()
        path_2.extend(cles[1:])
        points_retour = []
        path_critique = []
        cle = []
        critic_point_reached = False
    compteur += 1
    
# Tracage des chemins suivis par le robot
m.tracePath({a:path_2_initial}, delay=30)
couleurs = [COLOR.blue, COLOR.red, COLOR.green, COLOR.turquoise, COLOR.chocolate, COLOR.cyan,  COLOR.orange, COLOR.pink, COLOR.navajo, COLOR.yellow, COLOR.black]
les_agents = []
for k in range(len(liste_points_critiques)):
    chemin_2 = dico_critique[liste_points_critiques[k]]
    chemin_3 = les_backtracking[liste_points_critiques[k]]
    robot = agent(m, chemin_2[0][0], chemin_2[0][1], goal=goal_20, filled=True, footprints=True, color=couleurs[k+1])
    robot_2 = agent(m, chemin_3[0][0], chemin_3[0][1], goal=goal_20, filled=False, footprints=True, color=couleurs[k+1])
    m.tracePath({robot_2:chemin_3}, delay=30)
    m.tracePath({robot:chemin_2}, delay=30)


m.run()
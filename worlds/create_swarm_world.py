#!/usr/bin/python
import random,math
import fileinput

world_template_file="surface.world.template"
world_file="surface.world"

RNG_SEED=111


ASV_POSITIONS=""

#dimensions of arena in meters
arena_min_x=-10.0
arena_max_x=+10.0
arena_min_y=-10.0
arena_max_y=+10.0
arena_water_level=10.0

import sys

random.seed(RNG_SEED)

if len(sys.argv) == 3:
    RNG_SEED=int(sys.argv[1])
    NUM_ROBOTS=int(sys.argv[2])
else :
    print "Random number generator seed and number of robots not specified as argument. Taking default values of 111 for seed, and 2 for the number of robots in swarm"
    RNG_SEED=111
    NUM_ROBOTS=2


for x in range(0, NUM_ROBOTS):
    x_pos=str(random.uniform(arena_min_x,arena_max_x))
    y_pos=str(random.uniform(arena_min_y,arena_max_y))
    z_pos=str(arena_water_level)

    x_rot=str(0.0) #rotation around x-axis in radians (axis in body frame of reference)
    y_rot=str(0.0) #rotation around y-axis in radians (axis in body frame of reference)
    z_rot=str(random.uniform(-math.pi,math.pi)) #rotation around z-axis in radians (axis in body frame of reference)

    pos=x_pos + " " + y_pos + " " + z_pos
    rot=x_rot + " " + y_rot + " " + z_rot
    ASV_POSITION="<include><uri>model://surfacevehicle_"+str(x)+"</uri><pose>"+pos+" "+rot+"</pose></include>"
    ASV_POSITIONS=ASV_POSITIONS+"\n\n\t"+ASV_POSITION
    #print "We're on time %s %s" % (ASV_POSITIONS,x_pos)


templatefile = open(world_template_file, 'r')
worldfile = open(world_file, 'w')

for line in fileinput.input(world_template_file):
    if "INCLUDE_ASVS_IN_WORLD_HERE" in line :
        worldfile.write(line.replace("INCLUDE_ASVS_IN_WORLD_HERE", ASV_POSITIONS))
    elif "WATER_LEVEL" in line :
        worldfile.write(line.replace("WATER_LEVEL", str(arena_water_level)))    
    else:
        worldfile.write(line)

templatefile.close()
worldfile.close()


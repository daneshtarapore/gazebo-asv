#!/bin/bash

NUM_ROBOTS=2

for (( i=0;i<$NUM_ROBOTS;i++)); do
    mkdir -p surfacevehicle_$i/meshes
    /bin/cp -f asv_model/model.sdf surfacevehicle_$i
    /bin/cp -f asv_model/model.config surfacevehicle_$i
    cp asv_model/meshes/proto7_meters.dae surfacevehicle_$i/meshes
    sed -i -e s/surfacevehicle/surfacevehicle_${i}/g surfacevehicle_${i}/model.sdf
done 


rm ./image/left/*
rm ./image/right/*

mkdir image
mkdir image/left
mkdir image/right

./bin/MakeSceneEurocStereoVIO_ros ./examples/simulationCamera.yaml

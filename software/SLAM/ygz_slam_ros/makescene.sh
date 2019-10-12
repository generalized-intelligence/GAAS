cd image/left
rm *
cd ../right
rm *
cd ../..

./bin/MakeSceneEurocStereoVIO_ros ./examples/simulationCamera.yaml

rm test.log
rm image/left/*
rm image/right/*
./bin/MakeSceneEurocStereoVIO_ros ./examples/simulationCamera.yaml >> test.log

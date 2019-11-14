rm test.log

mkdir loopclosure_result
mkdir loopclosure_result2

rm loopclosure_result/*.png
rm loopclosure_result2/*.png

.bin/demo_stereo "./image/scene.scn" "./image/small_voc.yml.gz" './image/left/' './image/right/' "./config/Q_mat.yaml"
# ./build/src/demo_stereo "./image/scene.scn" "./image/small_voc.yml.gz" './image/left/17.png' './image/right/17.png' "./config/Q_mat.yaml" >> test.log

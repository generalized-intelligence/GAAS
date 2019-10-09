rm -r loopclosure_result
mkdir loopclosure_result

rm log_controller_*
rm all.txt

./bin/controller_node "./image/scene.scn" "./image/small_voc.yml.gz"

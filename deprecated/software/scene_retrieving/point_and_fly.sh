rm -r loopclosure_result
mkdir loopclosure_result

rm relative_and_average_file.txt
rm relative_distance_file.txt

rm log_controller_*
rm all.txt

#gdb --args ./bin/controller_node "./image/scene.scn" "./image/small_voc.yml.gz"
./bin/controller_node "./image/scene.scn" "./image/small_voc.yml.gz"

#!/bin/bash
echo "Enter release version x.y.z [ENTER]:"
read version

# first do the submodule
cd okvis
git archive --format=zip --prefix=okvis/ --output=okvis-$version.zip master
#doxygen doxygen.config # regenerate docu
#scp -rq documentation/html/* sleutene@shell1.doc.ic.ac.uk:public_html/software/docs/okvis/$version/
#if [$? -ne 0]; then
#  echo "upload to server failed"
#  exit 1
#fi
scp okvis-$version.zip sleutene@shell1.doc.ic.ac.uk:public_html/software/
if [$? -ne 0]; then
  echo "upload to server failed"
  exit 1
fi

# now the whole thing
cd ..
#doxygen doxygen.config # regenerate docu
git archive --format=zip --prefix=okvis_ros/ --output=okvis_ros.zip master
unzip okvis_ros.zip -d .
rmdir okvis_ros/okvis
rm okvis_ros/.gitmodules
unzip okvis/okvis-$version.zip -d okvis_ros/
rm okvis_ros/release.sh # don't want to release this script...
zip -r okvis_ros-$version.zip okvis_ros/*
rm okvis_ros.zip
rm -rf okvis_ros
#scp -rq documentation/html/* sleutene@shell1.doc.ic.ac.uk:public_html/software/docs/okvis_ros/$version/
#if [ $? -ne 0 ]; then
#  echo "upload to server failed"
#  exit 1
#fi
scp okvis_ros-$version.zip sleutene@shell1.doc.ic.ac.uk:public_html/software/
#if [ $? -ne 0 ]; then
#  echo "upload to server failed"
#  exit 1
#fi
rm okvis/okvis-$version.zip
rm okvis_ros-$version.zip

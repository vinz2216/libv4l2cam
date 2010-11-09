make clean
make gstreamer
rm -rf deb/usr
mkdir deb/usr
mkdir deb/usr/bin
mkdir deb/usr/lib
mkdir deb/usr/lib/libcvm
cp v4l2stereo deb/usr/bin/
cp /usr/lib/libcvm/libcvm*.so deb/usr/lib/libcvm/
sudo chmod -R 0755 deb/usr
dpkg -b deb v4l2stereo.deb

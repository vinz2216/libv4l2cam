make clean
rm -rf ~/temp/pcloud
mkdir ~/temp
mkdir ~/temp/pcloud
mkdir ~/temp/pcloud/deb
mkdir ~/temp/pcloud/deb/DEBIAN
mkdir ~/temp/pcloud/deb/usr
mkdir ~/temp/pcloud/deb/usr/bin
make
cp pcloud ~/temp/pcloud/deb/usr/bin
cp deb/DEBIAN/control ~/temp/pcloud/deb/DEBIAN
cp deb/DEBIAN/copyright ~/temp/pcloud/deb/DEBIAN
cp deb/DEBIAN/docs ~/temp/pcloud/deb/DEBIAN
cp deb/DEBIAN/install ~/temp/pcloud/deb/DEBIAN
sudo chmod -R 0755 ~/temp/pcloud/deb/usr
dpkg -b ~/temp/pcloud/deb pcloud.deb
rm -rf ~/temp/pcloud

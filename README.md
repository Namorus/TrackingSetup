# TrackingSetup

Code for the TrackingSetup.

## Dependencies

- [libusb > 1.0.8](https://github.com/libusb/libusb)
- [libFTDI](http://www.intra2net.com/en/developer/libftdi/repository.php)
- [GeographicLib](http://geographiclib.sourceforge.net/html/)

### HowTo install dependencies
The following libraries may have dependencies on their own. The respective project homepage should clarify those.

#### libusb
```shell
git clone https://github.com/libusb/libusb.git
cd libusb
./autogen.sh
make
sudo make install
```
#### libFTDI
```shell
git clone git://developer.intra2net.com/libftdi
cd libftdi
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX="/usr" ../
make
sudo make install
```

#### GeographicLib
```shell
sudo apt-get install libgeographiclib-dev
```
You'll need a [magnetic model](http://geographiclib.sourceforge.net/html/magnetic.html) to be able to run TrackingSetup.

#include $(shell rospack find mk)/cmake.mk
ADAPTOBRANCH=adapto_0.2
# TODO: put tag in here
MORSERELEASE=adapto_0.2.5


all: morse update

morse: morse-src/build/bin
	cd morse-src/build;make install

morse-src:
	mkdir morse-src
	cd morse-src;git init
	cd morse-src;git remote add origin "https://github.com/kargm/morse.git" 

update: morse-src
	cd morse-src;git fetch
	cd morse-src;git checkout ${ADAPTOBRANCH}
	cd morse-src;git reset --hard ${MORSERELEASE}

morse-src/build/bin: update
  #create dir if does not exist
	mkdir -p morse-src/build
	cd morse-src/build;cmake  -D BUILD_ROS_SUPPORT=ON -D CMAKE_INSTALL_PREFIX=`pwd`/../../morse ..

clean:
	rm -rf morse
	rm -rf morse-src

#include $(shell rospack find mk)/cmake.mk
BLENDVERSION=2.63a
# TODO: get the right variant depending on system
BLENDRELEASE=blender-${BLENDVERSION}-linux-glibc27-x86_64
BLENDTAR=${BLENDRELEASE}.tar.bz2
BLENDURL=http://ftp.halifax.rwth-aachen.de/blender/release/Blender2.63/${BLENDTAR}

all: bin/blender

bin/blender: ${BLENDRELEASE}
  #create dir if does not exist
	mkdir -p bin
	cd bin;ln -fs ../${BLENDRELEASE}/blender

clean:
	rm -rf ${BLENDTAR}
	rm -rf ${BLENDRELEASE}
	rm -rf bin

${BLENDRELEASE}: ${BLENDTAR}
	tar -xf ${BLENDTAR}

${BLENDTAR}:
	wget ${BLENDURL}

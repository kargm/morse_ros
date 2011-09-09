# include $(shell rospack find mk)/cmake.mk
#include $(shell rospack find mk)/cmake.mk

PYYAMLRELEASE=PyYAML-3.09

PYYAMLTAR=${PYYAMLRELEASE}.tar.gz
PYYAMLURL=http://pyyaml.org/download/pyyaml/${PYYAMLTAR}

all: pyyaml


clean:
	rm -rf ${PYYAMLTAR}
	rm -rf ${PYYAMLRELEASE}
	rm -rf pyyaml

${PYYAMLRELEASE}: ${PYYAMLTAR}
	tar -xf ${PYYAMLTAR}

${PYYAMLTAR}:
	wget ${PYYAMLURL}

pyyaml: ${PYYAMLRELEASE}
	cd ${PYYAMLRELEASE};python3.2 setup.py install --prefix `pwd`/../pyyaml

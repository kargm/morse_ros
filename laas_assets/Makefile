# include $(shell rospack find mk)/cmake.mk
PKGVERSION=             0.7
DISTNAME=               gbM-${PKGVERSION}
PREFIX=`pwd`/..
#`rospack find gbM`

all: laas-assets

clean:
#	rm -rf laas-assets

laas-assets: Makefile
	git clone http://trac.laas.fr/git/robots/move3d-assets.git laas-assets

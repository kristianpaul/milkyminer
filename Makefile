#
# Authors: Xiangfu Liu <xiangfu@sharism.cc>
#                      bitcoin: 1CGeqFzCZnAPEEcigr8LzmWTqf8cvo8toW
#
# License GPLv3 or later.  NO WARRANTY.
#

BASEDIR=${CURDIR}

SYNTOOL?=xst
BOARD?=milkymist-one


host:
	make -C ${BASEDIR}/tools


bitstream: host
	make -C ${BASEDIR}/boards/${BOARD}/synthesis -f Makefile.${SYNTOOL}


load-bitstream: bitstream
	make -C ${BASEDIR}/boards/${BOARD}/synthesis -f Makefile.${SYNTOOL} load


.PHONY: clean load-bitstream
clean:
	make -C ${BASEDIR}/boards/milkymist-one/synthesis -f common.mak clean
	make -C ${BASEDIR}/boards/milkymist-one/flash clean
	make -C ${BASEDIR}/tools clean

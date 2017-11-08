#
# (C) Copyright 2014 CompuLab, Ltd. <www.compulab.co.il>
#
# Authors: Nikita Kiryanov <nikita@compulab.co.il>
#
# SPDX-License-Identifier:	GPL-2.0+

COMMON_DIR=../common
ccflags-y +=-I${COMMON_DIR}
#
ifdef CONFIG_SPL_BUILD
obj-y = ${COMMON_DIR}/common.o ${COMMON_DIR}/memory.o ${COMMON_DIR}/spl.o
else
obj-y = ${COMMON_DIR}/common.o petersime_ihmi_spl.o
endif

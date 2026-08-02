ccflags-y := -DLA9310_HOST_SW_VERSION="${VERSION_STRING}" -I${COMMON_DIR} -I${UAPI_DIR}
ccflags-y += -Wall -Wno-unused-function -Wno-unused-variable
ccflags-y += -Wno-error

# must stay '+=': ':=' here silently wiped the version define and every warning flag above
ccflags-y += -std=gnu99 -Wno-declaration-after-statement


ifeq (${DEBUG}, 1)
ccflags-y += -g -DDEBUG
endif

# RFNM_BUILD_DIR = the directory that CONTAINS this repo checkout (module legs below reference
# sibling SDK trees by ../../../ relative paths, so this must be the repo's parent directory).
RFNM_BUILD_DIR ?= $(abspath $(obj)/../../..)
# Optional vendor SDKs - a daughterboard module leg is built only when its SDK tree is present.
DIORA_SDK ?= ${RFNM_BUILD_DIR}/diora-sdk-open-source
ICEWINGS_SDK ?= ${RFNM_BUILD_DIR}/g_icewings

ccflags-y += -I${LA9310_DRV_HEADER_DIR} -I${LA9310_COMMON_HEADERS}
ccflags-y += -I${RFNM_BUILD_DIR}/la93xx_host_sw/LimeSuiteNG/embedded/include
ccflags-y += -I${RFNM_BUILD_DIR}/la93xx_host_sw/kernel_driver/la9310rfnm
ccflags-y += -I${KERNEL_DIR}

ifneq ($(wildcard ${DIORA_SDK}),)
ccflags-y += -I${DIORA_SDK}/api -I${DIORA_SDK}/drv -I${DIORA_SDK}/firmware
endif
ifneq ($(wildcard ${ICEWINGS_SDK}),)
ccflags-y += -I${ICEWINGS_SDK}/system  -I${ICEWINGS_SDK}/core -I${ICEWINGS_SDK}/api -I${ICEWINGS_SDK}/kernel
endif




obj-m := la9310rfnm.o rfnm_usb_function.o rfnm_usb.o 

ifneq ($(wildcard ${ICEWINGS_SDK}),)
rfnm_granita-objs := rfnm_granita_m.o
rfnm_granita-objs += rfnm_fe_generic.o
rfnm_granita-objs += rfnm_fe_granita0.o
rfnm_granita-objs += rfnm_granita_rffc.o

rfnm_granita-objs += ../../../g_icewings/api/ErrHandling.o
rfnm_granita-objs += ../../../g_icewings/kernel/SiKernel.o
rfnm_granita-objs += ../../../g_icewings/system/SiSystem.o
rfnm_granita-objs += ../../../g_icewings/api/SiIceWing_parser.o
rfnm_granita-objs += ../../../g_icewings/core/SiIceWing_Core.o
endif

ifneq ($(wildcard ${DIORA_SDK}),)
rfnm_yucca-objs := rfnm_yucca_m.o
rfnm_yucca-objs += rfnm_fe_generic.o
rfnm_yucca-objs += rfnm_fe_yucca0.o


rfnm_yucca-objs += ../../../diora-sdk-open-source/drv/mt3812_drv.o
rfnm_yucca-objs += ../../../diora-sdk-open-source/drv/mt3812_cmdapi.o
rfnm_yucca-objs += ../../../diora-sdk-open-source/drv/mt3812_cmdapi_optim.o
rfnm_yucca-objs += ../../../diora-sdk-open-source/port/mt3812_l1al.o
rfnm_yucca-objs += ../../../diora-sdk-open-source/port/mt3812_osal.o
rfnm_yucca-objs += ../../../diora-sdk-open-source/port/mt3812_phal.o
rfnm_yucca-objs += ../../../diora-sdk-open-source/firmware/fw_init_mem_R2_7_001.o
rfnm_yucca-objs += ../../../diora-sdk-open-source/firmware/fw_prog_mem_R2_7_001.o
endif


rfnm_lime-objs := rfnm_lime_m.o
rfnm_lime-objs += rfnm_lime_mcu.o
rfnm_lime-objs += rfnm_fe_generic.o
rfnm_lime-objs += rfnm_fe_lime0.o

rfnm_lime-objs += ../../LimeSuiteNG/embedded/lms7002m/lms7002m.o
rfnm_lime-objs += ../../LimeSuiteNG/embedded/lms7002m/calibrations.o
rfnm_lime-objs += ../../LimeSuiteNG/embedded/lms7002m/privates.o
rfnm_lime-objs += ../../LimeSuiteNG/embedded/lms7002m/save_state.o
rfnm_lime-objs += ../../LimeSuiteNG/embedded/lms7002m/spi.o

rfnm_breakout-objs := rfnm_breakout_m.o 

ifneq ($(wildcard ${ICEWINGS_SDK}),)
obj-m += rfnm_granita.o
endif
ifneq ($(wildcard ${DIORA_SDK}),)
obj-m += rfnm_yucca.o
endif
obj-m += rfnm_lime.o
obj-m += rfnm_breakout.o


obj-m += rfnm_daughterboard.o
obj-m += rfnm_gpio.o
obj-m += rfnm_usb_boost.o
obj-m += rfnm_qec.o
rfnm_qec-objs := rfnm_qec_main.o rfnm_qec_neon.o
obj-m += rfnm_agc.o
obj-m += rfnm_lalib.o
obj-m += rfnm_eth.o

la9310rfnm-objs := la9310_rfnm.o rfnm_neon.o cache.o pack16to12.o unpack12to16.o remap_packed.o

CFLAGS_REMOVE_rfnm_neon.o += -mgeneral-regs-only
CFLAGS_REMOVE_rfnm_qec_neon.o += -mgeneral-regs-only
CFLAGS_rfnm_qec_neon.o += -ffreestanding -isystem $(shell $(CC) -print-file-name=include)
CFLAGS_REMOVE_rfnm_granita_rffc.o += -mgeneral-regs-only
CFLAGS_REMOVE_../../../g_icewings/system/SiSystem.o += -mgeneral-regs-only
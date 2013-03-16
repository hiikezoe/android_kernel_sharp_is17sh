ifeq ($(CONFIG_WIFIPM),y)
 EXTRA_CFLAGS += -DWIFIPM_INI=0
 EXTRA_CFLAGS += -DWIFIPM_DEBUG
 obj-y	+= wifipm.o
endif

ifeq ($(CONFIG_SHWLAN_BCM4329_1),y)
 BCM_SRC_PATH := ../../../../system/wlan/broadcom/4.218_1/
endif

ifeq ($(CONFIG_SHWLAN_BCM4330_1),y)
 ifeq ($(CONFIG_SH_BCMDHD),y)
  BCM_SRC_PATH := ../../../../system/wlan/broadcom/5.90_1/bcmdhd/
 else
  BCM_SRC_PATH := ../../../../system/wlan/broadcom/5.90_1/
 endif
endif

obj-y += $(BCM_SRC_PATH)

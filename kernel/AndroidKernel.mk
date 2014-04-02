#Android makefile to build kernel as a part of Android Build

TARGET_PREBUILT_INT_KERNEL := kernel/arch/arm/boot/zImage


TARGET_PREBUILT_KERNEL := $(TARGET_PREBUILT_INT_KERNEL)

LOGO_FILE := kernel/drivers/video/logo/logo_linux_clut224.o
file_exist:=$(shell if [ -f $(LOGO_FILE) ]; then echo "exist"; else echo "notexist"; fi;)

KERNEL_MALATA_DEFCONFIG := malata_kernel_tmp_defconfig
KERNEL_ROOT_CONFIG_PATH := kernel/arch/arm/configs/
KERNEL_TMP_DEFCONFIG := $(KERNEL_ROOT_CONFIG_PATH)$(KERNEL_MALATA_DEFCONFIG)

tmp_config_exist:=$(shell if [ -f $(KERNEL_TMP_DEFCONFIG) ]; then echo "exist"; else echo "notexist"; fi;)

define generate-config
cp kernel/arch/arm/configs/$(KERNEL_DEFCONFIG) $(KERNEL_TMP_DEFCONFIG)
for param in $(KERNEL_SPECIAL_CONFIG); \
	do \
		echo $$param >> $(KERNEL_TMP_DEFCONFIG); \
done

for param in $(KERNEL_DEL_SPECIAL_CONFIG); \
	do \
		sed -i "s/$$param/#$$param/g" $(KERNEL_TMP_DEFCONFIG); \
done
endef

define copy-kernel-image
cp kernel/arch/arm/boot/Image $(PRODUCT_OUT)/kernel
endef

ifneq ($(KERNEL_DEFCONFIG), )
$(KERNEL_CONFIG):
	$(MAKE) -C kernel $(KERNEL_MALATA_DEFCONFIG)

$(TARGET_PREBUILT_INT_KERNEL):
ifneq ($(KERNEL_LOGO_FILE), )
	cp $(KERNEL_LOGO_FILE) kernel/drivers/video/logo/
ifeq ($(file_exist), exist)
	rm $(LOGO_FILE)
else
	echo "$(LOGO_FILE) not exist"
endif
endif

ifeq ($(tmp_config_exist), exist)
	rm $(KERNEL_TMP_DEFCONFIG)
endif
	$(generate-config)
	$(MAKE) -C kernel $(KERNEL_MALATA_DEFCONFIG)
	$(MAKE) -C kernel kernel.img
	$(copy-kernel-image)

kernelconfig: $(KERNEL_CONFIG)
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel menuconfig
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel savedefconfig
	cp kernel/defconfig kernel/arch/arm/configs/$(KERNEL_DEFCONFIG)

else
$(TARGET_PREBUILT_INT_KERNEL):
	$(copy-kernel-image)
endif

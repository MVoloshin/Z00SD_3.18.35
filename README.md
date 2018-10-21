# Z00SD_3.18.35
This is the unofficial Z00SD (Asus Zenfone Go ZC451TG) kernel;

If you have gt9xx touchscreen, uncomment it in defconfig (do not forget to disable ft5x0x to prevent conflicts);
Also edit vz6580_we_i_n.dts and set use-tpd-button to 0 (becuase gt9xx button parameters are stored in its driver instead of dts);

Hardware:
LCM: zaw700_hx8379c_fwvga_dsi_vdo_txd_TXDT450SKP zaw700_ili9806e_fwvga_dsi_vdo_DJN_1504 zaw700_otm8019_fwvga_dsi_vdo_boyi;
Touchscreen: gt9xx, ft5x0x (one of these);
Cameras: ov5670_mipi_raw, gc0310_mipi_yuv_qh;
Autofocus: DW9714AF;
Accelerometer: KXTJ2_1009;
Light sensor: STK3X1X;
Charger: PMIC;

Build commands:
export ARCH=arm;
export SUBARCH=arm;
export CROSS_COMPILE=/home/user/Z00SD_3.18.35/util/arm-eabi-4.8/bin/arm-eabi-;
make dep;
make clean;
make mrproper;
make O=out TARGET_ARCH=arm vz6580_we_i_n_defconfig;
make -j4 O=out TARGET_ARCH=arm | tee build.log;

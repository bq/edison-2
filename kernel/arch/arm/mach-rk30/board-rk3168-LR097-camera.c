#ifdef CONFIG_VIDEO_RK29
/*---------------- Camera Sensor Macro Define Begin  ------------------------*/
/*---------------- Camera Sensor Configuration Macro Begin ------------------------*/
#if defined(CONFIG_SOC_CAMERA_FLASH_0C30C2)
#define CONFIG_SENSOR_FALSH_PIN		  RK30_PIN0_PC3
#define CONFIG_SENSOR_TORCH_PIN        RK30_PIN0_PC2
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL RK29_CAM_FLASHACTIVE_H
#if defined(CONFIG_MALATA_D1012)||defined(CONFIG_MALATA_D1014)||defined(CONFIG_MALATA_D1015)
#define CONFIG_SENSOR_TORCHACTIVE_LEVEL RK29_CAM_TORCHACTIVE_H
#else
#define CONFIG_SENSOR_TORCHACTIVE_LEVEL RK29_CAM_TORCHACTIVE_L
#endif
#else
#define CONFIG_SENSOR_FALSH_PIN		  INVALID_GPIO
#define CONFIG_SENSOR_TORCH_PIN        INVALID_GPIO
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL RK29_CAM_FLASHACTIVE_L
#define CONFIG_SENSOR_TORCHACTIVE_LEVEL RK29_CAM_TORCHACTIVE_L
#endif

#define CONFIG_SENSOR_0 RK29_CAM_SENSOR_GC2035						/* back camera sensor */
#define CONFIG_SENSOR_IIC_ADDR_0		0x78
#define CONFIG_SENSOR_IIC_ADAPTER_ID_0	  3
#define CONFIG_SENSOR_CIF_INDEX_0                    0
#if defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D7005)
#define CONFIG_SENSOR_ORIENTATION_0 	  180
#elif defined(CONFIG_MALATA_D7022)
#define CONFIG_SENSOR_ORIENTATION_0 	  0
#else
#define CONFIG_SENSOR_ORIENTATION_0 	  90
#endif
#define CONFIG_SENSOR_POWER_PIN_0		  INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_0		  INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_0 	  RK30_PIN3_PB5
#define CONFIG_SENSOR_FALSH_PIN_0		  CONFIG_SENSOR_FALSH_PIN
#define CONFIG_SENSOR_TORCH_PIN_0         CONFIG_SENSOR_TORCH_PIN
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_0 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_0 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_0 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_0 CONFIG_SENSOR_FLASHACTIVE_LEVEL
#define CONFIG_SENSOR_TORCHACTIVE_LEVEL_0 CONFIG_SENSOR_TORCHACTIVE_LEVEL
#define CONFIG_SENSOR_DVDD_VALUE_0 VDD_18V
#define FOV_V_0 56
#define FOV_H_0 56

#define CONFIG_SENSOR_QCIF_FPS_FIXED_0		15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_0   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_0		15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_0		15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_0		15000
#define CONFIG_SENSOR_480P_FPS_FIXED_0		15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_0		15000
#define CONFIG_SENSOR_720P_FPS_FIXED_0		15000

#define CONFIG_SENSOR_01  RK29_CAM_SENSOR_GT2005                   /* back camera sensor 1 */
#define CONFIG_SENSOR_IIC_ADDR_01 	              0x78 
#define CONFIG_SENSOR_CIF_INDEX_01                    0
#define CONFIG_SENSOR_IIC_ADAPTER_ID_01    3
#if defined(CONFIG_MALATA_D8006)
#define CONFIG_SENSOR_ORIENTATION_01       270
#elif defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D7005)
#define CONFIG_SENSOR_ORIENTATION_01	  180
#else
#define CONFIG_SENSOR_ORIENTATION_01       90
#endif
#define CONFIG_SENSOR_POWER_PIN_01         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_01         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_01      RK30_PIN3_PB5
#define CONFIG_SENSOR_FALSH_PIN_01         CONFIG_SENSOR_FALSH_PIN
#define CONFIG_SENSOR_TORCH_PIN_01         CONFIG_SENSOR_TORCH_PIN
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_01 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_01 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_01 RK29_CAM_POWERDNACTIVE_L   
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_01 CONFIG_SENSOR_FLASHACTIVE_LEVEL
#define CONFIG_SENSOR_TORCHACTIVE_LEVEL_01 CONFIG_SENSOR_TORCHACTIVE_LEVEL
#define CONFIG_SENSOR_DVDD_VALUE_01 VDD_18V
#define FOV_V_01 60
#define FOV_H_01 60

#define CONFIG_SENSOR_QCIF_FPS_FIXED_01      15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_01   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_01      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_01       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_01       15000
#define CONFIG_SENSOR_480P_FPS_FIXED_01      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_01      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_01     15000

#define CONFIG_SENSOR_02 RK29_CAM_SENSOR_OV5640                      /* back camera sensor 2 */
#define CONFIG_SENSOR_IIC_ADDR_02 	    0x78
#define CONFIG_SENSOR_CIF_INDEX_02                    0
#define CONFIG_SENSOR_IIC_ADAPTER_ID_02    3
#if defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D7005)
#define CONFIG_SENSOR_ORIENTATION_02       180
#else
#define CONFIG_SENSOR_ORIENTATION_02       90
#endif
#define CONFIG_SENSOR_POWER_PIN_02         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_02         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_02       RK30_PIN3_PB5
#define CONFIG_SENSOR_FALSH_PIN_02		  CONFIG_SENSOR_FALSH_PIN
#define CONFIG_SENSOR_TORCH_PIN_02         CONFIG_SENSOR_TORCH_PIN
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_02 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_02 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_02 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_02 CONFIG_SENSOR_FLASHACTIVE_LEVEL
#define CONFIG_SENSOR_TORCHACTIVE_LEVEL_02 CONFIG_SENSOR_TORCHACTIVE_LEVEL
#define CONFIG_SENSOR_DVDD_VALUE_02 VDD_15V
#define FOV_V_02 57
#define FOV_H_02 57

#define CONFIG_SENSOR_QCIF_FPS_FIXED_02      15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_02   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_02      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_02       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_02       15000
#define CONFIG_SENSOR_480P_FPS_FIXED_02      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_02      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_02      15000

#define CONFIG_SENSOR_03 RK29_CAM_SENSOR_SID130B_BACK/* back camera sensor 2 */
#define CONFIG_SENSOR_IIC_ADDR_03 	    0x6e   //   6E
#define CONFIG_SENSOR_CIF_INDEX_03                    0
#define CONFIG_SENSOR_IIC_ADAPTER_ID_03    3
#define CONFIG_SENSOR_ORIENTATION_03       90
#define CONFIG_SENSOR_FALSH_PIN_03         CONFIG_SENSOR_FALSH_PIN
#define CONFIG_SENSOR_TORCH_PIN_03         CONFIG_SENSOR_TORCH_PIN
#define CONFIG_SENSOR_POWER_PIN_03         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_03         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_03      RK30_PIN3_PB5
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_03 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_03 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_03 RK29_CAM_POWERDNACTIVE_L
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_03 CONFIG_SENSOR_FLASHACTIVE_LEVEL
#define CONFIG_SENSOR_TORCHACTIVE_LEVEL_03 CONFIG_SENSOR_TORCHACTIVE_LEVEL
#define CONFIG_SENSOR_DVDD_VALUE_03 VDD_15V
#define FOV_V_03 60
#define FOV_H_03 60


#define CONFIG_SENSOR_QCIF_FPS_FIXED_03      15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_03   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_03      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_03       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_03      15000
#define CONFIG_SENSOR_480P_FPS_FIXED_03      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_03      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_03      15000

#define CONFIG_SENSOR_04 RK29_CAM_SENSOR_OV2655/* back camera sensor 4 */
#define CONFIG_SENSOR_IIC_ADDR_04 	    0x60   //   6E
#define CONFIG_SENSOR_CIF_INDEX_04                    0
#define CONFIG_SENSOR_IIC_ADAPTER_ID_04    3
#define CONFIG_SENSOR_ORIENTATION_04       90
#define CONFIG_SENSOR_FALSH_PIN_04         CONFIG_SENSOR_FALSH_PIN
#define CONFIG_SENSOR_TORCH_PIN_04         CONFIG_SENSOR_TORCH_PIN
#define CONFIG_SENSOR_POWER_PIN_04         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_04         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_04      RK30_PIN3_PB5
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_04 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_04 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_04 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_04 CONFIG_SENSOR_FLASHACTIVE_LEVEL
#define CONFIG_SENSOR_TORCHACTIVE_LEVEL_04 CONFIG_SENSOR_TORCHACTIVE_LEVEL
#define CONFIG_SENSOR_DVDD_VALUE_04 VDD_15V
#define FOV_V_04 60
#define FOV_H_04 60

#define CONFIG_SENSOR_QCIF_FPS_FIXED_04      15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_04   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_04      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_04       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_04      15000
#define CONFIG_SENSOR_480P_FPS_FIXED_04      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_04      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_04      15000

#define CONFIG_SENSOR_1 RK29_CAM_SENSOR_GC2035_FRONT                      /* front camera sensor 0 */
#define CONFIG_SENSOR_IIC_ADDR_1 	    0x78
#define CONFIG_SENSOR_IIC_ADAPTER_ID_1	  3
#define CONFIG_SENSOR_CIF_INDEX_1				  0
#if defined(CONFIG_MALATA_D8006)
#define CONFIG_SENSOR_ORIENTATION_1       90
#elif defined(CONFIG_MALATA_D7022)
#define CONFIG_SENSOR_ORIENTATION_1       0
#else
#define CONFIG_SENSOR_ORIENTATION_1       270
#endif
#define CONFIG_SENSOR_POWER_PIN_1         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_1         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_1 	  RK30_PIN3_PB4
#define CONFIG_SENSOR_FALSH_PIN_1         INVALID_GPIO
#define CONFIG_SENSOR_TORCH_PIN_1         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_1 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_1 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_1 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_1 RK29_CAM_FLASHACTIVE_L
#define CONFIG_SENSOR_TORCHACTIVE_LEVEL_1 RK29_CAM_TORCHACTIVE_L
#define CONFIG_SENSOR_DVDD_VALUE_1 VDD_18V
#define FOV_V_1 60
#define FOV_H_1 60

#define CONFIG_SENSOR_QCIF_FPS_FIXED_1		15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_1   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_1		15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_1		15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_1		15000
#define CONFIG_SENSOR_480P_FPS_FIXED_1		15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_1		15000
#define CONFIG_SENSOR_720P_FPS_FIXED_1		15000

#define CONFIG_SENSOR_11 RK29_CAM_SENSOR_GC0308                      /* front camera sensor 1 */
#define CONFIG_SENSOR_IIC_ADDR_11 	    0x42
#define CONFIG_SENSOR_IIC_ADAPTER_ID_11    3
#define CONFIG_SENSOR_CIF_INDEX_11				  0
#if defined(CONFIG_MALATA_D7022)
#define CONFIG_SENSOR_ORIENTATION_11       0
#else
#define CONFIG_SENSOR_ORIENTATION_11       270
#endif
#define CONFIG_SENSOR_POWER_PIN_11         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_11         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_11       RK30_PIN3_PB4//RK30_PIN1_PB7
#define CONFIG_SENSOR_FALSH_PIN_11         INVALID_GPIO
#define CONFIG_SENSOR_TORCH_PIN_11         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_11 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_11 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_11 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_11 RK29_CAM_FLASHACTIVE_L
#define CONFIG_SENSOR_TORCHACTIVE_LEVEL_11 RK29_CAM_TORCHACTIVE_L
#define CONFIG_SENSOR_DVDD_VALUE_11 VDD_18V
#define FOV_V_11 60
#define FOV_H_11 60

#define CONFIG_SENSOR_QCIF_FPS_FIXED_11      15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_11   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_11      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_11       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_11       15000
#define CONFIG_SENSOR_480P_FPS_FIXED_11      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_11      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_11      15000

#define CONFIG_SENSOR_12 RK29_CAM_SENSOR_GT2005_FRONT//RK29_CAM_SENSOR_OV2655                      /* front camera sensor 2 */
#define CONFIG_SENSOR_IIC_ADDR_12 	   0x78
#define CONFIG_SENSOR_IIC_ADAPTER_ID_12    3
#define CONFIG_SENSOR_CIF_INDEX_12				  0
#define CONFIG_SENSOR_ORIENTATION_12       270
#define CONFIG_SENSOR_POWER_PIN_12         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_12         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_12       RK30_PIN3_PB4//RK30_PIN1_PB7
#define CONFIG_SENSOR_FALSH_PIN_12         INVALID_GPIO
#define CONFIG_SENSOR_TORCH_PIN_12         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_12 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_12 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_12 RK29_CAM_POWERDNACTIVE_L   
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_12 RK29_CAM_FLASHACTIVE_L
#define CONFIG_SENSOR_TORCHACTIVE_LEVEL_12 RK29_CAM_TORCHACTIVE_L
#define CONFIG_SENSOR_DVDD_VALUE_12 VDD_18V
#define FOV_V_12 60
#define FOV_H_12 60

#define CONFIG_SENSOR_QCIF_FPS_FIXED_12      15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_12   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_12      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_12       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_12       15000
#define CONFIG_SENSOR_480P_FPS_FIXED_12      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_12      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_12      15000

#define CONFIG_SENSOR_13 RK29_CAM_SENSOR_HI253_FRONT//RK29_CAM_SENSOR_OV2655                      /* front camera sensor 2 */
#define CONFIG_SENSOR_IIC_ADDR_13 	   0x40
#define CONFIG_SENSOR_IIC_ADAPTER_ID_13    3
#define CONFIG_SENSOR_CIF_INDEX_13				  0
#if defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D7005)
#define CONFIG_SENSOR_ORIENTATION_13       180
#else
#define CONFIG_SENSOR_ORIENTATION_13       270
#endif
#define CONFIG_SENSOR_POWER_PIN_13         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_13         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_13       RK30_PIN3_PB4//RK30_PIN1_PB7
#define CONFIG_SENSOR_FALSH_PIN_13         INVALID_GPIO
#define CONFIG_SENSOR_TORCH_PIN_13         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_13 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_13 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_13 RK29_CAM_POWERDNACTIVE_H   
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_13 RK29_CAM_FLASHACTIVE_L
#define CONFIG_SENSOR_TORCHACTIVE_LEVEL_13 RK29_CAM_TORCHACTIVE_L
#define CONFIG_SENSOR_DVDD_VALUE_13 VDD_18V
#define FOV_V_13 60
#define FOV_H_13 60

#define CONFIG_SENSOR_QCIF_FPS_FIXED_13      15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_13   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_13      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_13       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_13       15000
#define CONFIG_SENSOR_480P_FPS_FIXED_13      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_13      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_13      15000

#define CONFIG_SENSOR_14 RK29_CAM_SENSOR_SID130B//RK29_CAM_SENSOR_OV2655                      /* front camera sensor 2 */
#define CONFIG_SENSOR_IIC_ADDR_14 	   0x6e
#define CONFIG_SENSOR_IIC_ADAPTER_ID_14    3
#define CONFIG_SENSOR_CIF_INDEX_14				  0
#if defined(CONFIG_MALATA_D7005)
#define CONFIG_SENSOR_ORIENTATION_14       0
#elif defined(CONFIG_MALATA_D7022)
#define CONFIG_SENSOR_ORIENTATION_14       0
#else
#define CONFIG_SENSOR_ORIENTATION_14       270
#endif
#define CONFIG_SENSOR_POWER_PIN_14         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_14         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_14       RK30_PIN3_PB4//RK30_PIN1_PB7
#define CONFIG_SENSOR_FALSH_PIN_14         INVALID_GPIO
#define CONFIG_SENSOR_TORCH_PIN_14         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_14 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_14 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_14 RK29_CAM_POWERDNACTIVE_L   
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_14 RK29_CAM_FLASHACTIVE_L
#define CONFIG_SENSOR_TORCHACTIVE_LEVEL_14 RK29_CAM_TORCHACTIVE_L
#define CONFIG_SENSOR_DVDD_VALUE_14 VDD_15V
#define FOV_V_14 60
#define FOV_H_14 60

#define CONFIG_SENSOR_QCIF_FPS_FIXED_14     15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_14   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_14      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_14       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_14       15000
#define CONFIG_SENSOR_480P_FPS_FIXED_14      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_14      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_14      15000

#define CONFIG_SENSOR_15 RK29_CAM_SENSOR_GC0329 //RK29_CAM_SENSOR_OV2655                      /* front camera sensor 2 */
#define CONFIG_SENSOR_IIC_ADDR_15 	   0x62
#define CONFIG_SENSOR_IIC_ADAPTER_ID_15    3
#define CONFIG_SENSOR_CIF_INDEX_15				  0
#if defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D7005)
#define CONFIG_SENSOR_ORIENTATION_15      180
#elif defined(CONFIG_MALATA_D7022)
#define CONFIG_SENSOR_ORIENTATION_15      0
#else
#define CONFIG_SENSOR_ORIENTATION_15      270
#endif
#define CONFIG_SENSOR_POWER_PIN_15         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_15         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_15       RK30_PIN3_PB4//RK30_PIN1_PB7
#define CONFIG_SENSOR_FALSH_PIN_15         INVALID_GPIO
#define CONFIG_SENSOR_TORCH_PIN_15         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_15 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_15 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_15 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_15 RK29_CAM_FLASHACTIVE_L
#define CONFIG_SENSOR_TORCHACTIVE_LEVEL_15 RK29_CAM_TORCHACTIVE_L
#define CONFIG_SENSOR_DVDD_VALUE_15 VDD_18V
#define FOV_V_15 60
#define FOV_H_15 60

#define CONFIG_SENSOR_QCIF_FPS_FIXED_15     15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_15   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_15      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_15       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_15       15000
#define CONFIG_SENSOR_480P_FPS_FIXED_15      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_15      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_15      15000

#define CONFIG_SENSOR_16 RK29_CAM_SENSOR_OV2659//RK29_CAM_SENSOR_OV2655                      /* front camera sensor 2 */
#define CONFIG_SENSOR_IIC_ADDR_16 	   0x60
#define CONFIG_SENSOR_IIC_ADAPTER_ID_16    3
#define CONFIG_SENSOR_CIF_INDEX_16				  0
#if defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D7005)
#define CONFIG_SENSOR_ORIENTATION_16       180
#else
#define CONFIG_SENSOR_ORIENTATION_16       270
#endif
#define CONFIG_SENSOR_POWER_PIN_16         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_16         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_16       RK30_PIN3_PB4//RK30_PIN1_PB7
#define CONFIG_SENSOR_FALSH_PIN_16         INVALID_GPIO
#define CONFIG_SENSOR_TORCH_PIN_16         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_16 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_16 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_16 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_16 RK29_CAM_FLASHACTIVE_L
#define CONFIG_SENSOR_TORCHACTIVE_LEVEL_16 RK29_CAM_TORCHACTIVE_L
#define CONFIG_SENSOR_DVDD_VALUE_16 VDD_15V
#define FOV_V_16 60
#define FOV_H_16 60

#define CONFIG_SENSOR_QCIF_FPS_FIXED_16     15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_16   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_16      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_16       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_16       15000
#define CONFIG_SENSOR_480P_FPS_FIXED_16      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_16      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_16      15000
#define CONFIG_SENSOR_17 RK29_CAM_SENSOR_SP2518//RK29_CAM_SENSOR_OV2655                      /* front camera sensor 2 */
#define CONFIG_SENSOR_IIC_ADDR_17 	   0x60
#define CONFIG_SENSOR_IIC_ADAPTER_ID_17    3
#define CONFIG_SENSOR_CIF_INDEX_17				  0
#define CONFIG_SENSOR_ORIENTATION_17       270
#define CONFIG_SENSOR_POWER_PIN_17         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_17         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_17       RK30_PIN3_PB4//RK30_PIN1_PB7
#define CONFIG_SENSOR_FALSH_PIN_17         INVALID_GPIO
#define CONFIG_SENSOR_TORCH_PIN_17         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_17 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_17 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_17 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_17 RK29_CAM_FLASHACTIVE_L
#define CONFIG_SENSOR_TORCHACTIVE_LEVEL_17 RK29_CAM_TORCHACTIVE_L
#define CONFIG_SENSOR_DVDD_VALUE_17 VDD_18V
#define FOV_V_17 60
#define FOV_H_17 60

#define CONFIG_SENSOR_QCIF_FPS_FIXED_17     15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_17   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_17      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_17       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_17       15000
#define CONFIG_SENSOR_480P_FPS_FIXED_17      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_17      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_17      15000
#endif  //#ifdef CONFIG_VIDEO_RK29

/*---------------- Camera Sensor Configuration Macro End------------------------*/
#include "../../../drivers/media/video/rk30_camera.c"
/*---------------- Camera Sensor Macro Define End  ---------*/

#define PMEM_CAM_SIZE PMEM_CAM_NECESSARY
/*****************************************************************************************
 * camera  devices
 * author: ddl@rock-chips.com
 *****************************************************************************************/
#ifdef CONFIG_VIDEO_RK29
#define CONFIG_SENSOR_POWER_IOCTL_USR	   1 //define this refer to your board layout
#define CONFIG_SENSOR_RESET_IOCTL_USR	   0
#define CONFIG_SENSOR_POWERDOWN_IOCTL_USR	   0
#define CONFIG_SENSOR_FLASH_IOCTL_USR	   0
static void all_rk_cif_power(int on,enum rk29camera_vdd_val vdd_val)
{
    struct regulator *ldo_18,*ldo_28;

	ldo_28 = regulator_get(NULL, "act_ldo8");	// vcc28_cif
	ldo_18 = regulator_get(NULL, "act_ldo3");	// vcc18_cif
	if (ldo_28 == NULL || IS_ERR(ldo_28) || ldo_18 == NULL || IS_ERR(ldo_18)){
        printk("get cif ldo failed!\n");
		return;
	    }
    if(on == 0){	
		while(regulator_is_enabled(ldo_28)>0)
    		regulator_disable(ldo_28);
    	regulator_put(ldo_28);

		while(regulator_is_enabled(ldo_18)>0)
    		regulator_disable(ldo_18);
    	regulator_put(ldo_18);
		mdelay(150);
		}
	else{
		regulator_set_voltage(ldo_28, 2800000, 2800000);
		regulator_enable(ldo_28);
   //	printk("%s set ldo7 vcc28_cif=%dmV end\n", __func__, regulator_get_voltage(ldo_28));
		regulator_put(ldo_28);

    	if(vdd_val == VDD_18V)
		regulator_set_voltage(ldo_18, 1800000, 1800000);
	else
		regulator_set_voltage(ldo_18, 1500000, 1500000);
    //	regulator_set_suspend_voltage(ldo, 1800000);
    	regulator_enable(ldo_18);
    //	printk("%s set ldo1 vcc18_cif=%dmV end\n", __func__, regulator_get_voltage(ldo_18));
    	regulator_put(ldo_18);
		mdelay(150);
	}
}

static void rk_cif_power(struct rk29camera_gpio_res *res,int on,enum rk29camera_vdd_val vdd_val)
{
    struct regulator *ldo_18,*ldo_28;
	int camera_power = res->gpio_power;
	int camera_ioflag = res->gpio_flag;
	int camera_io_init = res->gpio_init;
     printk("honghaishen_0626 camera power is %d",on);
	ldo_28 = regulator_get(NULL, "act_ldo8");	// vcc28_cif
	ldo_18 = regulator_get(NULL, "act_ldo3");	// vcc18_cif
	if (ldo_28 == NULL || IS_ERR(ldo_28) || ldo_18 == NULL || IS_ERR(ldo_18)){
        printk("get cif ldo failed!\n");
		return;
	    }
    if(on == 0){	
		while(regulator_is_enabled(ldo_28)>0)
    		regulator_disable(ldo_28);
    	regulator_put(ldo_28);

		while(regulator_is_enabled(ldo_18)>0)
    		regulator_disable(ldo_18);
    	regulator_put(ldo_18);
    	
		mdelay(150);
	if (camera_power != INVALID_GPIO)  {
		  if (camera_io_init & RK29_CAM_POWERACTIVE_MASK) {
			  gpio_set_value(camera_power, (((~camera_ioflag)&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));
			//	dprintk("%s..%s..PowerPin=%d ..PinLevel = %x	 \n",__FUNCTION__,res->dev_name, camera_power, (((~camera_ioflag)&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));
			}
		}
		}
	else{
		regulator_set_voltage(ldo_28, 2800000, 2800000);
		regulator_enable(ldo_28);
   //	printk("%s set ldo7 vcc28_cif=%dmV end\n", __func__, regulator_get_voltage(ldo_28));
		regulator_put(ldo_28);

    	if(vdd_val == VDD_18V)
		regulator_set_voltage(ldo_18, 1800000, 1800000);
	else
		regulator_set_voltage(ldo_18, 1500000, 1500000);
    //	regulator_set_suspend_voltage(ldo, 1800000);
    	regulator_enable(ldo_18);
    //	printk("%s set ldo1 vcc18_cif=%dmV end\n", __func__, regulator_get_voltage(ldo_18));
    	regulator_put(ldo_18);
		mdelay(150);
	if (camera_power != INVALID_GPIO)  {
		  if (camera_io_init & RK29_CAM_POWERACTIVE_MASK) {
			gpio_set_value(camera_power, ((camera_ioflag&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));
  
		  }

}


	}
}

#if CONFIG_SENSOR_POWER_IOCTL_USR
static int sensor_power_usr_cb (struct rk29camera_gpio_res *res,int on,enum rk29camera_vdd_val vdd_val)
{
	//#error "CONFIG_SENSOR_POWER_IOCTL_USR is 1, sensor_power_usr_cb function must be writed!!";
	rk_cif_power(res,on, vdd_val);
	return 0;
}
#endif
#if CONFIG_SENSOR_FLASH_IOCTL_USR
static int sensor_flash_usr_cb (struct rk29camera_gpio_res *res,int on)
{
	#error "CONFIG_SENSOR_FLASH_IOCTL_USR is 1, sensor_flash_usr_cb function must be writed!!";
}
#endif

static struct rk29camera_platform_ioctl_cb	sensor_ioctl_cb = {
	#if CONFIG_SENSOR_POWER_IOCTL_USR
	.sensor_power_cb = sensor_power_usr_cb,
	#else
	.sensor_power_cb = NULL,
	#endif

	#if CONFIG_SENSOR_RESET_IOCTL_USR
	.sensor_reset_cb = sensor_reset_usr_cb,
	#else
	.sensor_reset_cb = NULL,
	#endif

	#if CONFIG_SENSOR_POWERDOWN_IOCTL_USR
	.sensor_powerdown_cb = sensor_powerdown_usr_cb,
	#else
	.sensor_powerdown_cb = NULL,
	#endif

	#if CONFIG_SENSOR_FLASH_IOCTL_USR
	.sensor_flash_cb = sensor_flash_usr_cb,
	#else
	.sensor_flash_cb = NULL,
	#endif
};

#if CONFIG_SENSOR_IIC_ADDR_0
static struct reginfo_t rk_init_data_sensor_reg_0[] =
{
		{0x0000, 0x00,0,0}
	};
static struct reginfo_t rk_init_data_sensor_winseqreg_0[] ={
	{0x0000, 0x00,0,0}
	};
#endif

#if CONFIG_SENSOR_IIC_ADDR_1
static struct reginfo_t rk_init_data_sensor_reg_1[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_1[] =
{
       {0x0000, 0x00,0,0}
};
#endif
#if CONFIG_SENSOR_IIC_ADDR_01
static struct reginfo_t rk_init_data_sensor_reg_01[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_01[] =
{
       {0x0000, 0x00,0,0}
};
#endif
#if CONFIG_SENSOR_IIC_ADDR_02
static struct reginfo_t rk_init_data_sensor_reg_02[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_02[] =
{
       {0x0000, 0x00,0,0}
};
#endif
#if CONFIG_SENSOR_IIC_ADDR_03
static struct reginfo_t rk_init_data_sensor_reg_03[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_03[] =
{
       {0x0000, 0x00,0,0}
};
#endif

#if CONFIG_SENSOR_IIC_ADDR_04
static struct reginfo_t rk_init_data_sensor_reg_04[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_04[] =
{
       {0x0000, 0x00,0,0}
};
#endif

#if CONFIG_SENSOR_IIC_ADDR_11
static struct reginfo_t rk_init_data_sensor_reg_11[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_11[] =
{
       {0x0000, 0x00,0,0}
};
#endif
#if CONFIG_SENSOR_IIC_ADDR_12
static struct reginfo_t rk_init_data_sensor_reg_12[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_12[] =
{
       {0x0000, 0x00,0,0}
};
#endif
#if CONFIG_SENSOR_IIC_ADDR_13
static struct reginfo_t rk_init_data_sensor_reg_13[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_13[] =
{
       {0x0000, 0x00,0,0}
};
#endif
#if CONFIG_SENSOR_IIC_ADDR_14
static struct reginfo_t rk_init_data_sensor_reg_14[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_14[] =
{
       {0x0000, 0x00,0,0}
};
#endif
#if CONFIG_SENSOR_IIC_ADDR_15
static struct reginfo_t rk_init_data_sensor_reg_15[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_15[] =
{
       {0x0000, 0x00,0,0}
};
#endif

#if CONFIG_SENSOR_IIC_ADDR_16
static struct reginfo_t rk_init_data_sensor_reg_16[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_16[] =
{
       {0x0000, 0x00,0,0}
};
#endif

#if CONFIG_SENSOR_IIC_ADDR_17
static struct reginfo_t rk_init_data_sensor_reg_17[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_17[] =
{
       {0x0000, 0x00,0,0}
};
#endif


static rk_sensor_user_init_data_s rk_init_data_sensor[RK_CAM_NUM] = 
{
    #if CONFIG_SENSOR_IIC_ADDR_0
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_0,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_0,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_0) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_0) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif
    #if CONFIG_SENSOR_IIC_ADDR_1
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_1,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_1,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_1) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_1) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif
    #if CONFIG_SENSOR_IIC_ADDR_01
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_01,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_01,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_01) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_01) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif
    #if CONFIG_SENSOR_IIC_ADDR_02
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_02,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_02,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_02) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_02) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif

	    #if CONFIG_SENSOR_IIC_ADDR_03
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_03,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_03,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_03) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_03) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif

    #if CONFIG_SENSOR_IIC_ADDR_04
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_04,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_04,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_04) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_04) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif

    #if CONFIG_SENSOR_IIC_ADDR_11
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_11,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_11,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_11) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_11) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif
    #if CONFIG_SENSOR_IIC_ADDR_12
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_12,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_12,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_12) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_12) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif

    #if CONFIG_SENSOR_IIC_ADDR_13
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_13,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_13,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_13) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_13) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif
    #if CONFIG_SENSOR_IIC_ADDR_14
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_14,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_14,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_14) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_14) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif
    #if CONFIG_SENSOR_IIC_ADDR_15
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_15,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_15,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_15) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_15) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif

 #if CONFIG_SENSOR_IIC_ADDR_16
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_16,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_16,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_16) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_16) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif

 #if CONFIG_SENSOR_IIC_ADDR_17
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_17,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_17,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_17) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_17) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif

 };
#include "../../../drivers/media/video/rk30_camera.c"

#endif /* CONFIG_VIDEO_RK29 */


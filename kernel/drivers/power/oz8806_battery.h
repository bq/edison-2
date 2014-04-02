

/*
 * -------------------------------------------------------------------------
 * Definition of OZ8806 chip register
 * -------------------------------------------------------------------------
 */

#define MUTEX_TIMEOUT 5000
// Those functions get information of smart battery.
#define MYDRIVER				"oz8806"
#define OZ8806Addr				0x2F
#define ControlStatus			0x09
	#define CtBitSoftwareReset		0x80
	#define CtBitSleepMode			0x40
	#define CtBitSleepOCVEn			0x20
	#define CtBitChargeON			0x10
#define	VoltageOffset			0x0A
#define BatteryID				0x0B
#define CellTempLow				0x0C
#define CellTempHigh			0x0D
	#define CellTempMASK			0x0FFF
#define CellVoltLow				0x0E
#define CellVoltHigh			0x0F
	#define CellVoltMASK			0x0FFF
#define CellOCVLow				0x10
#define CellOCVHigh				0x11
	#define CellOCVMASK				0x0FFF
	#define CellOCVSleepMask		0x02
	#define	CellOCVPoOnMask			0x01
#define CellCurrLow				0x12
#define CellCurrHigh			0x13
	#define CellCurrMASK			0xFFFF
#define CellCARLow				0x14
#define CellCARHigh				0x15
	#define CellCARMASK				0xFFFF
#define	CurrentOffsetLow		0x16
#define CurrentOffsetHight		0x17
#define BoardOffsetLow			0x18
#define BoardOffsetHigh			0x19
	#define	BoardOffsetMask			0x0FFF



#define		STATEIDLE				0x00
#define		STATECHARGING			0x01
#define		STATEDISCHARGING		0x02
#define		STATEEOC				0x10
#define		STATEEOD				0x20
#define		STATEFULLEOC			0x40
#define		STATEFULLEOD			0x80
#define		STATEENDMASK			0xF0
#define		VOLTCVMODE				4100		//volt reaching to CV mode
#define		BUFSIZE					4096



/*
 * -------------------------------------------------------------------------
 * battery information structure
 * -------------------------------------------------------------------------
 */

struct struct_batt_data {
	int		PowerStatus;	//
	int		fRsense;		//= 20;						//Rsense value of chip, in mini ohm
//	BYTE	yCtrlReg;
	int		dbCARLSB;		//= 5.0;					//LSB of CAR, comes from spec
	int		dbCurrLSB;		//391 (3.90625*100);		//LSB of Current, comes from spec
	int		fVoltLSB;		//250 (2.5*100);			//LSB of Voltage, comes from spec
	int		fBattIDLSB;		//= 40.0;					//LSB of battery ID, comes from spec
	int		fFCC;			//= 6900;					//Fully Charged Capacity, depends on battery
	int		fEOC;			//= 345;						//the current threshold of End of Charged
	int		fRC;			//= 0;						//Remaining Capacity, indicates how many mAhr in battery
	int		fRSOC;			//50 = 50%;					//Relative State Of Charged, present percentage of battery capacity
	int		fVolt;			//= 0;						//Voltage of battery, in mV
	int		fCurr;			//= 0;						//Current of battery, in mA; plus value means charging, minus value means discharging
	int		fPrevCurr;		//= 0;						//last one current reading
	int		fOCVVolt;		//= 0;						//Open Circuit Voltage
	int		fCellTemp;		//= 0;						//Temperature of battery
	int		fReserved;		//= 0;						//Reserved Capacity for 
	int		fVoltFCHG;		//= 4160;					//Voltage at fully charged, this value needs to consider the voltage drop when having loading
	int		fPerFromOCV;	//= 0;
	int		fSuspmA;		//= 5;						//mA, power consumption when OS in suspend
	int		sCaMAH;			//= 0;						//adjusted residual capacity
	int		sCrMAH;			//= 100;					//residual capacity at last fully DSG, initially equals to fReserved
	int		sCeodMAH;		//= 100;
	int		sCfMAH;			//= 1400;					//predictive full capacity, initially equals to FCC
	int		fRCDelta;		//= 0;
	int		fRCPrev;		//= 0;						//previous CAR value
	int		fVoltAdjLow;	//= 3775;					//mV
	int		fDisCOCV;		//= 100;					//OCV table is made by discharging current = 100C
};



/*
 * -------------------------------------------------------------------------
 * Exported functions
 * -------------------------------------------------------------------------
 */


//extern u8 OZ8806_control_register(struct i2c_client *client, int op, u8 indata);


//extern void OZ8806_sleep_control(struct i2c_client *client, int sleepEn, int sleepOCV);


//extern void OZ8806_CAR_Write(struct i2c_client *client);


//extern void OZ8806_CAR_Reset(struct i2c_client *client);


//--------------------------------------------------------------------------
// Function: OZ8806_init_chip
// 
// Purpose:	PowerOn Initialization of OZ8806, 
//			read OCV/voltage value and calculate capacity of battery, 
//			then set CAR register of OZ8806
// In:		i2_client -- i2c_client structure specific for OZ8806 device
// Out:		void
//--------------------------------------------------------------------------

//extern void OZ8806_init_chip(struct i2c_client *client);


//extern bool OZ8806_PollingLoop(void);


//extern bool OZ8806_GaugeAdjustion(void);

//extern bool OZ8806_getbatinfo(struct struct_batt_data *rkbatt_info);


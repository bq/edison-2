/*
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <mach/gpio.h>
#include <linux/power_supply.h>
#include <mach/board.h>

#include <asm/irq.h>

//Note that .h file maybe put in kernel/include/linux/i2c/ folder
//i put it over here just only for convenience
#include "oz8806_battery.h"
//#include <linux/i2c/OZ8806.h>


int gPreFrsoc = -1;
int gAdapatRemove = 0;
//struct delayed_work gAdapatRemoveWork;


/*-------------------------------------------------------------------------*/
struct struct_batt_data batt_info = {
	0, 20, 5, 391, 250, 40, 7700, 280, 0, 0, 0, 0, 0, 0, 0, 0, 4160, 0, 5, 0, 100, 100, 7700, 0, 0, 3775, 100 
};
// Addresses to scan, in 7-bit presentation
//static const unsigned short normal_i2c[] = { OZ8806Addr, I2C_CLIENT_END };

//I2C_CLIENT_INSMOD;
//I2C_CLIENT_INSMOD_1(OZ8806);

struct OZ8806_data {
	struct power_supply bat;
	struct power_supply ac;
	struct delayed_work work;
	struct work_struct dcwakeup_work;
	unsigned int interval;
	unsigned int dc_det_pin;

	struct i2c_client	*myclient;
	struct mutex		update_lock;

	u32					valid;
	unsigned long		last_updated;
	u8					control;
	u16					aout16;
};

static struct OZ8806_data *the_OZ8806;

static enum power_supply_property oz8806_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
	//POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	//POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	//POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
};

static enum power_supply_property oz8806_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

int		sECxMAH[4]		= {0,0,0,0};		//tables of EOC adjustment point
int		fStateDtm		= 10;				//current below this value, state => idle
u8		yOZState, yPrevOZState;
int		iVoltAdjHigh	= 3800;				//mV
int		iEODLowVolt		= 3300;				//mV, low voltage of battery, base on battery spec
bool	bHadFullEOD		= false;			//Record EOD process

/*-------------------------------------------------------------------------*/

//
//declaration of temperature table
//
#define NPoints		31
struct RTPoint
{
	int			oC;
	int			Ohm;
	int			mV;
};
//value of 1st point depends on HW connection, demo board using 120Kohm+1.8V, oC can be any value
struct RTPoint			RTtable[NPoints] = {
				{ 1800, 120000, 1800},
				{ -20, 69204, 0},	{ -15, 54464, 0},	{ -10, 43139, 0},
				{  -5, 34386, 0},	{   0, 27580, 0},	{   5, 22257, 0},
				{  10, 18067, 0},	{  15, 14752, 0},	{  20, 12112, 0},
				{  25, 10000, 0},	{  30,  8300, 0},	{  35,  6924, 0},
				{  40,  5806, 0},	{  45,  4891, 0},	{  50,  4140, 0},
				{  55,  3521, 0},	{  60,  3007, 0},	{  65,  2578, 0},
				{  70,  2221, 0},	{  75,  1920, 0},	{  80,  1666, 0},
				{  85,  1451, 0},	{  90,  1269, 0},	{  95,  1113, 0},
				{ 100,   980, 0},	{ 105,   865, 0},	{ 110,   766, 0},
				{ 115,   681, 0},	{ 120,   606, 0},	{ 125,   542, 0}};


#define	OCVNPoints	21
struct SOCVPoint
{
	int			iVoltage;
	int			iRSOC;				//this value save in xx%, really value * 100
};

//To Customer: OCVtable[] is used when OS goes into Suspend and OZ8806 ever did SleepOCV detection
//When OS resume from Suspend Mode, battery driver checks OZ8806 SleepOCV value, and use SleepOCV voltage to re-initialize battery capacity.
/*struct SOCVPoint	OCVtable[OCVNPoints] = {
				{3000, 00},	{3400, 05},	{3475, 10},	{3518, 15},	{3557, 20}, 
				{3587, 25},	{3608, 30},	{3624, 35},	{3638, 40},	{3655, 45}, 
				{3675, 50},	{3705, 55},	{3743, 60},	{3786, 65},	{3832, 70}, 
				{3882, 75},	{3933, 80},	{3990, 85},	{4050, 90},	{4114, 95}, 
				{4180, 100}};*/

struct SOCVPoint   OCVtable[OCVNPoints] = {

                                     {3000, 00},        {3665, 05},        {3703, 10},        {3730, 15},        {3760, 20}, 

                                      {3773, 25},        {3778, 30},        {3780, 35},        {3788, 40},        {3798, 45}, 

                                     {3816, 50},        {3850, 55},        {3880, 60},        {3905, 65},        {3933, 70}, 

                                     {3963, 75},        {4005, 80},        {4050, 85},        {4090, 90},        {4140, 95}, 

                                     {4180, 100}};




// Loading table for initialization of battery
struct SOCVPoint	PoOntable[OCVNPoints] = {
				{2930, 00},	{3590, 05},	{3625, 10},	{3660, 15},	{3690, 20}, 
				{3700, 25},	{3705, 30},	{3715, 35},	{3720, 40},	{3728, 45}, 
				{3745, 50},	{3780, 55},	{3710, 60},	{3835, 65},	{3865, 70}, 
				{3895, 75},	{3935, 80},	{3980, 85},	{4020, 90},	{4073, 95}, 
				{4130, 100}};

//
//declaration of RC table
//
#define ZAxis		8
#define YAxis		2
#define XAxis		25

// RC table X Axis value
int				XAxisElement[XAxis] = { 3000, 3045, 3135, 3200, 3260, 3305, 3345, 3380,
										3405, 3430, 3455, 3480, 3500, 3520, 3535, 3555,
										3575, 3590, 3605, 3625, 3645, 3660, 3680, 3695,
										3700};	//mV
// RC table Y Axis value
int				YAxisElement[YAxis] = {  700, 1500};	//10000C (1C=DesignCapacity)
// RC table Z Axis value, in 10*'C format
int			ZAxisElement[ZAxis] = { -75, -25,  50, 150, 250, 350, 450, 550};

// contents of RC table, its unit is 10000C, 1C = DesignCapacity
int				RCtable[YAxis*ZAxis][XAxis]={
{ 478, 480, 513, 561, 621, 688, 775, 873, 961,1066,1195,1354,1518,1714,1920,2307,2881,3518,4236,5116,5902,6378,6880,7077,7109},		//-7.5'C
{1194,1239,1380,1567,1900,2265,2718,3281,3810,4559,5744,7154,8074,8765,9108,9450,9800,10000,10000,10000,10000,10000,10000,10000,10000},
{ 228, 234, 270, 313, 366, 420, 484, 554, 615, 685, 769, 872, 982,1141,1316,1624,2052,2499,3029,3773,4507,4987,5546,5883,5981},		//-2.5'C
{ 608, 640, 740, 861,1057,1267,1522,1839,2162,2650,3387,4317,5047,5715,6152,6606,6945,7161,7394,7710,8003,8189,8386,8516,8639},
{  78,  86, 123, 164, 212, 258, 309, 362, 405, 455, 512, 582, 658, 795, 951,1211,1551,1884,2302,2963,3666,4148,4741,5163,5300},		//5'C
{ 255, 278, 353, 435, 549, 665, 801, 969,1168,1499,1965,2584,3199,3855,4351,4995,5594,5993,6335,6730,7076,7280,7511,7674,7752},
{  21,  31,  68, 108, 155, 198, 243, 290, 327, 369, 417, 473, 538, 667, 815,1058,1365,1655,2031,2661,3353,3836,4442,4895,5046},		//15'C
{ 124, 143, 209, 277, 360, 441, 532, 645, 799,1070,1435,1940,2511,3163,3681,4395,5092,5558,5941,6365,6731,6942,7185,7361,7422},
{   8,  18,  56,  96, 142, 185, 229, 274, 310, 350, 396, 449, 511, 638, 785,1024,1324,1604,1971,2594,3283,3766,4375,4836,4990},		//25'C
{  95, 114, 177, 242, 318, 391, 472, 573, 716, 975,1318,1796,2359,3009,3532,4262,4980,5461,5853,6284,6654,6867,7113,7292,7349},
{   5,  16,  53,  93, 139, 182, 226, 271, 306, 346, 391, 444, 505, 632, 779,1016,1315,1593,1957,2579,3268,3751,4361,4823,4978},		//35'C
{  88, 107, 170, 234, 308, 380, 459, 557, 698, 954,1292,1764,2325,2974,3499,4232,4955,5440,5834,6266,6637,6850,7096,7276,7333},
{   5,  15,  53,  92, 139, 181, 225, 270, 305, 345, 390, 443, 504, 630, 777,1015,1313,1590,1954,2576,3264,3747,4357,4820,4975},		//45'C
{  87, 105, 169, 232, 306, 377, 456, 554, 694, 949,1286,1757,2317,2967,3492,4226,4950,5435,5829,6262,6633,6847,7093,7273,7329},
{   5,  15,  53,  92, 138, 181, 225, 270, 305, 345, 390, 443, 503, 630, 777,1014,1312,1590,1954,2575,3263,3747,4357,4819,4974},		//55'C
{  86, 105, 168, 232, 306, 377, 456, 553, 693, 948,1285,1756,2315,2965,3490,4224,4948,5434,5828,6261,6633,6846,7092,7272,7328}};
/*-------------------------------------------------------------------------*/

static void OZ8806_TemperatureInit(void)
{
	int				i;
	struct RTPoint			*pRTPoint;

	for (i=1;i<NPoints;i++)
	{
		pRTPoint = &RTtable[i];
		pRTPoint->mV = RTtable[0].oC * pRTPoint->Ohm;
		pRTPoint->mV = pRTPoint->mV / (RTtable[0].Ohm + pRTPoint->Ohm);
	}	
}

int OZ8806_TemperaturemV2oC(int mVvalue)
{
	int		i,j;
	int	res;

	for (j=0;j<NPoints;j++)
	{
	   if (RTtable[j].Ohm==0)
			break;
	}
	for (i=NPoints-1; i>0; i--)
	{
		if (mVvalue < RTtable[i].mV) // increase
		{
			break;
		}
	}
	if (i==0)
	{
		res = RTtable[i].oC;
		res = res - (((RTtable[i].mV-mVvalue)*RTtable[i].oC)/RTtable[i].mV);
	}
	else if(i==j-1)
	{
		res = (mVvalue+RTtable[i-1].mV)*(RTtable[i-1].oC)/(RTtable[i-1].mV);
	}
	else if (mVvalue==RTtable[i-1].mV)
	{
		res=RTtable[i-1].oC;
	}
	else
	{
		res=((mVvalue-RTtable[i-1].mV)*(RTtable[i].oC-RTtable[i-1].oC)/(RTtable[i].mV-RTtable[i-1].mV) + RTtable[i-1].oC);
	}

	return res;
}

/*-------------------------------------------------------------------------*/

static int OZ8806_LoadingVoltToRC(void)
{
	int j;
	int res;

	for (j=0;j<OCVNPoints;j++)
	{
		if (PoOntable[j].iVoltage==batt_info.fVolt)
		{
			res = PoOntable[j].iRSOC;
			return res;
		}
		if(PoOntable[j].iVoltage > batt_info.fVolt)
			break;
	}
	if(j == 0)
		res = PoOntable[j].iRSOC;
	else if(j == OCVNPoints)
		res = PoOntable[j-1].iRSOC;
	else
	{
		res = ((batt_info.fVolt-PoOntable[j-1].iVoltage)*
				(PoOntable[j].iRSOC-PoOntable[j-1].iRSOC));
		res = res / (PoOntable[j].iVoltage-PoOntable[j-1].iVoltage);
		res += PoOntable[j-1].iRSOC;
	}

	return res;
}

static int OZ8806_PowerOnVoltToRC(void)
{
	int j;
	int res;

	for (j=0;j<OCVNPoints;j++)
	{
		if (OCVtable[j].iVoltage==batt_info.fVolt)
		{
			res = OCVtable[j].iRSOC;
			return res;
		}
		if(OCVtable[j].iVoltage > batt_info.fVolt)
			break;
	}
	if(j == 0)
		res = OCVtable[j].iRSOC;
	else if(j == OCVNPoints)
		res = OCVtable[j-1].iRSOC;
	else
	{
		res = ((batt_info.fVolt-OCVtable[j-1].iVoltage)*
				(OCVtable[j].iRSOC-OCVtable[j-1].iRSOC));
		res = res / (OCVtable[j].iVoltage-OCVtable[j-1].iVoltage);
		res += OCVtable[j-1].iRSOC;
	}

	return res;
}

/*-------------------------------------------------------------------------*/

static u8 OZ8806_control_register(struct i2c_client *client, int op, u8 indata)
{
	struct		OZ8806_data *data = i2c_get_clientdata(client);
	u8			bRet = 0;

	data->control	= ControlStatus;
	data->aout16	= indata;

	if(op == 1)
	{
		bRet =i2c_smbus_write_byte_data(client, data->control, data->aout16);
	}
	else
	{
		bRet = i2c_smbus_read_byte_data(client, data->control);
	}

	return bRet;
}

static void OZ8806_sleep_control(struct i2c_client *client, int sleepEn, int sleepOCV)
{
	u8			data8;
	data8= 0x05;
	if(sleepEn != 0)
	{
		data8 = data8 | CtBitSleepMode;
		if(sleepOCV != 0)
		{
			data8 = data8 | CtBitSleepOCVEn;
		}
	}
	else
	{
		data8 = data8 & ~CtBitSleepMode;
	}
	OZ8806_control_register(client, 1, data8);				//write to control register
}


static void OZ8806_CAR_Write(struct i2c_client *client)
{
	int			tmpVl;
	struct		OZ8806_data *data = i2c_get_clientdata(client);

	tmpVl = (((batt_info.fRC)*batt_info.fRsense)/batt_info.dbCARLSB);		//transfer to CAR

	data->control	= CellCARLow;
	data->aout16	= (u16)tmpVl;
	i2c_smbus_write_word_data(client, data->control, data->aout16);
	batt_info.fRC = ((data->aout16*batt_info.dbCARLSB)/batt_info.fRsense);
	//synchronize with CAR, cause LSB of CAR is 0.25mAhr
															//skip meaningless decimal point
															//if fRC = 10.0125, make it = 10.00
}

static void OZ8806_CAR_Reset(struct i2c_client *client)
{
	batt_info.fRC = batt_info.fRSOC * batt_info.fFCC / 100;
	OZ8806_CAR_Write(client);
	//Reserved Capacity should not used by OS
	batt_info.fRSOC = ((batt_info.fRC - batt_info.fReserved) * 100) / (batt_info.fFCC - batt_info.fReserved);
	if(batt_info.fRSOC >= 100)		batt_info.fRSOC = 100;
	if(batt_info.fRSOC <= 0)		batt_info.fRSOC = 0.0;
}

/*-------------------------------------------------------------------------*/

void OZ8806_init_chip(struct i2c_client *client)
{
	struct OZ8806_data *data = i2c_get_clientdata(client);
	u16		ADCValue;
	u8		bRet = 0, tmpADC;

	// Called when we have found OZ8806.
	// Initialize OZ8806 chip
	mutex_lock(&data->update_lock);

	OZ8806_sleep_control(client, 0, 0);				//wake up OZ8806 into FullPower mode

	data->control = ControlStatus;
	tmpADC = i2c_smbus_read_byte_data(client, data->control);
	if((tmpADC & CtBitChargeON) != 0)
	{
		batt_info.PowerStatus = 1;
	}
	else
	{
		batt_info.PowerStatus = 0;
	}

	data->control = CellVoltLow;
	ADCValue = i2c_smbus_read_word_data(client, data->control);
	if(ADCValue > 0)
	{
		ADCValue=ADCValue >> 4;
		ADCValue=ADCValue & CellVoltMASK;
		batt_info.fVolt = ((int)ADCValue * batt_info.fVoltLSB) / 100;
	}
	else
	{
		batt_info.fVolt = 3900;										//just for case
	}

	data->control = CellOCVLow;
	ADCValue = i2c_smbus_read_word_data(client, data->control);
	printk("CellOCVLow = 0x%x\n",ADCValue);
	if((ADCValue > 0) &&(ADCValue & CellOCVPoOnMask))				//Power On OCV detect
	{
		bRet = 1;
		ADCValue=ADCValue >> 4;									//if no OCV flag, use old value for pretended initialization
		ADCValue=ADCValue & CellOCVMASK;
		batt_info.fOCVVolt = ((int)ADCValue * batt_info.fVoltLSB) / 100;
	}
	
	printk("000...batt_info.fOCVVolt is %d\n",batt_info.fOCVVolt);

	if((bRet == 1) && (batt_info.PowerStatus == 0))					//system goes normal booting
	{
		printk("if((bRet == 1) && (batt_info.PowerStatus == 0))\n");
		batt_info.fPerFromOCV = OZ8806_LoadingVoltToRC();					//use CellVolt to initial
		//if((fPerFromOCV>=(fRSOC+0.10)) || (fPerFromOCV<=(fRSOC-0.10)))		// out of +/- 10% 
		{											//delta percentage is big
			batt_info.fRSOC = batt_info.fPerFromOCV;					//use fRSOC from OCV table to initialize CAR
			OZ8806_CAR_Reset(client);
		}
//		else
//		{
//			OZ8806_CAR_Write(client);						//use fRC value from Registry
//		}
	}
	else if((bRet == 1) && (batt_info.PowerStatus == 1))			//AC-in and boot up system
	{
		printk("else if((bRet == 1) && (batt_info.PowerStatus == 1))\n");
		batt_info.fPerFromOCV = OZ8806_PowerOnVoltToRC();					//use PoOCV to initial
		//if((fPerFromOCV>=(fRSOC+0.10)) || (fPerFromOCV<=(fRSOC-0.10)))		// out of +/- 10% 
		{											//delta percentage is big
			batt_info.fRSOC = batt_info.fPerFromOCV;					//use fRSOC from OCV table to initialize CAR
			OZ8806_CAR_Reset(client);
		}
//		else
//		{
//			OZ8806_CAR_Write(client);						//use fRC value from Registry
//		}
	}
	else
	{
		data->control = CellCARLow;
		ADCValue = i2c_smbus_read_word_data(client, data->control);
		ADCValue=ADCValue & CellCARMASK;
		batt_info.fRC = (((short)ADCValue * batt_info.dbCARLSB) / batt_info.fRsense);
		//if((fRC >= 0.1*fFCC) && (fRC <= fFCC))				//a reasonable capacity in OZ8806
		{
			batt_info.fRSOC = ((batt_info.fRC - batt_info.fReserved) * 100 )/ 
				(batt_info.fFCC - batt_info.fReserved);
		}
	}

	//initial sCaMAH equals to fRC
	batt_info.sCaMAH = batt_info.fRC;

	mutex_unlock(&data->update_lock);
}

bool OZ8806_PollingLoop(void)
{
	struct OZ8806_data *data = i2c_get_clientdata(the_OZ8806->myclient);
	u16			ADCValue;
	
	batt_info.fPrevCurr = batt_info.fCurr;							//saved previous current
	batt_info.fRCPrev = batt_info.fRC;

	mutex_lock(&data->update_lock);
//	data->control = CellTempLow;
//	ADCValue = i2c_smbus_read_word_data(the_OZ8806->myclient, data->control);
//		ADCValue=ADCValue >> 4;
//		ADCValue=ADCValue & CellTempMASK;
//		batt_info.fCellTemp = ADCValue * batt_info.fVoltLSB;		//fVoltLSB = 250 (2.5 mV)
//		batt_info.fCellTemp = OZ8806_TemperaturemV2oC(batt_info.fCellTemp/100);
		batt_info.fCellTemp = 25;

	data->control = CellVoltLow;
	ADCValue = i2c_smbus_read_word_data(the_OZ8806->myclient, data->control);
		ADCValue=ADCValue >> 4;
		ADCValue=ADCValue & CellTempMASK;
		batt_info.fVolt = (ADCValue * batt_info.fVoltLSB) / 100;	//fVoltLSB = 250 (2.5 mV)
		//printk("batt_info.fVolt is %d\n",batt_info.fVolt);
	data->control = CellCurrLow;
	ADCValue = i2c_smbus_read_word_data(the_OZ8806->myclient, data->control);
		ADCValue=ADCValue & CellCurrMASK;
		batt_info.fCurr = (short)ADCValue * batt_info.dbCurrLSB;	//dbCurrLSB = 391 (3.90625 mA)
		batt_info.fCurr = (batt_info.fCurr / batt_info.fRsense) / 100;
		batt_info.fCurr /= 2;
	data->control = CellCARLow;
	ADCValue = i2c_smbus_read_word_data(the_OZ8806->myclient, data->control);
		ADCValue=ADCValue & CellCARMASK;
		batt_info.fRC = (short)(ADCValue) * batt_info.dbCARLSB;
		batt_info.fRC = batt_info.fRC / batt_info.fRsense;
		batt_info.fRSOC = (batt_info.fRC - batt_info.fReserved) * 100;
		batt_info.fRSOC = batt_info.fRSOC / (batt_info.fFCC - batt_info.fReserved);
		if(batt_info.fRSOC >= 100)		batt_info.fRSOC = 100;
		if(batt_info.fRSOC <= 0)		batt_info.fRSOC = 0;
		if((batt_info.fCurr < batt_info.fEOC) 
			&& (batt_info.fVolt >= batt_info.fVoltFCHG))				//EOC condition, this may be different during each project
		{		//EOC condition, this may be different during each project
			if((batt_info.fPrevCurr < batt_info.fCurr+2) && (batt_info.fCurr >= 0))
			{
				batt_info.fRSOC = 100;
				OZ8806_CAR_Reset(the_OZ8806->myclient);
			}
		}
		batt_info.fRC = (batt_info.fRSOC * batt_info.fFCC) / 100;

	batt_info.fRCDelta = batt_info.fRC - batt_info.fRCPrev;
	
	
	printk("111...batt_info.fVolt is %d\n",batt_info.fVolt);
	printk("111...batt_info.fRSOC is %d\n",batt_info.fRSOC);
	printk("111...batt_info.fRC is %d\n",batt_info.fRC);
	printk("111...batt_info.fCurr is %d\n",batt_info.fCurr);
  printk("111...batt_info.fOCVVolt is %d\n",batt_info.fOCVVolt);
	mutex_unlock(&data->update_lock);

	return true;
}

/*-------------------------------------------------------------------------*/

void OZ8806_EOCXSet(void)
{
	int i;

	for(i=4;i>0;i--)
	{
		if(batt_info.fCurr > (i+1)*batt_info.fEOC)
		{
			sECxMAH[i-1] = -1;
		}
		else
		{
			if(sECxMAH[i-1] == -1)
			{
				sECxMAH[i-1] = batt_info.fRC;
			}
		}
	}
}

void OZ8806_EOCBlend(void)
{
	int		l=0,h=3;
	int		fTemp;

	while(l<=3)
	{
		if(sECxMAH[l] > 0)
			break;
		l++;
	}

	while(h>=0)
	{
		if(sECxMAH[h] > 0)
			break;
		h--;
	}

	if(h>=0)
	{
		int fraction;

		fraction = ((batt_info.fCurr - batt_info.fEOC) * 100) / (batt_info.fEOC * (h+1));
		fTemp = fraction * (batt_info.fRC) + ((100-fraction)*batt_info.sCfMAH);
		if(fTemp > 100*(batt_info.sCaMAH + 2*batt_info.fRCDelta))					//prevent fRSOC jump
		{
			batt_info.sCaMAH += 2*batt_info.fRCDelta;
		}
		else
		{
			batt_info.sCaMAH = fTemp / 100;
		}
	}
}

void OZ8806_ProcessEndCharging(void)
{
	//sCrMAH = sCrpMAH;
	if(yOZState  & STATEFULLEOC)
	{
		if(bHadFullEOD)														//do Fully EOD first then do Fully EOC
		{
			batt_info.fFCC		= batt_info.fRC;
			batt_info.sCfMAH	= batt_info.fFCC;							//learning cycle, update sCfMAH
			batt_info.sCaMAH	= batt_info.sCfMAH;
		}
		else
		{
			batt_info.fRC		= batt_info.sCaMAH;
			OZ8806_CAR_Write(the_OZ8806->myclient);										//synchronize CAR
			batt_info.sCaMAH	= batt_info.fRC;							//LSB of CAR is 0.25mAhr, skip meaningless decimal point
		}
	}
	else if (yOZState & STATEEOC)											//not Fully EOC
	{
		if(batt_info.sCaMAH != (batt_info.fRC))								//had done EOC blending, but not Fully EOC
		{
			batt_info.fRC		= batt_info.sCaMAH;
			OZ8806_CAR_Write(the_OZ8806->myclient);
			batt_info.sCaMAH	= batt_info.fRC;							//LSB of CAR is 0.25mAhr, skip meaningless decimal point
		}
	}
	bHadFullEOD = false;
}

void OZ8806_ProcessEndDischarging(void)
{
	//int		RC1;

	if(yOZState & STATEFULLEOD)
	{
		batt_info.sCrMAH		= batt_info.fRC;
		batt_info.sCaMAH		= batt_info.sCrMAH;
		bHadFullEOD = true;
	}
	else if(yOZState & STATEEOD)
	{
		if(batt_info.fRSOC < 98)
		{
			batt_info.sCrMAH	= (batt_info.fRC*100 - batt_info.sCfMAH * batt_info.fRSOC) / (100-batt_info.fRSOC);
			batt_info.sCrMAH	/= 100;
		}
		batt_info.sCaMAH		= batt_info.fRC;
		bHadFullEOD = false;
	}
}


int OZ8806_ma2c10k(int infCurr)
{
	return (int)((infCurr*10000)/batt_info.fFCC);
}


int OZ8806_c10k2mah(int capIn10kC)
{
	return (int)((capIn10kC*batt_info.fFCC) / 10000);
}


bool OZ8806_LookUpRCTable(int infTemp, int infCurr, int infVolt, int *infCal)
{
	bool	bRet = true;
	int		indexX, indexY, indexZ;
	long	fRCtemp1, fRCtemp2, fRCInter1, fRCInter2, flongCal;

	for(indexX=1;indexX<XAxis;indexX++)
	{
		if((XAxisElement[indexX-1] <= infVolt) && (XAxisElement[indexX] > infVolt))
		{
			break;
		}
	}
	for(indexY=0;indexY<YAxis;indexY++)
	{
		if(YAxisElement[indexY] >= infCurr)
		{
			break;
		}
	}
	for(indexZ=0;indexZ<ZAxis;indexZ++)
	{
		if(ZAxisElement[indexZ] >= infTemp)
		{
			break;
		}
	}

	if((indexY != 0) && (indexZ !=0) && (indexY != YAxis) && (indexZ != ZAxis))
	{
		fRCtemp1 = (long)(RCtable[(indexY-1)+(indexZ-1)*YAxis][indexX-1])*100;
		fRCtemp1 +=((long)(infVolt - XAxisElement[indexX-1])*100 / 
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY-1)+(indexZ-1)*YAxis][indexX] - RCtable[(indexY-1)+(indexZ-1)*YAxis][indexX-1]); 
		fRCtemp2 = (long)(RCtable[(indexY)+(indexZ-1)*YAxis][indexX-1])*100;
		fRCtemp2 +=((long)(infVolt - XAxisElement[indexX-1])*100 / 
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY)+(indexZ-1)*YAxis][indexX] - RCtable[(indexY)+(indexZ-1)*YAxis][indexX-1]); 
		fRCInter1 = fRCtemp1;
		fRCInter1 +=(long)((infCurr - YAxisElement[indexY-1]) /
					(long)(YAxisElement[indexY] - YAxisElement[indexY-1])) *
					(fRCtemp2 - fRCtemp1);

		fRCtemp1 = (long)(RCtable[(indexY-1)+(indexZ)*YAxis][indexX-1])*100;
		fRCtemp1 +=((long)(infVolt - XAxisElement[indexX-1])*100 / 
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY-1)+(indexZ)*YAxis][indexX] - RCtable[(indexY-1)+(indexZ)*YAxis][indexX-1]); 
		fRCtemp2 = (long)(RCtable[(indexY)+(indexZ)*YAxis][indexX-1])*100;
		fRCtemp2 +=((long)(infVolt - XAxisElement[indexX-1])*100 / 
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY)+(indexZ)*YAxis][indexX] - RCtable[(indexY)+(indexZ)*YAxis][indexX-1]); 
		fRCInter2 = fRCtemp1;
		fRCInter2 +=(long)((infCurr - YAxisElement[indexY-1]) /
					(long)(YAxisElement[indexY] - YAxisElement[indexY-1])) *
					(fRCtemp2 - fRCtemp1);
		flongCal = fRCInter1;
		flongCal +=((long)(infTemp - ZAxisElement[indexZ-1]) /
				(long)(ZAxisElement[indexZ] - ZAxisElement[indexZ-1])) *
				(fRCInter2 - fRCInter1);
		*infCal = (int)(flongCal/100);
	}
	else if(indexY == 0)						//current is too low, but no matter temperature is below
	{
		fRCInter1 = (long)OZ8806_PowerOnVoltToRC()*100;
		//fRCInter1 *= 10000;
		fRCInter1 *= 100;
		fRCtemp1 = (long)(RCtable[(indexY)+(indexZ-1)*YAxis][indexX-1])*100;
		fRCtemp1 +=((long)(infVolt - XAxisElement[indexX-1])*100 / 
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY)+(indexZ-1)*YAxis][indexX] - RCtable[(indexY)+(indexZ-1)*YAxis][indexX-1]); 
		fRCtemp2 = (long)(RCtable[(indexY)+(indexZ)*YAxis][indexX-1])*100;
		fRCtemp2 +=((long)(infVolt - XAxisElement[indexX-1])*100 / 
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY)+(indexZ)*YAxis][indexX] - RCtable[(indexY)+(indexZ)*YAxis][indexX-1]); 
		fRCInter2 = fRCtemp1;
		fRCInter2 +=(long)((infTemp - ZAxisElement[indexZ-1]) /
					(long)(ZAxisElement[indexZ] - ZAxisElement[indexZ-1])) *
					(fRCtemp2 - fRCtemp1);
		flongCal = fRCInter1;
		flongCal +=((long)(infCurr - batt_info.fDisCOCV) /
				(long)(YAxisElement[indexY] - batt_info.fDisCOCV)) *
				(fRCInter2 - fRCInter1);
		*infCal = (int)(flongCal/100);
	}
	else if((indexZ == 0) || (indexZ == ZAxis))
	{
		if(indexZ >= ZAxis)			indexZ = ZAxis - 1;
		fRCtemp1 = (long)(RCtable[(indexY-1)+(indexZ)*YAxis][indexX-1])*100;
		fRCtemp1 +=((long)(infVolt - XAxisElement[indexX-1])*100 / 
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY-1)+(indexZ)*YAxis][indexX] - RCtable[(indexY-1)+(indexZ)*YAxis][indexX-1]); 
		fRCtemp2 = (long)(RCtable[(indexY)+(indexZ)*YAxis][indexX-1])*100;
		fRCtemp2 +=((long)(infVolt - XAxisElement[indexX-1]) / 
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY)+(indexZ)*YAxis][indexX] - RCtable[(indexY)+(indexZ)*YAxis][indexX-1]); 
		fRCInter1 = fRCtemp1;
		fRCInter1 +=(long)((infCurr - YAxisElement[indexY-1]) /
					(long)(YAxisElement[indexY] - YAxisElement[indexY-1])) *
					(fRCtemp2 - fRCtemp1);
		*infCal = (int)(fRCInter1/100);
	}
	else if(indexY == YAxis)
	{
		indexY--;
		fRCtemp1 = (long)(RCtable[(indexY-1)+(indexZ-1)*YAxis][indexX-1])*100;
		fRCtemp1 +=((long)(infVolt - XAxisElement[indexX-1])*100 / 
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY-1)+(indexZ-1)*YAxis][indexX] - RCtable[(indexY-1)+(indexZ-1)*YAxis][indexX-1]); 
		fRCtemp2 = (long)(RCtable[(indexY)+(indexZ-1)*YAxis][indexX-1])*100;
		fRCtemp2 +=((long)(infVolt - XAxisElement[indexX-1]) / 
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY)+(indexZ-1)*YAxis][indexX] - RCtable[(indexY)+(indexZ-1)*YAxis][indexX-1]); 
		fRCInter1 = fRCtemp1;
		fRCInter1 +=(long)((infCurr - YAxisElement[indexY-1]) /
					(long)(YAxisElement[indexY] - YAxisElement[indexY-1])) *
					(fRCtemp2 - fRCtemp1);

		fRCtemp1 = (long)(RCtable[(indexY-1)+(indexZ)*YAxis][indexX-1])*100;
		fRCtemp1 +=((long)(infVolt - XAxisElement[indexX-1])*100 / 
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY-1)+(indexZ)*YAxis][indexX] - RCtable[(indexY-1)+(indexZ)*YAxis][indexX-1]); 
		fRCtemp2 = (long)(RCtable[(indexY)+(indexZ)*YAxis][indexX-1])*100;
		fRCtemp2 +=((long)(infVolt - XAxisElement[indexX-1])*100 / 
					(long)(XAxisElement[indexX] - XAxisElement[indexX-1])) *
					(long)(RCtable[(indexY)+(indexZ)*YAxis][indexX] - RCtable[(indexY)+(indexZ)*YAxis][indexX-1]); 
		fRCInter2 = fRCtemp1;
		fRCInter2 +=(long)((infCurr - YAxisElement[indexY-1]) /
					(long)(YAxisElement[indexY] - YAxisElement[indexY-1])) *
					(fRCtemp2 - fRCtemp1);
		flongCal = fRCInter1;
		flongCal +=((long)(infTemp - ZAxisElement[indexZ-1]) /
				(long)(ZAxisElement[indexZ] - ZAxisElement[indexZ-1])) *
				(fRCInter2 - fRCInter1);
		*infCal = (int)(flongCal/100);
	}




	return bRet;
}


bool OZ8806_GaugeAdjustion(void)
{
	struct OZ8806_data *data = i2c_get_clientdata(the_OZ8806->myclient);

	mutex_lock(&data->update_lock);

	if(batt_info.fCurr > fStateDtm)											//charging
	{
		yPrevOZState = yOZState;
		yOZState = STATECHARGING;
		if(yPrevOZState == STATEDISCHARGING)								//end discharging then charge immediately
		{
			yOZState |= STATEEOD;
			OZ8806_ProcessEndDischarging();									//calculate EOD first, then do CHG process
			yOZState &= (~STATEEOD);
		}
		OZ8806_EOCXSet();
		batt_info.sCaMAH += batt_info.fRCDelta;
		if((batt_info.fVolt >= VOLTCVMODE) && (batt_info.fCurr >= batt_info.fEOC))
		{
			OZ8806_EOCBlend();
		}
		if((batt_info.fCurr < batt_info.fEOC) && (batt_info.fVolt >= batt_info.fVoltFCHG))						//EOC condition, this may be different during each project
		{		//EOC condition, this may be different during each project
			if((batt_info.fPrevCurr < batt_info.fCurr+2) && (batt_info.fCurr >= 0))
			{
				if(!(yOZState & STATEFULLEOC))
				{
					yOZState |= STATEFULLEOC;
					OZ8806_ProcessEndCharging();
//					yOZState = STATEIDLE;
				}
			}
		}
		batt_info.fRSOC = ((batt_info.sCaMAH - batt_info.sCrMAH) * 100) / (batt_info.sCfMAH - batt_info.sCrMAH);		//get RSOC
		if(batt_info.fRSOC >= 100)
		{
			batt_info.fRSOC = 100;
			OZ8806_CAR_Reset(the_OZ8806->myclient);							//prevent overflow
		}
		if(batt_info.fRSOC <= 0)
		{
			batt_info.fRSOC = 0;
			OZ8806_CAR_Reset(the_OZ8806->myclient);							//prevent overflow
		}
	}
	else if(batt_info.fCurr < (-fStateDtm))									//discharging
	{
		yPrevOZState = yOZState;
		yOZState = STATEDISCHARGING;
		if(yPrevOZState == STATECHARGING)									//end charging then discharge immediately
		{
			yOZState |= STATEEOC;
			OZ8806_ProcessEndCharging();									//calculate EOC first then do DSG process
			yOZState &= (~STATEEOC);
		}
		if(batt_info.fVolt > iVoltAdjHigh)									//voltage higher than adjust
		{
			batt_info.sCaMAH += batt_info.fRCDelta;							//do coulomb counting
			//sCeodMAH = sCaMAH;
			if(batt_info.sCaMAH > batt_info.sCfMAH+50)
			{
				batt_info.sCaMAH = batt_info.sCfMAH - 1;					//set to 99%
				batt_info.fRCPrev = batt_info.fRC = batt_info.sCaMAH;		//synchronize variables
				OZ8806_CAR_Write(the_OZ8806->myclient);											//write to CAR register
			}
		}
		else
		{
			if(batt_info.fVolt > iEODLowVolt)								//higher than Low Volt
			{																//calculate sCa by RC table reference
				int		RC1, RC2;
				OZ8806_LookUpRCTable(batt_info.fCellTemp*10, 
									-1*(OZ8806_ma2c10k(batt_info.fCurr)), 
									batt_info.fVolt, &RC1);
				OZ8806_LookUpRCTable(batt_info.fCellTemp*10, 
									-1*(OZ8806_ma2c10k(batt_info.fCurr)), 
									iEODLowVolt, &RC2);
				batt_info.sCeodMAH = batt_info.sCrMAH + OZ8806_c10k2mah(RC1 - RC2);
				if(batt_info.sCaMAH > batt_info.sCeodMAH)						//if greater than table calculated value, faster to close it
				{
					batt_info.sCaMAH += (int)((150*batt_info.fRCDelta)/100);
				}
				else
				{
					batt_info.sCaMAH += batt_info.fRCDelta;
				}
				//batt_info.sCaMAH += batt_info.fRCDelta;
			}
			else															//under voltage stage
			{
				if(!(yOZState & STATEFULLEOD))
				{
					yOZState |= STATEFULLEOD;
					OZ8806_ProcessEndDischarging();
//					yOZState = STATEIDLE;
				}
			}
		}
		//get RSOC
		batt_info.fRSOC = ((batt_info.sCaMAH - batt_info.sCrMAH) * 100) / (batt_info.sCfMAH - batt_info.sCrMAH);
		if(batt_info.fRSOC >= 100)
		{
			batt_info.fRSOC = 100;
			OZ8806_CAR_Reset(the_OZ8806->myclient);							//prevent overflow
		}
		if(batt_info.fRSOC <= 1)		batt_info.fRSOC = 0;				//below 1%=> fully EndOfDischarge
		{
			if(yOZState & STATEFULLEOD)
			{
				yOZState |= STATEFULLEOD;
				OZ8806_ProcessEndDischarging();
//				yOZState = STATEIDLE;
			}
		}
	}
	else																	//idle
	{
		yPrevOZState = yOZState;
		yOZState = STATEIDLE;
		//get RSOC
		if(yPrevOZState & STATECHARGING)									//going idle from CHG, 
		{
			yOZState |= STATEEOC;
			OZ8806_ProcessEndCharging();
			yOZState &= (~STATEEOC);
		}
		else if(yPrevOZState & STATEDISCHARGING)							//going idle from DSG
		{
			yOZState |= STATEEOD;
			OZ8806_ProcessEndDischarging();
			yOZState &= (~STATEEOD);
		}
		else
		{
			batt_info.sCaMAH = batt_info.fRC;								//synchronize sCaMAH with coulomb counting
		}
		batt_info.fRSOC = ((batt_info.sCaMAH - batt_info.sCrMAH) * 100) / (batt_info.sCfMAH - batt_info.sCrMAH);
		if(batt_info.fRSOC >= 100)		batt_info.fRSOC = 100;
		if(batt_info.fRSOC <= 0)		batt_info.fRSOC = 0;
	}

	if(gAdapatRemove > 0 && gAdapatRemove < 8)
	{
		gAdapatRemove++;
	}else if(gAdapatRemove >= 8)
	{
		gAdapatRemove = 0;
	}

	mutex_unlock(&data->update_lock);

	return true;
}

/*-------------------------------------------------------------------------*/

static int OZ8806_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE_DATA
				     | I2C_FUNC_SMBUS_READ_BYTE))
		return -ENODEV;

	strlcpy(info->type, MYDRIVER, I2C_NAME_SIZE);

	return 0;
}

static int oz8806_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	
	struct OZ8806_data *data = container_of(psy, struct OZ8806_data, bat);
	switch (psp) {
	
	case POWER_SUPPLY_PROP_STATUS:
		if(gpio_get_value(data->dc_det_pin))
			val->intval = 3;	/*discharging*/
		else
			val->intval = 1;	/*charging*/
		//printk("cdy ----- battery ---%d\n" , val->intval);
		
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = batt_info.fVolt;
		//printk("oz8806_battery_get_property batt_info.fVolt =%d \n",batt_info.fVolt);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;//batt_info.fRSOC<0 ? 0:1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = batt_info.fCurr;
		printk("333...oz8806_battery_get_property batt_info.fCurr =%d \n",batt_info.fCurr);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if(gAdapatRemove> 0 && gAdapatRemove < 8){
			val->intval = 100;
		}else{
			val->intval = batt_info.fRSOC;
		}
		gPreFrsoc = val->intval;
		printk("333...oz8806_battery_get_property batt_info.fRSOC =%d \n",batt_info.fRSOC);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

static int oz8806_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	struct OZ8806_data *data = container_of(psy, struct OZ8806_data, ac);
	
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS){
			if(gpio_get_value(data->dc_det_pin))
				val->intval = 0;	/*discharging*/
			else
				val->intval = 1;	/*charging*/
		}
		break;
		
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static void oz8806_powersupply_init(struct OZ8806_data *data)
{
	data->bat.name = "battery";
	data->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	data->bat.properties = oz8806_battery_props;
	data->bat.num_properties = ARRAY_SIZE(oz8806_battery_props);
	data->bat.get_property = oz8806_battery_get_property;
	
	data->ac.name = "ac";
	data->ac.type = POWER_SUPPLY_TYPE_MAINS;
	data->ac.properties = oz8806_ac_props;
	data->ac.num_properties = ARRAY_SIZE(oz8806_ac_props);
	data->ac.get_property = oz8806_ac_get_property;
}





static void oz8806_battery_work(struct work_struct *work)
{
	struct OZ8806_data *data = container_of(work, struct OZ8806_data, work.work); 
	
	OZ8806_PollingLoop();
	OZ8806_GaugeAdjustion();
	printk("222...batt_info.fVolt is %d\n",batt_info.fVolt);
	printk("222...batt_info.fRSOC is %d\n",batt_info.fRSOC);
	printk("222...batt_info.fRC is %d\n",batt_info.fRC);
	printk("222...batt_info.fCurr is %d\n",batt_info.fCurr);
	power_supply_changed(&data->bat);
	/* reschedule for the next time */
	schedule_delayed_work(&data->work, data->interval);
}

static irqreturn_t OZ8806_dc_wakeup(int irq, void *dev_id)
{   
    schedule_work(&the_OZ8806->dcwakeup_work);
    return IRQ_HANDLED;
}

static void OZ8806_dcdet_delaywork(struct work_struct *work)
{
    int ret;

	struct OZ8806_data *data = container_of(work, struct OZ8806_data, dcwakeup_work); 
    
	int irq      = gpio_to_irq(data->dc_det_pin);
    int irq_flag = gpio_get_value (data->dc_det_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
    
    rk28_send_wakeup_key();
    
    free_irq(irq, NULL);
    ret = request_irq(irq, OZ8806_dc_wakeup, irq_flag, "oz8806_dc_det", NULL);
	if (ret) {
		free_irq(irq, NULL);
	}

	if(irq_flag == IRQF_TRIGGER_FALLING && batt_info.fRSOC == 99 && gPreFrsoc == 100)
	{
		gAdapatRemove = 1;
	}
	power_supply_changed(&data->bat);
}

static int OZ8806_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret, irq, irq_flag;
	struct OZ8806_data *data;

	if (!(data = kzalloc(sizeof(struct OZ8806_data), GFP_KERNEL)))
		return -ENOMEM;

	//Note that mainboard definition file, ex: arch/arm/mach-msm/board-xxx.c, must has declared
	// static struct i2c_board_info xxx_i2c_devs[] __initdata = {....}
	// and it must add including this "I2C_BOARD_INFO("OZ8806", 0x2F)," 
	// otherwise, probe will occur error
	// string is matching with definition in OZ8806_id id table

	// Init real i2c_client 
	i2c_set_clientdata(client, data);

	the_OZ8806 = data;
	data->myclient = client;
	data->interval = msecs_to_jiffies(4 * 1000);
	data->dc_det_pin = RK30_PIN6_PA5;

	if (data->dc_det_pin != INVALID_GPIO)
	{
		ret = gpio_request(data->dc_det_pin, "oz8806_dc_det");
		if (ret != 0) {
			gpio_free(data->dc_det_pin);
			printk("fail to request dc_det_pin\n");
			return -EIO;
		}

		INIT_WORK(&data->dcwakeup_work, OZ8806_dcdet_delaywork);
		irq = gpio_to_irq(data->dc_det_pin);
	        
		irq_flag = gpio_get_value (data->dc_det_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
	    	ret = request_irq(irq, OZ8806_dc_wakeup, irq_flag, "oz8806_dc_det", NULL);
	    	if (ret) {
	    		printk("failed to request dc det irq\n");
				return -EIO;
	    	}
	    	enable_irq_wake(irq);	
	}

	mutex_init(&data->update_lock);

	
	INIT_DELAYED_WORK(&data->work, oz8806_battery_work);
	schedule_delayed_work(&data->work, data->interval);

	// Init OZ8806 chip
	OZ8806_init_chip(client);
	OZ8806_TemperatureInit();

	//OZ8806_PollingLoop();
	//OZ8806_GaugeAdjustion();
	
	oz8806_powersupply_init(data);
	ret = power_supply_register(&client->dev, &the_OZ8806->bat);
	if (ret) {
		printk(KERN_ERR "failed to register battery\n");
		return ret;
	}
	ret = power_supply_register(&client->dev, &the_OZ8806->ac);
	if (ret) {
		printk(KERN_ERR "failed to register ac\n");
		return ret;
	}
	printk("444...batt_info.fVolt is %d\n",batt_info.fVolt);
	printk("444...batt_info.fRSOC is %d\n",batt_info.fRSOC);
	printk("444...batt_info.fRC is %d\n",batt_info.fRC);
	printk("444...batt_info.fCurr is %d\n",batt_info.fCurr);
	printk("%s %d",__FUNCTION__,__LINE__);
	return 0;					//return Ok
}

static int OZ8806_remove(struct i2c_client *client)
{
	struct OZ8806_data *data = i2c_get_clientdata(client);

	power_supply_unregister(&data->bat);
	power_supply_unregister(&data->ac);
	cancel_delayed_work(&data->work);
	gpio_free(data->dc_det_pin);
	kfree(data);

	return 0;
}

static int OZ8806_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct OZ8806_data *data = i2c_get_clientdata(client);

	cancel_delayed_work(&data->work);
	mutex_lock(&data->update_lock);
	OZ8806_CAR_Write(client);
	OZ8806_sleep_control(client, 1, 1);
	mutex_unlock(&data->update_lock);
	return 0;
}

static int OZ8806_resume(struct i2c_client *client)
{
	struct OZ8806_data *data = i2c_get_clientdata(client);
	u16			ADCValue;
	int				tmpfRSOC;

	mutex_lock(&data->update_lock);
	data->control = CellOCVLow;
	ADCValue = i2c_smbus_read_word_data(client, data->control);
	if(ADCValue & CellOCVSleepMask)							//Sleep OCV detect
	{
		ADCValue=ADCValue >> 4;								//if no OCV flag, use old value for pretended initialization
		ADCValue=ADCValue & CellOCVMASK;
		batt_info.fOCVVolt = (ADCValue * batt_info.fVoltLSB) / 100;
		tmpfRSOC= OZ8806_PowerOnVoltToRC();
		if(tmpfRSOC < batt_info.fRSOC)
		{
			batt_info.fRSOC = tmpfRSOC;
			OZ8806_CAR_Reset(client);
			//sCaMAH = fRCPrev = fRC;
		}
	}
	else													//no Sleep OCV detection, use current CAR
	{
		data->control = CellCARLow;
		ADCValue = i2c_smbus_read_word_data(client, data->control);
			ADCValue=ADCValue & CellCARMASK;
			batt_info.fRC = (short)(ADCValue)*batt_info.dbCARLSB;
			batt_info.fRC = batt_info.fRC / batt_info.fRsense;
			batt_info.fRSOC = (batt_info.fRC - batt_info.fReserved) * 100;
			batt_info.fRSOC = batt_info.fRSOC / (batt_info.fFCC - batt_info.fReserved);
			if(batt_info.fRSOC >= 100)		batt_info.fRSOC = 100;
			if(batt_info.fRSOC <= 0)		batt_info.fRSOC = 0;
	}
	OZ8806_sleep_control(client, 0, 0);
	mutex_unlock(&data->update_lock);

	schedule_delayed_work(&data->work, data->interval);
	return 0;
}


/*-------------------------------------------------------------------------*/

static const struct i2c_device_id OZ8806_id[] = {
	{ MYDRIVER, 0 },							//string, id??
	{ }
};
MODULE_DEVICE_TABLE(i2c, OZ8806_id);

static struct i2c_driver OZ8806_driver = {
	.driver = {
		.name	= MYDRIVER,
	},
	.probe			= OZ8806_probe,
	.remove			= OZ8806_remove,
	//.suspend		= OZ8806_suspend,
	//.resume			= OZ8806_resume,
	.id_table		= OZ8806_id,

	//auto-detection function
	//.class			= I2C_CLASS_HWMON,			// Nearest choice
	//.detect			= OZ8806_detect,
	//.address_data	= &addr_data,
};

/*-------------------------------------------------------------------------*/

static int __init OZ8806_init(void)
{
	return i2c_add_driver(&OZ8806_driver);
}

static void __exit OZ8806_exit(void)
{
	i2c_del_driver(&OZ8806_driver);
}

/*-------------------------------------------------------------------------*/

#define	DRIVER_VERSION	"24 June 2009"
#define	DRIVER_NAME	(OZ8806_driver.driver.name)

MODULE_DESCRIPTION("OZ8806 Battery Monitor IC Driver");
MODULE_LICENSE("O2Micro");

module_init(OZ8806_init);
module_exit(OZ8806_exit);


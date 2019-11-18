

//////////////////////////////
#include "hrs3300.h"
//#include "hrs3300_alg.h"
#include "hrs3300_reg_init.h"
//////////////////////////////
//#define GSENSER_DATA

// hrs3300 customer config
const uint8_t  hrs3300_bp_timeout_grade = 0;  // max 15
const uint8_t  hrs3300_agc_init_stage = 0x04;  // init AGC state  
const uint8_t  hrs3300_bp_power_grade = 0;
const uint8_t  hrs3300_accurate_first_shot = 0;
const uint8_t  hrs3300_up_factor = 3;
const uint8_t  hrs3300_up_shift = 2;
const uint16_t hrs3300_AMP_LTH = 120;
const uint16_t hrs3300_hr_AMP_LTH = 150;
const uint16_t hrs3300_hr_PVAR_LTH = 10;
// hrs3300 customer config end

//20161117 added by ericy for "low power in no_touch state"
static bool hrs3300_power_up_flg = 0 ;
uint8_t reg_0x7f ;
uint8_t reg_0x80 ;
uint8_t reg_0x81 ;
uint8_t reg_0x82 ;
//20161117 added by ericy for "low power in no_touch state"

#if 0
bool Hrs3300_write_reg(uint8_t addr, uint8_t data) 
{
	// I2c custom  return  0(success), 1(fail)
#if 0
    uint8_t data_buf[2];	
		
	  data_buf[0] = addr;
	  data_buf[1] = data;

	  twi_pin_switch(1);
    twi_master_transfer(0x88, data_buf, 2, true);	//write	
#endif
	return 0;  	
}

uint8_t Hrs3300_read_reg(uint8_t addr) 
{
    uint8_t data_buf = 0;	
#if 0
	  twi_pin_switch(1);
	  twi_master_transfer(0x88, &addr, 1, false);	//write
	  twi_master_transfer(0x89, &data_buf, 1, true);//read
#endif
	  return data_buf;  	
}
#endif

uint16_t Hrs3300_read_hrs(void)
{
	uint8_t  databuf[3];
	uint16_t data;

	databuf[0] = Hrs3300_read_reg(0x09);	// addr09, bit
  databuf[1] = Hrs3300_read_reg(0x0a);	// addr0a, bit
  databuf[2] = Hrs3300_read_reg(0x0f);	// addr0f, bit
	
	data = ((databuf[0]<<8)|((databuf[1]&0x0F)<<4)|(databuf[2]&0x0F));

	printk("%s (%d)	: ==huangwenjun==Hrs3300_read_hrs== hrs data = 0x%x\n", __func__, __LINE__, data);  // add by hwj

	return data;
}

uint16_t Hrs3300_read_als(void)
{
	uint8_t  databuf[3];
	uint16_t data;

	databuf[0] = Hrs3300_read_reg(0x08);	// addr09, bit [10:3]
  databuf[1] = Hrs3300_read_reg(0x0d);	// addr0a, bit [17:11]
  databuf[2] = Hrs3300_read_reg(0x0e);	// addr0f, bit [2:0]
	
	data = ((databuf[0]<<3)|((databuf[1]&0x3F)<<11)|(databuf[2]&0x07));
	
	if (data > 32767) data = 32767;  // prevent overflow of other function

	return data;
}



bool Hrs3300_chip_init(void)
{
	int i =0 ;
	uint8_t id =0;
	printk(" hrs3300 init \n");
	for(i = 0; i < INIT_ARRAY_SIZE;i++)
	{
		if ( Hrs3300_write_reg( init_register_array[i][0],
		                init_register_array[i][1]) != 0 )
		{
		goto RTN;
		}
	}	


	//20161117 added by ericy for "low power in no_touch state"		
	if(hrs3300_power_up_flg == 0){
		reg_0x7f=Hrs3300_read_reg(0x7f) ;
		reg_0x80=Hrs3300_read_reg(0x80) ;
		reg_0x81=Hrs3300_read_reg(0x81) ;
		reg_0x82=Hrs3300_read_reg(0x82) ;		
		hrs3300_power_up_flg =  1; 
	}
	//20161117 added by ericy for "low power in no_touch state"

	id = Hrs3300_read_reg(0x00);
	printk("<<< hrs3300 init done id = 0x%x \r\n", id); // 0x21	
	printk("%s (%d)	: ==huangwenjun==Hrs3300_chip_init== init done id = 0x%x\n", __func__, __LINE__, id);  // add by hwj

	return true;
	RTN:
	return false;		
}

void Hrs3300_chip_enable()
{	
	Hrs3300_write_reg( 0x16, 0x78 );
	Hrs3300_write_reg( 0x01, 0xd0 );	
	Hrs3300_write_reg( 0x0c, 0x2e );
	printk("%s (%d)	: ==huangwenjun==Hrs3300_chip_enable== END\n", __func__, __LINE__);  // add by hwj
	return ;	
}

void Hrs3300_chip_disable()
{
	Hrs3300_write_reg( 0x01, 0x08 );
	Hrs3300_write_reg( 0x02, 0x80 );
	Hrs3300_write_reg( 0x0c, 0x4e );
	
	Hrs3300_write_reg( 0x16, 0x88 );
	
	Hrs3300_write_reg( 0x0c, 0x22 );
	Hrs3300_write_reg( 0x01, 0xf0 );
	Hrs3300_write_reg( 0x0c, 0x02 );

	Hrs3300_write_reg( 0x0c, 0x22 );
	Hrs3300_write_reg( 0x01, 0xf0 );
	Hrs3300_write_reg( 0x0c, 0x02 );
	
	Hrs3300_write_reg( 0x0c, 0x22 );
	Hrs3300_write_reg( 0x01, 0xf0 );
	Hrs3300_write_reg( 0x0c, 0x02 );
	
	Hrs3300_write_reg( 0x0c, 0x22 );
	Hrs3300_write_reg( 0x01, 0xf0 );
	Hrs3300_write_reg( 0x0c, 0x02 );
	printk("%s (%d)	: ==huangwenjun==Hrs3300_chip_disable== END\n", __func__, __LINE__);  // add by hwj
	return ;	
}

extern unsigned int hr_date;
extern unsigned int up_date;
extern unsigned int down_date;
void heart_rate_meas_timeout_handler(void * p_context)
{
   // uint32_t        err_code;
   // uint16_t        heart_rate;
	  uint16_t hrm_raw_data;
	uint16_t als_raw_data;
//	  uint8_t gsen_data;	
	  hrs3300_results_t alg_results;
#ifdef BP_CUSTDOWN_ALG_LIB		
	hrs3300_bp_results_t	bp_alg_results ;	
#endif	
	  static uint16_t timer_index =0;
#ifdef GSENSER_DATA
	  AxesRaw_t gsen_buf;
#endif

//    UNUSED_PARAMETER(p_context);
#ifdef GSENSER_DATA
	  LIS3DH_GetAccAxesRaw(&gsen_buf);
#endif
	  hrm_raw_data = Hrs3300_read_hrs();
	  als_raw_data = Hrs3300_read_als();  // 20170430
#ifdef GSENSER_DATA
    Hrs3300_alg_send_data(hrm_raw_data, als_raw_data, gsen_buf.AXIS_X, gsen_buf.AXIS_Y, gsen_buf.AXIS_Z, 0); 
#else
	  Hrs3300_alg_send_data(hrm_raw_data, als_raw_data, 0, 0, 0,0);
#endif
		#if 0
	   alg_results = Hrs3300_alg_get_results();	
	   SEGGER_RTT_printf(0,"%d %d %d %d %d %d %d\n", hrm_raw_data, als_raw_data,gsen_buf.AXIS_X,gsen_buf.AXIS_Y,gsen_buf.AXIS_Z,alg_results.hr_result,alg_results.alg_status);
	   #endif

	  timer_index ++;
    if (timer_index >= 25)  {    // get result per second
			  timer_index =0;
		alg_results = Hrs3300_alg_get_results();

		if (alg_results.alg_status == MSG_NO_TOUCH)
		{
			//opr_display_hr(0);    // customer can print no touch information here
			hr_date = 0;
		}
		else if (alg_results.alg_status == MSG_PPG_LEN_TOO_SHORT)
		{
			//opr_display_wait( ((alg_results.data_cnt/100)%3)+1);  // customer can print waiting information here
			hr_date = 0;
		}
		else
		{
#ifdef BP_CUSTDOWN_ALG_LIB					
        bp_alg_results = Hrs3300_alg_get_bp_results();  
        if (bp_alg_results.sbp!= 0){
			up_date = bp_alg_results.sbp;
			down_date = bp_alg_results.dbp;
           //opr_display_bp(bp_alg_results.sbp, bp_alg_results.dbp);
        }
#endif				

			if (alg_results.object_flg == 1){ 
				// 固体检测处�?
			}
			hr_date = alg_results.hr_result;
			//opr_display_hr(alg_results.hr_result);  // customer can print real heart rate here
		}
	}
}

void blood_presure_meas_timeout_handler(void * p_context)
{
#ifdef  BP_CUSTDOWN_ALG_LIB
	uint16_t hrm_raw_data;
	//uint8_t gsen_data;	
	hrs3300_bp_results_t bp_alg_results;
	static uint16_t timer_index =0;

//    UNUSED_PARAMETER(p_context);
	  hrm_raw_data = Hrs3300_read_hrs();
//	  SEGGER_RTT_printf(0,"------------- %d\n", hrm_raw_data);

	  Hrs3300_bp_alg_send_data(hrm_raw_data);
//		SEGGER_RTT_printf(0,"status is %d\n", alg_status);
	  timer_index ++;
	if (timer_index >= 50)  {    // get result per second
		timer_index =0;
		bp_alg_results = Hrs3300_alg_get_bp_results();
			
		if (bp_alg_results.object_flg == 1){ 
			// 固体检测处�?
		}
		if (bp_alg_results.bp_alg_status == MSG_BP_NO_TOUCH)
		{
			//opr_display_hr(0);    // customer can print no touch information here
		}
	        else if (bp_alg_results.bp_alg_status == MSG_BP_PPG_LEN_TOO_SHORT)
		{
			//opr_display_hr(0);  // customer can print waiting information here
		}	          
		else if (bp_alg_results.bp_alg_status == MSG_BP_ALG_TIMEOUT)
		{
			if (bp_alg_results.sbp != 0)
			{
			}
			else
			{
				//bp_alg_results.sbp = 120 + rand()%8;
				//bp_alg_results.dbp = 68 + rand()%8;
			}
			//opr_display_bp(bp_alg_results.sbp, bp_alg_results.dbp);  // customer can print real heart rate here
		}
		else if (bp_alg_results.bp_alg_status == MSG_BP_READY)
		{
			//opr_display_bp(bp_alg_results.sbp, bp_alg_results.dbp);  // customer can print real heart rate here
		}


	}
#endif
}


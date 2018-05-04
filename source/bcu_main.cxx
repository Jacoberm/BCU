/*
 * Copyright (c) 2012，MIT
 * All right reserved.
 *
 * 文件名称： bcu_main.c
 * 文件标识：
 * 摘    要： 广播控制盒主函数
 *
 * 当前版本： V1.0.0
 * 作    者： wilson
 * 完成日期：2012-08-22
 *
 * 取代版本： V0.0.0
 * 原作者  ： wilson
 * 完成日期：
 */

#include "bcu_main.h"
#include "TouchScreen_horizontal.h"


#define CONFIG_TEST_EAMP_factory


device_id_info_t bcu_device;
#define CYGSEM_HAL_5034_V1_DEVICES_TYPE_BCU

unsigned char wherther_start_auto_ann = 0;	//20160824
unsigned char sim_auto_flag = 0;
send_infomation_t send_to_eamp_for_auto_test;
cyg_handle_t  counter_handle_sim_auto;
cyg_handle_t alarm_handle_sim_auto;
cyg_alarm alarm_object_sim_auto;

//extern cyg_handle_t  counter_handle_sync;
//extern cyg_handle_t alarm_handle_sync;
//extern cyg_alarm alarm_object_sync;

cyg_handle_t  counter_handle_sync;
cyg_handle_t alarm_handle_sync;
cyg_alarm alarm_object_sync;

void alarm_func_handle_sync(cyg_handle_t counter_handle, cyg_addrword_t data)
{
	wherther_send_sync_signal = 1;
	sync_timer_flag++;
}

void CreateSYNCTimer()
{///<创建同步定时器
	cyg_clock_to_counter(cyg_real_time_clock(),&counter_handle_sync);
	cyg_alarm_create(counter_handle_sync,alarm_func_handle_sync,0,&alarm_handle_sync,&alarm_object_sync);
	cyg_alarm_initialize(alarm_handle_sync,cyg_current_time()+400,400);
	cyg_alarm_disable(alarm_handle_sync);
	diag_printf("CreateSYNCTimer\n");
}
void RestartSYNCTimer()
{///<重启同步定时器
	cyg_alarm_initialize(alarm_handle_sync,cyg_current_time()+400,400);
	cyg_alarm_enable(alarm_handle_sync);
	diag_printf("RestartSYNCTimer\n");
}
void CloseSYNCTimer()
{///<关闭同步定时器
	cyg_alarm_disable(alarm_handle_sync);
	diag_printf("CloseSYNCTimer\n");
}

void alarm_func_handle_sim_auto(cyg_handle_t counter_handle, cyg_addrword_t data)
{
	wherther_start_auto_ann = 1;
}

void CreateAUTOSIMTimer()
{
	cyg_clock_to_counter(cyg_real_time_clock(),&counter_handle_sim_auto);
	cyg_alarm_create(counter_handle_sim_auto,alarm_func_handle_sim_auto,0,&alarm_handle_sim_auto,&alarm_object_sim_auto);
	cyg_alarm_initialize(alarm_handle_sim_auto,cyg_current_time()+5000,5000); 
	cyg_alarm_disable(alarm_handle_sim_auto);
	diag_printf("CreateAUTOSIMTimer\n");
}
void RestartAUTOSIMTimer()
{
	cyg_alarm_initialize(alarm_handle_sim_auto,cyg_current_time()+5000,5000);
	cyg_alarm_enable(alarm_handle_sim_auto);
	diag_printf("RestartAUTOSIMTimer\n");
}
void CloseAUTOSIMTimer()
{
	cyg_alarm_disable(alarm_handle_sim_auto);
	diag_printf("CloseAUTOSIMTimer\n");
}


void SendAtoTestLineSet(send_infomation_t *line_set_send_package)
{
//	diag_printf("####### run into SendAtoTestLineSet function !!!\n");
	strcpy(line_set_send_package->src_devices_name,bcu_state.bcu_info.devices_name);
	line_set_send_package->src_devices_no = bcu_state.bcu_info.devices_no;
	line_set_send_package->event_type_ann = LINE_NUMBER_CHANGE_EVENT;
	line_set_send_package->event_type_intercom = INTERCOM_IDLE;
	line_set_send_package->event_info_ann.line_info.line_no = 4;
	line_set_send_package->event_info_ann.line_info.region_no = 1;
	line_set_send_package->event_info_ann.line_info.get_or_set = 0;
	line_set_send_package->event_info_ann.line_info.start_station_no = 1;
	line_set_send_package->event_info_ann.line_info.up_or_down = 0;
	line_set_send_package->event_info_ann.line_info.terminal_station_no = 30;
	line_set_send_package->event_info_ann.line_info.whether_the_cycle = 0;
	SendCmd(&line_set_send_package,"EAMP",MUL_DST_NO);
}


void StartSimlateAnn()
{
	//if(sim_auto_flag == 1)
	if(1)
	{
//		diag_printf("zhw &&&&& run into StartSimulateAnn function!!!\n");
		SendAtoTestLineSet(&send_to_eamp_for_auto_test);
		RestartAUTOSIMTimer();
	}
	else
	{
		//SendAtoEnd(&send_to_eamp_for_auto_test);
		//CloseAUTOSIMTimer();
	}
}

void SimlateAtuoANNInofFill(send_infomation_t *param_send_package)
{
	static unsigned char record_pre_status = 0;
	static unsigned char door_side = 1;
	static unsigned char up_down = 1;
	static unsigned char stop_or_leave = 0;
	static unsigned char start_station_code = 1;
	static unsigned char end_station_code = 15;
	static unsigned char current_station_code = 1;
	static unsigned char next_station_code = 2;
	diag_printf("zhw %%%%%% ---- @@@@@@@@@@@@@\n");
	diag_printf("door_side = %d,up_down = %d,stop_or_leave = %d,start = %d,end = %d,current = %d--next = %d",
			door_side,up_down,stop_or_leave,start_station_code,end_station_code,current_station_code,next_station_code);

	strcpy(param_send_package->src_devices_name,bcu_state.bcu_info.devices_name);
	param_send_package->src_devices_no = bcu_state.bcu_info.devices_no;
	param_send_package->event_type_ann = TMS_AUTO_ANN_EVENT;
	param_send_package->event_type_intercom = INTERCOM_IDLE;
	param_send_package->event_info_ann.tms_auto_announce.tms_auto_begin_or_over = 1;
	param_send_package->event_info_ann.tms_auto_announce.tms_auto_active = 1;
	param_send_package->event_info_ann.tms_auto_announce.tms_auto_response = 0;
	param_send_package->event_info_ann.tms_auto_announce.line_number = 3;
	param_send_package->event_info_ann.tms_auto_announce.tms_auto_pre_read = 0;

	param_send_package->event_info_ann.tms_auto_announce.door_side = door_side;
	param_send_package->event_info_ann.tms_auto_announce.up_down = up_down;
	param_send_package->event_info_ann.tms_auto_announce.stop_or_leave = stop_or_leave;
	param_send_package->event_info_ann.tms_auto_announce.start_station_code  = start_station_code;
	param_send_package->event_info_ann.tms_auto_announce.end_station_code = end_station_code;
	param_send_package->event_info_ann.tms_auto_announce.current_station_code = current_station_code;
	param_send_package->event_info_ann.tms_auto_announce.next_station_code = next_station_code;

	if(stop_or_leave == 0)
	{
		record_pre_status = 1;
	}
	else
	{
		record_pre_status = 2;
	}

	RestartSYNCTimer();///<重置同步定时器

	SendCmd(&param_send_package,"EAMP",MUL_DST_NO);
	
	//ANNStateHangle(&recv_cmd_info_of_ann,param_send_package);/*Handle announcement event*/

	if(current_station_code >= 15 && current_station_code > 0)
	{
	up_down = 1;
	door_side = 1;
	start_station_code = 1;
	end_station_code = 15;
	current_station_code = 1;
	next_station_code = 2;
	stop_or_leave = 0;
	}
	else
	{
	up_down = 1;
	door_side = 1;
	start_station_code = 1;
	end_station_code = 15;
	if(record_pre_status == 1)
	{
	stop_or_leave = 1;
	} //pre ann
	else
	{
	stop_or_leave = 0;
	current_station_code++;
	next_station_code++;
	}
	}
}



extern char pcu_alarm_audio[];
extern int size_pcu_audio_buffer;
extern required_station_info_t station_info_from_network;

static cyg_uint8  audio_rx_0525[1024 * 8] = { 0 } ;
static char tmp_buff[3*1024];
struct global_info bcu_devices_info_IP; 
unsigned char idle_pcu_req_count; ///< add,0616

void SetAutoAnnWhenIntercomIdle(void)
{
   extern unsigned char pc_state_flag;
   static unsigned char last_auto_set_flag = 0;
   if(bcu_state.bcu_active_intercom_state->state_id == INTERCOM_IDLE &&
   	  bcu_state.bcu_active_ann_state->state_id == ANN_IDLE )
   {
      if( pc_state_flag > 2 && last_auto_set_flag==0 )
      {
         last_auto_set_flag = 1;
	  }
      else if( pc_state_flag > 2 && last_auto_set_flag==1 )
      {
         last_auto_set_flag = 0;
	  }
	  else
	  {
         //diag_printf("<0827>: unsupported evt\n!!");
	  }
	  if( last_auto_set_flag == 1)
	  	atuo_set_flag = 1;
	  else
	  	atuo_set_flag = 0;
   }
   
   return ;
}

int main_testGPIO(int argc, char **argv)
{
    diag_printf("551-main: test PA button LED\n");
    int i = 0;
    for(;;) {
      BCU_LED_BUTTON1_ON;
      for(i=0; i<3000000; i++);
      BCU_LED_BUTTON1_DIS;
     for(i=0; i<3000000; i++);

     BCU_LED_BUTTON2_ON;
      for(i=0; i<3000000; i++);
      BCU_LED_BUTTON2_DIS;
     for(i=0; i<3000000; i++);

     BCU_LED_BUTTON3_ON;
      for(i=0; i<3000000; i++);
      BCU_LED_BUTTON3_DIS;
     for(i=0; i<3000000; i++);

     BCU_LED_BUTTON4_ON;
      for(i=0; i<3000000; i++);
      BCU_LED_BUTTON4_DIS;
     for(i=0; i<3000000; i++);
    }

    return 0;
}

int PlayAudioTwice_0525(cyg_io_handle_t audio_handle, int audio_source_buffer_id, int pending_audio_buffer_id)
{
	unsigned int current_total_bytes = 0;
	unsigned int main_current_total_bytes = 0;
	unsigned int pending_current_total_bytes = 0;
	unsigned int right_bytes = 0;
	cyg_uint32 snd_set_local;
	int ret_bwrite = 0;
	cyg_uint32 len = 0, len_backup = 0, left_len = 0;
	int ret = 0;
	//char tmp_buff[3*1024];
	int tmp_read_len = 0;
	int index = 0;
	int total_read_len = 0;
	unsigned char  buffer_clear_flag  = CharBufferCheckClearFlag(audio_source_buffer_id);
	unsigned char if_read_main_buffer = 0;
	int border_of_audia_data_size = 2234;
	CharBufferCurrentReadPointer_temp(pending_audio_buffer_id, &pending_current_total_bytes, &right_bytes);
	CharBufferCurrentReadPointer_temp(audio_source_buffer_id, &main_current_total_bytes, &right_bytes);
	current_total_bytes = main_current_total_bytes + pending_current_total_bytes;
	debug_print(("main_current_total_bytes = %d\n",main_current_total_bytes));
	debug_print(("pending_current_total_bytes = %d\n",pending_current_total_bytes));
	//if( current_total_bytes <=  border_of_audia_data_size )
	if( current_total_bytes <=  1024*3+64 )		
	{
		return -5;
	}

	if( buffer_clear_flag != CharBufferCheckClearFlag(audio_source_buffer_id) )
	{
		 debug_print(("PlayAudioTwice-0525: before bwrite, buffer cleared: %d,so return -1\n",current_total_bytes));
		 return -1;
	}
	if( current_total_bytes > border_of_audia_data_size )
	{
	    ret = 0;
		ret = CharBufferRead(pending_audio_buffer_id, tmp_buff, sizeof(tmp_buff) );
		if( ret >0 )
		{
			tmp_read_len = sizeof(tmp_buff) - ret;
			index = ret;
			total_read_len += ret;
		   if( total_read_len < sizeof(tmp_buff) )
		   {
				 if_read_main_buffer = 1;
		   }
		}
		else if( -2 == ret ) ///< pending is empty
		{
			tmp_read_len = sizeof(tmp_buff);
			index = 0;
			total_read_len = 0;
			if_read_main_buffer = 1;
		}
		else
		{
		   debug_print(("PlayAudioTwice-0525:  read pending=%d, This is unlikely\n", ret));
		   return -6; ///< error
		}
		if( if_read_main_buffer==1)
		{
			ret = 0;
			ret = CharBufferRead(audio_source_buffer_id, &tmp_buff[index], tmp_read_len );
			if( ret > 0 )
			{
				total_read_len += ret;
			}
		}
		if( total_read_len < border_of_audia_data_size)
		{
			  debug_print(("PlayAudioTwice-0525:  total(%d) < 2KiB, This is unlikely\n", total_read_len));
			  return -6;
		}
		if( buffer_clear_flag != CharBufferCheckClearFlag(audio_source_buffer_id) )///<2013-11-11
		{
			 debug_print(("PlayAudioTwice-0525: before bwrite, buffer cleared-02\n"));
			 return -1;
		}

		len = (cyg_uint32)total_read_len;
		len_backup = len;
		snd_set_local = (1<<8) | CYG_SND_AUDIO_STREAM;
		debug_print(("PlayAudioTwice-0525:  Before cyg_io_bwrite,%d \n",len));
		ret_bwrite = cyg_io_bwrite( audio_handle, (cyg_uint8 *)tmp_buff, &len, snd_set_local);
		debug_print(("PlayAudioTwice-0525: after cyg_io_bwrite \n"));
		if( buffer_clear_flag != CharBufferCheckClearFlag(audio_source_buffer_id) ) //add, 2013
		{
			debug_print(("PlayAudioTwice-0525: after bwrite, buffer cleared,so return -1\n"));
			 return -1;
		}
		debug_print(("PlayAudioTwice-0525: bw=%d, total=%d, left=%d, dec=%d \n", 
                       ret_bwrite, current_total_bytes, len,  current_total_bytes-len ));

		if( len > len_backup )
		{
			left_len = 0;
		}
		else if ( len >= 0 )
		{
			left_len = len;
		}
		if( left_len > 0 ) ///< write into pending buffer
		{
		   CharBufferWrite(pending_audio_buffer_id, &tmp_buff[total_read_len-left_len], left_len );
		}
		ret = 0; ///< write 2KiB,ok
	}
	else
	{
		ret = -2;
	}
	return ret;
}

void PutMicSampleDataintoBuffer_0525(cyg_io_handle_t audio_handle, int buffer_id)
{
	int wr_ret = 0;
	int button_state = 0;
	//button_state = (read_button_snd())? (0):(1);
	button_state = 1;

	//cyg_uint8  audio_rx[1024 * 8] = { 0 } ;
	cyg_uint8  *audio_tx = audio_rx_0525;
	cyg_uint32 len = 100,len1  = 100;
	cyg_uint32 len_origin = 0;
	cyg_uint32 snd_set ;
	Cyg_ErrNo stat_rw;

	len = sizeof (snd_set) ;
	snd_set = (1<<8) | CYG_SND_AUDIO_STREAM;
	stat_rw = cyg_io_bread(bcu_audio_handle, (void *)audio_tx, &len, snd_set);
	len1 = len;
	if( button_state == 1)
	{
		if (stat_rw == -EIO ||stat_rw<0  )
		{
			len = 0 ;
		        debug_print(("<0525>,error, stat_rw = %d\n",stat_rw));
		}

		audio_tx = audio_rx_0525 ;
		len_origin = len;

		if(len > 0)
		{
			if(1)
			{
				wr_ret = CharBufferWrite(buffer_id, audio_tx, len);
			}
		}
		debug_print(("<0525>len = %d,wr_ret = %d\n",len,wr_ret));
	}
	else
	{
		len = 0 ;
		audio_tx = audio_rx_0525;
	}
	return ;
}

#define SCIF_BAUDRATE CYGNUM_SERIAL_BAUD_38400

#define STACK_SIZE 0x1000

static char stack[STACK_SIZE];
static cyg_thread thread_data;
static cyg_handle_t thread_handle;

int main(int argc, char **argv)
{
	int return_value_of_thread_create = 0;/*the return of thread create*/
#ifdef CMDTOCMU //add by zhouwei
	void *return_value_of_join_thread[9];/*the return of thread wait*/
#else
	void *return_value_of_join_thread[7];/*the return of thread wait*/
#endif

	int return_init_buffer = 0;/*the value of init buffer*/

	/*the declaration of thread attributions*/
	pthread_attr_t attr_of_screen;/*the attribution of touch sreen thread*/
	pthread_attr_t attr_of_network;/*the attribution of network thread*/
	pthread_attr_t attr_of_control;/*the attribution of control thread*/
	pthread_attr_t attr_of_sample_and_play;/*the attribution of mic sample and audio play thread*/
	pthread_attr_t attr_of_failure_statics;/*the attribution of failure statics thread*/
	pthread_attr_t attr_of_gd_sync;/*the attribution of global device sync thread*/
	pthread_attr_t attr_of_demao_thread;
	pthread_attr_t attr_of_tftp_queue;/*the attribution of the tftp thread-20150109*/
	pthread_attr_t attr_of_serial;
#ifdef CMDTOCMU
	cyg_flag_init(&bcu_cmd_to_cmu);
	pthread_attr_t attr_of_bcu_to_cmu; //add by zhouwei 2014-4-17
#endif

	/*设备软件版本号登记*/
	strncpy(bcu_state.pa_software_version.software_version,"683-PA-BCU-app-1-20180504",30);
	strncpy(bcu_state.pa_software_version.software_date,"v1.5.0",30);
	strncpy(bcu_state.pa_software_version.db_version_audio,"NULL",30);
	strncpy(bcu_state.pa_software_version.db_version_config,"NULL",30);

	diag_printf("Just for test\n");
	diag_printf("%s\n",bcu_state.pa_software_version.software_version);
	diag_printf("%s\n",bcu_state.pa_software_version.software_date);
	diag_printf("ann:%s\n",bcu_state.pa_software_version.db_version_audio);
	diag_printf("config:%s\n",bcu_state.pa_software_version.db_version_config);

   /*global device initialize for zhouwei*/
   InitGlobalArray();

   /*Initialize the bcu_state*/
   InitBCUState();

	/*Acquire this device number*/
	GetCurrentDeviceInfomation(&bcu_device);

	/*Update current device information*/
	UpdateCurrentDeviceInformation(&bcu_state.bcu_info,bcu_device);

	/*根据指定格式设置本设备信息*/
	SetThisDevInfo(bcu_state.bcu_info.devices_name,bcu_state.bcu_info.devices_no);

	/*Acquire this device number*/
	GetOtherDeviceNo(bcu_device);

	/*初始化设备数据库版本号*/
    BcuReadDeviceVersionTableItemInit(); ///< add, 1010

	/*Initial the mutex*/
	InitMutex();

	/*通过串口打印本设备信息*/
	diag_printf("%s%d-%s%d\n",bcu_state.bcu_info.devices_name,bcu_state.bcu_info.devices_no,bcu_state.bcu_info.devices_name,bcu_state.opposite_bcu_no);

	/*Initialize the Buffer*/
	return_init_buffer = InitBuffer();
	if(return_init_buffer == -1)
	{
		debug_print(("Create buffer error!\n"));
		pthread_exit(NULL);
	}

	/*Initialize the semaphore*/
	InitSemaphore(&sem_wakeup_bcu_mic_sample);
	InitSemaphore(&sem_demao);

#ifdef CONFIG_TEST_SND_IN_MULTI_THREAD
	InitSemaphore(&sem_snd_card_self_test);
#endif

	/*init the info which is send by bcu to ccu*/
	InitBCULogInfo(&bcu_send_log_info_to_ccu);

	/*initialize the share variable,which is used to identify the audio data's destination*/
	InitAudioDataDestination();

	/*Intialize the station information*/
	InitStationInformation();

	/*系统上电之后将音频输入切换为PTT  0：PTT   1：外接3D5接口*/
	bcu_audio_in_ctrl(0);

	/*设置系统默认音量*/
	bcu_state.device_volume.d2d_volume = DEFAULT_INTERCOMM_VOLUME;
	bcu_6d5w_ctrl_wilson(bcu_state.device_volume.d2d_volume);

	/*Lookup the sound card,which is used on bcu*/
	LookupSndCard(&bcu_audio_handle);

	/*Open the sound card,the open mode is CYG_SND_PLAYBACK_MODE*/
	//OpenSndCard(&bcu_audio_handle,"codec");
	int optime = 0;        
	for(optime=0; optime<10; optime++)         
	{	   
		if( OpenSndCard(&bcu_audio_handle,"codec") < 0 ) 
			{              cyg_thread_delay(10);           } 
		else {              break;           }        
	}

	/*声卡打开之后才能调节声卡的线性音量*/
	AdjustVolumeAfterCODEC();

	/*Initial all device information*/
	InitAllDeviceInfo();

	/*Set the package of PTT state*/
	SetPTTStateInfoPackage();

	/*初始化数据库版本信息-目前没有使用*/
	InitDBVersionInfo();

	AdjustVolumeByGPIO(1);

	bcu_state.current_d2d_running_mode = ANALOG_MODE;
	bcu_state.current_pa_running_mode = ANALOG_MODE;

	//配置线程信息，优先级、调度方式、堆栈大小、继承方式等。
	//入口参数：线程名，优先级大小，堆栈基址，堆栈大小

	hal_set_pin_function( CYGHWR_HAL_KINETIS_PIN(F, 25, 1, KINETIS_PIN_PULLUP) );
	hal_gpio_pin_ddr_out( CYGHWR_HAL_KINETIS_PIN(F, 25, 1, KINETIS_PIN_PULLUP) );

	hal_set_pin_function( CYGHWR_HAL_KINETIS_PIN(A, 9, 1, KINETIS_PIN_PULLUP) );
	hal_gpio_pin_ddr_out( CYGHWR_HAL_KINETIS_PIN(A, 9, 1, KINETIS_PIN_PULLUP) );

	hal_set_pin_function( CYGHWR_HAL_KINETIS_PIN(A, 10, 1, KINETIS_PIN_PULLUP) );
	hal_gpio_pin_ddr_out( CYGHWR_HAL_KINETIS_PIN(A, 10, 1, KINETIS_PIN_PULLUP) );

	hal_set_pin_function( CYGHWR_HAL_KINETIS_PIN(D, 11, 1, KINETIS_PIN_PULLUP) ); //tc speaker
	hal_gpio_pin_ddr_out( CYGHWR_HAL_KINETIS_PIN(D, 11, 1, KINETIS_PIN_PULLUP) );
	hal_gpio_pin_clear( CYGHWR_HAL_KINETIS_PIN(D, 11, 1, KINETIS_PIN_PULLUP) );  

	hal_set_pin_function( CYGHWR_HAL_KINETIS_PIN(E, 4, 1, KINETIS_PIN_PULLUP) );
	hal_gpio_pin_ddr_out( CYGHWR_HAL_KINETIS_PIN(E, 4, 1, KINETIS_PIN_PULLUP) );

    BCU_LED_BUTTON1_DIS;
    BCU_LED_BUTTON2_DIS;	
    BCU_LED_BUTTON3_DIS;
    diag_printf("[1117-1200]: after button\n");

    ///< init pp_station_info,2016
	bcu_state.pp_station_info = station;
	bcu_state.bcu_region_info[0].total_regions_number = 32+4;
	bcu_state.bcu_region_info[0].p_regin_info =
			(region_station_t *)malloc((32+4) * sizeof(region_station_t));
	bcu_state.bcu_region_info[1].total_regions_number = 32+4;
	bcu_state.bcu_region_info[1].p_regin_info =
			(region_station_t *)malloc((32+4) * sizeof(region_station_t));

	/*空白数据，口播按钮松开之后补发，从而及时将采集的口播数据发送给EAMP并且播放出来*/
	blank_audio_data = (char *)malloc((1024 * 1) * sizeof(char));
	memset((void *)blank_audio_data,0xff,(1024 *1));

#ifdef CMDTOCMU
	ThreadAttributionConfigure(&attr_of_bcu_to_cmu,BCU_PRIORIT,
						  (thread_stack_of_bcu_to_cmu + (1024 * 3)),(1024 * 3));
#endif

	ThreadAttributionConfigure(&attr_of_screen,(BCU_PRIORIT),///<触摸屏线程属性值初始化
								thread_stack_of_screen+(1024 * 20),(1024 * 25));

	ThreadAttributionConfigure(&attr_of_network,BCU_PRIORIT,///<线程属性值初始化
						  (thread_stack_of_network + (1024 * 16)),(1024 * 25));
	pthread_attr_setschedpolicy(&attr_of_network,SCHED_FIFO);

	ThreadAttributionConfigure(&attr_of_control,BCU_PRIORIT,///<控制线程属性值初始化
									 (thread_stack_of_control + (1024 * 24)),(1024 * 24));

	diag_printf("%x-%x\n",thread_stack_of_control, (thread_stack_of_control + (1024 * 24)));
	ThreadAttributionConfigure(&attr_of_sample_and_play,BCU_HIGH_MIC_SAMPLE_PRIORITY,///<音频采集与播放线程属性值初始化
									 (thread_stack_of_sample_amd_play + (1024 * 20)),(1024 * 20));

	ThreadAttributionConfigure(&attr_of_failure_statics,BCU_PRIORIT,///<故障统计线程属性值初始化
									 (thread_stack_of_failure_statics + (1024 * 3)),(1024 * 3));

	ThreadAttributionConfigure(&attr_of_gd_sync,BCU_PRIORIT,///<同步线程属性值初始化
									 (thread_stack_of_gd_sync + (1024 * 3)),(1024 * 3));

	ThreadAttributionConfigure(&attr_of_demao_thread,BCU_HIGH_MIC_SAMPLE_PRIORITY+2,///<
									 (thread_stack_of_dameo + (1024 * 3)),(1024 * 3));

	ThreadAttributionConfigure(&attr_of_tftp_queue, BCU_PRIORIT,
							                    (thread_stack_of_tftp_queue + (1024 * 3)),(1024 * 3));

	ThreadAttributionConfigure(&attr_of_serial, BCU_PRIORIT,
						                    (thread_stack_of_serial + (1024 * 4)),(1024 * 4));


	return_value_of_thread_create = pthread_create(&thread_of_demao,
														&attr_of_demao_thread,
														DemaoEntry,
														NULL
														);

	/*Create the thread of touch screen*/
	return_value_of_thread_create = pthread_create(&thread_of_screen,
														&attr_of_screen,
														TouchScreenEntry,
														NULL
														);
	 CYG_TEST_CHECK(return_value_of_thread_create == 0, "touch screen thread returned error");
	 debug_print(("I am the touch screen thread!\n"));

//	 /*Create the thread of network*/
	return_value_of_thread_create = pthread_create(&thread_of_network,
														&attr_of_network,
														NetworkHandleEntry,
														NULL
														);
	 CYG_TEST_CHECK(return_value_of_thread_create == 0,"network thread returned error");
	 debug_print(("I am the network thread\n"));

	 /*Create the thread of control*/
	 return_value_of_thread_create = pthread_create(&thread_of_control,
														 &attr_of_control,
														 SystemControl,
														 NULL
														 );
	 CYG_TEST_CHECK(return_value_of_thread_create == 0,"control thread returned error");
	 debug_print(("I am the control thread!\n"));

//	 /*Create the thread of sample and play*/
	 return_value_of_thread_create = pthread_create(&thread_of_sample_and_play,
														 &attr_of_sample_and_play,
														 BcuMicSampleAndPlayEntry,
														 NULL
														 );
	 CYG_TEST_CHECK(return_value_of_thread_create == 0,"mic get thread returned error");
	 debug_print(("I am the sample and play thread!\n"));

	 /*Create the thread of failure statics*/
	 return_value_of_thread_create = pthread_create(&thread_of_failure_statics,
														 &attr_of_failure_statics,
														 BcuFailureStaticsEntry,
														 NULL
														 );
	 CYG_TEST_CHECK(return_value_of_thread_create == 0,"failure statics thread returned error");
	 debug_print(("I am failure statics thread!\n"));

	 /*Create the thread of gd sync*/
	 return_value_of_thread_create = pthread_create(&thread_of_gd_sync,
														 &attr_of_gd_sync,
														 BcuGdThreadEntry,
														 NULL
														 );
	 CYG_TEST_CHECK(return_value_of_thread_create == 0,"gd sync thread returned error");
	 debug_print(("I am gd sync thread!\n"));

#ifdef CMDTOCMU
return_value_of_thread_create = pthread_create(&thread_of_bcu_to_cmu,
													 &attr_of_bcu_to_cmu,
													 SendCmdToCmuEntry,
													 NULL
													 );
CYG_TEST_CHECK(return_value_of_thread_create == 0,"bcu send cmd to cmu thread returned error");
debug_print(("I am bcu_to_cmu thread!\n"));
#endif

	/*Create the thread of tftp thread*/
	 return_value_of_thread_create = pthread_create(&thread_of_tftp_queue,
														 &attr_of_tftp_queue,
														 BCUTftpThreadEntry,
														 NULL
														 );


	 return_value_of_thread_create = pthread_create(&thread_of_serial,
													 &attr_of_tftp_queue,
													 BCUSerialThreadEntry,
													 NULL
													 );

	 /*Wait the end of thread*/
//	 pthread_join(thread_of_screen, &return_value_of_join_thread[0] );
	 pthread_join(thread_of_network, &return_value_of_join_thread[1] );
	 pthread_join(thread_of_control, &return_value_of_join_thread[2] );
	 pthread_join(thread_of_sample_and_play, &return_value_of_join_thread[3] );
	 pthread_join(thread_of_failure_statics, &return_value_of_join_thread[4] );
	 pthread_join(thread_of_gd_sync, &return_value_of_join_thread[5] );
	 pthread_join(thread_of_screen, &return_value_of_join_thread[0] );
	 pthread_join(thread_of_demao, &return_value_of_join_thread[7] );
#ifdef CMDTOCMU
	 pthread_join(thread_of_bcu_to_cmu, &return_value_of_join_thread[6] );
	 pthread_join(thread_of_tftp_queue, &return_value_of_join_thread[8]);
	 pthread_join(thread_of_serial, &return_value_of_join_thread[9]);
#endif
	 debug_print(("I am main function, but I have finished my work now.\n"));

	return 0;
}


void *DemaoEntry(void *arg)
{
	diag_printf("####\n");
	for(;;)
	{
		sem_wait(&sem_demao);
		PthreadPriorityChangeForSchedRr(thread_of_sample_and_play, BCU_PRIORIT-1);
		diag_printf("!!!-SND Error-No return-2-%d\n",sample_flag);
	}
	return NULL;
}

/*the handle function of touch screen thread*/
void *TouchScreenEntry(void *arg)
{
	debug_print(("I am success entry touch screen handle thread!\n"));

	int screen_send_cmd_to_control,ts_dev_vol_info_buffer_id;
	/*触摸屏发送给控制线程的事件命令包*/
	screen_send_cmd_to_control = BlockBufferOpen("bcu-screen-cmd-tx");

	/*触摸屏发送给控制线程的设备音量信息包*/
	ts_dev_vol_info_buffer_id = BlockBufferOpen("bcu-control-recv-ts-info");

	TouchScreenMain(screen_send_cmd_to_control,ts_dev_vol_info_buffer_id);
	return NULL;
}
void *SendCmdToCmuEntry(void *arg)
{
	int cmd_send_buf = 0 ;
	cmd_send_buf = BlockBufferOpen("bcu-network-cmd-tx");
	ZhwBcuToCmuEntry(cmd_send_buf);
	return NULL;
}

//网络线程处理函数
void *NetworkHandleEntry(void *arg)
{
	 debug_print(("I am success entry network handle thread!\n"));

	 network_buffer_t network_buffer_info;
	 /*neteork send cmd information buffer*/
	 network_buffer_info.udp_cmd_socket_buffer.udp_cmd_socket_send_buffer = BlockBufferOpen("bcu-network-cmd-tx");
	 /*neteork receive cmd information buffer*/
	 network_buffer_info.udp_cmd_socket_buffer.udp_cmd_socket_recv_buffer = BlockBufferOpen("bcu-network-cmd-rx");

	 /*neteork send audio data buffer*/
	 network_buffer_info.udp_data_socket_buffer.udp_data_socket_send_buffer = CharBufferOpen("bcu-network-audio-tx");
	 /*neteork receive audio data buffer*/
	 network_buffer_info.udp_data_socket_buffer.udp_data_socket_recv_buffer = CharBufferOpen("bcu-network-audio-rx");

	 /*neteork send common cmd information buffer*/
	 network_buffer_info.udp_common_socket_buffer.udp_common_socket_send_buffer = BlockBufferOpen("network-udp-common-send");
	 /*neteork receive common cmd information buffer*/
	 network_buffer_info.udp_common_socket_buffer.udp_common_socket_recv_buffer = BlockBufferOpen("network-udp-common-recv");

	 /*neteork send common cmd information buffer*/
	 network_buffer_info.udp_common_socket_big_buffer.udp_common_socket_send_big_buffer = BlockBufferOpen("network-udp-common-big-send");
	 /*neteork receive common cmd information buffer*/
	 network_buffer_info.udp_common_socket_big_buffer.udp_common_socket_recv_big_buffer = BlockBufferOpen("network-udp-common-big-recv");

	 network_buffer_info.udp_heart_socket_buffer.udp_heart_socket_recv_buffer = BlockBufferOpen("network-udp-heart-recv");
	 network_buffer_info.udp_heart_socket_buffer.udp_heart_socket_send_buffer = BlockBufferOpen("network-udp-heart-send");

	 network_buffer_info.udp_shell_socket_buffer.udp_shell_socket_recv_buffer = BlockBufferOpen("network-udp-shell-recv");
	 network_buffer_info.udp_shell_socket_buffer.udp_shell_socket_send_buffer = BlockBufferOpen("network-udp-shell-send");
	 	
	 /*网络线程处理函数*/
	 NetWorkMain(network_buffer_info);
	 return 0;
}


#ifdef CONFIG_TEST_EAMP_factory
unsigned char enable_manual_auto_test = 0;
unsigned char  cycle_manual_ann_start = 0;
unsigned char station_no = 1;
cyg_handle_t  counter_handle_cycle_manual_ann;
cyg_handle_t alarm_handle_cycle_manual_ann;
cyg_alarm alarm_object_cycle_manual_ann;
void alarm_func_cycle_manual_ann(cyg_handle_t counter_handle, cyg_addrword_t data)
{
	cycle_manual_ann_start  = 1;
}
void cycle_manual_ann_timer(void)
{
	cyg_clock_to_counter(cyg_real_time_clock(), &counter_handle_cycle_manual_ann);
	cyg_alarm_create(counter_handle_cycle_manual_ann, alarm_func_cycle_manual_ann, 0,
		  &alarm_handle_cycle_manual_ann, &alarm_object_cycle_manual_ann);
	cyg_alarm_initialize(alarm_handle_cycle_manual_ann, cyg_current_time()+3000, 3000); ///< 3000*10ms = 30s
	cyg_alarm_enable(alarm_handle_cycle_manual_ann);
}
void cycle_manual_ann_test(int net_cmd_recv_buffer_id)
{
       int ret = -1;
	network_send_package_t tms_to_eamp_package = {"\0",0};

	memset(&tms_to_eamp_package, 0, sizeof(tms_to_eamp_package));

#if 0 ///< tms auto
	strcpy(tms_to_eamp_package.dst_devices_name, "EAMP");
	tms_to_eamp_package.dst_devices_no = MUL_DST_NO;
	tms_to_eamp_package.send_information.src_devices_no = 1;
	strcpy(tms_to_eamp_package.send_information.src_devices_name,"CCU");
	tms_to_eamp_package.send_information.event_type_ann = TMS_AUTO_ANN_EVENT;
	tms_to_eamp_package.send_information.event_type_intercom = INTERCOM_IDLE;

	tms_to_eamp_package.send_information.event_info_ann.tms_auto_announce.line_number = 3;
	if( station_no >= 17 )
		station_no = 1;
	tms_to_eamp_package.send_information.event_info_ann.tms_auto_announce.current_station_code = station_no++;
	tms_to_eamp_package.send_information.event_info_ann.tms_auto_announce.tms_auto_pre_read = 0;
	tms_to_eamp_package.send_information.event_info_ann.tms_auto_announce.stop_or_leave = 1;
	tms_to_eamp_package.send_information.event_info_ann.tms_auto_announce.tms_auto_active = 1;
	tms_to_eamp_package.send_information.event_info_ann.tms_auto_announce.tms_auto_begin_or_over = 1;
	tms_to_eamp_package.send_information.event_info_ann.tms_auto_announce.door_side = 1;
	tms_to_eamp_package.send_information.event_info_ann.tms_auto_announce.up_down = 1;
	tms_to_eamp_package.send_information.event_info_ann.tms_auto_announce.start_station_code = 1;
       tms_to_eamp_package.send_information.event_info_ann.tms_auto_announce.end_station_code = 17;
#else
	strcpy(tms_to_eamp_package.dst_devices_name, "EAMP");
	tms_to_eamp_package.dst_devices_no = MUL_DST_NO;
	tms_to_eamp_package.send_information.src_devices_no = 1;
	strcpy(tms_to_eamp_package.send_information.src_devices_name,"BCU");
	tms_to_eamp_package.send_information.event_type_ann = MANUAL_ANN_EVENT;
	tms_to_eamp_package.send_information.event_type_intercom = INTERCOM_IDLE;

	tms_to_eamp_package.send_information.event_info_ann.manual_annnounce.manual_active = 1;
	tms_to_eamp_package.send_information.event_info_ann.manual_annnounce.manual_pre_read = 0;
	tms_to_eamp_package.send_information.event_info_ann.manual_annnounce.manual_begin_or_over = 1;
	tms_to_eamp_package.send_information.event_info_ann.manual_annnounce.manual_response = 0;
	tms_to_eamp_package.send_information.event_info_ann.manual_annnounce.line_number = 3;
	tms_to_eamp_package.send_information.event_info_ann.manual_annnounce.up_down = 1;
	tms_to_eamp_package.send_information.event_info_ann.manual_annnounce.stop_or_leave = 1;
	tms_to_eamp_package.send_information.event_info_ann.manual_annnounce.start_station_code = 1;
	if( station_no >= 13 )
		station_no = 1;
	tms_to_eamp_package.send_information.event_info_ann.manual_annnounce.current_station_code = station_no++;
	tms_to_eamp_package.send_information.event_info_ann.manual_annnounce.next_station_code = station_no+1;
	tms_to_eamp_package.send_information.event_info_ann.manual_annnounce.end_station_code = 13;
	tms_to_eamp_package.send_information.event_info_ann.manual_annnounce.door_side = 1;
	tms_to_eamp_package.send_information.event_info_ann.manual_annnounce.key_side = 0;
#endif


	BlockBufferWrite(net_cmd_recv_buffer_id, &tms_to_eamp_package, sizeof(tms_to_eamp_package));

	return ;
}
#endif

unsigned char  cycle_analog_count = 0;
cyg_handle_t  counter_handle_cycle_analog;
cyg_handle_t alarm_handle_cycle_analog;
cyg_alarm alarm_object_cycle_analog;
void alarm_func_cycle_analog(cyg_handle_t counter_handle, cyg_addrword_t data)
{
		cyg_alarm_initialize(alarm_handle_cycle_analog, cyg_current_time()+200, 200); ///< 200*10ms = 2s
		cyg_alarm_enable(alarm_handle_cycle_analog);
	
	 ///< ask dot table:1.detect which pcu is calling
	if(serial_info.cur_status == IDLE)
			BcuSetRs485State( READ_ALL_REGS );
	else
	{
		cyg_alarm_initialize(alarm_handle_cycle_analog, cyg_current_time()+200, 200); ///< 200*10ms = 2s
		cyg_alarm_enable(alarm_handle_cycle_analog);
	}
}

void cycle_rs485_read_timer(void)
{
	cyg_clock_to_counter(cyg_real_time_clock(), &counter_handle_cycle_analog);
	cyg_alarm_create(counter_handle_cycle_analog, alarm_func_cycle_analog, 0,
		  &alarm_handle_cycle_analog, &alarm_object_cycle_analog);
	cyg_alarm_initialize(alarm_handle_cycle_analog, cyg_current_time()+200, 200); ///< 200*10ms = 2s
	cyg_alarm_enable(alarm_handle_cycle_analog);
}


void *SystemControl(void *arg)
{
    int ret = 0;
	unsigned char rs485_res[64] ;
	unsigned char multi_pcu_request[12];
	unsigned char multi_pcu_request_trigger[12];
	memset(multi_pcu_request,0xff,12);
	memset(multi_pcu_request_trigger,0xff,12);

    diag_printf("#-%x-%x\n",&ret,&whe_change_dev_type_failure);
	debug_print(("I am success entry control handle thread!\n"));

	BCURuleTableInit();/*Initialize the bcu rule tables*/

	BCUControlMainBufferInit();/*initial bcu control thread data buffer*/

	RegisterCommonBufferRecvBuffer();///<注册接受相应命令包缓存区

	CreateD2DHangUpD2PTimeOutTimer();/*create D2D hangup D2P timeout timer*/

	CreateAudioSampleTimer();/*create  audio sample timer*/

	CreateLiveMonitorTimer();

	BCUStateInit();/*initialize the original state of bcu*/

	send_infomation_t recv_send_info_from_touch_screen;///<从触摸屏线程接收的命令
	ts_dev_volume_info_t ctrl_recv_from_ts;///<触摸屏音量设置信息命令
	network_send_package_t recv_network_info_from_network;///<从网络上接收的命令
	common_package_t bcu_recv_tcms_cmu_state;
	network_shell_package_t recv_shell_from_network;
	///<相关命令包的初始化
	ControlCommomPackageInit(&dev_vol_to_eamp,&dev_vol_to_pcu,&dev_vol_to_bcu,&dev_vol_to_ccu);

	int counts_of_control_running = 0;///<控制线程运行次数
	int whether_ts_is_running = 0;
	int temp_current_pcu_request_number = 0;
	bcu_state.other_bcu_intercomm_state = INTERCOM_IDLE;

	///<创建获取钥匙信号定时器，定时从CCU获取钥匙信号
	CreateAskKeyInfoTimer();

	///<创建同步定时器
	CreateSYNCTimer();
	AskCCUTcmsCMUState();

	
	CreateAUTOSIMTimer();//20160824
	StartSimlateAnn();		

#ifdef CONFIG_TEST_EAMP_factory
	cycle_manual_ann_timer();
#endif

	cycle_rs485_read_timer();
	
	for(;;)
	{
		///获取所有外部按钮当前状态
		GetPAAllOuterButtonState();

		bcu_state.this_bcu_is_active = 1;

		//口播处理
		if(bcu_state.this_bcu_is_active == 1 
			&& bcu_state.bcu_active_intercom_state->state_id == INTERCOM_IDLE)
		{
			PAHandle();
		}

		if(bcu_state.pcu_is_connecting_without_ts == 0 && bcu_state.opposite_live_ann_active == 0
			&& bcu_state.bcu_active_ann_state->state_id == ANN_IDLE)
		{
			PCHandle();
		}

		if(bcu_state.opposite_d2p_connect_active == 0 && bcu_state.bcu_active_intercom_state->state_id == INTERCOM_IDLE
			&& bcu_state.opposite_live_ann_active == 0 && bcu_state.bcu_active_ann_state->state_id == ANN_IDLE )
		{
			MRHandle();
		}

		///<获取对端PPT状态
		ReadOtherBCUPttState();

		/*D2D hangup D2P timeout handle*/
		D2DHangUpD2PTimeOutHandle();

		//司机对讲请求与挂断处理
		D2DReqAndResponseHandle();

		RequestBCUKeyInfo();

		KeepComminicationWithPCU();

		//BCU 解析串口发送过来的Serial-485总线命令
		if( BlockBufferRead(bcu_state.rs485_cmd_recv_buffer_id, rs485_res, sizeof(rs485_res)) > 0 )
		{
		    if( rs485_res[0]==READ_ALL_REGS && rs485_res[1]==0x01 && rs485_res[2]==0x03 )
		    {
	             unsigned short *preg_val = (unsigned short *)&(rs485_res[4]);
				 unsigned char pcu_request_no = ((preg_val[6])>>8) | ((preg_val[6] & 0xff)<<8);
				 unsigned char pcu_recept_no = ((preg_val[7])>>8) | ((preg_val[7] & 0xff)<<8);
				 unsigned char opposite_bcu_request_status = ((preg_val[8])>>8) | ((preg_val[8] & 0xff)<<8);
				 unsigned char local_bcu_request_status = ((preg_val[9])>>8) | ((preg_val[9] & 0xff)<<8);
				 unsigned char d2d_connect_status = ((preg_val[10])>>8) | ((preg_val[10] & 0xff)<<8);

				 int temp_request_pcu_number = 0;
				 int i,j = 0;
				 for(i = 0; i < 6; i++)
				 {
					 unsigned char high_byte = (preg_val[13+i]>>8) & 0xff;
				 	 unsigned char low_byte = preg_val[13+i] & 0xff;
				 	 if(high_byte < 0xff)
				 	 {
				 		 multi_pcu_request[j] = high_byte - 100;
				 		 temp_request_pcu_number ++;
				 	 }
				 	 else
				 		 multi_pcu_request[j] = 0;
				 	 if(multi_pcu_request_trigger[j] == 0 && multi_pcu_request[j] != 0 )
				 	 {
				 		 SetMultiPCUquest(multi_pcu_request[j]);
				 	 }
				 	 if(low_byte < 0xff)
				 	 {
				 		 multi_pcu_request[j+1] = low_byte - 100;
				 		 temp_request_pcu_number ++;
				 	 }
				 	 else
				 		 multi_pcu_request[j+1] = 0;
				 	 if(multi_pcu_request_trigger[j+1] == 0 && multi_pcu_request[j+1] != 0)
				 	 {
				 		 SetMultiPCUquest(multi_pcu_request[j+1]);
				 		temp_request_pcu_number++;
				 	 }
				 	diag_printf("multi_pcu_request[%d]:%x,multi_pcu_request[%d]:%x\n",j,multi_pcu_request[j],j+1,multi_pcu_request[j+1]);
				 	multi_pcu_request_trigger[j] = multi_pcu_request[j];
				 	multi_pcu_request_trigger[j+1] = multi_pcu_request[j+1];
				 	j = j+2;
				 }

				 bcu_state.opposite_live_ann_active = ((((preg_val[1])>>8) | ((preg_val[1] & 0xff)<<8)) >> 4) & 0x1;
				 bcu_state.opposite_d2p_connect_active =  ((((preg_val[1])>>8) | ((preg_val[1] & 0xff)<<8)) >> 5) & 0x1;

					 debug_print2(("\n<1222> pcu_request_no %d ,pcu_recept_no %d,request_number %d\n",
					 pcu_request_no,pcu_recept_no,bcu_state.pcu_request_info.request_number));
					 if( ( pcu_request_no != 0xff) && pcu_recept_no == 0xff && /*bcu_state.pcu_request_info.request_number == 0*/ temp_request_pcu_number != 0 ) ///< pcu calling on 485-bus so we request local  pcu alarm
					 {
						 debug_print2(("<1221-1 pcu calling 485-bus>: pcu_request_%d \n", pcu_request_no-100 ));
						 bcu_state.pcu_request_info.current_request_pcu_no = pcu_request_no - 100;
//						 SetPCURequestAccordingToRs485();
					 }
					 else if ( pcu_recept_no != 0xff  && bcu_state.pcu_request_info.request_number > 0 )
					 {
						 debug_print2(("<1221-2 pcu recept local>\n"));
						 bcu_state.pcu_request_info.current_conect_pcu_no = pcu_recept_no - 100;
						 if(bcu_state.pcu_request_info.current_request_pcu_no != 0xff)
						 {
							 debug_print2(("just test\n"));
							 SetPCUConnectAccordingToRs485();
							 bcu_state.pcu_request_info.current_request_pcu_no = 0xff;
						 }
					 }
					 /*/////////////////mutil pcu request///////////////////////
							   	   pcu_request_no  pcu_recept_no
					pcu req1--->	   pcu1_no 		      255
									 	255 			pcu1_no   
					pcu req2--->	   pcu2_no 		    pcu1_no
									   pcu2_no 		      255
									 	255 			pcu2_no 
									 	255 			  255
					//////////////////////////////////////////////////////////*/
					 else if ((bcu_state.pcu_request_info.current_request_pcu_no != 0 && (pcu_request_no == 0xff) && pcu_recept_no == 0xff  && bcu_state.pcu_request_info.request_number > 0) ||
							 (bcu_state.pcu_request_info.current_request_pcu_no == 0xff && pcu_recept_no == 0xff  && bcu_state.pcu_request_info.request_number > 0) )
					 {
						 debug_print2(("<1221-3 pcu clear 485-bus>\n"));
//						 SetPCUClearAccordingToRs485();
						 if(bcu_state.pcu_request_info.current_request_pcu_no != 0xff || bcu_state.pcu_request_info.current_conect_pcu_no != 0xff)
						 {
							 SetPCUClearAccordingToRs485();
							 bcu_state.pcu_request_info.request_number  = 0;
							 bcu_state.pcu_request_info.current_request_pcu_no = 0xff;
							 bcu_state.pcu_request_info.current_conect_pcu_no = 0xff;
						 }
					 }
				if( opposite_bcu_request_status == 1) ///< D2D calling on 485-bus so we request local D2D
				{
					 debug_print2(("<1222-1 d2d calling 485-bus>\n"));
				 	 SetD2DRequestAccordingToRs485();
				}
				if( d2d_connect_status == 1 ) ///< D2D is connect on 485-bus
				{
					 debug_print2(("<1222-2 d2d connect 485-bus>\n"));
					 bcu_state.d2d_is_connecting_without_ts = 1;
				 	 SetD2DRequestAccordingToRs485();
				}
				else if(bcu_state.d2d_is_connecting_without_ts == 1) 
				{
					debug_print2(("<1222-3 d2d finish 485-bus>\n"));
					FinishD2DAccordingToRs485();
					bcu_state.d2d_is_connecting_without_ts = 0;
				}
				
				memset((void *)rs485_res, 0, sizeof(rs485_res));
			}
		}
		//BCU 接收CCU发送过来的TCMS有关CMU的状态
		if(BlockBufferRead(bcu_state.bcu_recv_tcms_cmu_state_buffer_id,&bcu_recv_tcms_cmu_state,sizeof(bcu_recv_tcms_cmu_state)) > 0)
		{
			if(bcu_recv_tcms_cmu_state.pkg_type == COMMON_PACKAGE_TYPE_TCMS_CMU_STATE && bcu_recv_tcms_cmu_state.common_data_u.tcms_cmu_state.get_or_set ==0)
			{
				diag_printf("cmu_state:%d\n",bcu_recv_tcms_cmu_state.common_data_u.tcms_cmu_state.tcms_current_cmu_state);
				if(bcu_state.bcu_current_tcms_cmu_state != bcu_recv_tcms_cmu_state.common_data_u.tcms_cmu_state.tcms_current_cmu_state)
				{
					bcu_state.bcu_current_tcms_cmu_state = bcu_recv_tcms_cmu_state.common_data_u.tcms_cmu_state.tcms_current_cmu_state;
				}
			}
		}
		if(BlockBufferRead(bcu_state.udp_common_recv_dev_colueme,&recv_dev_col_bcu,sizeof(recv_dev_col_bcu)) > 0)
		{
			if(recv_dev_col_bcu.pkg_type == COMMON_PACKAGE_TYPE_DEVICE_INFO)
			{
				if(strcmp(recv_dev_col_bcu.common_data_u.ts_dev_volume_info.device_name,"BCU") == 0 &&
						recv_dev_col_bcu.common_data_u.ts_dev_volume_info.device_volume >= 0 &&
						recv_dev_col_bcu.common_data_u.ts_dev_volume_info.device_volume <= 4)
				{

					diag_printf("old:d2d_volume = %d\n",bcu_state.device_volume.d2d_volume);
					if(bcu_state.device_volume.d2d_volume != recv_dev_col_bcu.common_data_u.ts_dev_volume_info.device_volume)
					{
						last_control_flag = control_flag;control_flag = 59;
						bcu_state.device_volume.d2d_volume = recv_dev_col_bcu.common_data_u.ts_dev_volume_info.device_volume;
						bcu_6d5w_ctrl_wilson(bcu_state.device_volume.d2d_volume) ;
						if(bcu_state.bcu_active_intercom_state->state_id != INTERCOM_IDLE)
						{
							AdjustVolumeAfterCODEC();
						}
						diag_printf("new:d2d_volume = %d\n",bcu_state.device_volume.d2d_volume);
					}
				}
				else if(strcmp(recv_dev_col_bcu.common_data_u.ts_dev_volume_info.device_name,"PCU") == 0 &&
						recv_dev_col_bcu.common_data_u.ts_dev_volume_info.device_volume >= 0 &&
							recv_dev_col_bcu.common_data_u.ts_dev_volume_info.device_volume <= 4)
				{
					if(bcu_state.device_volume.intercomm_volume != recv_dev_col_bcu.common_data_u.ts_dev_volume_info.device_volume)
					{
						bcu_state.device_volume.intercomm_volume = recv_dev_col_bcu.common_data_u.ts_dev_volume_info.device_volume;
					}
					diag_printf("bcu_state.device_volume.intercomm_volume = %d\n",bcu_state.device_volume.intercomm_volume);
				}
			}
		}
		if(BlockBufferRead(bcu_state.bcu_recv_monitor_control_info_id,&monitor_ctrl_info,sizeof(monitor_ctrl_info)) > 0)
		{
			if(monitor_ctrl_info.pkg_type == COMMON_PACKAGE_TYPE_RUNNING_MODE_INFO)
			{
				bcu_state.current_pa_running_mode = monitor_ctrl_info.common_data_u.monitor_control_info.pa_running_mode;
				diag_printf("bcu_state.current_pa_running_mode  %s\n",bcu_state.current_pa_running_mode ? "DIGTAL":"ANALOG");
				bcu_state.current_d2d_running_mode = monitor_ctrl_info.common_data_u.monitor_control_info.d2d_running_mode;
				diag_printf("bcu_state.current_d2d_running_mode  %s\n",bcu_state.current_d2d_running_mode ? "DIGTAL":"ANALOG");
			}else if(monitor_ctrl_info.pkg_type == COMMON_PACKAGE_TYPE_VOLUME_ADJ_INFO)
			{
				if(bcu_state.current_d2d_running_mode == DIGTAL_MODE)
				{
					bcu_state.device_volume.d2d_volume = monitor_ctrl_info.common_data_u.monitor_control_info.d2d_volume;
					diag_printf("DIGTAL_MODE:bcu_state.d2d_volume = %d\n",bcu_state.device_volume.d2d_volume);
					AdjustVolumeAfterCODEC();
				}
				else if(bcu_state.current_d2d_running_mode == ANALOG_MODE)
				{
					bcu_state.device_volume.d2d_volume = monitor_ctrl_info.common_data_u.monitor_control_info.d2d_volume;
					diag_printf("ANALOG_MODE:bcu_state.d2d_volume = %d\n",bcu_state.device_volume.d2d_volume);
					AdjustVolumeByGPIO(bcu_state.device_volume.d2d_volume);
				}
			}
			else if(monitor_ctrl_info.pkg_type == COMMON_PACKAGE_TYPE_PCU_CONTROL)
			{
				diag_printf("current: recept_pcu_no = %d,refuse_pcu_no = %d\n",bcu_state.pcu_request_info.recept_pcu_no,bcu_state.pcu_request_info.refuse_pcu_no);
				diag_printf("monitor_ctrl:pcu_recept_no = %d,pcu_refuse_no\n",monitor_ctrl_info.common_data_u.monitor_control_info.pcu_recept_no,
						monitor_ctrl_info.common_data_u.monitor_control_info.pcu_recept_no);
				bcu_state.pcu_request_info.refuse_pcu_no= monitor_ctrl_info.common_data_u.monitor_control_info.pcu_refuse_no;
				bcu_state.pcu_request_info.recept_pcu_no = monitor_ctrl_info.common_data_u.monitor_control_info.pcu_recept_no;
				SetAnyPCUAccordingToCCU(multi_pcu_request);
			}
		}
#if 1
		/*if there have cmd information comes from network thread,we should response it as soon as possible*/
		if(BlockBufferRead(bcu_state.cmd_recv_buffer_id,(void *)&recv_network_info_from_network, sizeof(recv_network_info_from_network))>0 &&
				BCUGetDataFromCCUState() == 3 && recv_network_info_from_network.send_information.src_devices_no <= 12)
		{
			diag_printf("<control>recv from %s-%d,ann=%d,intercom=%d,%d,%d,%d\n",
					recv_network_info_from_network.send_information.src_devices_name,
					recv_network_info_from_network.send_information.src_devices_no,
					recv_network_info_from_network.send_information.event_type_ann,
					recv_network_info_from_network.send_information.event_type_intercom,
					recv_network_info_from_network.send_information.event_info_intercom.d2d_intercomm.d2d_intercomm_active,
					recv_network_info_from_network.send_information.event_info_intercom.d2d_intercomm.d2d_intercomm_request_or_over,
					recv_network_info_from_network.send_information.event_info_intercom.d2d_intercomm.d2d_intercomm_response);

			/*connect two cars*/
			if(strncmp(recv_network_info_from_network.send_information.src_devices_name,"CMU",3) == 0)
			{
				BCUCMUCommunication(recv_network_info_from_network.send_information);
			}

			if(recv_network_info_from_network.send_information.event_type_intercom == D2D_INTERCOMM_EVENT)
			{
				/*Judge whether have D2D request from network*/
				JudgeWhetherHaveD2DRequest(&recv_cmd_info_of_intercom,&recv_network_info_from_network);
			}

			if(recv_network_info_from_network.send_information.event_type_ann == PA_KEY_INFORMATION)
			{
				if(recv_network_info_from_network.send_information.event_info_ann.pa_key_info.get_or_set_key_info == 0)
				{
					if(bcu_state.this_bcu_is_active == 1 &&
							recv_network_info_from_network.send_information.event_info_ann.pa_key_info.bcu_status[bcu_state.bcu_info.devices_no - 1] == 0)
					{
						//ClosePA();
					}
					bcu_state.this_bcu_is_active = recv_network_info_from_network.send_information.event_info_ann.pa_key_info.bcu_status[bcu_state.bcu_info.devices_no - 1];			
				}
				diag_printf("bcu_state.this_bcu_is_active = %d\n",bcu_state.this_bcu_is_active);
			}
			temp_current_pcu_request_number = bcu_state.pcu_request_info.request_number;
			if(recv_network_info_from_network.send_information.event_type_intercom == D2P_INTERCOMM_EVENT)
			{
				diag_printf("temp_current_pcu_request_number = %d, src_pcu_no=%d\n",
					temp_current_pcu_request_number,
					recv_network_info_from_network.send_information.src_devices_no );
					if(recv_network_info_from_network.send_information.event_info_intercom.d2p_intercomm.d2p_intercomm_active == 1 &&
							recv_network_info_from_network.send_information.event_info_intercom.d2p_intercomm.d2p_intercomm_request_or_over == 1 &&
							recv_network_info_from_network.send_information.event_info_intercom.d2p_intercomm.d2p_intercomm_response == 0)
					{
						bcu_state.pcu_request_info.request_number ++;
						bcu_state.pcu_request_info_without_ts[recv_network_info_from_network.send_information.src_devices_no-1] = bcu_state.pcu_request_info.request_number; 
						//bcu_state.pcu_request_info_without_ts[recv_network_info_from_network.send_information.src_devices_no-1]++;
						diag_printf("1:bcu_state.pcu_request_info.request_number = %d,%d\n",bcu_state.pcu_request_info.request_number,recv_network_info_from_network.send_information.src_devices_no-1);
					}
					else 	if(recv_network_info_from_network.send_information.event_info_intercom.d2p_intercomm.d2p_intercomm_active == 1 &&
							recv_network_info_from_network.send_information.event_info_intercom.d2p_intercomm.d2p_intercomm_request_or_over == 0 &&
							recv_network_info_from_network.send_information.event_info_intercom.d2p_intercomm.d2p_intercomm_response == 1 &&
							recv_network_info_from_network.send_information.event_info_intercom.d2p_intercomm.d2p_intercomm_bcu_device_no != bcu_state.bcu_info.devices_no)
					{
						BcuResetPlayAlarmAudioWhenD2pReq();
						bcu_state.pcu_is_connecting_without_ts = 1;
					}
					else if(recv_network_info_from_network.send_information.event_info_intercom.d2p_intercomm.d2p_intercomm_active == 0 &&
							bcu_state.pcu_request_info.request_number > 0 &&
							bcu_state.pcu_request_info_without_ts[recv_network_info_from_network.send_information.src_devices_no-1] !=0 &&
							strcmp(recv_network_info_from_network.send_information.src_devices_name,"BCU") != 0)
					{
						int l = 0;
						for(l = 0;l < 12;l++)
						{
							if(bcu_state.pcu_request_info_without_ts[l] != 0 && bcu_state.pcu_request_info_without_ts[l] >= 1)  //20161222 bug-fix: cancle pcu alarm
							{
								bcu_state.pcu_request_info_without_ts[l] --;
							}
						}

						//bcu_state.pcu_request_info_without_ts[recv_network_info_from_network.send_information.src_devices_no-1] = 0;
//						bcu_state.pcu_request_info.request_number --;
						diag_printf("3:bcu_state.pcu_request_info.request_number = %d,%d\n",bcu_state.pcu_request_info.request_number,recv_network_info_from_network.send_information.src_devices_no-1);
						bcu_state.pcu_is_connecting_without_ts = 0;
						diag_printf("###########################---%d\n",bcu_state.pcu_is_connecting_without_ts );
					}
			}
			if(recv_network_info_from_network.send_information.event_type_ann == MANUAL_ANN_EVENT )
			{
				station_info_from_network.door_side = recv_network_info_from_network.send_information.event_info_ann.manual_annnounce.door_side;
				station_info_from_network.current_station_code = recv_network_info_from_network.send_information.event_info_ann.manual_annnounce.current_station_code;
				station_info_from_network.next_station_code = recv_network_info_from_network.send_information.event_info_ann.manual_annnounce.next_station_code;
				diag_printf("$$$$ door_side %d\n",station_info_from_network.door_side);
				diag_printf("$$$$ current_station_code %d\n",station_info_from_network.current_station_code);
				diag_printf("$$$$ next_station_code %d\n",station_info_from_network.next_station_code);

				AlarmTSToChangeScreen(2);

			}
			if(recv_network_info_from_network.send_information.event_type_ann == LINE_NUMBER_CHANGE_EVENT)
			{
				diag_printf("$$$$ LINE_NUMBER_CHANGE_EVENT\n");
				if(BCUGetSyncState()>0)
				{
					AlarmTSToChangeScreen(1);
				}
			}
			if(bcu_state.pcu_request_info.request_number == 0 && temp_current_pcu_request_number > 0)
			{
				diag_printf("SendNoPCUToCCU\n");
				///<发送没有PCU请求给CCU，确保监控联动
				BcuResetPlayAlarmAudioWhenD2pReq(); 
				//SendNoPCUToCCU();   
			}
			if(temp_current_pcu_request_number != bcu_state.pcu_request_info.request_number)
			{
				StartOrBrokeBroadcastPcuRequestAlarmAudioData();
				idle_pcu_req_count = 0;
				debug_print(("[2016]: SSSSSSexitSSSSSSS\n"));
			}
		}
		if(BlockBufferRead(bcu_state.shell_recv_buffer_id,(void *)&recv_shell_from_network,sizeof(recv_shell_from_network))>0)
		{	
			int shell_loop;
			for(shell_loop = 0 ; shell_loop < SHELL_CMD_NUMBER ; shell_loop++)
			{
				if(recv_shell_from_network.cmd == shellcmd[shell_loop].cmd)
				{
					shellcmd[shell_loop].cmd_function(recv_shell_from_network);
					break;
				}
			}
		}

		//SetAutoAnnWhenIntercomIdle();
		if( atuo_set_flag == 1)
		{
			//发送PA设备同步信号
			DevSyncHandle();
			
			//模拟自动报站--add by pan--2015-12-25
			if(wherther_start_auto_ann == 1)
			{
				SimlateAtuoANNInofFill(&recv_send_info_from_touch_screen);
				wherther_start_auto_ann = 0;
			}
		}
		
		//到此已经进入到具体的状态机里面了
		bcu_state.bcu_active_ann_state->process(&recv_cmd_info_of_ann);
		ClearProcessPackage(&recv_cmd_info_of_ann);
		bcu_state.bcu_active_intercom_state->process(&recv_cmd_info_of_intercom);
		ClearProcessPackage(&recv_cmd_info_of_intercom);

#endif
		///处理设备版本号检查
		GetBCUDeviceVersionInfo();
		sched_yield();

	}
	return NULL;
}

int times_of_show_led = 0;
#ifndef CONFIG_TEST_SND_IN_MULTI_THREAD
void *BcuMicSampleAndPlayEntry(void *arg)
{//---265
	debug_print(("I am success entry sample and play handle thread!\n"));
	int whether_send_blank_audio_data = 0;
    struct timeval tv_start, tv_end;
    CreateSndEnableTimer();
	for(;;)
	{
		DisableSndEnableTimer();
		sem_wait(&sem_wakeup_bcu_mic_sample);
		EnableSndEnableTimer();
		GetPAAllOuterButtonState2();

		if(bcu_state.bcu_active_ann_state->state_id == MIC_3D5_OUTER_EVENT &&
		   bcu_state.bcu_active_intercom_state->state_id != D2P_INTERCOMM_EVENT &&
		   bcu_state.bcu_info.current_state_ann == MIC_3D5_OUTER_EVENT &&
		   bcu_state.bcu_mcu_connect_state == 0)
		{
			///<获取外接3D5音频数据
			PutMicSampleDataintoBuffer_EAMP_Outer(bcu_audio_handle, bcu_state.audio_data_send_buffer_id);
			///<改变网络线程优先级，确保网络数据及时发出
			PthreadPriorityChangeForSchedRr(thread_of_network, BCU_HIGH_NETWORK_PRIORITY);
		}

		if(bcu_state.bcu_active_ann_state->state_id == LIVE_ANN_EVENT &&
		   bcu_state.bcu_active_intercom_state->state_id != D2P_INTERCOMM_EVENT &&
		   bcu_state.bcu_info.current_state_ann == LIVE_ANN_EVENT) //d2d??
		{
			if(GetLiveMonitorCounts() >= 60)
			{
				if(bcu_state.live_button_state == 1)///<exit live
				{
					ExitFromLiveWithoutTS();
				}
			}
			if(bcu_state.current_pa_running_mode == ANALOG_MODE)
			{
				if(bcu_state.this_bcu_ptt_state != GetPTTState())
				{
				    if( bcu_state.this_bcu_ptt_state == 0 )
					{  ///< PTT pressed
						debug_print2(("PTT-1\n"));
	                    hal_gpio_pin_clear( CYGHWR_HAL_KINETIS_PIN(A, 9, 1, KINETIS_PIN_PULLUP) );
	                    hal_gpio_pin_set( CYGHWR_HAL_KINETIS_PIN(A, 10, 1, KINETIS_PIN_PULLUP) ); 
					}
					else if( bcu_state.this_bcu_ptt_state == 1 )
					{  ///< release PTT
						debug_print2(("PTT-0\n"));
	                    hal_gpio_pin_set( CYGHWR_HAL_KINETIS_PIN(A, 9, 1, KINETIS_PIN_PULLUP) );
	                    hal_gpio_pin_clear( CYGHWR_HAL_KINETIS_PIN(A, 10, 1, KINETIS_PIN_PULLUP) ); 
					}
						
					bcu_state.this_bcu_ptt_state = GetPTTState();
					if(bcu_state.this_bcu_ptt_state == 0)
					{
						whether_send_blank_audio_data = 5;
					}
				}
			}
			else
			{
				///<采集口播音频数据
				PutMicSampleDataintoBuffer_EAMP(bcu_audio_handle, bcu_state.audio_data_send_buffer_id );
				///<改变网络优先级，确保音频数据能够及时的发出
				PthreadPriorityChangeForSchedRr(thread_of_network, BCU_HIGH_NETWORK_PRIORITY);
			}
		}
		else if(bcu_state.bcu_active_intercom_state->state_id == D2P_INTERCOMM_EVENT)
		{
			if(bcu_state.this_bcu_ptt_state != GetPTTState())
			{
			       if( bcu_state.this_bcu_ptt_state == 0 )
				{  ///< PTT pressed
					debug_print2(("PTT-1\n"));
                    hal_gpio_pin_clear( CYGHWR_HAL_KINETIS_PIN(A, 9, 1, KINETIS_PIN_PULLUP) );
                    hal_gpio_pin_set( CYGHWR_HAL_KINETIS_PIN(A, 10, 1, KINETIS_PIN_PULLUP) ); 
                    hal_gpio_pin_clear( CYGHWR_HAL_KINETIS_PIN(D, 11, 1, KINETIS_PIN_PULLUP) ); 
					BcuSetRs485State(SET_LOCAL_PTT);
				}
				else if( bcu_state.this_bcu_ptt_state == 1 )
				{  ///< release PTT
					debug_print2(("PTT-0\n"));
                    hal_gpio_pin_set( CYGHWR_HAL_KINETIS_PIN(A, 9, 1, KINETIS_PIN_PULLUP) );
                    hal_gpio_pin_clear( CYGHWR_HAL_KINETIS_PIN(A, 10, 1, KINETIS_PIN_PULLUP) ); 
                    hal_gpio_pin_set( CYGHWR_HAL_KINETIS_PIN(D, 11, 1, KINETIS_PIN_PULLUP) ); 
					BcuSetRs485State(SET_REMOTE_PTT);
				}					
				bcu_state.this_bcu_ptt_state = GetPTTState();
			}

			///<发送PTT状态给PCU
			SendPTTStateToPCU();
		}
		else if(bcu_state.bcu_active_intercom_state->state_id == D2D_INTERCOMM_EVENT)
		{
			if(bcu_state.current_d2d_running_mode == ANALOG_MODE)
			{
				if(bcu_state.this_bcu_ptt_state != GetPTTState())
				{
				    if( bcu_state.this_bcu_ptt_state == 0 )
					{  ///< PTT pressed
						debug_print2(("PTT-1\n"));
	                    hal_gpio_pin_clear( CYGHWR_HAL_KINETIS_PIN(A, 9, 1, KINETIS_PIN_PULLUP) );
	                    hal_gpio_pin_set( CYGHWR_HAL_KINETIS_PIN(A, 10, 1, KINETIS_PIN_PULLUP) );
	                    hal_gpio_pin_clear( CYGHWR_HAL_KINETIS_PIN(D, 11, 1, KINETIS_PIN_PULLUP) ); 
						BcuSetRs485State(SET_LOCAL_PTT); 
					}
					else if( bcu_state.this_bcu_ptt_state == 1 )
					{  ///< release PTT
						debug_print2(("PTT-0\n"));
	                    hal_gpio_pin_set( CYGHWR_HAL_KINETIS_PIN(A, 9, 1, KINETIS_PIN_PULLUP) );
	                    hal_gpio_pin_clear( CYGHWR_HAL_KINETIS_PIN(A, 10, 1, KINETIS_PIN_PULLUP) ); 
	                    hal_gpio_pin_set( CYGHWR_HAL_KINETIS_PIN(D, 11, 1, KINETIS_PIN_PULLUP) ); 
						BcuSetRs485State(SET_REMOTE_PTT);
					}					
					bcu_state.this_bcu_ptt_state = GetPTTState();
				}
			}
			else
			{
				if(bcu_state.bcu_mcu_connect_state == 0 ||
						(bcu_state.bcu_mcu_connect_state == 1 &&
								bcu_state.this_bcu_ptt_state == 1 &&
								bcu_state.other_bcu_ptt_state == 0))
				{
					///<采集司机对讲音量数据
					PutMicSampleDataintoBuffer_BCU(bcu_audio_handle, bcu_state.audio_data_send_buffer_id );
	//				diag_printf("2:bcu_state.bcu_mcu_connect_state = %d\n",bcu_state.bcu_mcu_connect_state);
				}
		
				if(bcu_state.bcu_mcu_connect_state == 1)
				{///<联挂状态下处理
					if(bcu_state.other_bcu_ptt_state == 0)
					{
						SendPTTStateToBCU(); /*Send ptt state to bcu*/
					}
					ReadOtherBCUPttState();/*Acquire other bcu ptt state*/
				}
				///<改变网络优先级，确保音频数据能够及时的发出
				PthreadPriorityChangeForSchedRr(thread_of_network, BCU_HIGH_NETWORK_PRIORITY);
				if(play_audio == 0)
				{
					unsigned int current_total_bytes = 0;
					unsigned int current_right_delta = 0;
					CharBufferCurrentReadPointer_temp(bcu_state.audio_data_recv_buffer_id, &current_total_bytes,&current_right_delta);
					if( current_total_bytes >= 1024*3)
					{
						play_audio = 1;
						whe_d2d_souder_have_been_worked = 0;
						whe_d2d_souder_stable = 0;
					}
				}

				if(1 == play_audio)
				{
					///<播放司机对讲音频数据
					PlayAudioTwice_D2D(bcu_audio_handle,bcu_state.audio_data_recv_buffer_id,bcu_state.pending_buffer_id);
				}
				
			}		
		}
		else if(bcu_state.bcu_active_intercom_state->state_id == D2D_HANGUP_D2P_EVENT )
		{
			if(bcu_state.bcu_mcu_connect_state == 1)
			{///<联挂状态下处理
				if(bcu_state.other_bcu_ptt_state == 0)
				{
					SendPTTStateToBCU(); /*Send ptt state to bcu*/
				}
				ReadOtherBCUPttState();/*Acquire other bcu ptt state*/
			}
			///<改变网络优先级，确保音频数据能够及时的发出
			//PthreadPriorityChangeForSchedRr(thread_of_network, BCU_HIGH_NETWORK_PRIORITY);
			if(bcu_state.this_bcu_ptt_state != GetPTTState())
			{
			    if( bcu_state.this_bcu_ptt_state == 0 )
				{  ///< PTT pressed
					debug_print2(("PTT-1\n"));
                    hal_gpio_pin_clear( CYGHWR_HAL_KINETIS_PIN(A, 9, 1, KINETIS_PIN_PULLUP) );
                    hal_gpio_pin_set( CYGHWR_HAL_KINETIS_PIN(A, 10, 1, KINETIS_PIN_PULLUP) ); 
                    hal_gpio_pin_clear( CYGHWR_HAL_KINETIS_PIN(D, 11, 1, KINETIS_PIN_PULLUP) ); 
					BcuSetRs485State(SET_LOCAL_PTT);
				}
				else if( bcu_state.this_bcu_ptt_state == 1 )
				{  ///< release PTT
					debug_print2(("PTT-0\n"));
                    hal_gpio_pin_set( CYGHWR_HAL_KINETIS_PIN(A, 9, 1, KINETIS_PIN_PULLUP) );
                    hal_gpio_pin_clear( CYGHWR_HAL_KINETIS_PIN(A, 10, 1, KINETIS_PIN_PULLUP) ); 
                    hal_gpio_pin_set( CYGHWR_HAL_KINETIS_PIN(D, 11, 1, KINETIS_PIN_PULLUP) ); 
					BcuSetRs485State(SET_REMOTE_PTT);
				}					
				bcu_state.this_bcu_ptt_state = GetPTTState();
			}
		}
		else if(bcu_state.bcu_active_intercom_state->state_id == INTERCOM_IDLE) ///< ach add, begin, 2014-01-08
		{
			if(bcu_state.pcu_request_info.request_number !=0)
			{
				hal_gpio_pin_set( CYGHWR_HAL_KINETIS_PIN(D, 11, 1, KINETIS_PIN_PULLUP) );
				hal_gpio_pin_set( CYGHWR_HAL_KINETIS_PIN(F, 25, 1, KINETIS_PIN_PULLUP) );
				
                if ( bcu_state.pcu_request_info.request_number !=0 )
                {
                	debug_print(("Bcu,mp3 thread[idle]:  %d,%d \n",
                			bcu_state.pcu_request_info.request_number,idle_pcu_req_count));
                       if( idle_pcu_req_count++ >= 3 )
                       {
                            idle_pcu_req_count = 0;
                            hal_gpio_pin_toggle( BUTTON_2_LED_PC_ON );
                       }
                }

			     if( 1==bcu_state.pcu_request_info.open_snd_playback )
			     {
				   debug_print(("Bcu,mp3 thread:  play alarm audio to tell LCD \n"));
				   cyg_thread_delay(20);
				   bcu_state.pcu_request_info.open_snd_playback = 2;
				   if( 2==GetSndCurrentMode() )
				   {
						//OpenSndCard(&bcu_audio_handle, "playback");
				   }
				   MicAdPcmWavheader(bcu_state.alarm_audio_data_buffer_id);	 
				    //ReadAlarmAudioDataToBuffer(bcu_state.alarm_audio_data_buffer_id);
				   AlarmAudioDataIntoBuffer(bcu_state.alarm_audio_data_buffer_id);
				   debug_print(("Bcu,mp3 thread:  alarm audio into Buffer,so play \n"));
			     }
				 
			     if( bcu_state.pcu_request_info.pcu_alarm_playing==1 )
			     {
					if( -5==PlayAudioTwice(bcu_audio_handle, bcu_state.alarm_audio_data_buffer_id, bcu_state.pending_buffer_id) )
					{
						char tmp_array[64];
						CloseAudioSampleTimerForPcuAlarm();
						debug_print(("Bcu,mp3 thread:  alarm aud io play, try again \n"));
						CharBufferBrush(bcu_state.alarm_audio_data_buffer_id);
						CharBufferRead(bcu_state.alarm_audio_data_buffer_id, tmp_array, 48);
						PlayAudioTwice(bcu_audio_handle, bcu_state.alarm_audio_data_buffer_id, bcu_state.pending_buffer_id);
						CharBufferClear(bcu_state.pending_buffer_id);
						bcu_state.pcu_request_info.pcu_alarm_playing_again = 1;
					}
				 }
			}
			else
			{
				debug_print((">>>Bcu,IDLE<<< \n"));

				hal_set_pin_function( BUTTON_2_LED_PC_ON );
				hal_gpio_pin_ddr_out( BUTTON_2_LED_PC_ON );
				hal_gpio_pin_clear( BUTTON_2_LED_PC_ON );

				hal_gpio_pin_clear( CYGHWR_HAL_KINETIS_PIN(D, 11, 1, KINETIS_PIN_PULLUP) );
				hal_gpio_pin_clear( CYGHWR_HAL_KINETIS_PIN(F, 25, 1, KINETIS_PIN_PULLUP) );
				if( bcu_state.pcu_request_info.pcu_alarm_playing==1 )
				{
					debug_print(("Bcu,mp3 thread:  close alarm play and reset \n"));
					BcuResetPlayAlarmAudioWhenD2pReq();
				}
			}
		}		
		debug_print(("this bcu ppt = %d,bcu_mcu_state = %d\n",bcu_state.this_bcu_ptt_state,bcu_state.bcu_mcu_connect_state));
	}
	 return NULL;
}
#endif

void *BcuFailureStaticsEntry(void *arg)
{
	debug_print(("I am success entry failure statics handle thread!\n"));
	udp_heart_socket_buffer_t udp_heart_socket_buffer_info;
	udp_heart_socket_buffer_info.udp_heart_socket_recv_buffer = BlockBufferOpen("network-udp-heart-recv");
	udp_heart_socket_buffer_info.udp_heart_socket_send_buffer = BlockBufferOpen("network-udp-heart-send");
	ZhwDevTestModule(udp_heart_socket_buffer_info);
	return NULL;
}


void *BCUTftpThreadEntry(void *arg)
{
  debug_print(("I am enter BCUTftpThreadEntry!!!!!"));
  char *db_server_ip = bcu_devices_info_IP.dev_ip;
  int loop_time = 0;
  unsigned char tftp_created = 0;
//	memset((void *) db_server_ip,0,sizeof(db_server_ip));

  for(;;)
  {	
		if(loop_time++ >= 50000)
		{
	//		debug_print(("EAMP tftp:  running!!!!\n"));
		    loop_time = 0;
	//		debug_print(("serverIP == %s\n",db_server_ip));
		}

		if(tftp_created == 0)
		{
			TftpServerAPI(db_server_ip);
			tftp_created = 1;
		}
		
	sched_yield();
  }

  return 0;

}

// This routine will be called if the read "times out"
static void
do_abort(void *handle)
{
	cyg_uint32 len;
	debug_print2(("<1214> time out\n"));
	
    cyg_io_handle_t io_handle = (cyg_io_handle_t)handle;
	cyg_io_get_config(io_handle, CYG_IO_GET_CONFIG_SERIAL_ABORT, NULL, &len);
}


void SerialInit(cyg_io_handle_t *param_io,char param_dev_string[],int param_baud)
{
	Cyg_ErrNo err;
	cyg_serial_info_t ser_info;
	unsigned int len;
	unsigned char buffer[16];

	err = cyg_io_lookup(param_dev_string, param_io);
	if (ENOERR == err)
	{
		debug_print2(("%s found\n",param_dev_string));
		len = sizeof(buffer);

		cyg_io_get_config(*param_io, CYG_IO_GET_CONFIG_SERIAL_INPUT_FLUSH, &buffer[0], &len);
		cyg_io_get_config(*param_io, CYG_IO_GET_CONFIG_SERIAL_OUTPUT_FLUSH, &buffer[0], &len);

		// cyg_io_set_config
		if(param_baud == 115200)
		{
			ser_info.baud = CYGNUM_SERIAL_BAUD_115200;
		}
		else if(param_baud == 38400)
		{
			ser_info.baud = CYGNUM_SERIAL_BAUD_38400;
		}
		else if(param_baud == 9600)
		{
			ser_info.baud = CYGNUM_SERIAL_BAUD_9600;
		}
		else
		{
				ser_info.baud = CYGNUM_SERIAL_BAUD_115200;
		}
		ser_info.word_length = CYGNUM_SERIAL_WORD_LENGTH_8;
		ser_info.stop = CYGNUM_SERIAL_STOP_1;
		ser_info.parity = CYGNUM_SERIAL_PARITY_NONE;
		ser_info.flags = CYGNUM_SERIAL_FLOW_NONE;

		len = sizeof(ser_info);
		err = cyg_io_set_config(*param_io, CYG_IO_SET_CONFIG_SERIAL_INFO,
							&ser_info, &len);
        if (err != ENOERR)
            diag_printf("<2016-0322> set %s config: err=%d\n",
            		param_dev_string, err);
      cyg_uint32 cfg_data;
      cfg_data = 0;
      len = sizeof(cfg_data);
	  err = cyg_io_set_config(*param_io, CYG_IO_SET_CONFIG_SERIAL_WRITE_BLOCKING,
			&cfg_data, &len); ///< non-block
      if (err != ENOERR)
        diag_printf("<2016-0322> set %s wr non-block: err=%d\n",
        		param_dev_string, err);
      cfg_data = 1;
	  err = cyg_io_set_config(*param_io, CYG_IO_SET_CONFIG_SERIAL_READ_BLOCKING,
			&cfg_data, &len); ///< read block
      if (err != ENOERR)
            diag_printf("<2016-0322> set %s  rd non-block: err=%d\n",
            		param_dev_string, err);
	}
	else
	{
		debug_print2(("%s not found\n",param_dev_string));
		diag_printf("%s not found\n",param_dev_string);
	}
}
void *BCUSerialThreadEntry(void *arg)
{
	cyg_int32 stamp;
	cyg_io_handle_t ser;
	cyg_uint32 send_byte,recv_byte,len;
	int i,ret;
	char recv_send_buf[64];char tmp_buf[64];
	serial_info.rs485_cmd_recv_buffer = BlockBufferOpen("rs485-recv-cmd-buffer");
	serial_info.rs485_cmd_send_buffer = BlockBufferOpen("rs485-send-cmd-buffer");

	SerialInit(&ser, "/dev/ser2", 9600);

	char *state_str[] = {"IDLE","LIVE_ANN_EXIT","LIVE_ANN_ENTER","D2D_INTERCOMM_EXIT",
					"D2D_INTERCOMM_ENTER","D2D_INTERCOMM_REQUEST","D2P_INTERCOMM_EXIT",
					"D2P_INTERCOMM_ENTER","READ_ALL_REGS","SET_LOCAL_PTT","SET_REMOTE_PTT","RESET_ALL_DEV"};

	memset(recv_send_buf,0,sizeof(recv_send_buf));
	memset(tmp_buf,0,sizeof(tmp_buf));
	cyg_io_get_config(ser, CYG_IO_GET_CONFIG_SERIAL_INPUT_FLUSH, &recv_send_buf[0], &len);
	cyg_io_get_config(ser, CYG_IO_GET_CONFIG_SERIAL_OUTPUT_FLUSH, &recv_send_buf[0], &len);

	for(;;)
	{
		if( BlockBufferRead(serial_info.rs485_cmd_send_buffer, recv_send_buf, sizeof(recv_send_buf)) > 0 )
		{
  	        PthreadPriorityChangeForSchedRr(pthread_self(), BCU_HIGH_NETWORK_PRIORITY+10); ///< up
  	                
			hal_gpio_pin_set( CYGHWR_HAL_KINETIS_PIN(E, 4, 1, KINETIS_PIN_PULLUP) );

			send_byte = serial_info.send_count;	
			ret = cyg_io_write(ser, recv_send_buf, &send_byte);
			cyg_thread_delay(3);
			if(0 && serial_info.cur_status != READ_ALL_REGS)
			{
				for(i = 0; i < send_byte; i++ )
				{   
					debug_print2(( " 0x%02x, ", (unsigned char)(recv_send_buf[i]) ));
				}
				debug_print2(("   ##serial thread state %s write %d byte ok\n",state_str[serial_info.cur_status],serial_info.send_count));		
			}

			cyg_io_get_config(ser, CYG_IO_GET_CONFIG_SERIAL_OUTPUT_FLUSH, &recv_send_buf[0], &len);

			hal_gpio_pin_clear( CYGHWR_HAL_KINETIS_PIN(E, 4, 1, KINETIS_PIN_PULLUP) );

			memset(tmp_buf,0,sizeof(tmp_buf));

			stamp = timeout(100, do_abort, ser);
			recv_byte = 2;
			cyg_io_read(ser,tmp_buf,&recv_byte);			
			untimeout(stamp);
			if( tmp_buf[0] == 0x01 && tmp_buf[1] == 0x10  )
			{
				  stamp = timeout(100, do_abort, ser);
			      recv_byte = 6;
                    	cyg_io_read(ser, &tmp_buf[2], &recv_byte);
			      recv_byte = 2+6;
				  untimeout(stamp);
			}
			else if( tmp_buf[0] == 0x01 && tmp_buf[1] == 0x90  )
			{
				  stamp = timeout(100, do_abort, ser);
			      recv_byte = 3;
                        cyg_io_read(ser, &tmp_buf[2], &recv_byte);	
		          recv_byte = 2+3;
				  untimeout(stamp);
				  Rs485ErrorHandle();
			}
			else if( tmp_buf[0] == 0x01 && tmp_buf[1] == 0x03  )
			{
				  stamp = timeout(100, do_abort, ser);
			      recv_byte = 29+12;
                        cyg_io_read(ser, &tmp_buf[2], &recv_byte);	
		          recv_byte = 2+29+12;
				  untimeout(stamp);
			}			
			else
			{
                diag_printf( "<wrong bytes>: 0x%02x, 0x%02x, rs485 bus maybe disconnect!!\n", 
				 (unsigned char)(tmp_buf[0]),
				  (unsigned char)(tmp_buf[1]) );
			}
			if(serial_info.cur_status != READ_ALL_REGS)
			{
				for(i = 0; i < recv_byte;++i)
				{
					debug_print2((" 0x%02x, ", (unsigned char)(tmp_buf[i]) ));
				}
				debug_print2(("    ##serial thread state %s read %d byte ok\n",state_str[serial_info.cur_status],recv_byte));
			}
		
			recv_send_buf[0] = serial_info.cur_status;
		        memcpy((void *)&recv_send_buf[1], tmp_buf, recv_byte);
			BlockBufferWrite(serial_info.rs485_cmd_recv_buffer, recv_send_buf, recv_byte+1);

			cyg_io_get_config(ser, CYG_IO_GET_CONFIG_SERIAL_INPUT_FLUSH, &recv_send_buf[0], &len);

			memset(tmp_buf,0,sizeof(tmp_buf));
			memset(recv_send_buf,0,sizeof(recv_send_buf));
			serial_info.cur_status = IDLE;

			PthreadPriorityChangeForSchedRr(pthread_self(), BCU_PRIORIT); ///< down
		}
		
		sched_yield();
	}

	return 0;
}


/*---------------------------end of bcu beta 1.1-------------------------------*/

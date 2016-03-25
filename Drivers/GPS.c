#include "at91sam7s256.h"
#include "GPS.h"
#include "../Main/global.h"
#define GPS_BUFFER_SIZE 160
#if OUTDOOR
char gps_buffer0[GPS_BUFFER_SIZE];//change bigger?
char gps_buffer1[GPS_BUFFER_SIZE];
unsigned char gpsbuffernumber=0;
unsigned int gps_buf_ptr=0;
//stores the array offset of the last received byte
unsigned int l_buf_ptr=0;
unsigned char buf_ptr_buffer=0;
unsigned char l_buf_ptr_buffer=0;
unsigned int package_len=0;
//length between two timouts
unsigned char buffer_change_times=0;
//cleared at timeout, ++ during dma interrupt
void gps_init(void)
{
#if OUTDOOR
	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	*AT91C_PMC_PCER|=(1<<AT91C_ID_US1);		
	*AT91C_PIOA_PDR|=0x00600000;		
	*AT91C_PIOA_ASR|=0x00600000;		
	*AT91C_US1_CR=0XAC;				
	*AT91C_US1_MR=0X8C0;
	*AT91C_US1_RTOR=20;			
	*AT91C_US1_IER=0x00;
	*AT91C_US1_IDR=0xFFFF;	
	*AT91C_US1_IER=AT91C_US_ENDRX;
	*AT91C_US1_IER |= AT91C_US_TIMEOUT;			
#if ORIGINAL_FREQ
#if LEA6H_GPS
	*AT91C_US1_BRGR=78;
#elif M8N_GPS
	*AT91C_US1_BRGR=312;
#endif
#elif DOUBLED_FREQ
	*AT91C_US1_BRGR=156;
#endif		
	*AT91C_US1_RPR=(unsigned int)gps_buffer0;
	*AT91C_US1_RCR=GPS_BUFFER_SIZE;
	*AT91C_US1_RNPR=(unsigned int)gps_buffer1;
	*AT91C_US1_RNCR=GPS_BUFFER_SIZE;
	pAIC->AIC_SMR[AT91C_ID_US1] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 6;
	pAIC->AIC_SVR[AT91C_ID_US1] = (unsigned long) gps_int_handler;
	pAIC->AIC_ICCR |= (1 << AT91C_ID_US1); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_US1); 
	*AT91C_US1_CR=0X10;	//only receiving enabled
	*AT91C_US1_PTCR=AT91C_PDC_RXTEN|AT91C_PDC_TXTDIS;
#endif
}

float char_to_float(char *a)//transfer ascII code(with)
{
	float value=0.0;
	int i=0;
	int j=1;
	int sign=0;
	float decade=1;
	int point=-1;//if there is no decimal point, point==-1
	if(a[0]=='-')
		sign=1;	
	for(i=0;i<14;i++){ //find decimal position
	 	if(a[i+sign]=='.'){
		   point=i;
		   break;
		}
	}
	for(j=1;j<(point);j++){//find the decade for max bit
		decade=decade*10;
	}
	for(i=0;i<point;i++){		
	   value+=(a[i+sign]-48)*decade;
	   decade=decade*0.1f;
	}//now we have got the int part
	decade=0.1;

	for(i=point+1;i<14;i++){
	 	if(a[i+sign]!=0){
			value+=(a[i+sign]-48)*decade;
			decade=decade*0.1;
		}
		else{
			break;
		}
	}
	if(sign)
		value=-value;
	return value;
}
void get_gps_data(void)//read the data we want in gps_buffer[160]
{
#if OUTDOOR
	char string[15];//to store ascII of a data between two ","
	int string_offset=0;
	int received_type=0;//the type of the data between two ","
	int frame=0;//GGA or RMC
	int gps_temp_data[2]={0,0};//to store unsigned LAT and LON, and wait for the NSEW letter
	unsigned char gps_in;
	unsigned int i,j=0;
	unsigned int reading_ptr = l_buf_ptr;
	unsigned char reading_buf = l_buf_ptr_buffer;	
	for(j=0;j<package_len;j++){
		reading_ptr++;
		if(reading_ptr > GPS_BUFFER_SIZE - 1){
			reading_ptr = 0;
			reading_buf = (!reading_buf) & 1;//either 0 or 1
		}
		if(reading_buf == 0){
			gps_in=gps_buffer0[reading_ptr]; 
		}
		else{
			gps_in=gps_buffer1[reading_ptr]; 
		}		
		if(gps_in=='$'){
			string_offset=0;
			received_type=0;
		}
		else if(gps_in==','){//deal with one string
			string_offset=0;		
			if(received_type==0){
			#if LEA6H_GPS
				if(string[0]=='G'&&string[1]=='P'&&string[2]=='G'
				&&string[3]=='G'&&string[4]=='A')
					frame=GPGGA;
				else if(string[0]=='G'&&string[1]=='P'&&string[2]=='R'
				&&string[3]=='M'&&string[4]=='C')
					frame=GPRMC;
				else 
				 	frame=OTH_FRM;
			#elif M8N_GPS
				if(string[0]=='G'&&string[1]=='N'&&string[2]=='G'
				&&string[3]=='G'&&string[4]=='A')
					frame=GPGGA;	
				else if(string[0]=='G'&&string[1]=='N'&&string[2]=='R'
				&&string[3]=='M'&&string[4]=='C')
					frame=GPRMC;
				else 
				 	frame=OTH_FRM;
			#endif
			}
			if(frame==OTH_FRM)
				continue;
			else if(frame==GPGGA){
				switch(received_type){
				case 6:
				//gps status: 0 not positioning, 1 non-diff positioning, 
				//2 diff positioning, 3 invalid PPS, 6 estimating
					gps.status=(string[0]-48);
					break;
				case 7://# of sat in use (00~12)
					gps.sat=(string[0]-48)*10+(string[1]-48);
					break;
				case 9://altitude -9,999.9 ~ 99,999.9
					gps.alt=char_to_float(string)*1000;//unit of mm
					break;
				default:
					break;
				}
			}
			else if(frame==GPRMC){
				switch(received_type){
				case 3:
				//latitude ddmm.mmmmm, 0.00001m correspond to 18mm
					gps_temp_data[LAT]=
					(string[0]-48)*100000000
					+(string[1]-48)*10000000
						+((string[2]-48)*10000000
						+(string[3]-48)*1000000
						+(string[5]-48)*100000
						+(string[6]-48)*10000
						+(string[7]-48)*1000
						+(string[8]-48)*100
						+(string[9]-48)*10)/6;//10^7deg
					break;
				case 4://latitude NS
					if(string[0]=='N'){
						gps.lat=gps_temp_data[LAT];
					}
					else if(string[0]=='S'){
						gps.lat=-gps_temp_data[LAT];
					}
					break;
				case 5://longitude dddmm.mmmmm
					gps_temp_data[LON]=
					(string[0]-48)*1000000000
					+(string[1]-48)*100000000
					+(string[2]-48)*10000000
						+((string[3]-48)*10000000
						+(string[4]-48)*1000000
						+(string[6]-48)*100000
						+(string[7]-48)*10000
						+(string[8]-48)*1000
						+(string[9]-48)*100
						+(string[10]-48)*10)/6;
					break;
				case 6://longitude EW			
					if(string[0]=='E'){
						gps.lon=gps_temp_data[LON];
					}
					else if(string[0]=='W'){
						gps.lon=-gps_temp_data[LON];
					}
					break;
				case 7://velocity Knots
					gps.vel=char_to_float(string)*514.4f;//1 knot = 0.5144m/s = 514.4mm/s
					break;
				case 8://azimuth deg			
					gps.azm=char_to_float(string) * DEG2RAD;
					break;
				default:
					break;
				}
			}
			received_type++;
			string_offset=0;
			for(i=0;i<15;i++){
				string[i]='0';
			}
		}
		else if(gps_in=='\r'||gps_in=='\n'){
		}
		else if(gps_in=='*'){
		}
		else{
			if(string_offset<15)
				string[string_offset]=gps_in;
			string_offset++;
		}
	}
#endif
}
#if OUTDOOR
__irq void gps_int_handler(void)
{
	if(*AT91C_US1_CSR & AT91C_US_ENDRX){//change DMA buffer
		buffer_change_times++;
		if (gpsbuffernumber==0){
			gpsbuffernumber=1;
   	   		*AT91C_US1_RNPR=(unsigned int)gps_buffer0;
			*AT91C_US1_RNCR=GPS_BUFFER_SIZE;
		}
		else{
			gpsbuffernumber=0;
			*AT91C_US1_RNPR=(unsigned int)gps_buffer1;
			*AT91C_US1_RNCR=GPS_BUFFER_SIZE;
		}
	}
	if(*AT91C_US1_CSR & AT91C_US_TIMEOUT){//one package fully received
		*AT91C_US1_CR|=AT91C_US_STTTO;		
		l_buf_ptr = gps_buf_ptr;
		l_buf_ptr_buffer = buf_ptr_buffer;
		gps_buf_ptr = GPS_BUFFER_SIZE - *AT91C_US1_RCR - 1;
		buf_ptr_buffer = gpsbuffernumber;
		package_len = gps_buf_ptr + buffer_change_times * GPS_BUFFER_SIZE - l_buf_ptr;		
		buffer_change_times = 0;
		if(package_len < 2 * GPS_BUFFER_SIZE)
			gps.gpsflag=1;
	}
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_US1);
	*AT91C_AIC_EOICR=1;
}
#endif
#endif


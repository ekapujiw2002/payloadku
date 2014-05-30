/* 
 * File:   main.c
 * Author: EX4
 * 
 * Created on January 31, 2013, 3:49 AM
 * Note on HM TRP Module :
 * When the serial data rate is below 9600bps,HM-TR module supports continuous transmission and the maximum data stream can reach 1000000bytes; however, the data transmitted each time should not exceed 32bytes in high-speed applications(>9600bps). HM-TR module work in half-duplex mode. When receiving 32 Bytes from the serial port, it will send data out at once. If the data package received is below 32 Bytes, the module will wait for about 30 ms and then send it. In order to send data immediately, 32 Bytes data per transmission is necessary. After each transmission, HM-TR module will be switched to receiver mode automatically. The switch time is about 5ms. 
 */

//linux or windows
#define VERBOSE         1       	//show status or not
#define KEYBOARD_TEST   0       	//test using keyboard or not

//standar lib
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/unistd.h>

//ini parser
#include "iniparser.h"

//rs232 lib
#include "rs232.h"

//opencv
#include <cv.h>
#include <highgui.h>

#ifdef __linux__
	#include <wiringPi.h>
	//#include <wiringSerial.h>

	#include "libNativeI2C.h"
	#include "adxl345.h"
	#include "itg3200.h"
	#include "i2c_gps.h"
#endif

//typedef

typedef enum {
    STATE_IDLE,
    STATE_GET_G_DATA,
    STATE_GET_CAM_DATA,
    STATE_EXIT,
    STATE_GET_STATUS,
    STATE_SHUTDOWN_OS,
    STATE_RESTART_OS,
    STATE_REINIT_PERIPHERAL,
    STATE_READ_WRITE_CONFIGURATION
} STATE_OPS_ENUM;

//periferal bit status
typedef enum {
	BIT_CAM_READY,
	BIT_I2C_BUS_READY,
	BIT_ADXL345_READY,
	BIT_ITG3200_READY
} STATUS_PERIPH_ENUM;

/*
typedef enum {
    CMD_EMPTY,
    CMD_STOP,
    CMD_GET_G_DATA,
    CMD_GET_CAM_DATA,
    CMD_GET_STATUS
} CMD_TYPE_ENUM;
 */

//titel info
#define ID_NAME			"MDP"
#define APP_TITEL       "KOMURINDO 2014 MDP BLUE SKY\r\n"
#define	CFG_NAME		"cfg.ini"	//file config
#define	COMMA			","

//delay buffer rf module
#define	RF_DELAY_US				4000
#define RF_DELAY_LESS_PACKET_US	35000
#define RF_DELAY_HALF_DUPLEX_US	12000
#define	RF_BUFFER_NUM			128	//64 byte buffer with 10ms delay per packet

//com port
#define COM_BUFF_NUM    30
#define COM_SPEED       57600

#ifdef __linux__
#define COM_PORT        22        //22 utk ttyAMA0
#else
#define COM_PORT        0
#endif

//i2c sensors
#define adxl345_addr 	0x53
#define itg3200_addr 	0x68
#define i2c_dev			"/dev/i2c-1"

//command list
#define CMD_COUNTER     9
//unsigned char cmdList[CMD_COUNTER][2] = {"i", "g", "c", "x", "s", "o", "r", "z", "f"};
unsigned char cmdList[CMD_COUNTER] = {'i', 'g', 'c', 'x', 's', 'o', 'r', 'z', 'f'};

//global var
unsigned char opsState = STATE_IDLE;
unsigned char statusPeripheral = 0;

//proto void
void SaveRGBToFile(char* OutFileName, char* ImageFile);
void SetCameraFrameSize(CvCapture* aCamHandle, int frameWidth, int frameHeight);
void GrabAndSendCameraData(CvCapture* aCamHandle, int i2c_handle, int skippedFrame, int comportHandle, unsigned int idPayloadNum, GPS_DATA *last_data, unsigned char rf_buffer_len); //comportHandle ignored for windows version
void ADXL345_Init_Start(int i2c_bus_handle, unsigned char sensor_addr, unsigned char *statusPeripheralOutput);
void Cam_Init_Start(CvCapture *camHdlOutput, unsigned char *statusPeripheralOutput);
void GetAndSendAccGyro(int i2c_handle, int i2c_acc_addr, int i2c_gyro_addr, unsigned int idPayloadNum, unsigned int delaySend, GPS_DATA *last_data, unsigned char rf_buffer_len );
int I2C_Init_Bus(char *dev_i2c_name, unsigned char *statusPeripheralOutput);
void ITG3200_Init_Start(int i2c_bus_handle, unsigned char sensor_addr, unsigned char *statusPeripheralOutput);
unsigned char GetAndFormatGPSData(int i2c_bus_handle, unsigned char* gps_str_out, GPS_DATA *last_gps_data);
int file_exist(char *filename);

//check if file exist
int file_exist(char *filename)
{
  struct stat   buffer;   
  return (stat (filename, &buffer) == 0);
}

//save rgb of picture to file

void SaveRGBToFile(char* OutFileName, char* ImageFile) {
    //write and parse jpg to extract the rgb value
    FILE *file;
    file = fopen(OutFileName, "w+");

    IplImage* image = cvLoadImage(ImageFile, CV_LOAD_IMAGE_COLOR);
    int x, y;
    for (y = 0; y < image->height; y++) {
        unsigned char* row = &CV_IMAGE_ELEM(image, unsigned char, y, 0);

        fprintf(file, "%c%.3d", 255, (y + 1));
        for (x = 0; x < image->width * image->nChannels; x += image->nChannels) {
            //            row[x] = {R,G,B};
            fprintf(file, "%c%c%c", row[x], row[x + 1], row[x + 2]);
        }
    }

    fclose(file); /*done!*/
}

//setup camera frame

void SetCameraFrameSize(CvCapture* aCamHandle, int frameWidth, int frameHeight) {
    cvSetCaptureProperty(aCamHandle, CV_CAP_PROP_FRAME_HEIGHT, frameHeight);
    cvSetCaptureProperty(aCamHandle, CV_CAP_PROP_FRAME_WIDTH, frameWidth);
}

//send camera data to serial port

void GrabAndSendCameraData(CvCapture* aCamHandle, int i2c_handle, int skippedFrame, int delayPixelSending, unsigned int idPayloadNum, GPS_DATA *last_data, unsigned char rf_buffer_len) {
    unsigned char frameBuff[645*200], buff_gps_data[40];
    unsigned int i, idPixel;

    //capture camera, skip n frame
    IplImage *img = 0;
    uint8_t cntSkipFrame = 0;
    while (cntSkipFrame < skippedFrame) {
        //grab frame
        img = cvQueryFrame(aCamHandle);
        if (img) cntSkipFrame++;
    }

    //get picture and resize it
    //cvSaveImage("/dev/shm/capture.jpg", img, 0);

    //create image to store
    IplImage *dest = cvCreateImage(cvSize(200, 200), img->depth, img->nChannels);

    //resize it
    cvResize(img, dest, CV_INTER_LANCZOS4);

    //save the resized image
    //~ cvSaveImage("resized.jpg", dest, 0);

    //view pixel info 
    int hcnt, wcnt;

    //method 1:
    //print header paket gambar
    snprintf((char*) frameBuff, 10, "\r%.3d\xff\r", idPayloadNum);
    SendBuf(COM_PORT, frameBuff, 7);
    usleep(RF_DELAY_LESS_PACKET_US);
    
    //print pixel
    idPixel = 0;
    for (hcnt = 0; hcnt < dest->height; hcnt++) { //loop height
        unsigned char* row = &CV_IMAGE_ELEM(dest, unsigned char, hcnt, 0); //row scanline

        //print header frame
        //~ uchar dataTX[4] = {0xff}; //header data
        //~ dataTX[1] = ((hcnt + 1) / 100) + 0x30; //counter height
        //~ dataTX[2] = (((hcnt + 1) % 100) / 10) + 0x30;
        //~ dataTX[3] = ((hcnt + 1) % 10) + 0x30;        
        
        frameBuff[idPixel++] = 0xff;
        frameBuff[idPixel++] = ((hcnt + 1) / 100) + 0x30; //counter height
        frameBuff[idPixel++] = (((hcnt + 1) % 100) / 10) + 0x30;
        frameBuff[idPixel++] = ((hcnt + 1) % 10) + 0x30;

#if VERBOSE==1
        printf("Processing frame #%d\r\n", hcnt + 1);
#endif        

        //send header
        //~ SendBuf(COM_PORT, dataTX, 4);
        //~ usleep(RF_DELAY_LESS_PACKET_US);

        //loop per width
        for (wcnt = 0; wcnt < dest->width * dest->nChannels; wcnt += dest->nChannels) {
			// opencv rgb data sequence = BGR
            //            row[x] = {R,G,B};
			//~ frameBuff[wcnt + 4] = (row[wcnt + 2] == 0xff) ? 0xfe : row[wcnt + 2];
            //~ frameBuff[wcnt + 1 + 4] = (row[wcnt + 1] == 0xff) ? 0xfe : row[wcnt + 1];
            //~ frameBuff[wcnt + 2 + 4] = (row[wcnt + 0] == 0xff) ? 0xfe : row[wcnt + 0];
            frameBuff[idPixel++] = (row[wcnt + 2] == 0xff) ? 0xfe : row[wcnt + 2];
            frameBuff[idPixel++] = (row[wcnt + 1] == 0xff) ? 0xfe : row[wcnt + 1];
            frameBuff[idPixel++] = (row[wcnt + 0] == 0xff) ? 0xfe : row[wcnt + 0];
			
            //~ SendByte(COM_PORT, (row[wcnt + 2] == 0xff) ? 0xfe : row[wcnt + 2]);
            //usleep(5);
            //~ SendByte(COM_PORT, (row[wcnt + 1] == 0xff) ? 0xfe : row[wcnt + 1]);
            //usleep(5);
            //~ SendByte(COM_PORT, (row[wcnt + 0] == 0xff) ? 0xfe : row[wcnt + 0]);
            //usleep(5);
            
            //delay for buffer radio
            //~ if(wcnt%32==0)
				//~ usleep(delayPixelSending);
        }

        //send frame package
        //SendBuf(COM_PORT, frameBuff, 600);

        //delay for transceiver
        //usleep(delayPixelSending);
        
        //get gps data
		//~ GetAndFormatGPSData(i2c_handle, buff_gps_data, last_data);
		GetAndFormatGPSData(i2c_handle, &frameBuff[idPixel], last_data);
	
		//send the gps data
		//~ SendBuf(COM_PORT, buff_gps_data, 18); 
		//~ usleep(2000);
		//~ SendBuf(COM_PORT, &buff_gps_data[18], 19); 
		//~ usleep(RF_DELAY_US);
		
		//delay buffer radio
		//~ usleep(delayPixelSending+2000);
		
		//tail
		//~ snprintf(buff_gps_data, 5, " INA");
		idPixel += 37;
		snprintf(&frameBuff[idPixel], 6, " %s", "GPS");
		idPixel += 4;
		
		//send the data
		//~ SendBuf(COM_PORT, buff_gps_data, 4); 
		
		//~ for(i=0;i<672;i+=32)
		//~ {
			//~ SendBuf(COM_PORT, &frameBuff[i], 32); 
			//~ usleep(RF_DELAY_HALF_DUPLEX_US);
		//~ }
		
		//~ for(i=0;i<672;i++)
		//~ {
			//~ SendByte(COM_PORT, frameBuff[i]); 
			//~ if(i%32==0)
				//~ usleep(RF_DELAY_HALF_DUPLEX_US);
		//~ }
		//~ usleep(50000);
		
		//~ printf("F#%.3d = %.3d\r\n", hcnt, idPixel);
		
		//~ if(hcnt==0) break;
    }
    
    //save to file
    //~ FILE *fp;
	//~ fp=fopen("raw_cam.log", "wb+");
	//~ fwrite(
		//~ frameBuff, 
		//~ sizeof(frameBuff[0]), 
		//~ /* sizeof(frameBuff)/sizeof(frameBuff[0]) */
		//~ idPixel, 
		//~ fp);
	//~ fclose(fp);
    
    //send all data
    printf("Going to send %ld byte with delay %d us\r\n", idPixel, delayPixelSending);    
    for(i=0;i<(idPixel);i++)
    {
		SendByte(COM_PORT, frameBuff[i]); 
		if(i%(rf_buffer_len-1)==0)
			//~ usleep(RF_DELAY_HALF_DUPLEX_US);
			usleep(delayPixelSending);
	}
    
    usleep(RF_DELAY_LESS_PACKET_US);
}

/*
 * send g and gyro data
 * */

void GetAndSendAccGyro(int i2c_handle, int i2c_acc_addr, int i2c_gyro_addr, unsigned int idPayloadNum, unsigned int delaySend, GPS_DATA *last_data, unsigned char rf_buffer_len )
{
	unsigned int gxvalue, gyvalue, gzvalue, gyrox, gyroy, gyroz;
	unsigned char buff_data[100], buff_gps_data[40];
	
	//tes clock length
	//clock_t c0,c1,c2;
	//double el1,el2;
	
	//c0 = clock();
	
	//set buff to all 0
	memset(buff_data,0,100);
	
	//get g data
	adxl345_get_gdata(i2c_handle, i2c_acc_addr, &gxvalue, &gyvalue, &gzvalue);
	
	//c1 = clock();
	
	//get gyro
	itg3200_get_gyro_data(i2c_handle, i2c_gyro_addr, &gyrox, &gyroy, &gyroz);
	
	//c2 = clock();
	
	//duration
	//el1 = ((double) c1-c0) / CLOCKS_PER_SEC;
	//el2 = ((double) c2-c1) / CLOCKS_PER_SEC;
	//printf("t1 = %f\t t2 = %f\n", el1, el2);
	
	//print g and gyro data
	//printf("\r%.3d %.3d %.3d %.3d %.3d %.3d %.3d %.3d",idPayloadNum, adxl345_normalize_gdata(adxl345_calc_gvalue(gxvalue), 0, 999) , adxl345_normalize_gdata(adxl345_calc_gvalue(gyvalue), 0, 999), adxl345_normalize_gdata(adxl345_calc_gvalue(gzvalue),0,999),itg3200_normalize_gyrodata(itg3200_calc_gyro_value(gyrox), 0, 999) , itg3200_normalize_gyrodata(itg3200_calc_gyro_value(gyroy), 0, 999), itg3200_normalize_gyrodata(itg3200_calc_gyro_value(gyroz),0,999), idPayloadNum);
	
	//c0 = clock();
	
	//format the data header+gyro acc
	snprintf(buff_data, 34, "%c%.3d %.3d %.3d %.3d %.3d %.3d %.3d ",
	13,
	idPayloadNum, 
	adxl345_normalize_gdata(adxl345_calc_gvalue(gxvalue), 0, 999) , 
	adxl345_normalize_gdata(adxl345_calc_gvalue(gyvalue), 0, 999), 
	adxl345_normalize_gdata(adxl345_calc_gvalue(gzvalue),0,999),
	itg3200_normalize_gyrodata(itg3200_calc_gyro_value(gyrox), 0, 999) , 
	itg3200_normalize_gyrodata(itg3200_calc_gyro_value(gyroy), 0, 999), 
	itg3200_normalize_gyrodata(itg3200_calc_gyro_value(gyroz),0,999)
	);
	
	//c1 = clock();
	
	//~ printf("%s", buff_data);
	
	//send the data
	//~ SendBuf(COM_PORT, buff_data, 29); 
	//~ cprintf(COM_PORT, buff_data);
	
	//~ usleep(delaySend);
	
	//get gps data
	//~ GetAndFormatGPSData(i2c_handle, buff_gps_data, last_data);
	GetAndFormatGPSData(i2c_handle, &buff_data[29], last_data);
	
	//send the gps data
	//~ SendBuf(COM_PORT, buff_gps_data, 37); 
	
	//~ usleep(RF_DELAY_US);
	
	//tail
	//~ snprintf(buff_data, 5, " %.3d",	idPayloadNum);
	//~ snprintf(&buff_data[64], 35, " %26.3s%4.3dX", ID_NAME, idPayloadNum);
	snprintf(&buff_data[66], 35, " %3.3s%4.3d", ID_NAME, idPayloadNum);
	
	//send the data
	//~ SendBuf(COM_PORT, buff_data, 4); 
	
	//c2 = clock();
	//el1 = ((double) c1-c0) / CLOCKS_PER_SEC;
	//el2 = ((double) c2-c1) / CLOCKS_PER_SEC;
	//printf("t1 = %f\t t2 = %f\n", el1, el2);
	
	//~ buff_data[0] = '\n';
	//~ printf("%s\n", buff_data);
	//~ buff_data[0] = '\r';
	
	unsigned int i;
	//~ for(i=0;i<96;i+=32)
	//~ {
		//~ SendBuf(COM_PORT, &buff_data[i], 32); 
		//~ usleep(delaySend);
	//~ }

	for(i=0;i<74;i++)
    {
		SendByte(COM_PORT,  buff_data[i]); 
		//~ if(i%(rf_buffer_len-1)==0)
		if(i%(16-1)==0)
			usleep(delaySend);
	}
	
	//~ SendByte(COM_PORT,  13); 
	//~ SendByte(COM_PORT,  'A'); 
	
	usleep(delaySend);
}

//bus i2c init
int I2C_Init_Bus(char *dev_i2c_name, unsigned char *statusPeripheralOutput)
{
	int i2c_hdlx;
	
	if((i2c_hdlx = openBus(dev_i2c_name)) < 0)	//fail opening
	{
		printf("Failed opening i2c device");
		//return errno;
		*statusPeripheralOutput &= ~(1<<BIT_I2C_BUS_READY);
	}
	else
	{
		*statusPeripheralOutput |= (1<<BIT_I2C_BUS_READY);
		
	#if VERBOSE==1
		printf("i2c port opened\r\n");
	#endif 
	}
	
	return i2c_hdlx;
}

//init and start adxl345
void ADXL345_Init_Start(int i2c_bus_handle, unsigned char sensor_addr, unsigned char *statusPeripheralOutput)
{
if(i2c_bus_handle>=0)
{
	//init adxl
	if(!adxl345_init(i2c_bus_handle, sensor_addr))
	{
		printf("init adxl fail\n");
		//return errno;
		*statusPeripheralOutput &= ~(1<<BIT_ADXL345_READY);
	}
	else
	{
		*statusPeripheralOutput |= (1<<BIT_ADXL345_READY);
	}
#if VERBOSE==1
	printf("adxl345 init success\r\n");
#endif	
	
	//start adxl
	if(!adxl345_start(i2c_bus_handle, sensor_addr))
	{
		printf("cannot start adxl\n");
		//return errno;
		*statusPeripheralOutput &= ~(1<<BIT_ADXL345_READY);
	}
	else
	{
		*statusPeripheralOutput |= (1<<BIT_ADXL345_READY);
	}
#if VERBOSE==1
	printf("adxl345 started\r\n");
#endif	
}
else
{
#if VERBOSE==1
	printf("i2c bus not ready\r\n");
#endif	
}
}

//init dan start itg3200
void ITG3200_Init_Start(int i2c_bus_handle, unsigned char sensor_addr, unsigned char *statusPeripheralOutput)
{
if(i2c_bus_handle>=0)
{
	//init itg3200
	if(!itg3200_init(i2c_bus_handle, sensor_addr))
	{
		printf("init itg3200 fail\n");
		//return errno;	
		*statusPeripheralOutput &= ~(1<<BIT_ITG3200_READY);
	}	
	else
	{
		*statusPeripheralOutput |= (1<<BIT_ITG3200_READY);
	}
#if VERBOSE==1
	printf("init itg3200 success and started\r\n");
#endif
}
else
{
#if VERBOSE==1
	printf("i2c bus not ready\r\n");
#endif
}
}

//init cam
void Cam_Init_Start(CvCapture *camHdlOutput, unsigned char *statusPeripheralOutput)
{
	//open cam
    if (!camHdlOutput) { //check fail or not
        fprintf(stderr, "Open cam failed : %s!!!\r\n!!!", strerror(errno));
        //return errno;
        *statusPeripheralOutput &= ~(1<<BIT_CAM_READY);
    }
    else
    {
		*statusPeripheralOutput |= (1<<BIT_CAM_READY);
		
#if VERBOSE==1
    printf("camera opened\r\n");
#endif    

    //setup the camera
    SetCameraFrameSize(camHdlOutput, 320, 240);

#if VERBOSE==1
    printf("camera set to 320x240\r\nready for command\r\n");
#endif 
	}   
}

//save ini file
void Save_INI_File(dictionary *iniDictIn, char *fName)
{
	//dump dict content to the file
	FILE *fp;	
	fp = fopen(fName,"w+");
	if(fp)
	{
		iniparser_dump_ini(iniDictIn, fp);
		fclose(fp);
	}	
}

/*
 * read token value
 * */
 
char* Read_Token_Value(int lenIn, char *strIn, int numToken, char *delimiter){
	char *tok;
	char cpyIn[lenIn];
	int idx = 1;
	
	memcpy(cpyIn, strIn, lenIn);
	
	//printf("%d = %s\n", lenIn, cpyIn);
	
	tok = strtok(cpyIn, delimiter);
	if(numToken>1)
	{
		while(idx!=numToken)
		{
			idx++;
			tok = strtok(NULL, delimiter);
		}
	}
	
	return (tok!=NULL) ? tok:"";
}

/*
 * cek for command
 * */
char CheckCommand(char *cmd_list_def, int input_number, char *input_data) 
{
	char cmdx = 0;
	int i,j;
	
	for(i=0;i<CMD_COUNTER;i++)
	{
		for(j=0;j<input_number;j++)
		{
			if(input_data[j]==cmd_list_def[i])
			{
				cmdx = i;
				break;
			}
		}
	}	
	
	return cmdx;
}

/*
 * get and format gps data
 * from i2cgps
 * format to char array :
 * status lat lon height timestamp
 * status		= 2byte hex
 * lat			= 8byte hex
 * lon			= 8byte hex
 * altitude		= 2byte hex
 * timestamp	= 8byte hex
 * speed		= 2byte hex
 * separator spasi
 * total byte	= 35
 * */
unsigned char GetAndFormatGPSData(int i2c_bus_handle, unsigned char* gps_str_out, GPS_DATA *last_gps_data)
{
	GPS_DATA gpsdtx;
	
	//init result
	gpsdtx.gps_status = gpsdtx.gps_loc.lat = gpsdtx.gps_loc.lon = 0;
	gpsdtx.gps_speed = gpsdtx.gps_altitude = gpsdtx.gps_time = 0;
	
	//read gps fix
	if((gpsdtx.gps_status = i2c_gps_read_status(i2c_bus_handle)))
	{
		//read speed and altitude
		i2c_gps_read_speed_altitude(i2c_bus_handle, &gpsdtx.gps_speed, &gpsdtx.gps_altitude);
		
		//read lat lon
		i2c_gps_read_lat_lon(i2c_bus_handle, &gpsdtx.gps_loc.lat, &gpsdtx.gps_loc.lon);
		
		//read time
		i2c_gps_read_time(i2c_bus_handle, &gpsdtx.gps_time);
		
		//update new data gps
		*last_gps_data = gpsdtx;
	}	
	else
	{
		gpsdtx = *last_gps_data;
	}
	
	//output to char*
	snprintf(gps_str_out, 40,"%02x %08lx %08lx %04x %08lx %02x", gpsdtx.gps_status, gpsdtx.gps_loc.lat, gpsdtx.gps_loc.lon, gpsdtx.gps_altitude, gpsdtx.gps_time, gpsdtx.gps_speed);
	
	//~ printf("%s\r\n", gps_str_out);
	
	//return value
	return gpsdtx.gps_status;
}

/*
 * main program
 */
int main(int argc, char** argv) {
    //global var
    time_t sec0, sec1;
    unsigned int delayPixel= 120000, delayGyrodataSending = 25000;
    unsigned int idPayload = 303, useKeyboardCmd = 0, buffer_max_num = 64;
    unsigned char keyChar;
    int gpioOK = 0;
	dictionary *iniDict;	//ini dictionery  
	char strPathCfg[255];
	
	GPS_DATA last_posx;
	
	//init last gps pos
	last_posx.gps_status = last_posx.gps_loc.lat = last_posx.gps_loc.lon = 0;
	last_posx.gps_speed = last_posx.gps_altitude = last_posx.gps_time = 0;	
    
    //show titel
#if VERBOSE==1
    printf("%s", APP_TITEL);
#endif

	//init pin switch
	gpioOK = wiringPiSetup();
	
	if(gpioOK != -1)
	{
		pinMode(6, INPUT);
		pullUpDnControl(6, PUD_UP);
	}
	else
		printf("gpio init eror\r\n");
		
	//parsing argumen jika ada
	if(argc>2)
	{
		//default cfg file
		getcwd(strPathCfg, sizeof(strPathCfg));
		strcat(strPathCfg, "/");
		strcat(strPathCfg, CFG_NAME);
	
		int i;
		for(i=1;i<(argc-1);i++)
		{
			//delay pixel
			if(strcmp("-d",argv[i])==0)
				delayPixel = atoi(argv[++i]);
				
			//id payload
			if(strcmp("-i",argv[i])==0)
				idPayload = atoi(argv[++i]);
				
			//pake keyboard ato tidak
			if(strcmp("-k",argv[i])==0)
				useKeyboardCmd = atoi(argv[++i]);
				
			//gyro send
			if(strcmp("-g",argv[i])==0)
				delayGyrodataSending = atoi(argv[++i]);
				
			//buffer size
			if(strcmp("-b",argv[i])==0)
				buffer_max_num = atoi(argv[++i]);
		}
	}
	else //baca dari cfg file
	{				
		//load ini file
		//char strCfg[80];
		
		if(argc==2)
		{
			strcpy(strPathCfg, argv[1]);
		}
		else
		{		
			//get cwd
			if (getcwd(strPathCfg, sizeof(strPathCfg)) != NULL)
			   printf("Current working dir: %s\r\n", strPathCfg);
			else
			{
				strcpy(strPathCfg, argv[1]);			
			}
		}
		
		//check file exists
		strcat(strPathCfg, "/");
		strcat(strPathCfg, CFG_NAME);
		if(!file_exist(strPathCfg))
		{
			printf("Configuration file %s not found!!!\r\n", strPathCfg );
			return(EXIT_FAILURE);
		}
		
		//load from config file
		printf("Loading configuration file from %s\r\n", strPathCfg);
		
		//load ini parser
		iniDict = iniparser_load(strPathCfg);
		
		//read cfg value
		if(iniDict)
		{
			idPayload = iniparser_getint(iniDict,"payload:id",100);
			delayPixel = iniparser_getint(iniDict,"payload:cam_delay",120000);
			delayGyrodataSending = iniparser_getint(iniDict,"payload:g_delay",20000);
			buffer_max_num = iniparser_getint(iniDict,"payload:buffer",64);
		}
	}
	
	//show config setup
	printf("======================================\r\n");
	printf("Configuration :\r\n");
	printf("Delay Pixel = %d uS\r\n", delayPixel);
	printf("ID Payload = %d\r\n", idPayload);
	printf("Use Keyboard = %d\r\n", useKeyboardCmd);
	printf("Gyro Delay = %d uS\r\n", delayGyrodataSending);
	printf("Buffer = %d byte\r\n", buffer_max_num);
	printf("======================================\r\n");    
	
	//init bus i2c
	int i2c_hdl = I2C_Init_Bus(i2c_dev, &statusPeripheral);
	
	//init start adxl345
	ADXL345_Init_Start(i2c_hdl, adxl345_addr, &statusPeripheral);
	
	//init itg3200
	ITG3200_Init_Start(i2c_hdl, itg3200_addr, &statusPeripheral);
	
    //open port
    unsigned char buffRX[COM_BUFF_NUM];
    int buffNumRX;
    
    //make it as null string
    buffRX[COM_BUFF_NUM-1] = 0;

    if (OpenComport(COM_PORT, COM_SPEED)) {
        fprintf(stderr, "Open port %d failed : %s\r\n!!!", COM_PORT, strerror(errno));
        return errno;
    }

#if VERBOSE==1
    printf("communication port opened\r\n");
#endif   

	//tes port com
	//while(1){
		////ambil perintah dari comport
        //buffNumRX = PollComport(COM_PORT, buffRX, (COM_BUFF_NUM - 2));
        //if (buffNumRX > 0) {
			////printf("%d = %s\n", buffNumRX, buffRX);
			//int i;
			//for(i=0;i<buffNumRX;i++)
			//{
				//printf("%X %s", buffRX[i], (i<buffNumRX-1) ? "":"\n");
			//}
			
			//char ss[10];
			//snprintf(ss,6,"%d",(buffRX[1]<<8)+buffRX[2]);
			//printf("id = %s\n", ss);
		//}
	//}

	//open cam
	CvCapture *camHdl = cvCaptureFromCAM(CV_CAP_ANY);
	Cam_Init_Start(camHdl, &statusPeripheral);

    //main loop
    while (1) {
		if(useKeyboardCmd)
		{
			//dummy keyboard input
			printf("press command key\r\n");
			keyChar = getchar();

			//if (keyChar == cmdList[3][0])
				//opsState = STATE_EXIT;

			//if (keyChar == cmdList[2][0])
				//opsState = STATE_GET_CAM_DATA;
				
			//if (keyChar == cmdList[1][0])
				//opsState = STATE_GET_G_DATA;
				
			if (keyChar == cmdList[3])
				opsState = STATE_EXIT;

			if (keyChar == cmdList[2])
				opsState = STATE_GET_CAM_DATA;
				
			if (keyChar == cmdList[1])
				opsState = STATE_GET_G_DATA;
		}

        //ambil perintah dari comport
        buffNumRX = PollComport(COM_PORT, buffRX, (COM_BUFF_NUM - 2));

        //data diterima -> cek printah
        if (buffNumRX > 0) {
            //int i;
            //unsigned char *chrPos;

            ////make it null string
            //if (buffRX[buffNumRX] != 0)
                //buffRX[buffNumRX] = 0;
                
            ////default ops state = idle
            //opsState = STATE_IDLE;

			////add new state
            //for (i = 0; i < CMD_COUNTER; i++) {
                //chrPos = strstr(buffRX,cmdList[i]);

                //if ((chrPos != NULL) && ((chrPos - buffRX + 1) == 1)) { //got a match?
                    ////proses state
                    //opsState = i;

                    //break;
                //}
            //}
            
            opsState = CheckCommand(cmdList, buffNumRX, buffRX);
        }
        
        //cek tobmol
        if(gpioOK != -1)
        {
			//printf("%d\r\n", digitalRead(6));
			//SendByte(COM_PORT, digitalRead(6)+0x30);
			//usleep(1e5);
		if(digitalRead(6) == 0)
		{
			usleep(5e5);
			if(digitalRead(6) == 0)
				opsState = STATE_SHUTDOWN_OS;
		}
	}

        //lakukan proses seuai ops state
        if (opsState == STATE_EXIT) {
#if VERBOSE==1
            printf("exiting...\r\n");
            //fflush(stdout);   
#endif            
            break;
        }
        
        //shutdown os
        if (opsState == STATE_SHUTDOWN_OS)
        {
			printf("shutdown...\r\n");
			break;
		}
		
		//restart os
        if (opsState == STATE_RESTART_OS)
        {
			printf("restart...\r\n");
			break;
		}

		//other state
        switch (opsState) {
            case STATE_IDLE: //idle state
                //SendByte(COM_PORT, 's');
                break;

            case STATE_GET_G_DATA: //ambil data g
                //SendByte(COM_PORT, 'g');
                
                GetAndSendAccGyro(i2c_hdl, adxl345_addr, itg3200_addr, idPayload, delayGyrodataSending, &last_posx, buffer_max_num);
                break;

            case STATE_GET_CAM_DATA://ambil data cam
                //SendByte(COM_PORT, 'c');
#if VERBOSE==1
                printf("grab camera start\r\n");
                sec0 = time(NULL);
#endif                                

                GrabAndSendCameraData(camHdl, i2c_hdl, 5, delayPixel, idPayload, &last_posx, buffer_max_num);                
#if VERBOSE==1
                sec1 = time(NULL);
                printf("grab camera finish in %lds\r\n", (sec1 - sec0));
#endif                
				opsState = STATE_IDLE;
				
				//usleep(5e4);
				//ComportFlush(COM_PORT);
				//usleep(1.5e6);
                break;

            case STATE_GET_STATUS: //ambil status payload
                snprintf((char*)buffRX, 16, "s,%.3d,%d,%d\r", idPayload, 990, statusPeripheral);
                //SendBuf(COM_PORT, buffRX, 10);
                //snprintf((char*)buffRX, 16, "s,%.3d,%d\r", idPayload, 990);
                cprintf(COM_PORT, buffRX);
                opsState = STATE_IDLE; //go back to idle
                
                #if VERBOSE==1
                printf("sendstatus reply = %s\r\n", buffRX);
                #endif
                break;
                
            case STATE_REINIT_PERIPHERAL:	//reinit peripheral
				//init cam
				Cam_Init_Start(camHdl, &statusPeripheral);
				
				//init bus i2c
				i2c_hdl = I2C_Init_Bus(i2c_dev, &statusPeripheral);
				
				//init start adxl345
				ADXL345_Init_Start(i2c_hdl, adxl345_addr, &statusPeripheral);
				
				//init itg3200
				ITG3200_Init_Start(i2c_hdl, itg3200_addr, &statusPeripheral);
				
				opsState = STATE_IDLE; //go back to idle
				break;
				
			case STATE_READ_WRITE_CONFIGURATION: //read/write config file
				printf("Opening configuration file %s\r\n", strPathCfg);
				
				//load ini file
				iniDict = iniparser_load(strPathCfg);
				
				//write if neccessary
				if(buffRX[1]==1)	//write setting
				{
					char stmp[10];
					
					//id
					snprintf(stmp,10,"%d", (buffRX[2]<<8)+buffRX[3]);					
					iniparser_set(iniDict,"payload:id",stmp);
					
					//cam delay in ms
					snprintf(stmp,10,"%d", buffRX[4]*1000);					
					iniparser_set(iniDict,"payload:cam_delay",stmp);
					
					//g delay in ms
					snprintf(stmp,10,"%d", buffRX[5]*1000);					
					iniparser_set(iniDict,"payload:g_delay",stmp);
					
					//buffer max
					snprintf(stmp,10,"%d", buffRX[6]);					
					iniparser_set(iniDict,"payload:buffer",stmp);
					
					Save_INI_File(iniDict, strPathCfg);
				}	
				
				//reload ini file
				iniDict = iniparser_load(strPathCfg);
				
				//read cfg value
				if(iniDict)
				{
					idPayload = iniparser_getint(iniDict,"payload:id",100);
					delayPixel = iniparser_getint(iniDict,"payload:cam_delay",120000);
					delayGyrodataSending = iniparser_getint(iniDict,"payload:g_delay",20000);
					buffer_max_num = iniparser_getint(iniDict,"payload:buffer",64);
				}
				
				//send reply always
				snprintf((char*)buffRX, 25, "f,%d,%d,%d,%d\r", idPayload, delayPixel, delayGyrodataSending,buffer_max_num);
                cprintf(COM_PORT, buffRX);
                opsState = STATE_IDLE; //go back to idle
                
                #if VERBOSE==1
                printf("send config reply = %s\r\n", buffRX);
                #endif				
				break;

            default:
                break;
        }
    }

    //release cam
    cvReleaseCapture(&camHdl);

    //release port
    CloseComport(COM_PORT);
    
    //close bus
	closeBus(i2c_hdl);
	
	//free ini handler
	if(iniDict)
		iniparser_freedict(iniDict);
	
	//cek shutdown atau restart
	switch (opsState) 
	{
		case STATE_SHUTDOWN_OS:
			system("sudo shutdown now");
			break;
			
		case STATE_RESTART_OS:
			system("sudo shutdown -r now");
			break;
			
		default:
			break;
	}

    //return
    return (EXIT_SUCCESS);
}

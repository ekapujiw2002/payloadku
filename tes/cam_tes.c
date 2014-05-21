/*
opencv camera test
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

//opencv
#include <cv.h>
#include <highgui.h>

//custom type
//periferal bit status
typedef enum {
	BIT_CAM_READY,
	BIT_I2C_BUS_READY,
	BIT_ADXL345_READY,
	BIT_ITG3200_READY
} STATUS_PERIPH_ENUM;

//global var
unsigned char statusPeripheral = 0;

//proto func and void
void SetCameraFrameSize(CvCapture* aCamHandle, int frameWidth, int frameHeight);
void Cam_Init_Start(CvCapture *camHdlOutput, unsigned char *statusPeripheralOutput);

//detail func and void

//setup camera frame
void SetCameraFrameSize(CvCapture* aCamHandle, int frameWidth, int frameHeight) {
    cvSetCaptureProperty(aCamHandle, CV_CAP_PROP_FRAME_HEIGHT, frameHeight);
    cvSetCaptureProperty(aCamHandle, CV_CAP_PROP_FRAME_WIDTH, frameWidth);
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
    //SetCameraFrameSize(camHdlOutput, 320, 240);
    SetCameraFrameSize(camHdlOutput, 320, 240);

#if VERBOSE==1
    printf("camera set to 320x240\r\nready for command\r\n");
#endif 
	}   
}

/*
 * main program
 */
int main(int argc, char** argv) {
	//open cam
	CvCapture *camHdl = cvCaptureFromCAM(CV_CAP_ANY);
	Cam_Init_Start(camHdl, &statusPeripheral);
	
	//display it
	if(camHdl){
		//prepare var
		IplImage *img = 0;
		
		//create the window
		cvNamedWindow( "cam_display", CV_WINDOW_AUTOSIZE );
		
		//move it
		cvMoveWindow("cam_display", 10, 10);
		
		//display it
		while(1){
			//grab frame
			uint8_t cntSkipFrame = 0;
			
			while(cntSkipFrame<5){
				img = cvQueryFrame(camHdl);
				if (img) cntSkipFrame++;
			}
			
			//view it if valid frame
			if(img){
				cvShowImage("cam_display",img);
				//cvSaveImage("/dev/shm/capture.jpg", img, 0);
				//printf("image saved\r\n");
			}
			
			//get keyboard ESC
			if( (cvWaitKey(10) & 255) == 27 ) break;
		}
	}
	
	//release cam
    cvReleaseCapture(&camHdl);
    
    //destroy all wind
    cvDestroyAllWindows();
    
	//return
	return (EXIT_SUCCESS);
}

#include "controller.h"
#include "sdio_sd.h"
#include "ff.h"
#include "diskio.h"
#include <stdio.h>



volatile FATFS fs;         /* Work area (file system object) for logical drive */
volatile FIL fsrc;         /* file objects */
volatile FRESULT res;
volatile UINT br;

void DiskTest(void)
{

	if(disk_initialize(0)==0)
	{
		  if (f_mount(0, &fs) == FR_OK)
		  {
			  res = f_open( &fsrc , "0:/test.txt" ,  FA_OPEN_EXISTING|FA_WRITE);
			  if (res==FR_OK)
			  {
//			      uint8_t read_stat=0;
			      uint8_t buf[64];
//				  read_stat=f_read(&fsrc, &buf[0], sizeof(buf), &br);
			      uint32_t i=0;
			      for(i=0;i<100000;i++)
			      {
					  sprintf(buf,"%d\n",i);
					  f_write(&fsrc,buf,strlen(buf),&br);
			      }
			      f_close(&fsrc);
			  }
			  else
			  {

			  }
		  }
		  else
		  {

		  }
	}
	else
	{

	}

}


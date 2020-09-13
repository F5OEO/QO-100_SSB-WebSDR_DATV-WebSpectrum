/*
* Web based SDR Client for Pluto
* =============================================================
* Author: F5OEO
*
*   (c) DJ0ABR/F5OEO
*   www.dj0abr.de
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
* 
* pluto.c ... handles the Pluto SDR hardware
* 
*/
#define SDR_PLUTO
#ifdef SDR_PLUTO

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include "qo100websdr.h"
#include "ssbfft.h"
#include "wb_fft.h"
#include "setup.h"
#include <getopt.h>
#include <ctype.h>
#include <termios.h>
#include "ad9363.h"



int gainR = 60;
#ifdef WIDEBAND
    int bwkHz = 8000;   // default BW, possible values: 200,300,600,1536,5000,6000,7000,8000
#else
    int bwkHz = 5000;   // default BW, possible values: 200,300,600,1536,5000,6000,7000,8000
#endif // WIDEBAND


#define PLUTO_RXBUFSIZE (NB_FFT_LENGTH*2)  // no of samples per buffer call


void *plutodevproc(void *pdata);

int init_pluto()
{
    printf("Initialize Pluto hardware\n");
    fmc_init();

    double dqrg = tuned_frequency;
	fmc_set_frequency(dqrg,false);

	//fmc_set_sr(2.3e6,false);//rx
	//fmc_set_sr(2.3e6,true);//tx

	// configure the device
    int samprate = SDR_SAMPLE_RATE;

	
	fmc_set_sr(samprate*8,false);//rx
	fmc_set_sr(samprate*8,true);//tx

	//int HardUpsample = 4; // Decimation factor
	//fmc_load_rrc_filter( 1, HardUpsample,0.25,typelpf,0.25);
	set_int_dec_filter_rates(samprate);
	//fmc_set_sr(samprate,false);//rx
    
	fmc_set_analog_lpf(samprate,false);

	fmc_set_rx_level(gainR);
	
	fmc_initchannelrx(PLUTO_RXBUFSIZE, 8);

	pthread_t plutodev_pid = 0;
    int ret = pthread_create(&plutodev_pid,NULL,plutodevproc, NULL);
    if(ret)
    {
        printf("plutodev_pid: plutodevproc NOT started\n\r");
        return 0;
    } 

    return 1;
}

// clean up SDRplay device
void pluto_shutdown()
{
    
}

//extern double lastsdrqrg;
double lastsdrqrg = 0;

void Pluto_setTunedQrgOffset(int hz)
{
    if(lastsdrqrg == 0) lastsdrqrg = tuned_frequency;
    
    double off = (double)hz;
    lastsdrqrg = lastsdrqrg - off;
    printf("set tuner: new:%f offset:%f\n",lastsdrqrg,off);
    //mir_sdr_SetRf(lastsdrqrg,1,0);
	fmc_set_frequency(lastsdrqrg+SDR_SAMPLE_RATE/2,false);
    printf("set tuner : %.6f MHz\n",lastsdrqrg/1e6);
}

void reset_Qrg_Pluto()
{
    lastsdrqrg = tuned_frequency;
    
    printf("re-tune: %.6f MHz\n",lastsdrqrg/1e6);
    fmc_set_frequency(lastsdrqrg+SDR_SAMPLE_RATE/2,false);
	
}


void *plutodevproc(void *pdata)
{
	short ibuf[PLUTO_RXBUFSIZE];
	short qbuf[PLUTO_RXBUFSIZE];
	
	fprintf(stderr,"Starting Pluto proc thread");
    pthread_detach(pthread_self());
	 while(1)
    {
        
            int numSamples=fmc_rx_samples(ibuf, qbuf);
			if(numSamples!=PLUTO_RXBUFSIZE) fprintf(stderr,"PlutoRx underflow %d",numSamples);
			#ifdef WIDEBAND
			static int Count=0;
			//if(Count++>50)
			{
					wb_sample_processing(ibuf, qbuf, numSamples);
					Count=0;
			}		
			#else
				fssb_sample_processing(ibuf, qbuf, numSamples);
			#endif // WIDEBAND
       
    }

}


void Plutoremove()
{
	fmc_shutdown();
}
#endif // SDR_PLUTO

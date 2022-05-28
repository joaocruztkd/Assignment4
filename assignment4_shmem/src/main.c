/*
 * Paulo Pedreiras, 2022/02
 * Zephyr: Simple thread creation example (2)
 * 
 * One of the tasks is periodc, the other two are activated via a semaphore. Data communicated via sharem memory 
 *
 * Base documentation:
 *      https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/zephyr/reference/kernel/index.html
 * 
 */


#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/adc.h>
#include <drivers/pwm.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>
#include <timing/timing.h>
#include <stdlib.h>
#include <stdio.h>


/** Size of stack area used by each thread (can be thread specific, if necessary)*/
#define STACK_SIZE 1024

/** Thread scheduling priority */
#define thread_A_prio 1
#define thread_B_prio 1
#define thread_C_prio 1

/** Therad periodicity (in ms)*/
#define thread_A_period 1000


/** ADC definitions and includes*/
#include <hal/nrf_saadc.h>
#define ADC_NID DT_NODELABEL(adc) 
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_CHANNEL_ID 1  

#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1 

#define BUFFER_SIZE 1

/* Other defines */
/*#define TIMER_INTERVAL_MSEC 1000  Interval between ADC samples */

/** Refer to dts file */
#define GPIO0_NID DT_NODELABEL(gpio0) 
#define PWM0_NID DT_NODELABEL(pwm0) 
#define BOARDLED1 0x0d
                

/** Create thread stack space */
K_THREAD_STACK_DEFINE(thread_A_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_B_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_C_stack, STACK_SIZE);
  
/** Create variables for thread data */
struct k_thread thread_A_data;
struct k_thread thread_B_data;
struct k_thread thread_C_data;

/** Create task IDs */
k_tid_t thread_A_tid;
k_tid_t thread_B_tid;
k_tid_t thread_C_tid;

/** Global vars (shared memory between tasks A/B and B/C, resp) */
int ab = 0;
int bc = 0;

/** Semaphores for task synch */
struct k_sem sem_ab;
struct k_sem sem_bc;

/** Thread code prototypes */
void thread_A_code(void *argA, void *argB, void *argC);
void thread_B_code(void *argA, void *argB, void *argC);
void thread_C_code(void *argA, void *argB, void *argC);

/** ADC channel configuration */
static const struct adc_channel_cfg my_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_CHANNEL_ID,
	.input_positive = ADC_CHANNEL_INPUT
};

/** Global vars */
struct k_timer my_timer;
const struct device *adc_dev = NULL;
static uint16_t adc_sample_buffer[BUFFER_SIZE];

/** Takes one sample */
static int adc_sample(void)
{
	int ret;
	const struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_ID),
		.buffer = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution = ADC_RESOLUTION,
	};

	if (adc_dev == NULL) {
            printk("adc_sample(): error, must bind to adc first \n\r");
            return -1;
	}

	ret = adc_read(adc_dev, &sequence);
	if (ret) {
            printk("adc_read() failed with code %d\n", ret);
	}	

	return ret;
}



/* Main function */
void main(void) {
    
    /* Welcome message */
    printf("\n\r Illustration of the use of shmem + semaphores\n\r");
    
    /* Create and init semaphores */
    k_sem_init(&sem_ab, 0, 1);
    k_sem_init(&sem_bc, 0, 1);
    
    /* Create tasks */
    thread_A_tid = k_thread_create(&thread_A_data, thread_A_stack,
        K_THREAD_STACK_SIZEOF(thread_A_stack), thread_A_code,
        NULL, NULL, NULL, thread_A_prio, 0, K_NO_WAIT);

    thread_B_tid = k_thread_create(&thread_B_data, thread_B_stack,
        K_THREAD_STACK_SIZEOF(thread_B_stack), thread_B_code,
        NULL, NULL, NULL, thread_B_prio, 0, K_NO_WAIT);

    thread_B_tid = k_thread_create(&thread_C_data, thread_C_stack,
        K_THREAD_STACK_SIZEOF(thread_C_stack), thread_C_code,
        NULL, NULL, NULL, thread_C_prio, 0, K_NO_WAIT);
    
    return;

} 

/* Thread code implementation */
void thread_A_code(void *argA , void *argB, void *argC)
{
    /* Timing variables to control task periodicity */
    int64_t fin_time=0, release_time=0;

    /* Other variables */
    int err=0;
    
    printk("Thread A init (periodic)\n");

    /* Compute next release instant */
    release_time = k_uptime_get() + thread_A_period;

    /* ADC setup: bind and initialize */
    adc_dev = device_get_binding(DT_LABEL(ADC_NID));
    if (!adc_dev) {
        printk("ADC device_get_binding() failed\n");
    } 
    err = adc_channel_setup(adc_dev, &my_channel_cfg);
    if (err) {
        printk("adc_channel_setup() failed with error code %d\n", err);
    }
    
    /* It is recommended to calibrate the SAADC at least once before use, and whenever the ambient temperature has changed by more than 10 °C */
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;

    /* Thread loop */
    while(1) {
        
        /* Get one sample, checks for errors and prints the values */
        err=adc_sample();
        if(err) {
            printk("adc_sample() failed with error code %d\n\r",err);
        }
        else {
            if(adc_sample_buffer[0] > 1023) {
                printk("adc reading out of range\n\r");
                printk("adc reading: raw:%4u / %4u mV: \n\r",adc_sample_buffer[0],(uint16_t)(1000*adc_sample_buffer[0]*((float)3/1023)));
            }
            else {
                /* ADC is set to use gain of 1/4 and reference VDD/4, so input range is 0...VDD (3 V), with 10 bit resolution */
                /*printk("adc reading: raw:%4u / %4u mV: \n\r",adc_sample_buffer[0],(uint16_t)(1000*adc_sample_buffer[0]*((float)3/1023)));*/
                ab=adc_sample_buffer[0];
            }
        }

        /*ab=30;*/

        printk("Thread A set ab value to: %d \n",ab);  
        
        k_sem_give(&sem_ab);
       
        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_A_period;

        }
    }

}

void thread_B_code(void *argA , void *argB, void *argC)
{
    /* Other variables */
    int dados[10]={0,0,0,0,0,0,0,0,0,0}, sum1=0, sum2=0, cnt1=0, cnt2=0, Avg=0;
    int i=0, j=0;

    printk("Thread B init (sporadic, waits on a semaphore by task A)\n");
    while(1) {
        k_sem_take(&sem_ab,  K_FOREVER);
        
        printk("Task B read ab value: %d\n",ab);

        dados[0] = ab;

        /** Rotates the array clockwise*/
        
        for(int k = 0; k < 10; k++){    
           int l=0, x=0;                
           x = dados[9];    /* Stores the last element of the array*/           
           for(l = 9; l > 0; l--){                    
                  dados[l] = dados[l-1];   
           }               
           dados[0] = x;    /* Last element of the array will be added to the start of the array*/ 
        } 

        /** Prevent the initial zeros from messing up the average*/
        
        for(i = 0; i<10;i++){
          //printk("Passou aqui v0.2\n\r");
            if(dados[i] != 0){
            //printk("Passou aqui v0.3\n\r");
                sum1 = sum1 + dados[i];
                cnt1++; 
            }
            else{
                sum1=sum1;}
        }
        
        Avg=sum1/cnt1;
        
        /** Choose the values that are not acording to the average*/
        
        for(j=0;j<10;j++){
            if(dados[j] < (Avg - Avg*0.1) || dados[j] > (Avg + Avg*0.1))
                sum2=sum2;
            else{
                sum2 = sum2 + dados[j];
                cnt2++;
            } 
            j++;
        }
        
        Avg=sum2/cnt2;

        bc=Avg;
        printk("Thread B set bc value to: %d \n",bc);  
        k_sem_give(&sem_bc);        
  }
}

void thread_C_code(void *argA , void *argB, void *argC)
{
    /* Other variables */
    /*long int nact = 0;*/
    const struct device *gpio0_dev;         /* Pointer to GPIO device structure */
    const struct device *pwm0_dev;          /* Pointer to PWM device structure */
    int ret=0;                              /* Generic return value variable */
    
    unsigned int pwmPeriod_us = 1000;       /* PWM priod in us */
    //unsigned int dcValue[]={0,33,66,100};   /* Duty-cycle in % */
    //unsigned int dcIndex=0;                 /* DC Index */

    /* Bind to GPIO 0 and PWM0 */
    gpio0_dev = device_get_binding(DT_LABEL(GPIO0_NID));
    if (gpio0_dev == NULL) {
        printk("Error: Failed to bind to GPIO0\n\r");        
	return;
    }
    /*else {
        printk("Bind to GPIO0 successfull \n\r");        
    }*/
    
    pwm0_dev = device_get_binding(DT_LABEL(PWM0_NID));
    if (pwm0_dev == NULL) {
	printk("Error: Failed to bind to PWM0\n r");
	return;
    }
    /*else  {
        printk("Bind to PWM0 successful\n\r");            
    }*/

    printk("Thread C init (sporadic, waits on a semaphore by task B)\n");

    while(1) {
        k_sem_take(&sem_bc, K_FOREVER);
        
        ret = pwm_pin_set_usec(pwm0_dev, BOARDLED1,
		      pwmPeriod_us,(unsigned int)((pwmPeriod_us*bc)/100), PWM_POLARITY_NORMAL);
        if (ret) {
            printk("Error %d: failed to set pulse width\n", ret);
            return;
        }          
        printk("Task C read bc value: %d\n",bc);

        
  }
}


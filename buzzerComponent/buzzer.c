/**
 *
 * This file provides the implementation of buzzer frequency selection, period,
 *	duty cycle and enable switch.
 *
 * <hr>
 *
 * Copyright (C) Sierra Wireless Inc.
 */

#include "legato.h"
#include "interfaces.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) ((unsigned long) (sizeof (array) / sizeof ((array)[0])))
#endif
// #define RES_PATH_BUZZER			"buzzer"
#define RES_PATH_ENABLE 		"buzzerenable"
#define RES_PATH_FREQ_OUTPUT 		"frequency"
#define RES_PATH_FREQ_INPUT 		RES_PATH_FREQ_OUTPUT "/value"
#define RES_PATH_PERIOD_OUTPUT 		"buzzerperiod"
#define RES_PATH_PERIOD_INPUT 		RES_PATH_PERIOD_OUTPUT "/value"
#define RES_PATH_DC_ON_OUTPUT 		"duty-cycle-on-interval"
#define RES_PATH_DC_ON_INPUT 		RES_PATH_DC_ON_OUTPUT "/value"

#define TURN_OFF_CLKOUT			0x0

static const char BuzzerFreqPath[]   = "/sys/bus/i2c/drivers/rtc-pcf85063/8-0051/clkout_freq";

/* To be more generic the ref needs to be a union of the returned dhub PushHandlerRef_t types
 * and PushHandler needs to be a union of different types
 */
typedef struct actuator_DhResTypes_t {
    const char			*OutputPath;
    const char			*InputPath;
    dhubIO_DataType_t		dataType;
    const char			*units;
    void			(*OutputHandler)(double, double, void *);
    void			*ref; /* Need a union here */
} actuator_DhResTypes_t;

static void FreqConfigPushHandler (double timestamp, double freq, void *context);
static void PeriodConfigPushHandler (double timestamp, double period, void *context);
static void DcOnConfigPushHandler (double timestamp, double dcOnInterval, void *context);
static void EnablePushHandler (double timestamp, bool enable, void *context);

/* To be more generic the PushHandler dhub routine should be in the table */
static actuator_DhResTypes_t dhubResources[] = {
   {RES_PATH_ENABLE, NULL, DHUBIO_DATA_TYPE_BOOLEAN, "1/0", NULL},
   {RES_PATH_FREQ_OUTPUT, RES_PATH_FREQ_INPUT, DHUBIO_DATA_TYPE_NUMERIC, "Hz", FreqConfigPushHandler},
   {RES_PATH_PERIOD_OUTPUT, RES_PATH_PERIOD_INPUT, DHUBIO_DATA_TYPE_NUMERIC, "s", PeriodConfigPushHandler},
   {RES_PATH_DC_ON_OUTPUT, RES_PATH_DC_ON_INPUT, DHUBIO_DATA_TYPE_NUMERIC, "%", DcOnConfigPushHandler},
};

// The current frequency of square wave output by the rtc chip to the buzzer
static int current_freq = 0;
static int set_freq = 0;
static FILE *freqFp = NULL;

// The duty cycle percent of being on/off for the buzzer
static int dc_on = 0;

// The period of the duty cycle
static int period = 0;

// The timer used to run the duty cycle
static le_timer_Ref_t Timer = NULL;
static uint32_t dc_on_timer = 0;
static uint32_t dc_off_timer = 0;
static bool buzzer_on = false;

static void SetBuzzer()
{
    if (freqFp == NULL) {
    	freqFp = fopen(BuzzerFreqPath, "r+");
    	if (freqFp == NULL) {
        	LE_ERROR("Open clkout sysfs file('%s') failed(%d)", BuzzerFreqPath, errno);
        	return;
    	}
    }

    //LE_DEBUG("Clkout frequency %d Hz", current_freq);

    if (fprintf(freqFp, "%d", current_freq) == -1)
        LE_ERROR("Write clkout file('%s') failed (%d)", BuzzerFreqPath, errno);

    if (fflush(freqFp) == -1)
	LE_ERROR("fflush of /sys/bus/i2c/drivers/rtc-pcf85063/8-0051/clkout_freq failed");
}

static void DcHandler(le_timer_Ref_t DcTimerRef)
{
    le_timer_Stop(Timer);
    if (buzzer_on) {
	le_timer_SetMsInterval(Timer, dc_off_timer);
	current_freq = TURN_OFF_CLKOUT;
	SetBuzzer();
	buzzer_on = false;
	le_timer_Start(Timer);
    }
    else if (!buzzer_on) {
	le_timer_SetMsInterval(Timer, dc_on_timer);
	current_freq = set_freq;
	SetBuzzer();
	buzzer_on = true;
	le_timer_Start(Timer);
    }
    else
	LE_ERROR("Illegal state of buzzer");
}

static void EnablePushHandler (double timestamp, bool enable, void *context)
{

    if (enable) {
	// We are assuming that set_freq != 0 means that the clkout file is open
	if (period == 0 || dc_on == 0 || set_freq == 0) {
	    LE_ERROR("Enable buzzer with either these not set: Period, Duty Cycle, Frequency");
	    return;
	}
	
	dc_on_timer = ((double) dc_on)/100 * period * 1000;
	dc_off_timer = ((100 - ((double) dc_on))/100) * period * 1000;
	LE_INFO("period: %d dc_on: %d dc_on_timer: %u dc_off_timer: %u", period, dc_on, dc_on_timer, dc_off_timer);
    	le_timer_SetMsInterval(Timer, dc_on_timer);

	// We start on the duty cycle high
	buzzer_on = true;
	if (set_freq == TURN_OFF_CLKOUT) {
	    LE_ERROR("Turning on buzzer with no frequency");
	    return;
	}
	current_freq = set_freq;
	SetBuzzer();
	le_timer_Start(Timer);
    }

    if (!enable) {
	le_timer_Stop(Timer);
	if (current_freq != TURN_OFF_CLKOUT) {
	    current_freq = TURN_OFF_CLKOUT;
	    SetBuzzer();
	}
    }
}

// Note we are not checking the values here, the sysfs interface does.
static void FreqConfigPushHandler (double timestamp, double freq, void *context)
{
    set_freq = current_freq = (int) freq;
    //SetBuzzer();
    dhubIO_PushNumeric(RES_PATH_FREQ_INPUT, DHUBIO_NOW, freq);
}

static void PeriodConfigPushHandler (double timestamp, double PeriodConfig, void *context)
{
    period = (int) PeriodConfig;

    // Restricting from 2 sec. to 3600 (i.e. 1 hour)
    if (period < 2 || period > 3600) {
	LE_ERROR("Received invalid PeriodConfig for the Buzzer Duty Cycle - must be between 2 & 3600");
	return;
    }
    dhubIO_PushNumeric(RES_PATH_PERIOD_INPUT, DHUBIO_NOW, PeriodConfig);
}

static void DcOnConfigPushHandler (double timestamp, double DcOnInterval, void *context)
{
    dc_on = (int) DcOnInterval;

    if (dc_on < 1 || dc_on > 99) {
	LE_ERROR("Received invalid DcOnInterval for the Buzzer Duty Cycle - must be between 1 & 99");
	return;
    }

    dhubIO_PushNumeric(RES_PATH_DC_ON_INPUT, DHUBIO_NOW, DcOnInterval);
}

static le_result_t actuator_DhRegister (void)
{
    le_result_t result = LE_OK;

    
    for (int i = 0; i < ARRAY_SIZE(dhubResources); i++) {
	if (dhubResources[i].OutputPath != NULL)
            result = dhubIO_CreateOutput(dhubResources[i].OutputPath, 
                                 dhubResources[i].dataType, 
                                 dhubResources[i].units);
        if (LE_OK != result) {
            LE_ERROR("Failed to create output resource %s", dhubResources[i].OutputPath);
            break;
        }

	dhubIO_MarkOptional(dhubResources[i].OutputPath);

	/* We try to create an Input to keep all listeners apprised of the value change */
	if (dhubResources[i].InputPath != NULL)
       	    LE_ASSERT(LE_OK == dhubIO_CreateInput(dhubResources[i].InputPath, dhubResources[i].dataType, ""));

	/* To be more generic the PushHandler dhub routine should be in the table */
	if (dhubResources[i].OutputHandler != NULL) {
		//LE_INFO("Adding handler for %s", dhubResources[i].OutputPath);
		dhubResources[i].ref = dhubIO_AddNumericPushHandler(dhubResources[i].OutputPath,
			dhubResources[i].OutputHandler, NULL);
		if (NULL == dhubResources[i].ref) {
            	    LE_ERROR("Failed to add handler for output resource %s", dhubResources[i].OutputPath);
            	    result = LE_FAULT;
            	    break;
        	}
	}
	
    }

    // Need seperate code for Bool Push Handler - TODO fix with union
    if (result == LE_OK) {
    	dhubResources[0].ref = dhubIO_AddBooleanPushHandler(dhubResources[0].OutputPath,
				EnablePushHandler, NULL);
	if (NULL == dhubResources[0].ref) {
            LE_ERROR("Failed to add handler for output resource %s", dhubResources[0].OutputPath);
            result = LE_FAULT;
	}
    }

    return result;
}

//--------------------------------------------------------------------------------------------------
/**
 * SIGTERM handler to cleanly shutdown
 */
//--------------------------------------------------------------------------------------------------
static void actuator_SigTermHandler (int pSigNum)
{
    LE_INFO("Remove buzzer resources");

    for (int i = 0; i < ARRAY_SIZE(dhubResources); i++) {
        dhubIO_DeleteResource(dhubResources[i].OutputPath);
        dhubIO_DeleteResource(dhubResources[i].InputPath);
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Main program
 */
//--------------------------------------------------------------------------------------------------
COMPONENT_INIT
{
    ///< Catch application termination and shutdown cleanly
    le_sig_Block(SIGTERM);
    le_sig_SetEventHandler(SIGTERM, actuator_SigTermHandler);
    LE_ASSERT(LE_OK == actuator_DhRegister());

    Timer = le_timer_Create("Buzzer Timer");
    le_timer_SetRepeat(Timer, 0);
    le_timer_SetHandler(Timer, DcHandler);

}


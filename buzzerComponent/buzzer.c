/**
 * This component makes the control of the mangOH Yellow's buzzer available through the Data Hub.
 *
 * The buzzer can cycle on and off over a period of time.
 *
 * If period = 1 second,
 *    duty cycle = 20 %,
 *    enable = true
 * then the buzzer will emit a sound for 200 ms, turn off for 800 ms, and repeat.
 *
 * If enable is false, then no sound will be emitted, regardless of the other settings.
 *
 * <hr>
 *
 * The buzzer is driven by the CLKOUT signal from the RTC chip, which is controlled via a sysfs
 * file.  A Legato timer is used to implement the on/off duty cycle period.
 *
 * <hr>
 *
 * Copyright (C) Sierra Wireless Inc.
 */

#include "legato.h"
#include "interfaces.h"

// Data Hub resource paths, relative to the app's root.
#define RES_PATH_ENABLE     "enable"
#define RES_PATH_PERIOD     "period"
#define RES_PATH_DUTY_CYCLE "percent"

/// Frequency to use to turn the buzzer off.
#define BUZZER_OFF_FREQ 0

/// Frequency to use to turn the buzzer on.
/// The buzzer can physically produce a range of frequencies, but is designed to run at 4kHz.
/// The RTC chip can produce a set of specific frequencies, the closest of which is 4096Hz.
#define BUZZER_ON_FREQ 4096

/// Whether the buzzer is enabled or not.
static bool Enabled = false;

// The on percentage of the buzzer on/off duty cycle (0 to 100).
static uint DutyCycleOnPercent = 50;

// The total number of milliseconds in the full duty cycle period (on + off).
// Must be >= 100 and <= 3600000 (i.e. 1 hour).
static uint PeriodMs = 2000;

// The timer used to run the duty cycle
static le_timer_Ref_t Timer = NULL;

// true if the buzzer is currently on (buzzing).
static bool BuzzerOn = false;

//--------------------------------------------------------------------------------------------------
/**
 * Sets the buzzer control on or off.
 */
//--------------------------------------------------------------------------------------------------
static void SetBuzzer
(
    bool on ///< true = turn the buzzer on, false = turn the buzzer off.
)
{
    // Path to the RTC CLKOUT control file in sysfs.
    static const char BuzzerFreqPath[] = "/sys/bus/i2c/drivers/rtc-pcf85063/8-0051/clkout_freq";

    static FILE *FreqFile = NULL;

    if (FreqFile == NULL)
    {
        FreqFile = fopen(BuzzerFreqPath, "r+");
        if (FreqFile == NULL)
        {
            LE_FATAL("Opening file (%s) failed (%m)", BuzzerFreqPath);
        }
    }

    if (fprintf(FreqFile, "%d", on ? BUZZER_ON_FREQ : BUZZER_OFF_FREQ) == -1)
    {
        LE_FATAL("Write to file (%s) failed (%m)", BuzzerFreqPath);
    }

    if (fflush(FreqFile) != 0)
    {
        LE_FATAL("fflush of file (%s) failed (%m)", BuzzerFreqPath);
    }
}

//--------------------------------------------------------------------------------------------------
/**
 * Start at the beginning of a duty cycle.
 */
//--------------------------------------------------------------------------------------------------
static void StartCycle
(
    void
)
{
    SetBuzzer(true);
    BuzzerOn = true;
    le_timer_SetMsInterval(Timer, PeriodMs * DutyCycleOnPercent / 100);
    le_timer_Start(Timer);
}

//--------------------------------------------------------------------------------------------------
/**
 * Disable the buzzer, immediately stopping it, even if it's in the middle of a duty cycle.
 */
//--------------------------------------------------------------------------------------------------
static void StopCycle
(
    void
)
{
    le_timer_Stop(Timer);
    if (BuzzerOn)
    {
        SetBuzzer(false);
        BuzzerOn = false;
    }
}

//--------------------------------------------------------------------------------------------------
/**
 * Timer expiry handler function.
 */
//--------------------------------------------------------------------------------------------------
static void TimerExpiryHandler
(
    le_timer_Ref_t timer
)
{
    // If the buzzer is on, it's time to turn it off and adjust the timer for the off period.
    // Otherwise, it's time to turn it on and restart the timer for the on period.
    // NOTE: the timer will drift less if we leave it running while we update its interval,
    // rather than stopping the timer while updating its interval.
    if (BuzzerOn)
    {
        // If the duty cycle is 100%, then just leave the buzzer on.
        if (DutyCycleOnPercent < 100)
        {
            SetBuzzer(false);
            BuzzerOn = false;

            uint ms = PeriodMs * (100 - DutyCycleOnPercent) / 100;
            le_timer_SetMsInterval(timer, ms);
        }
    }
    else
    {
        // If the duty cycle is 0%, then just leave the buzzer off.
        if (DutyCycleOnPercent > 0)
        {
            SetBuzzer(true);
            BuzzerOn = true;

            uint ms = PeriodMs * DutyCycleOnPercent / 100;
            le_timer_SetMsInterval(timer, ms);
        }
    }
}

//--------------------------------------------------------------------------------------------------
/**
 * Handler function for updates to the enable setpoint from the Data Hub.
 */
//--------------------------------------------------------------------------------------------------
static void EnablePushHandler
(
    double timestamp,
    bool enable,
    void *context
)
{
    // Ignore updates that don't change the value.
    if (enable != Enabled)
    {
        Enabled = enable;

        if (enable)
        {
            StartCycle();
        }
        else
        {
            StopCycle();
        }
    }
}

//--------------------------------------------------------------------------------------------------
/**
 * Handler function for duty cycle period setpoint updates from the Data Hub.
 */
//--------------------------------------------------------------------------------------------------
static void PeriodPushHandler
(
    double timestamp,
    double period,
    void *context
)
{
    // Restrict the range
    if (period < 0.1 || period > 3600.0)
    {
        LE_ERROR("Received invalid duty cycle period (%lf seconds) - must be between 0.1 & 3600",
                 period);
    }
    else
    {
        uint periodMs = (uint)(period * 1000);  // Convert to integer number of milliseconds.
        if (PeriodMs != periodMs)
        {
            PeriodMs = periodMs;

            // If the buzzer is enabled, stop the buzzer and the timer and restart everything.
            if (Enabled)
            {
                StopCycle();
                StartCycle();
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------
/**
 * Handler function for the duty cycle percent setpoint updates from the Data Hub.
 */
//--------------------------------------------------------------------------------------------------
static void PercentPushHandler
(
    double timestamp,
    double percent,
    void *context
)
{
    if (percent < 0.0 || percent > 100.0)
    {
        LE_ERROR("Ignoring invalid duty cycle percentage (%lf) - must be between 0 & 100", percent);
    }
    else
    {
        uint intPercent = (uint)percent;
        if (DutyCycleOnPercent != intPercent)
        {
            DutyCycleOnPercent = intPercent;

            // If the buzzer is on, it's not too late to update the timer interval in this
            // cycle.  Otherwise, we have to wait for the off period to end before updating.
            // Also, the buzzer won't be on if it's disabled.
            if (BuzzerOn)
            {
                uint ms = PeriodMs * DutyCycleOnPercent / 100;
                le_timer_SetMsInterval(Timer, ms);
            }
        }
    }
}

COMPONENT_INIT
{
    Timer = le_timer_Create("Buzzer Timer");
    le_timer_SetRepeat(Timer, 0 /* number of iterations, where 0 = infinity */);
    le_timer_SetHandler(Timer, TimerExpiryHandler);

    LE_ASSERT(LE_OK == dhubIO_CreateOutput(RES_PATH_ENABLE, DHUBIO_DATA_TYPE_BOOLEAN, ""));
    LE_ASSERT(dhubIO_AddBooleanPushHandler(RES_PATH_ENABLE, EnablePushHandler, NULL));
    dhubIO_SetBooleanDefault(RES_PATH_ENABLE, Enabled);

    LE_ASSERT(LE_OK == dhubIO_CreateOutput(RES_PATH_PERIOD, DHUBIO_DATA_TYPE_NUMERIC, "s"));
    LE_ASSERT(dhubIO_AddNumericPushHandler(RES_PATH_PERIOD, PeriodPushHandler, NULL));
    dhubIO_SetNumericDefault(RES_PATH_PERIOD, ((double)PeriodMs) / 1000.0);

    LE_ASSERT(LE_OK == dhubIO_CreateOutput(RES_PATH_DUTY_CYCLE, DHUBIO_DATA_TYPE_NUMERIC, "%"));
    LE_ASSERT(dhubIO_AddNumericPushHandler(RES_PATH_DUTY_CYCLE, PercentPushHandler, NULL));
    dhubIO_SetNumericDefault(RES_PATH_DUTY_CYCLE, DutyCycleOnPercent);
}

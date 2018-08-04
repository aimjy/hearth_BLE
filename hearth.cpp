/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mbed.h"
#include "ble/BLE.h"
#include "ble/services/HeartRateService.h"
#include "ble/services/BatteryService.h"
#include "ble/services/DeviceInformationService.h"

DigitalOut led1(P0_11);                                                                         //setting output-mode on pin P0_11 of the BLE for the built-in led
AnalogIn analog_value(P0_4);                                                                    //setting analog input-mode on pin P0_4 of the BLE for the hearthratesensor
const static char     DEVICE_NAME[]        = "HRM1";                                            //Bleutooth SSID
static const uint16_t uuid16_list[]        = {GattService::UUID_HEART_RATE_SERVICE,
                                              GattService::UUID_DEVICE_INFORMATION_SERVICE};    //List of Services that the BLE can provide
static volatile bool  triggerSensorPolling = false;                                             //Boolean to notify when the sensor data has been querried & processed (every 15s here)

//Pulse sensing
volatile int thresh = 130;                                                                      //used to find instant moment of heart beat, seeded
volatile int BPM;                                                                               //used to hold the pulse rate
volatile int Signal;                                                                            //holds the incoming raw data
volatile bool QS = false;                                                                       //becomes true when Arduoino finds a beat.
uint8_t hrmCounter = 100;                                                                       //var which contains data send over bluetooth
bool peak = false;                                                                              //bool value if a peak has been detected
int counter15 = 0;                                                                              //timer to mesure 15seconds
int teller = 0;                                                                                 //to count the number of beats during 15seconds
int peak_value = 0;                                                                             //highest value of the signal
int trough_value = 250;                                                                         //lowest value of the signal
int reset_counter = 0;                                                                          //timer to mesure if the user has left and reset default value of thresh, peak & trough


HeartRateService         *hrService;
DeviceInformationService *deviceInfo;

/* called when the device is disconected */
void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    BLE::Instance(BLE::DEFAULT_INSTANCE).gap().startAdvertising(); // restart advertising
}

/* adapt the thresh to be between the peak & trough */
void changeThresh(void){
    if((peak_value != 0 || trough_value != 250) && peak_value > trough_value){
        thresh = (peak_value + trough_value)/2;
    }
}

/* function is called periodically every 2ms */
void periodicCallback_test(void){
    
    int Signal = 255 * analog_value.read();                                                     //read the signal of the analog pin (from the sensor)
    reset_counter += 2;                                                                         //add 2ms every iteration
    counter15 += 2;                                                                             //add 2ms every iteration
    
    if(Signal > thresh && Signal > peak_value){                                                 //when signal becomes higher than the actual peak, adapt peak & thresh
        peak_value = Signal;
        reset_counter = 0;
        changeThresh();
    }
    
    if(Signal < thresh && Signal < trough_value){                                               //when signal becomes lower than the actual through, adapt through & thresh
        trough_value = Signal;
        reset_counter = 0;
        changeThresh();
    }
    
    if(reset_counter >= 10000){                                                                 //when peak & through hasn't changed in 10s, reset the values to adapt again
        peak_value = 0;
        trough_value = 250;
        reset_counter = 10000;
    }
    
    if(Signal > thresh && !peak){                                                               //if a heartbeat is detected, add a heartbeat and set the led on. (peak is used to detect only once)
        led1 = 1;
        teller ++;
        peak = true;
        
    }
    
    if(Signal < thresh && peak){                                                                //when a heartbeat is over, set the led off & allow the next heartbeat
        led1 = 0;
        peak = false;    
    }
    
    if(counter15 >= 15000){                                                                     //after 15s, send the avarage heartbeat through bluetooth
        BPM = 4*teller;
        counter15 = 0;
        teller = 0;
        QS = true;
        triggerSensorPolling = true;
    }
    
}

/* initialisation of the BLE Components */
void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE &ble          = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        return;
    }

    ble.gap().onDisconnection(disconnectionCallback);

    /* Setup primary service. */
    
    hrService = new HeartRateService(ble, hrmCounter, HeartRateService::LOCATION_OTHER);

    /* Setup auxiliary service. */
    deviceInfo = new DeviceInformationService(ble, "ARM", "Model1", "SN1", "hw-rev1", "fw-rev1", "soft-rev1");

    /* Setup advertising. */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *)uuid16_list, sizeof(uuid16_list));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::GENERIC_HEART_RATE_SENSOR);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(1000); /* 1000ms */
    ble.gap().startAdvertising();
}

int main(void)
{
    led1 = 1;
    Ticker ticker;
    ticker.attach(periodicCallback_test, 0.002);                                            // call periodicCallback_test() every 2ms

    BLE& ble = BLE::Instance(BLE::DEFAULT_INSTANCE);
    ble.init(bleInitComplete);

    /* SpinWait for initialization to complete. This is necessary because the
     * BLE object is used in the main loop below. */
    while (ble.hasInitialized()  == false) { /* spin loop */ }

    // infinite loop
    while (1) {
        // check for trigger from periodicCallback_test()
         if (triggerSensorPolling && ble.getGapState().connected) {
            triggerSensorPolling = false;

            if (QS == true) {                                                               // Quantified Self flag is true when BLE finds a heartbeat
                // Do blocking calls or whatever is necessary for sensor polling.
                // In our case, we simply update the HRM measurement.
                hrmCounter = (uint8_t)(BPM);

                // update bps
                hrService->updateHeartRate(hrmCounter);
                QS = false;                                                                 // reset the Quantified Self flag for next time
            } else {
                hrmCounter = 7;

                // update bps
                hrService->updateHeartRate(hrmCounter);
            } 

        } else {
            ble.waitForEvent();                                                             // low power wait for event
        }
    }
}


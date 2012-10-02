/**
 * IEC 61850-9-2LE Sampled Values demonstration
 *
 * Copyright (c) 2012 Steven Blair
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "mbed.h"
#include "iec61850.h"

#include <stdio.h>

#define NUMBER_OF_SIGNALS           8
#define NUMBER_OF_SIGNALS_PHASES    6
#define NUMBER_OF_SAMPLES           80

#define PI                          3.1415926535897932384626433832795f
#define TWO_PI                      6.283185307179586476925286766559f
#define TWO_PI_OVER_THREE           2.0943951023931954923084289221863f


CTYPE_INT32 signal[NUMBER_OF_SIGNALS_PHASES][NUMBER_OF_SAMPLES] = {0};

float phi = 0.1 * PI;        // phase angle between voltage and current (rad)
float freq = 50.0;           // frequency of waveforms (Hz)
float w = 2.0 * PI * freq;
float Ts = 250e-6;           // timestep; should equal 1 / (freq * NUMBER_OF_SAMPLES)
float V = 8981.462390205;    // voltage magnitude; equals 11 kV * sqrt(2) / sqrt(3)
float Zmag = 15.0;           // impedance magnitude - defines the current magnitude
float harmonic = 7;          // harmonic number
float harmonicMagPu = 0.03;  // harmonic magnitude (p.u.); set to zero to ignore

unsigned char buf[1024] = {0};
int len = 0;

DigitalOut watchdogLED(LED1);
DigitalOut ethLink(p29);
DigitalOut ethAct(p30);
Ethernet eth;
Ticker sv;

/**
 *  Pre-calculate all voltage and current samples (only possible if exactly 50 Hz frequency is used).
 */
void preCalculate() {
    int t = 0;
    for (t = 0; t < NUMBER_OF_SAMPLES; t++) {
        double theta = w * (((float) t) * Ts);
        
        signal[0][t] = (CTYPE_INT32) (V * sin(theta) / ((float) LE_IED.S1.MUnn.IEC_61850_9_2LETVTR_1.Vol.sVC.scaleFactor) + (V * harmonicMagPu * sin(theta * harmonic) / ((float) LE_IED.S1.MUnn.IEC_61850_9_2LETVTR_1.Vol.sVC.scaleFactor)));
        signal[1][t] = (CTYPE_INT32) (V * sin(theta - TWO_PI_OVER_THREE) / LE_IED.S1.MUnn.IEC_61850_9_2LETVTR_2.Vol.sVC.scaleFactor) + (V * harmonicMagPu * sin(theta * harmonic - TWO_PI_OVER_THREE) / ((float) LE_IED.S1.MUnn.IEC_61850_9_2LETVTR_1.Vol.sVC.scaleFactor));
        signal[2][t] = (CTYPE_INT32) (V * sin(theta + TWO_PI_OVER_THREE) / LE_IED.S1.MUnn.IEC_61850_9_2LETVTR_3.Vol.sVC.scaleFactor) + (V * harmonicMagPu * sin(theta * harmonic + TWO_PI_OVER_THREE) / ((float) LE_IED.S1.MUnn.IEC_61850_9_2LETVTR_1.Vol.sVC.scaleFactor));
        
        signal[3][t] = (CTYPE_INT32) ((V / Zmag) * sin(theta - phi) / LE_IED.S1.MUnn.IEC_61850_9_2LETCTR_1.Amp.sVC.scaleFactor) + ((harmonicMagPu * V / Zmag) * sin(theta * harmonic - phi) / LE_IED.S1.MUnn.IEC_61850_9_2LETCTR_1.Amp.sVC.scaleFactor);
        signal[4][t] = (CTYPE_INT32) ((V / Zmag) * sin(theta - phi - TWO_PI_OVER_THREE) / LE_IED.S1.MUnn.IEC_61850_9_2LETCTR_2.Amp.sVC.scaleFactor) + ((harmonicMagPu * V / Zmag) * sin(theta * harmonic - phi - TWO_PI_OVER_THREE) / LE_IED.S1.MUnn.IEC_61850_9_2LETCTR_2.Amp.sVC.scaleFactor);
        signal[5][t] = (CTYPE_INT32) ((V / Zmag) * sin(theta - phi + TWO_PI_OVER_THREE) / LE_IED.S1.MUnn.IEC_61850_9_2LETCTR_3.Amp.sVC.scaleFactor) + ((harmonicMagPu * V / Zmag) * sin(theta * harmonic - phi + TWO_PI_OVER_THREE) / LE_IED.S1.MUnn.IEC_61850_9_2LETCTR_3.Amp.sVC.scaleFactor);
    }
}

/**
 * Transmit the next set of samples.
 */
void svSnapshot() {
    static int t = 0;
    
    LE_IED.S1.MUnn.IEC_61850_9_2LETVTR_1.Vol.instMag.i = signal[0][t];
    LE_IED.S1.MUnn.IEC_61850_9_2LETVTR_2.Vol.instMag.i = signal[1][t];
    LE_IED.S1.MUnn.IEC_61850_9_2LETVTR_3.Vol.instMag.i = signal[2][t];
    LE_IED.S1.MUnn.IEC_61850_9_2LETVTR_4.Vol.instMag.i = 0;

    LE_IED.S1.MUnn.IEC_61850_9_2LETCTR_1.Amp.instMag.i = signal[3][t];
    LE_IED.S1.MUnn.IEC_61850_9_2LETCTR_2.Amp.instMag.i = signal[4][t];
    LE_IED.S1.MUnn.IEC_61850_9_2LETCTR_3.Amp.instMag.i = signal[5][t];
    LE_IED.S1.MUnn.IEC_61850_9_2LETCTR_4.Amp.instMag.i = 0;

    len = sv_update_LE_IED_MUnn_MSVCB01(buf);
    
    if (len > 0) {
        ethAct = 1;
        eth.write((const char *) buf, len);
        eth.send();
        ethAct = 0;
    }
    
    if (++t >= NUMBER_OF_SAMPLES) {
        t = 0;
    }
}

/**
 * Overriding this function sets the MAC address.
 * Set to the destination MAC of all received SV or GOOSE packets to simplify hardware MAC filtering.
 */
extern "C" void mbed_mac_address(char *s) {
    char mac[6];
    
    mac[0] = 0x01;
    mac[1] = 0x0C;
    mac[2] = 0xCD;
    mac[3] = 0x04;
    mac[4] = 0x00;
    mac[5] = 0x00;

    memcpy(s, mac, 6);
}

int main() {
    initialise_iec61850();   
     
    // enable hardware MAC address filtering
    LPC_EMAC->RxFilterCtrl = 1 << 5;
    
    eth.set_link(eth.FullDuplex100);
    while (!eth.link()) {
        wait(1);
    }
    ethLink = 1;
    
    wait(1);
    
    preCalculate();
    
    sv.attach_us(&svSnapshot, 250);     // create 250 us (for 50 Hz, 80 samples/cycle) periodic timer

    // loop forever, toggling LED
    while(1) {
        watchdogLED = 1;
        wait(0.01);
        watchdogLED = 0;
        wait(2);
    }
}

// Modification for MobiDisc with FastHandOver
//
// 30/05/2015
// Vasileios Kotsiou

   
/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Implementation of the ContikiMAC power-saving radio duty cycling protocol
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 */

#include "contiki-conf.h"
#include "dev/leds.h"
#include "dev/radio.h"
#include "dev/watchdog.h"
#include "lib/random.h"
#include "net/mac/contikimac.h"
#include "net/netstack.h"
#include "net/rime.h"
#include "sys/compower.h"
#include "sys/pt.h"
#include "sys/rtimer.h"


#include <string.h>

#include "dev/cc2420.h"


/* TX/RX cycles are synchronized with neighbor wake periods */
#ifdef CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION
#define WITH_PHASE_OPTIMIZATION     1// CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION
#else /* CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION */
#define WITH_PHASE_OPTIMIZATION     1   
#endif /* CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION */
/* Two byte header added to allow recovery of padded short packets */
/* Wireshark will not understand such packets at present */
#ifdef CONTIKIMAC_CONF_WITH_CONTIKIMAC_HEADER
#define WITH_CONTIKIMAC_HEADER       CONTIKIMAC_CONF_WITH_CONTIKIMAC_HEADER
#else
#define WITH_CONTIKIMAC_HEADER       1
#endif
/* More aggressive radio sleeping when channel is busy with other traffic */
#ifndef WITH_FAST_SLEEP
#define WITH_FAST_SLEEP              1 
#endif
/* Radio does CSMA and autobackoff */
#ifndef RDC_CONF_HARDWARE_CSMA
#define RDC_CONF_HARDWARE_CSMA       0
#endif
/* Radio returns TX_OK/TX_NOACK after autoack wait */
#ifndef RDC_CONF_HARDWARE_ACK
#define RDC_CONF_HARDWARE_ACK        0
#endif
/* MCU can sleep during radio off */
#ifndef RDC_CONF_MCU_SLEEP
#define RDC_CONF_MCU_SLEEP           0
#endif

#if NETSTACK_RDC_CHANNEL_CHECK_RATE >= 64
#undef WITH_PHASE_OPTIMIZATION
#define WITH_PHASE_OPTIMIZATION 0
#endif

#if WITH_CONTIKIMAC_HEADER
#define CONTIKIMAC_ID 0x00

struct hdr {
  uint8_t id;
  uint8_t len;
};
#endif /* WITH_CONTIKIMAC_HEADER */

/* CYCLE_TIME for channel cca checks, in rtimer ticks. */
#ifdef CONTIKIMAC_CONF_CYCLE_TIME
#define CYCLE_TIME (CONTIKIMAC_CONF_CYCLE_TIME)
#else
#define CYCLE_TIME (RTIMER_ARCH_SECOND / NETSTACK_RDC_CHANNEL_CHECK_RATE)
#endif

/* CHANNEL_CHECK_RATE is enforced to be a power of two.
 * If RTIMER_ARCH_SECOND is not also a power of two, there will be an inexact
 * number of channel checks per second due to the truncation of CYCLE_TIME.
 * This will degrade the effectiveness of phase optimization with neighbors that
 * do not have the same truncation error.
 * Define SYNC_CYCLE_STARTS to ensure an integral number of checks per second.
 */
#if RTIMER_ARCH_SECOND & (RTIMER_ARCH_SECOND - 1)
#define SYNC_CYCLE_STARTS                    1
#endif

/* Are we currently receiving a burst? */
static int we_are_receiving_burst = 0;

/* INTER_PACKET_DEADLINE is the maximum time a receiver waits for the
   next packet of a burst when FRAME_PENDING is set. */
#define INTER_PACKET_DEADLINE               CLOCK_SECOND / 32

/* ContikiMAC performs periodic channel checks. Each channel check
   consists of two or more CCA checks. CCA_COUNT_MAX is the number of
   CCAs to be done for each periodic channel check. The default is
   two.*/
#ifdef CONTIKIMAC_CONF_CCA_COUNT_MAX
#define CCA_COUNT_MAX                      (CONTIKIMAC_CONF_CCA_COUNT_MAX)
#else
#define CCA_COUNT_MAX                     3
#endif

//  default value 2  - 3 ordinary anycast - bronicast


/* Before starting a transmission, Contikimac checks the availability
   of the channel with CCA_COUNT_MAX_TX consecutive CCAs */
#ifdef CONTIKIMAC_CONF_CCA_COUNT_MAX_TX
#define CCA_COUNT_MAX_TX                   (CONTIKIMAC_CONF_CCA_COUNT_MAX_TX)
#else
#define CCA_COUNT_MAX_TX                   6
#endif

/* CCA_CHECK_TIME is the time it takes to perform a CCA check. */
/* Note this may be zero. AVRs have 7612 ticks/sec, but block until cca is done */
#ifdef CONTIKIMAC_CONF_CCA_CHECK_TIME
#define CCA_CHECK_TIME                     (CONTIKIMAC_CONF_CCA_CHECK_TIME)
#else
#define CCA_CHECK_TIME                     RTIMER_ARCH_SECOND / 8192
#endif

/* CCA_SLEEP_TIME is the time between two successive CCA checks. */
/* Add 1 when rtimer ticks are coarse */
#if RTIMER_ARCH_SECOND > 8000
#define CCA_SLEEP_TIME                     RTIMER_ARCH_SECOND / 2000
#else
#define CCA_SLEEP_TIME                     (RTIMER_ARCH_SECOND / 2000) + 1  //2000
#endif

/* CHECK_TIME is the total time it takes to perform CCA_COUNT_MAX
   CCAs. */
#define CHECK_TIME                         (CCA_COUNT_MAX * (CCA_CHECK_TIME + CCA_SLEEP_TIME))

/* CHECK_TIME_TX is the total time it takes to perform CCA_COUNT_MAX_TX
   CCAs. */
#define CHECK_TIME_TX                      (CCA_COUNT_MAX_TX * (CCA_CHECK_TIME + CCA_SLEEP_TIME))

/* LISTEN_TIME_AFTER_PACKET_DETECTED is the time that we keep checking
   for activity after a potential packet has been detected by a CCA
   check. */
#define LISTEN_TIME_AFTER_PACKET_DETECTED  RTIMER_ARCH_SECOND / 80

/* MAX_SILENCE_PERIODS is the maximum amount of periods (a period is
   CCA_CHECK_TIME + CCA_SLEEP_TIME) that we allow to be silent before
   we turn of the radio. */
#define MAX_SILENCE_PERIODS                5

/* MAX_NONACTIVITY_PERIODS is the maximum number of periods we allow
   the radio to be turned on without any packet being received, when
   WITH_FAST_SLEEP is enabled. */
#define MAX_NONACTIVITY_PERIODS            10



/* STROBE_TIME is the maximum amount of time a transmitted packet
   should be repeatedly transmitted as part of a transmission. */
#define STROBE_TIME                        (CYCLE_TIME + 2 * CHECK_TIME)

/* GUARD_TIME is the time before the expected phase of a neighbor that
   a transmitted should begin transmitting packets. */
#define GUARD_TIME                         5 * CHECK_TIME + CHECK_TIME_TX

/* INTER_PACKET_INTERVAL is the interval between two successive packet transmissions */
#ifdef CONTIKIMAC_CONF_INTER_PACKET_INTERVAL
#define INTER_PACKET_INTERVAL              CONTIKIMAC_CONF_INTER_PACKET_INTERVAL
#else
#define INTER_PACKET_INTERVAL              RTIMER_ARCH_SECOND / 700
#endif


/* AFTER_ACK_DETECTECT_WAIT_TIME is the time to wait after a potential
   ACK packet has been detected until we can read it out from the
   radio. */
#ifdef CONTIKIMAC_CONF_AFTER_ACK_DETECTECT_WAIT_TIME
#define AFTER_ACK_DETECTECT_WAIT_TIME      CONTIKIMAC_CONF_AFTER_ACK_DETECTECT_WAIT_TIME
#else
#define AFTER_ACK_DETECTECT_WAIT_TIME      RTIMER_ARCH_SECOND / 1500
#endif

/* MAX_PHASE_STROBE_TIME is the time that we transmit repeated packets
   to a neighbor for which we have a phase lock. */
#define MAX_PHASE_STROBE_TIME              RTIMER_ARCH_SECOND / 30



#define MY_STROBE_TIME	 RTIMER_ARCH_SECOND / 20

/* SHORTEST_PACKET_SIZE is the shortest packet that ContikiMAC
   allows. Packets have to be a certain size to be able to be detected
   by two consecutive CCA checks, and here is where we define this
   shortest size.
   Padded packets will have the wrong ipv6 checksum unless CONTIKIMAC_HEADER
   is used (on both sides) and the receiver will ignore them.
   With no header, reduce to transmit a proper multicast RPL DIS. */
#ifdef CONTIKIMAC_CONF_SHORTEST_PACKET_SIZE
#define SHORTEST_PACKET_SIZE  CONTIKIMAC_CONF_SHORTEST_PACKET_SIZE
#else
#define SHORTEST_PACKET_SIZE               43
#endif


#define ACK_LEN 3

#include <stdio.h>
static struct rtimer rt;
static struct pt pt;

static volatile uint8_t contikimac_is_on = 0;
static volatile uint8_t contikimac_keep_radio_on = 0;

static volatile unsigned char we_are_sending = 0;
static volatile unsigned char radio_is_on = 0;

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTDEBUG(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#define PRINTDEBUG(...)
#endif

#if CONTIKIMAC_CONF_COMPOWER
static struct compower_activity current_packet;
#endif /* CONTIKIMAC_CONF_COMPOWER */

#if WITH_PHASE_OPTIMIZATION

#include "net/mac/phase.h"

#endif /* WITH_PHASE_OPTIMIZATION */

#define DEFAULT_STREAM_TIME (4 * CYCLE_TIME)

#ifndef MIN
#define MIN(a, b) ((a) < (b)? (a) : (b))
#endif /* MIN */

struct seqno {
  rimeaddr_t sender;
  uint8_t seqno;
};

#ifdef NETSTACK_CONF_MAC_SEQNO_HISTORY
#define MAX_SEQNOS NETSTACK_CONF_MAC_SEQNO_HISTORY
#else /* NETSTACK_CONF_MAC_SEQNO_HISTORY */
#define MAX_SEQNOS 16
#endif /* NETSTACK_CONF_MAC_SEQNO_HISTORY */
static struct seqno received_seqnos[MAX_SEQNOS];

#if CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT
static struct timer broadcast_rate_timer;
static int broadcast_rate_counter;
#endif /* CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT */


static int BroniCastFather=0, prevBronicastFather=-1, AnycastFather=0 ,prevAnycastFather=-1 ;
static int SendBroniCast=0,SendAnyCast=0;
static char *packet;
static int is_mobile=0;
static unsigned int my_seq=0,prev_seq=-1;
static unsigned int retrans=0;

static unsigned int is_Any_BroadCast=0;

static int my_rank=100, rcv_rank=100;


static int postpone_cycle=0, SendBurst=0;

static int num_tries =0;


static int min_hops=100;
static int min_father =-1, my_got_ack=0, BrPhase=0;


void WakeUp();


#define DATA 7
#define PROBE 9

/*---------------------------------------------------------------------------*/
static void
on(void)
{
  if(contikimac_is_on && radio_is_on == 0) {
    radio_is_on = 1;
    NETSTACK_RADIO.on();
  }
}
/*---------------------------------------------------------------------------*/
static void
off(void)
{
/*if (rimeaddr_node_addr.u8[0]==6) {
 // printf("OFF_1 %d \n",postpone_cycle);
}
*/  
if (postpone_cycle==1) {
   postpone_cycle=1;
  //printf("PostPone \n");
    return;
 }
  if(contikimac_is_on && radio_is_on != 0 &&
     contikimac_keep_radio_on == 0) {
    radio_is_on = 0;
    NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static volatile rtimer_clock_t cycle_start;
static char powercycle(struct rtimer *t, void *ptr);
static void
schedule_powercycle(struct rtimer *t, rtimer_clock_t time)
{
  int r;

  if(contikimac_is_on) {

    if(RTIMER_CLOCK_LT(RTIMER_TIME(t) + time, RTIMER_NOW() + 2)) {
      time = RTIMER_NOW() - RTIMER_TIME(t) + 2;
    }

    r = rtimer_set(t, RTIMER_TIME(t) + time, 1,
                   (void (*)(struct rtimer *, void *))powercycle, NULL);
    if(r != RTIMER_OK) {
      PRINTF("schedule_powercycle: could not set rtimer\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
schedule_powercycle_fixed(struct rtimer *t, rtimer_clock_t fixed_time)
{
  int r;

  if(contikimac_is_on) {

    if(RTIMER_CLOCK_LT(fixed_time, RTIMER_NOW() + 1)) {
      fixed_time = RTIMER_NOW() + 1;
    }

    r = rtimer_set(t, fixed_time, 1,
                   (void (*)(struct rtimer *, void *))powercycle, NULL);
    if(r != RTIMER_OK) {
      PRINTF("schedule_powercycle: could not set rtimer\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
powercycle_turn_radio_off(void)
{


 if (postpone_cycle==1) {
  // printf("ppstp \n");
  return;
 }

  
#if CONTIKIMAC_CONF_COMPOWER
  uint8_t was_on = radio_is_on;
#endif /* CONTIKIMAC_CONF_COMPOWER */
  
  if(we_are_sending == 0 && we_are_receiving_burst == 0) {
    off();
#if CONTIKIMAC_CONF_COMPOWER
    if(was_on && !radio_is_on) {
      compower_accumulate(&compower_idle_activity);
    }
#endif /* CONTIKIMAC_CONF_COMPOWER */
  }

}
/*---------------------------------------------------------------------------*/
static void
powercycle_turn_radio_on(void)
{
  if(we_are_sending == 0 && we_are_receiving_burst == 0) {
    on();
  }
}
/*---------------------------------------------------------------------------*/
static char
powercycle(struct rtimer *t, void *ptr)
{

#if SYNC_CYCLE_STARTS
  static volatile rtimer_clock_t sync_cycle_start;
  static volatile uint8_t sync_cycle_phase;
#endif

  PT_BEGIN(&pt);

#if SYNC_CYCLE_STARTS
  sync_cycle_start = RTIMER_NOW();
#else
  cycle_start = RTIMER_NOW();
#endif

  while(1) {
    static uint8_t packet_seen;
    static rtimer_clock_t t0;
    static uint8_t count;

#if SYNC_CYCLE_STARTS
    /* Compute cycle start when RTIMER_ARCH_SECOND is not a multiple
       of CHANNEL_CHECK_RATE */
    if(sync_cycle_phase++ == NETSTACK_RDC_CHANNEL_CHECK_RATE) {
      sync_cycle_phase = 0;
      sync_cycle_start += RTIMER_ARCH_SECOND;
      cycle_start = sync_cycle_start;
    } else {
#if (RTIMER_ARCH_SECOND * NETSTACK_RDC_CHANNEL_CHECK_RATE) > 65535
      cycle_start = sync_cycle_start + ((unsigned long)(sync_cycle_phase*RTIMER_ARCH_SECOND))/NETSTACK_RDC_CHANNEL_CHECK_RATE;
#else
      cycle_start = sync_cycle_start + (sync_cycle_phase*RTIMER_ARCH_SECOND)/NETSTACK_RDC_CHANNEL_CHECK_RATE;
#endif
    }
#else
    cycle_start += CYCLE_TIME;
#endif

    packet_seen = 0;

    postpone_cycle=0;
    reset_AnyBroadCast();
    
    
    for(count = 0; count < CCA_COUNT_MAX; ++count) {
      t0 = RTIMER_NOW();
      if(we_are_sending == 0 && we_are_receiving_burst == 0) {
        powercycle_turn_radio_on();
        /* Check if a packet is seen in the air. If so, we keep the
             radio on for a while (LISTEN_TIME_AFTER_PACKET_DETECTED) to
             be able to receive the packet. We also continuously check
             the radio medium to make sure that we wasn't woken up by a
             false positive: a spurious radio interference that was not
             caused by an incoming packet. */
        if(NETSTACK_RADIO.channel_clear() == 0) {
          packet_seen = 1;
          break;
        }
        powercycle_turn_radio_off();
      }
      schedule_powercycle_fixed(t, RTIMER_NOW() + CCA_SLEEP_TIME);
      PT_YIELD(&pt);
    }

    if(packet_seen) {
      static rtimer_clock_t start;
      static uint8_t silence_periods, periods;
      start = RTIMER_NOW();

      periods = silence_periods = 0;
      while(we_are_sending == 0 && radio_is_on &&
            RTIMER_CLOCK_LT(RTIMER_NOW(),
                            (start + LISTEN_TIME_AFTER_PACKET_DETECTED))) {

        /* Check for a number of consecutive periods of
             non-activity. If we see two such periods, we turn the
             radio off. Also, if a packet has been successfully
             received (as indicated by the
             NETSTACK_RADIO.pending_packet() function), we stop
             snooping. */
#if !RDC_CONF_HARDWARE_CSMA
       /* A cca cycle will disrupt rx on some radios, e.g. mc1322x, rf230 */
       /*TODO: Modify those drivers to just return the internal RSSI when already in rx mode */
        if(NETSTACK_RADIO.channel_clear()) {
          ++silence_periods;
        } else {
          silence_periods = 0;
        }
#endif

        ++periods;

        if(NETSTACK_RADIO.receiving_packet()) {
          silence_periods = 0;
        }
        if(silence_periods > MAX_SILENCE_PERIODS) {
          powercycle_turn_radio_off();
          break;
        }
        if(WITH_FAST_SLEEP &&
            periods > MAX_NONACTIVITY_PERIODS &&
            !(NETSTACK_RADIO.receiving_packet() ||
              NETSTACK_RADIO.pending_packet())) {
          powercycle_turn_radio_off();
          break;
        }
        if(NETSTACK_RADIO.pending_packet()) {
          break;
        }

        schedule_powercycle(t, CCA_CHECK_TIME + CCA_SLEEP_TIME);
        PT_YIELD(&pt);
      }
      if(radio_is_on) {
        if(!(NETSTACK_RADIO.receiving_packet() ||
             NETSTACK_RADIO.pending_packet()) ||
             !RTIMER_CLOCK_LT(RTIMER_NOW(),
                 (start + LISTEN_TIME_AFTER_PACKET_DETECTED))) {
          powercycle_turn_radio_off();
        }
      }
    }

    if(RTIMER_CLOCK_LT(RTIMER_NOW() - cycle_start, CYCLE_TIME - CHECK_TIME * 4)) {
      /* Schedule the next powercycle interrupt, or sleep the mcu
	 until then.  Sleeping will not exit from this interrupt, so
	 ensure an occasional wake cycle or foreground processing will
	 be blocked until a packet is detected */
#if RDC_CONF_MCU_SLEEP
      static uint8_t sleepcycle;
      if((sleepcycle++ < 16) && !we_are_sending && !radio_is_on) {
        rtimer_arch_sleep(CYCLE_TIME - (RTIMER_NOW() - cycle_start));
      } else {
        sleepcycle = 0;
        schedule_powercycle_fixed(t, CYCLE_TIME + cycle_start);
        PT_YIELD(&pt);
      }
#else
      schedule_powercycle_fixed(t, CYCLE_TIME + cycle_start);
      PT_YIELD(&pt);
#endif
    }
  }

  PT_END(&pt);
}
/*---------------------------------------------------------------------------*/
static int
broadcast_rate_drop(void)
{
#if CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT
  if(!timer_expired(&broadcast_rate_timer)) {
    broadcast_rate_counter++;
    if(broadcast_rate_counter < CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT) {
      return 0;
    } else {
      return 1;
    }
  } else {
    timer_set(&broadcast_rate_timer, CLOCK_SECOND);
    broadcast_rate_counter = 0;
    return 0;
  }
#else /* CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT */
  return 0;
#endif /* CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT */
}
/*---------------------------------------------------------------------------*/
static int
send_packet(mac_callback_t mac_callback, void *mac_callback_ptr,
	    struct rdc_buf_list *buf_list,
            int is_receiver_awake)
{
  rtimer_clock_t t0;
  rtimer_clock_t encounter_time = 0;
  int strobes;
  uint8_t got_strobe_ack = 0;
  int hdrlen, len,len1;
  uint8_t is_broadcast = 0;
  uint8_t is_reliable = 0;
  uint8_t is_known_receiver = 0;
  uint8_t collisions;
  int transmit_len;
  int ret;
  uint8_t contikimac_was_on;
  uint8_t seqno;
  

  
#if WITH_CONTIKIMAC_HEADER
  struct hdr *chdr;
#endif /* WITH_CONTIKIMAC_HEADER */

  /* Exit if RDC and radio were explicitly turned off */
   if(!contikimac_is_on && !contikimac_keep_radio_on) {
    PRINTF("contikimac: radio is turned off\n");
    return MAC_TX_ERR_FATAL;
  }
 
  if(packetbuf_totlen() == 0) {
    PRINTF("contikimac: send_packet data len 0\n");
    return MAC_TX_ERR_FATAL;
  }

#if !NETSTACK_CONF_BRIDGE_MODE
  /* If NETSTACK_CONF_BRIDGE_MODE is set, assume PACKETBUF_ADDR_SENDER is already set. */
  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &rimeaddr_node_addr);
#endif
  if(rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), &rimeaddr_null)) {
    is_broadcast = 1;
    PRINTDEBUG("contikimac: send broadcast\n");

    if(broadcast_rate_drop()) {
      return MAC_TX_COLLISION;
    }
  } else {
#if UIP_CONF_IPV6
    PRINTDEBUG("contikimac: send unicast to %02x%02x:%02x%02x:%02x%02x:%02x%02x\n",
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[1],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[2],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[3],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[4],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[5],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[6],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[7]);
#else /* UIP_CONF_IPV6 */
    PRINTDEBUG("contikimac: send unicast to %u.%u\n",
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[1]);
#endif /* UIP_CONF_IPV6 */
  }
  is_reliable = packetbuf_attr(PACKETBUF_ATTR_RELIABLE) ||
    packetbuf_attr(PACKETBUF_ATTR_ERELIABLE);

  packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);



    
// Initialization of MobiDisc
    if (packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0]==100) { // MobiDisc Address
      SendAnyCast=0;
      if  (BroniCastFather==0) {
	SendBroniCast=1;
	
	min_hops=100;
	min_father=0;
	my_got_ack=0;
	BrPhase=1;
	
	set_bronicast_phase(1);
      } else {
	
	set_bronicast_phase(0);
	
	
	SendBroniCast=0;
	rimeaddr_t receiver;
	receiver.u8[0] = BroniCastFather;
	receiver.u8[1] = 0;
	packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &receiver);  // Unicast
	
	prevBronicastFather=BroniCastFather;
      }
     
    }
  
// ----------------------------------------
  
   if (packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0]==99) {
      SendAnyCast=1;
      SendBroniCast=0;
       if (num_tries==0) {
	  if (AnycastFather!=0){
	    //printf("Father\n");
	    char *p=(char *)packetbuf_dataptr();
	    p[2]=AnycastFather ;
	    prevAnycastFather=AnycastFather;
	    
	    is_Any_BroadCast =0;
	    p[6]=rcv_rank;
	  } else {
	  
	    is_Any_BroadCast=1;
	  }
      }else {
	  char *p = (char *)packetbuf_dataptr();
	  p[2]=0;
	  
	  AnycastFather=0;
	  
	  is_Any_BroadCast =1;
	  
      }
            
    }
  
// --------------------------------

#if WITH_CONTIKIMAC_HEADER
  hdrlen = packetbuf_totlen();
  if(packetbuf_hdralloc(sizeof(struct hdr)) == 0) {
    /* Failed to allocate space for contikimac header */
    PRINTF("contikimac: send failed, too large header\n");
    return MAC_TX_ERR_FATAL;
  }
  chdr = packetbuf_hdrptr();
  chdr->id = CONTIKIMAC_ID;
  chdr->len = hdrlen;
  
  /* Create the MAC header for the data packet. */
  hdrlen = NETSTACK_FRAMER.create();
  if(hdrlen < 0) {
    /* Failed to send */
    PRINTF("contikimac: send failed, too large header\n");
    packetbuf_hdr_remove(sizeof(struct hdr));
    return MAC_TX_ERR_FATAL;
  }
  hdrlen += sizeof(struct hdr);
#else

  /* Create the MAC header for the data packet. */
  hdrlen = NETSTACK_FRAMER.create();
  if(hdrlen < 0) {
    /* Failed to send */
    PRINTF("contikimac: send failed, too large header\n");
    return MAC_TX_ERR_FATAL;
  }
#endif

  /* Make sure that the packet is longer or equal to the shortest
     packet length. */
  transmit_len = packetbuf_totlen();
  if(transmit_len < SHORTEST_PACKET_SIZE) {
    /* Pad with zeroes */
    uint8_t *ptr;
    ptr = packetbuf_dataptr();
    memset(ptr + packetbuf_datalen(), 0, SHORTEST_PACKET_SIZE - packetbuf_totlen());

    PRINTF("contikimac: shorter than shortest (%d)\n", packetbuf_totlen());
    transmit_len = SHORTEST_PACKET_SIZE;
  }


  packetbuf_compact();

#ifdef NETSTACK_ENCRYPT
  NETSTACK_ENCRYPT();
#endif /* NETSTACK_ENCRYPT */

  transmit_len = packetbuf_totlen();

  NETSTACK_RADIO.prepare(packetbuf_hdrptr(), transmit_len);

  /* Remove the MAC-layer header since it will be recreated next time around. */
  packetbuf_hdr_remove(hdrlen);

  if (is_mobile==0) {  
  if(!is_broadcast && !is_receiver_awake) {
#if WITH_PHASE_OPTIMIZATION
    ret = phase_wait(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                     CYCLE_TIME, GUARD_TIME,
                     mac_callback, mac_callback_ptr, buf_list);
    if(ret == PHASE_DEFERRED) {
      return MAC_TX_DEFERRED;
    }
    if(ret != PHASE_UNKNOWN) {
      is_known_receiver = 1;
    }
#endif /* WITH_PHASE_OPTIMIZATION */ 
  }
  } 
  


  /* By setting we_are_sending to one, we ensure that the rtimer
     powercycle interrupt do not interfere with us sending the packet. */
  we_are_sending = 1;

  /* If we have a pending packet in the radio, we should not send now,
     because we will trash the received packet. Instead, we signal
     that we have a collision, which lets the packet be received. This
     packet will be retransmitted later by the MAC protocol
     instread. */

  if(NETSTACK_RADIO.receiving_packet() || NETSTACK_RADIO.pending_packet()) {
    we_are_sending = 0;
    return MAC_TX_COLLISION;

  }

  
  /* Switch off the radio to ensure that we didn't start sending while
     the radio was doing a channel check. */
  off();


  strobes = 0;

  /* Send a train of strobes until the receiver answers with an ACK. */
  collisions = 0;

  got_strobe_ack = 0;

  /* Set contikimac_is_on to one to allow the on() and off() functions
     to control the radio. We restore the old value of
     contikimac_is_on when we are done. */
  contikimac_was_on = contikimac_is_on;
  contikimac_is_on = 1;

#if !RDC_CONF_HARDWARE_CSMA
    /* Check if there are any transmissions by others. */
    /* TODO: why does this give collisions before sending with the mc1322x? */
  if(is_receiver_awake == 0) {
    int i;
    for(i = 0; i < CCA_COUNT_MAX_TX; ++i) {
      t0 = RTIMER_NOW();
      on();
#if CCA_CHECK_TIME > 0
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + CCA_CHECK_TIME)) { }
#endif
      if(NETSTACK_RADIO.channel_clear() == 0) {
        collisions++;
        off();
        break;
      }
      off();
      t0 = RTIMER_NOW();
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + CCA_SLEEP_TIME)) { }
    }
  }

  if(collisions > 0) {
    we_are_sending = 0;
    off();
    //printf("contikimac1: collisions before sending\n");
    contikimac_is_on = contikimac_was_on;
    return MAC_TX_COLLISION;
  }
#endif /* RDC_CONF_HARDWARE_CSMA */

#if !RDC_CONF_HARDWARE_ACK
  if(!is_broadcast) {
    /* Turn radio on to receive expected unicast ack.  Not necessary
       with hardware ack detection, and may trigger an unnecessary cca
       or rx cycle */
     on();
  }
#endif

  watchdog_periodic();
  t0 = RTIMER_NOW();
  seqno = packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);

 
  if ( SendBroniCast==0){
	char *p=(char *)packetbuf_dataptr();
	 uint8_t s1=p[3];
	 uint8_t s2=p[4];
	 my_seq =s2*256+s1;
	 if (my_seq==prev_seq) {
	   ++retrans;

	 }else {
	   retrans=0;

	   prev_seq=my_seq;
	 }

  }

  
  for(strobes = 0, collisions = 0;
      got_strobe_ack == 0 && collisions == 0 &&
      RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + STROBE_TIME); strobes++) {


    
    
    
    
    watchdog_periodic();

   // MobiDisc Change Phases
 if (BrPhase!=2) {
   //HandOver
	if (num_tries==0) {
	    if(!is_broadcast && (is_receiver_awake || is_known_receiver) &&
	      !RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + MAX_PHASE_STROBE_TIME)) {
	     
	      break;
	    }
	  } else { 
	
	  if(!is_broadcast && (is_receiver_awake || is_known_receiver) &&
	    !RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + MY_STROBE_TIME)) {
	  
	    break;
	  }
	}
    
 } 
  
    len = 0;

    {
      rtimer_clock_t wt;
      rtimer_clock_t txtime;
      int ret;

      txtime = RTIMER_NOW();
      ret = NETSTACK_RADIO.transmit(transmit_len);

#if RDC_CONF_HARDWARE_ACK
     /* For radios that block in the transmit routine and detect the
	ACK in hardware */
      if(ret == RADIO_TX_OK) {
        if(!is_broadcast) {
          got_strobe_ack = 1;
          encounter_time = txtime;
          break;
        }
      } else if (ret == RADIO_TX_NOACK) {
      } else if (ret == RADIO_TX_COLLISION) {
          PRINTF("contikimac: collisions while sending\n");
          collisions++;
      }
      wt = RTIMER_NOW();
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + INTER_PACKET_INTERVAL)) { }
#else /* RDC_CONF_HARDWARE_ACK */


     /* Wait for the ACK packet */
      wt = RTIMER_NOW();
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + INTER_PACKET_INTERVAL)) { }

      if(!is_broadcast && (NETSTACK_RADIO.receiving_packet() ||
                           NETSTACK_RADIO.pending_packet() ||
                           NETSTACK_RADIO.channel_clear() == 0)) {
        uint8_t ackbuf[ACK_LEN];
        wt = RTIMER_NOW();
        while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + AFTER_ACK_DETECTECT_WAIT_TIME)) { }

        len = NETSTACK_RADIO.read(ackbuf, ACK_LEN);
        
        if(len == ACK_LEN) { 				
          
	       
	  if (SendBroniCast==0) {   // UniCast
	      got_strobe_ack = 1;
	      encounter_time = txtime;			
	      
	     AnycastFather =ackbuf[1];
	    rcv_rank=ackbuf[2];
	    
	    if (retrans>0 && prevBronicastFather != BroniCastFather && is_mobile ==1) {
	      printf("HVR %d %d %d , %d - %d\n",rimeaddr_node_addr.u8[0],my_seq,retrans,BroniCastFather,prevBronicastFather);
	    }
	      
	      break ;
	  }else {					// Select minimum rank
		 my_got_ack++;
		 if (ackbuf[2]<min_hops) {
		  
		  min_hops=ackbuf[2];
		  min_father=ackbuf[1];
		 }   
	    
	  }
	  

         
        } else {

	  if (SendBroniCast==1) {
	   ;// 
	   } else if (SendAnyCast==1) {
		    
	     if (is_Any_BroadCast!=1) {  
		    collisions++;
		    if (is_mobile==1) {
		    ;printf("Mobile Collision \n");
		    }
		  } else {
		  
		    ;
		  }

	   }else {  
	      collisions++;
	    
	  }

        }
        
        
      }
#endif /* RDC_CONF_HARDWARE_ACK */
    }
  }


  
  
 off();
 
  PRINTF("contikimac: send (strobes=%u, len=%u, %s, %s), done\n", strobes,
         packetbuf_totlen(),
         got_strobe_ack ? "ack" : "no ack",
         collisions ? "collision" : "no collision");
  

#if CONTIKIMAC_CONF_COMPOWER
  /* Accumulate the power consumption for the packet transmission. */
  compower_accumulate(&current_packet);

  /* Convert the accumulated poweHVRr consumption for the transmitted
     packet to packet attributes so that the higher levels can keep
     track of the amount of energy spent on transmitting the
     packet. */
  compower_attrconv(&current_packet);

  /* Clear the accumulated power consumption so that it is ready for
     the next packet. */
  compower_clear(&current_packet);
#endif /* CONTIKIMAC_CONF_COMPOWER */AVA_HOME environment variable not set, Cooja motes may not compile


  contikimac_is_on = contikimac_was_on;
  we_are_sending = 0;

  /* Determine the return value that we will return from the
     function. We must pass this value to the phase module before we
     return from the function.  */

	 
  if (SendBroniCast ==0) {			 // UniCast
   
	if(collisions > 0) {
	  ret = MAC_TX_COLLISION;
	} else if(!is_broadcast && !got_strobe_ack) {
	  ret = MAC_TX_NOACK;
	} else {
	  ret = MAC_TX_OK;
	}
  } else {					// My BroadCast
	    if (my_got_ack >0 ) {
	      ret=MAC_TX_OK;
	    }else {
	      ret =MAC_TX_NOACK;
	    }
	    
	    SendBroniCast =0;
	    BroniCastFather=min_father ;
	    printf("Select FatherID %d rank %d ack %d \n",BroniCastFather,min_hops,my_got_ack);
  }
  if (SendAnyCast==1 && AnycastFather!=0) {
  
    BroniCastFather=AnycastFather;
  }

  if (is_mobile==0) {  
#if WITH_PHASE_OPTIMIZATION
  if(is_known_receiver && got_strobe_ack) {
    PRINTF("no miss %d wake-ups %d\n",
	   packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0],
           strobes);
  }else if (strobes!=0) { 
  
    //printf("Miss %d wake-ups %d\n",   packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0],           strobes);
  }

  if(!is_broadcast) {
    if(collisions == 0 && is_receiver_awake == 0) {
      phase_update(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
		   encounter_time, ret);
    }
  }
#endif /* WITH_PHASE_OPTIMIZATION */
  }
//printf("Ret %d %d \n",ret,collisions);



if (is_mobile==1 && BrPhase!=1 && !got_strobe_ack && ret != MAC_TX_OK)  { //ret != MAC_TX_OK
 printf("SVR %d %d %d , %d - %d\n",rimeaddr_node_addr.u8[0],my_seq,retrans,BroniCastFather,prevBronicastFather);
}

  return ret;
}
/*---------------------------------------------------------------------------*/
static void
qsend_packet(mac_callback_t sent, void *ptr)
{

  int ret = send_packet(sent, ptr, NULL, 0);
  if(ret != MAC_TX_DEFERRED) {
    mac_call_sent_callback(sent, ptr, ret, 1);
  }
}
/*---------------------------------------------------------------------------*/
static void
qsend_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
  struct rdc_buf_list *curr = buf_list;
  struct rdc_buf_list *next;
  int ret;
  int is_receiver_awake;
  

  int pkt_num;
  pkt_num=0;
  BrPhase=0;

  
  if(curr == NULL) {
    return;
  }
  /* Do not send during reception of a burst */
  if(we_are_receiving_burst) {
    /* Prepare the packetbuf for callback */
    queuebuf_to_packetbuf(curr->buf);
    /* Return COLLISION so the MAC may try again later */
    mac_call_sent_callback(sent, ptr, MAC_TX_COLLISION, 1);
    return;
  }
  /* The receiver needs to be awoken before we send */
  is_receiver_awake = 0;
  
  // Initialization
    packet = (char *)packetbuf_dataptr();
    packet[2]=0;
    BroniCastFather=0;
    AnycastFather=0;
    
    num_tries=0; //HandOver
   
    
  // ---------    

  do { /* A loop sending a burst of packets from buf_list */

      ++pkt_num ;
  
      if (pkt_num==1 && is_mobile==1 && BroniCastFather==0) {
	
	queuebuf_to_packetbuf(curr->buf);
	
	packet=(char *)packetbuf_dataptr();
	packet[0]=PROBE;
	 
	packetbuf_set_attr(PACKETBUF_ATTR_PENDING, 1); 
	
	
	if (packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0]!=100 &&
	  packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0]!=99 	) { //Hard HandOvER 

	  rimeaddr_t receiver;
	  receiver.u8[0] = 100;
	  receiver.u8[1] = 0;
	  packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &receiver);
	}
	
	ret = send_packet(sent, ptr, curr, is_receiver_awake);

	if(ret != MAC_TX_DEFERRED) {
	  //mac_call_sent_callback(sent, ptr, ret, 1);
	}
	if(ret == MAC_TX_OK) {
	      is_receiver_awake = 1;      
	      
	} else {
	   printf("Exit \n");
	   mac_call_sent_callback(sent, ptr, ret, 1);
	   off(); 
	  return; 
	  }
	  
      }
       ++pkt_num ;
       if (pkt_num==2 && is_mobile==1) {
	 packet[0]=DATA ;
	 BrPhase=2;
       } else  {
	 BrPhase=0;
       }
  
   
   
      next = list_item_next(curr);

    /* Prepare the packetbuf */
    queuebuf_to_packetbuf(curr->buf);
    
   
	 
    if(next != NULL) {
      packetbuf_set_attr(PACKETBUF_ATTR_PENDING, 1);
     
      if (is_mobile==1) {
	SendBurst=1;
      }
    }

    /* Send the current packet */
  
    ret = send_packet(sent, ptr, curr, is_receiver_awake);  
    
    // Seamless  HandOver 
    
    if (is_mobile==1 && ret== MAC_TX_NOACK && num_tries==0) {
	++num_tries ;
	packet=(char *)packetbuf_dataptr();
	packet[0]=PROBE;
	
	  rimeaddr_t receiver;
	  receiver.u8[0] = 99;
	  receiver.u8[1] = 0;
	  packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &receiver);   // Allagh se anycast
	  printf("ANYCAST \n" );
	ret = send_packet(sent, ptr, curr, is_receiver_awake);    // Second Try na balw edw to
	
	
	packet[0]=DATA ;
	num_tries=0 ;
	if (ret==MAC_TX_OK) {
	      ret = send_packet(sent, ptr, curr, is_receiver_awake);
	}else {
	  printf("Fail \n");
	}
    }
    
    if(ret != MAC_TX_DEFERRED) {
      mac_call_sent_callback(sent, ptr, ret, 1);
    }
 //printf("aret %d \n",ret);
    if(ret == MAC_TX_OK) {
      if(next != NULL) {
        /* We're in a burst, no need to wake the receiver up again */
        is_receiver_awake = 1;
	
	  curr = next;  
	
      }
    } else {
      /* The transmission failed, we stop the burst */
      if (is_mobile==1) {
     // printf (" REs ACK %d packets -%d \n",ret,pkt_num);
	BroniCastFather=0;
      }
      next = NULL;
    }
  } while(next != NULL);

    if (is_mobile==1) {
     // printf ("Packets -%d \n",pkt_num);
      }
    off();
  
}
/*---------------------------------------------------------------------------*/
/* Timer callback triggered when receiving a burst, after having
   waited for a next packet for a too long time. Turns the radio off
   and leaves burst reception mode */
static void
recv_burst_off(void *ptr)
{

  off();
  we_are_receiving_burst = 0;
  

  set_RCV_burst(0);
  set_postpone_cycle(0);

}
/*---------------------------------------------------------------------------*/
static void
input_packet(void)
{
  //printf("Input \n"); 
  //printf("Lentgh %d \n", packetbuf_totlen());
   if (packetbuf_totlen()==3) {
      // printf("Input ACK \n");
	  on();
       return;
    }
    
    
    
    
    
  static struct ctimer ct;
  if(!we_are_receiving_burst) {
    off();
  }



#ifdef NETSTACK_DECRYPT
  NETSTACK_DECRYPT();
#endif /* NETSTACK_DECRYPT */

  if(packetbuf_totlen() > 0 && NETSTACK_FRAMER.parse() >= 0) {

#if WITH_CONTIKIMAC_HEADER
    struct hdr *chdr;
    chdr = packetbuf_dataptr();
    

   
    if(chdr->id != CONTIKIMAC_ID) {
    //  printf("contikimac: failed to parse hdr (%u)\n", packetbuf_totlen());
      return;
    }
    packetbuf_hdrreduce(sizeof(struct hdr));
    packetbuf_set_datalen(chdr->len);
#endif /* WITH_CONTIKIMAC_HEADER */
    

   
     packet = (char *)packetbuf_dataptr();
     
     
 
    
   if (packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0] == 99 && packet[2]!=0 && packet[2]!=rimeaddr_node_addr.u8[0]) {
  
     off() ;
     set_RCV_burst(0);
  }
  
    if(packetbuf_datalen() > 0 &&
       packetbuf_totlen() > 0 &&
       (rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), &rimeaddr_node_addr) ||   // Unicast
        rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), &rimeaddr_null)) || 	  // Broadcast
	(is_mobile==0 && packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0] == 99 && (packet[2]==0 || packet[2]==rimeaddr_node_addr.u8[0]))) {			//  Anycast
   // ---------
   
      /* This is a regular packet that is destined to us or to the
         broadcast address. */
	//printf(" Entering -Contiki mac \n");
      /* If FRAME_PENDING is set, we are receiving a packets in a burst */
      we_are_receiving_burst = packetbuf_attr(PACKETBUF_ATTR_PENDING);
      set_RCV_burst(we_are_receiving_burst);	
      if(we_are_receiving_burst) {
        on();
        /* Set a timer to turn the radio off in case we do not receive
	   a next packet */
        ctimer_set(&ct, INTER_PACKET_DEADLINE, recv_burst_off, NULL);
      } else  if(packet[0]!=PROBE) {
	on();
        /* Set a timer to turn the radio off in case we do not receive
	   a next packet */
        ctimer_set(&ct, INTER_PACKET_DEADLINE, recv_burst_off, NULL);
      }
      else{
        off();
        ctimer_stop(&ct);
	
      }

      /* Check for duplicate packet by comparing the sequence number
         of the incoming packet with the last few ones we saw. */
      
      if (packet[0]!=PROBE)                               

      {
        int i;
        for(i = 0; i < MAX_SEQNOS; ++i) {
          if(packetbuf_attr(PACKETBUF_ATTR_PACKET_ID) == received_seqnos[i].seqno &&
             rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_SENDER),
                          &received_seqnos[i].sender)) {
            /* Drop the packet. */
              // printf("Drop duplicate ContikiMAC layer packet\n");
            return;
          }
        }
        for(i = MAX_SEQNOS - 1; i > 0; --i) {
          memcpy(&received_seqnos[i], &received_seqnos[i - 1],
                 sizeof(struct seqno));
        }
        received_seqnos[0].seqno = packetbuf_attr(PACKETBUF_ATTR_PACKET_ID);
        rimeaddr_copy(&received_seqnos[0].sender,
                      packetbuf_addr(PACKETBUF_ADDR_SENDER));
      }

#if CONTIKIMAC_CONF_COMPOWER
      /* Accumulate the power consumption for the packet reception. */
      compower_accumulate(&current_packet);
      /* Convert the accumulated power consumption for the received
         packet to packet attributes so that the higher levels can
         keep track of the amount of energy spent on receiving the
         packet. */
      compower_attrconv(&current_packet);

      /* Clear the accumulated power consumption so that it is ready
         for the next packet. */
      compower_clear(&current_packet);
#endif /* CONTIKIMAC_CONF_COMPOWER */

      PRINTDEBUG("contikimac: data (%u)\n", packetbuf_datalen());
      
      NETSTACK_MAC.input();
      return;
    } else if (is_mobile==0 && packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0] == 100){
      
      
	static struct ctimer timerBr;
	uint8_t s1=packet[7];
	uint8_t s2=packet[8];
	static rtimer_clock_t t1;
	
	rtimer_clock_t wakeupTime=CYCLE_TIME - (s1 +s2*256); 
	printf("WakeUp time %d ~~ %d \n",wakeupTime,(s1 +s2*256) );
	
	t1=RTIMER_NOW();
	for(; RTIMER_CLOCK_LT(RTIMER_NOW(), t1 + wakeupTime+ RTIMER_ARCH_SECOND / 50);) {;}
	printf (" wakeeeeeeeeeeee\n");
	
	struct rtimer *rt1;
	void *pnt;
	
	
	
	//powercycle(rt1,pnt);
	//ctimer_set(&timerBr, CYCLE_TIME, WakeUp, NULL);
	  PRINTDEBUG("contikimac: data not for us\n");
    }
  } else {
    PRINTF("contikimac: failed to parse (%u)\n", packetbuf_totlen());
  }
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
  radio_is_on = 0;
  PT_INIT(&pt);

  rtimer_set(&rt, RTIMER_NOW() + CYCLE_TIME, 1,
             (void (*)(struct rtimer *, void *))powercycle, NULL);

  contikimac_is_on = 1;

 
  
#if WITH_PHASE_OPTIMIZATION
  phase_init();
#endif /* WITH_PHASE_OPTIMIZATION */

}
/*---------------------------------------------------------------------------*/
static int
turn_on(void)
{
  if(contikimac_is_on == 0) {
    contikimac_is_on = 1;
    contikimac_keep_radio_on = 0;
    rtimer_set(&rt, RTIMER_NOW() + CYCLE_TIME, 1,
               (void (*)(struct rtimer *, void *))powercycle, NULL);
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
turn_off(int keep_radio_on)
{


  contikimac_is_on = 0;
  contikimac_keep_radio_on = keep_radio_on;
  if(keep_radio_on) {
    radio_is_on = 1;
    return NETSTACK_RADIO.on();
  } else {
    radio_is_on = 0;
    return NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static unsigned short
duty_cycle(void)
{
  return (1ul * CLOCK_SECOND * CYCLE_TIME) / RTIMER_ARCH_SECOND;
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver contikimac_driver = {
  "ContikiMAC",
  init,
  qsend_packet,
  qsend_list,
  input_packet,
  turn_on,
  turn_off,
  duty_cycle,
};
/*---------------------------------------------------------------------------*/
uint16_t
contikimac_debug_print(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/


//  Extra Functions
void set_mobile_mac()
{
  is_mobile =1 ;
}


void set_rank_mac(int r)
{
  my_rank =r;
}

void set_postpone_cycle(int k) {
  postpone_cycle=k;
  
}

void WakeUp() {
  printf("WakeUp  ~~~ \n");
}
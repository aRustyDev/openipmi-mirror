/*
 * mc.c
 *
 * MontaVista IPMI code for handling management controllers
 *
 * Author: MontaVista Software, Inc.
 *         Corey Minyard <minyard@mvista.com>
 *         source@mvista.com
 *
 * Copyright 2002,2003 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public License
 *  as published by the Free Software Foundation; either version 2 of
 *  the License, or (at your option) any later version.
 *
 *
 *  THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this program; if not, write to the Free
 *  Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <string.h>

#include <OpenIPMI/ipmi_conn.h>
#include <OpenIPMI/ipmiif.h>
#include <OpenIPMI/ipmi_domain.h>
#include <OpenIPMI/ipmi_mc.h>
#include <OpenIPMI/ipmi_sdr.h>
#include <OpenIPMI/ipmi_sel.h>
#include <OpenIPMI/ipmi_entity.h>
#include <OpenIPMI/ipmi_sensor.h>
#include <OpenIPMI/ipmi_msgbits.h>
#include <OpenIPMI/ipmi_err.h>
#include <OpenIPMI/ipmi_int.h>
#include <OpenIPMI/ipmi_oem.h>

#include "ilist.h"
#include "opq.h"

/* Re-query the SEL every 10 seconds by default. */
#define IPMI_SEL_QUERY_INTERVAL 10

/* Timer structure for rereading the SEL. */
typedef struct mc_reread_sel_s
{
    int cancelled;
    ipmi_mc_t *mc;
} mc_reread_sel_t;
    
struct ipmi_mc_s
{
    ipmi_domain_t *domain;
    long          seq;
    ipmi_addr_t   addr;
    int           addr_len;

    /* True if the MC is a valid MC in the system, false if not.
       Primarily used to handle shutdown races, where the MC still
       exists in lists but has been shut down. */
    int valid;

    /* If the MC is known to be good in the system, then active is
       true.  If active is false, that means that there are sensors
       that refer to this MC, but the MC is not currently in the
       system. */
    int active;

    /* The device SDRs on the MC. */
    ipmi_sdr_info_t *sdrs;

    /* The sensors that came from the device SDR on this MC. */
    ipmi_sensor_t **sensors_in_my_sdr;
    unsigned int  sensors_in_my_sdr_count;

    /* Sensors that this MC owns (you message this MC to talk to this
       sensor, and events report the MC as the owner. */
    ipmi_sensor_info_t  *sensors;

    ipmi_control_info_t *controls;

    unsigned int in_domain_list : 1; /* Tells if we are in the list of
					our domain yet. */

    /* The system event log, for querying and storing events. */
    ipmi_sel_info_t *sel;

    /* The handler to call for delete event operations.  This is NULL
       normally and is only used if the MC has a special delete event
       handler. */
    ipmi_mc_del_event_cb sel_del_event_handler;

    /* Timer for rescanning the sel periodically. */
    os_hnd_timer_id_t *sel_timer;
    mc_reread_sel_t   *sel_timer_info;
    unsigned int      sel_scan_interval; /* seconds between SEL scans */


    /* The SEL time when the connection first came up.  Only used at
       startup, after the SEL has been read the first time this will
       be set to zero. */
    unsigned long startup_SEL_time;

    void *oem_data;

    ipmi_mc_oem_new_sensor_cb new_sensor_handler;
    void                      *new_sensor_cb_data;

    ipmi_oem_event_handler_cb oem_event_handler;
    void                      *oem_event_cb_data;

    ipmi_oem_event_handler_cb sel_oem_event_handler;
    void                      *sel_oem_event_cb_data;

    ipmi_mc_oem_removed_cb removed_mc_handler;
    void                   *removed_mc_cb_data;

    /* The rest is the actual data from the get device id and SDRs.
       There's the real version and the normal version, the real
       version is the one from the get device id response, the normal
       version may have been adjusted by the OEM code. */

    uint8_t device_id;

    uint8_t device_revision;

    unsigned int provides_device_sdrs : 1;
    unsigned int device_available : 1;

    unsigned int chassis_support : 1;
    unsigned int bridge_support : 1;
    unsigned int IPMB_event_generator_support : 1;
    unsigned int IPMB_event_receiver_support : 1;
    unsigned int FRU_inventory_support : 1;
    unsigned int SEL_device_support : 1;
    unsigned int SDR_repository_support : 1;
    unsigned int sensor_device_support : 1;

    uint8_t major_fw_revision;
    uint8_t minor_fw_revision;

    uint8_t major_version;
    uint8_t minor_version;

    uint32_t manufacturer_id;
    uint16_t product_id;

    uint8_t  aux_fw_revision[4];

    uint8_t real_device_id;

    uint8_t real_device_revision;

    unsigned int real_provides_device_sdrs : 1;
    unsigned int real_device_available : 1;

    unsigned int real_chassis_support : 1;
    unsigned int real_bridge_support : 1;
    unsigned int real_IPMB_event_generator_support : 1;
    unsigned int real_IPMB_event_receiver_support : 1;
    unsigned int real_FRU_inventory_support : 1;
    unsigned int real_SEL_device_support : 1;
    unsigned int real_SDR_repository_support : 1;
    unsigned int real_sensor_device_support : 1;

    uint8_t real_major_fw_revision;
    uint8_t real_minor_fw_revision;

    uint8_t real_major_version;
    uint8_t real_minor_version;

    uint32_t real_manufacturer_id;
    uint16_t real_product_id;

    uint8_t  real_aux_fw_revision[4];
};

static void mc_sel_new_event_handler(ipmi_sel_info_t *sel,
				     ipmi_mc_t       *mc,
				     ipmi_event_t    *event,
				     void            *cb_data);

/***********************************************************************
 *
 * Routines for creating and destructing MCs.
 *
 **********************************************************************/

int
_ipmi_create_mc(ipmi_domain_t *domain,
		ipmi_addr_t   *addr,
		unsigned int  addr_len,
		ipmi_mc_t     **new_mc)
{
    ipmi_mc_t *mc;
    int       rv = 0;

    if (addr_len > sizeof(ipmi_addr_t))
	return EINVAL;

    mc = ipmi_mem_alloc(sizeof(*mc));
    if (!mc)
	return ENOMEM;
    memset(mc, 0, sizeof(*mc));

    mc->domain = domain;

    mc->seq = ipmi_get_seq();

    mc->active = 1;

    mc->sensors = NULL;
    mc->sensors_in_my_sdr = NULL;
    mc->sensors_in_my_sdr_count = 0;
    mc->controls = NULL;
    mc->new_sensor_handler = NULL;
    mc->removed_mc_handler = NULL;
    mc->sel = NULL;
    mc->sel_timer_info = NULL;
    mc->sel_scan_interval = IPMI_SEL_QUERY_INTERVAL;

    memcpy(&(mc->addr), addr, addr_len);
    mc->addr_len = addr_len;
    mc->sdrs = NULL;

    rv = ipmi_sensors_alloc(mc, &(mc->sensors));
    if (rv)
	goto out_err;

    rv = ipmi_controls_alloc(mc, &(mc->controls));
    if (rv)
	goto out_err;

    rv = ipmi_sel_alloc(mc, 0, &(mc->sel));
    if (rv)
	goto out_err;

    rv = ipmi_sdr_info_alloc(domain, mc, 0, 1, &(mc->sdrs));
    if (rv)
	goto out_err;

    /* When we get new logs, handle them. */
    ipmi_sel_set_new_event_handler(mc->sel,
				   mc_sel_new_event_handler,
				   domain);

 out_err:
    if (rv)
	_ipmi_cleanup_mc(mc);
    else
	*new_mc = mc;

    return rv;
}

static os_handler_t *
mc_get_os_hnd(ipmi_mc_t *mc)
{
    ipmi_domain_t *domain = mc->domain;
    return ipmi_domain_get_os_hnd(domain);
}

void
_ipmi_cleanup_mc(ipmi_mc_t *mc)
{
    int           i;
    int           rv;
    ipmi_domain_t *domain = mc->domain;
    os_handler_t  *os_hnd = ipmi_domain_get_os_hnd(domain);

    /* First the device SDR sensors, since they can be there for any
       MC. */
    if (mc->sensors_in_my_sdr) {
	for (i=0; i<mc->sensors_in_my_sdr_count; i++) {
	    if (mc->sensors_in_my_sdr[i])
		ipmi_sensor_destroy(mc->sensors_in_my_sdr[i]);
	}
	ipmi_mem_free(mc->sensors_in_my_sdr);
	mc->sensors_in_my_sdr = NULL;
    }

    /* Make sure the timer stops. */
    if (mc->sel_timer_info) {
	mc->sel_timer_info->cancelled = 1;
	rv = os_hnd->stop_timer(os_hnd, mc->sel_timer);
	if (!rv) {
	    /* If we can stop the timer, free it and it's info.
	       If we can't stop the timer, that means that the
	       code is currently in the timer handler, so we let
	       the "cancelled" value do this for us. */
	    os_hnd->free_timer(os_hnd, mc->sel_timer);
	    ipmi_mem_free(mc->sel_timer_info);
	}
	mc->sel_timer_info = NULL;
    }

    /* Call the OEM handler for removal, if it has been registered. */
    if (mc->removed_mc_handler)
	mc->removed_mc_handler(domain, mc, mc->removed_mc_cb_data);

    if ((ipmi_controls_get_count(mc->controls) == 0)
	&& (ipmi_sensors_get_count(mc->sensors) == 0))
    {
	/* There are no sensors associated with this MC, so it's safe
           to delete it.  If there are sensors that stil reference
           this MC (such as from another MC's SDR repository, or the
           main SDR repository) we have to leave it inactive but not
           delete it. */
	_ipmi_remove_mc_from_domain(domain, mc);

	if (mc->sensors)
	    ipmi_sensors_destroy(mc->sensors);
	if (mc->controls)
	    ipmi_controls_destroy(mc->controls);
	if (mc->sdrs)
	    ipmi_sdr_info_destroy(mc->sdrs, NULL, NULL);
	if (mc->sel)
	    ipmi_sel_destroy(mc->sel, NULL, NULL);

	ipmi_mem_free(mc);
    } else {
	mc->active = 0;
    }
}

/***********************************************************************
 *
 * Event handling.
 *
 **********************************************************************/

/* Got a new event in the system event log that we didn't have before. */
static void
mc_sel_new_event_handler(ipmi_sel_info_t *sel,
			 ipmi_mc_t       *mc,
			 ipmi_event_t    *event,
			 void            *cb_data)
{
    _ipmi_domain_system_event_handler(cb_data, mc, event);
}

int
_ipmi_mc_check_oem_event_handler(ipmi_mc_t *mc, ipmi_event_t *event)
{
    if (mc->oem_event_handler)
	return (mc->oem_event_handler(mc, event, mc->oem_event_cb_data));
    else
	return 0;
}

int
_ipmi_mc_check_sel_oem_event_handler(ipmi_mc_t *mc, ipmi_event_t *event)
{
    if (mc->sel_oem_event_handler)
	return (mc->sel_oem_event_handler(mc, event,
					  mc->sel_oem_event_cb_data));
    else
	return 0;
}


/***********************************************************************
 *
 * SEL handling.
 *
 **********************************************************************/

void
_ipmi_mc_sel_event_add(ipmi_mc_t *mc, ipmi_event_t *event)
{
    ipmi_sel_event_add(mc->sel, event);
}

unsigned long
ipmi_mc_get_startup_SEL_time(ipmi_mc_t *mc)
{
    return mc->startup_SEL_time;
}

void
ipmi_mc_set_del_event_handler(ipmi_mc_t            *mc,
			      ipmi_mc_del_event_cb handler)
{
    mc->sel_del_event_handler = handler;
}

void
ipmi_mc_set_sel_rescan_time(ipmi_mc_t *mc, unsigned int seconds)
{
    CHECK_MC_LOCK(mc);

    mc->sel_scan_interval = seconds;
}

unsigned int
ipmi_mc_get_sel_rescan_time(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);

    return mc->sel_scan_interval;
}

typedef struct sel_op_done_info_s
{
    ipmi_mc_t  *mc;
    ipmi_mc_cb done;
    void       *cb_data;
} sel_op_done_info_t;

static void
sel_op_done(ipmi_sel_info_t *sel,
	    void            *cb_data,
	    int             err)
{
    sel_op_done_info_t *info = cb_data;

    /* No need to lock, the domain/mc should already be locked. */
    if (info->done)
        info->done(info->mc, err, info->cb_data);
    ipmi_mem_free(info);
}

int
_ipmi_mc_del_event(ipmi_mc_t                 *mc,
		   ipmi_event_t              *event, 
		   ipmi_mc_del_event_done_cb handler,
		   void                      *cb_data)
{
    sel_op_done_info_t *sel_info;
    int                rv;

    if (!mc->SEL_device_support)
	return EINVAL;

    /* If we have an OEM handler, call it instead. */
    if (mc->sel_del_event_handler) {
	rv = mc->sel_del_event_handler(mc, event, handler, cb_data);
	return rv;
    }

    sel_info = ipmi_mem_alloc(sizeof(*sel_info));
    if (!sel_info)
	return ENOMEM;

    sel_info->mc = mc;
    sel_info->done = handler;
    sel_info->cb_data = cb_data;

    rv = ipmi_sel_del_event(mc->sel, event, sel_op_done, sel_info);
    if (rv)
	ipmi_mem_free(sel_info);

    return rv;
}

int
_ipmi_mc_next_event(ipmi_mc_t *mc, ipmi_event_t *event)
{
    return ipmi_sel_get_next_event(mc->sel, event);
}

int
_ipmi_mc_prev_event(ipmi_mc_t *mc, ipmi_event_t *event)
{
    return ipmi_sel_get_prev_event(mc->sel, event);
}

int
_ipmi_mc_last_event(ipmi_mc_t *mc, ipmi_event_t *event)
{
    return ipmi_sel_get_last_event(mc->sel, event);
}

int
_ipmi_mc_first_event(ipmi_mc_t *mc, ipmi_event_t *event)
{
    return ipmi_sel_get_first_event(mc->sel, event);
}

int
_ipmi_mc_sel_count(ipmi_mc_t *mc)
{
    unsigned int val = 0;

    ipmi_get_sel_count(mc->sel, &val);
    return val;
}

int
_ipmi_mc_sel_entries_used(ipmi_mc_t *mc)
{
    unsigned int val = 0;

    ipmi_get_sel_entries_used(mc->sel, &val);
    return val;
}

int
ipmi_mc_set_oem_event_handler(ipmi_mc_t                 *mc,
			      ipmi_oem_event_handler_cb handler,
			      void                      *cb_data)
{
    mc->oem_event_handler = handler;
    mc->oem_event_cb_data = cb_data;
    return 0;
}

int
ipmi_mc_set_sel_oem_event_handler(ipmi_mc_t                 *mc,
				  ipmi_oem_event_handler_cb handler,
				  void                      *cb_data)
{
    mc->sel_oem_event_handler = handler;
    mc->sel_oem_event_cb_data = cb_data;
    return 0;
}

static void mc_reread_sel(void *cb_data, os_hnd_timer_id_t *id);

static void
sels_fetched_start_timer(ipmi_sel_info_t *sel,
			 int             err,
			 int             changed,
			 unsigned int    count,
			 void            *cb_data)
{
    mc_reread_sel_t *info = cb_data;
    ipmi_mc_t       *mc = info->mc;
    os_handler_t    *os_hnd;
    struct timeval  timeout;

    if (info->cancelled) {
	ipmi_mem_free(info);
	return;
    }

    /* After the first SEL fetch, disable looking at the timestamp, in
       case someone messes with the SEL time. */
    mc->startup_SEL_time = 0;

    os_hnd = mc_get_os_hnd(mc);

    timeout.tv_sec = mc->sel_scan_interval;
    timeout.tv_usec = 0;
    os_hnd->start_timer(os_hnd,
			mc->sel_timer,
			&timeout,
			mc_reread_sel,
			info);
}

static void
mc_reread_sel(void *cb_data, os_hnd_timer_id_t *id)
{
    mc_reread_sel_t *info = cb_data;
    ipmi_mc_t       *mc = info->mc;
    int             rv = EINVAL;

    if (info->cancelled) {
	ipmi_mem_free(info);
	return;
    }

    /* Only fetch the SEL if we know the connection is up. */
    if (ipmi_domain_con_up(mc->domain))
	rv = ipmi_sel_get(mc->sel, sels_fetched_start_timer, info);

    /* If we couldn't run the SEL get, then restart the timer now. */
    if (rv)
	sels_fetched_start_timer(mc->sel, 0, 0, 0, info);
}


/***********************************************************************
 *
 * Handling startup of a new MC
 *
 **********************************************************************/

static void
set_event_rcvr_done(ipmi_mc_t  *mc,
		    ipmi_msg_t *rsp,
		    void       *rsp_data)
{
    if (!mc)
	return; /* The MC went away, no big deal. */

    if (rsp->data[0] != 0) {
	/* Error setting the event receiver, report it. */
	ipmi_log(IPMI_LOG_WARNING,
		 "Could not set event receiver for MC at 0x%x",
		 ipmi_addr_get_slave_addr(&mc->addr));
    }
}

static void
send_set_event_rcvr(ipmi_mc_t *mc, unsigned int addr)
{
    ipmi_msg_t    msg;
    unsigned char data[2];
    
    msg.netfn = IPMI_SENSOR_EVENT_NETFN;
    msg.cmd = IPMI_SET_EVENT_RECEIVER_CMD;
    msg.data = data;
    msg.data_len = 2;
    data[0] = addr;
    data[1] = 0; /* LUN is 0 per the spec (section 7.2 of 1.5 spec). */
    ipmi_mc_send_command(mc, 0, &msg, set_event_rcvr_done, NULL);
    /* No care about return values, if this fails it will be done
       again later. */
}

static void
get_event_rcvr_done(ipmi_mc_t  *mc,
		    ipmi_msg_t *rsp,
		    void       *rsp_data)
{
    unsigned long addr = (long) rsp_data;

    if (!mc)
	return; /* The MC went away, no big deal. */

    if (rsp->data[0] != 0) {
	/* Error getting the event receiver, report it. */
	ipmi_log(IPMI_LOG_WARNING,
		 "Could not get event receiver for MC at 0x%x",
		 ipmi_addr_get_slave_addr(&mc->addr));
    } else if (rsp->data[1] != addr) {
	/* The event receiver doesn't match, so change it. */
	send_set_event_rcvr(mc, addr);
    }
}

static void
send_get_event_rcvr(ipmi_mc_t *mc, unsigned int addr)
{
    ipmi_msg_t    msg;
    
    msg.netfn = IPMI_SENSOR_EVENT_NETFN;
    msg.cmd = IPMI_GET_EVENT_RECEIVER_CMD;
    msg.data = NULL;
    msg.data_len = 0;
    ipmi_mc_send_command(mc, 0, &msg, get_event_rcvr_done,
			 (void *) (unsigned long) addr);
    /* No care about return values, if this fails it will be done
       again later. */
}

void
_ipmi_mc_check_event_rcvr(ipmi_mc_t *mc)
{
    if (mc && mc->IPMB_event_generator_support) {
	/* We have an MC that is live (or still live) and generates
	   events, make sure the event receiver is set properly. */
	unsigned int event_rcvr = ipmi_domain_get_event_rcvr(mc->domain);

	if (event_rcvr) {
	    send_get_event_rcvr(mc, event_rcvr);
	}
    }
}

static void
set_sel_time(ipmi_mc_t  *mc,
	     ipmi_msg_t *rsp,
	     void       *rsp_data)
{
    if (!mc) {
	ipmi_log(IPMI_LOG_WARNING, "MC went away during SEL time set");
	return;
    }

    if (rsp->data[0] != 0) {
	ipmi_log(IPMI_LOG_WARNING,
		 "Unable to set the SEL time due to error: %x",
		 rsp->data[0]);
	mc->startup_SEL_time = 0;
    }

    ipmi_sel_get(mc->sel, sels_fetched_start_timer, mc->sel_timer_info);
}

/* This is called after the first sensor scan for the MC, we start up
   timers and things like that here. */
static void
sensors_reread(ipmi_mc_t *mc, int err, void *cb_data)
{
    unsigned char  data[4];
    struct timeval now;

    if (mc) {
	unsigned int event_rcvr = 0;

	/* See if any presence has changed with the new sensors. */ 
	ipmi_detect_domain_presence_changes(mc->domain, 0);

	/* We set the event receiver here, so that we know all the SDRs
	   are installed.  That way any incoming events from the device
	   will have the proper sensor set. */
	if (mc->IPMB_event_generator_support)
	    event_rcvr = ipmi_domain_get_event_rcvr(mc->domain);
	else if (mc->SEL_device_support)
	    /* If it is an SEL device and not an event receiver, then
                set it's event receiver to itself. */
	    event_rcvr = ipmi_mc_get_address(mc);

	if (event_rcvr)
	    send_set_event_rcvr(mc, event_rcvr);
    }

    if (mc->SEL_device_support) {
	mc_reread_sel_t *info;
	int             rv;
	ipmi_msg_t      msg;
	os_handler_t    *os_hnd = mc_get_os_hnd(mc);

	/* If the MC supports an SEL, start scanning its SEL. */

	/* Allocate the system event log fetch timer. */
	info = ipmi_mem_alloc(sizeof(*info));
	if (!info) {
	    ipmi_log(IPMI_LOG_SEVERE,
		     "Unable to allocate info for system event log timer."
		     " System event log will not be queried");
	    return;
	}
	info->mc = mc;
	info->cancelled = 0;
	rv = os_hnd->alloc_timer(os_hnd, &(mc->sel_timer));
	if (rv) {
	    ipmi_mem_free(info);
	    ipmi_log(IPMI_LOG_SEVERE,
		     "Unable to allocate the system event log timer."
		     " System event log will not be queried");
	} else {
	    mc->sel_timer_info = info;
	}

	/* Set the current system event log time.  We do this here so
	   we can be sure that the entities are all there before
	   reporting events. */
	gettimeofday(&now, NULL);
	msg.netfn = IPMI_STORAGE_NETFN;
	msg.cmd = IPMI_SET_SEL_TIME_CMD;
	msg.data = data;
	msg.data_len = 4;
	ipmi_set_uint32(data, now.tv_sec);
	mc->startup_SEL_time = now.tv_sec;
	rv = ipmi_mc_send_command(mc, 0, &msg, set_sel_time, NULL);
	if (rv) {
	    ipmi_log(IPMI_LOG_DEBUG,
		     "Unable to start SEL time set due to error: %x\n",
		     rv);
	    mc->startup_SEL_time = 0;
	    ipmi_sel_get(mc->sel, sels_fetched_start_timer, mc->sel_timer_info);
	}
    }
}


static void
mc_sdr_handler(ipmi_sdr_info_t *sdrs,
	       int             err,
	       int             changed,
	       unsigned int    count,
	       void            *cb_data)
{
    ipmi_mc_t  *mc = (ipmi_mc_t *) cb_data;

    if (err) {
	_ipmi_cleanup_mc(mc);
	return;
    }

    /* Scan all the sensors and call sensors_reread() when done. */
    ipmi_mc_reread_sensors(mc, sensors_reread, NULL);
}

int
_ipmi_mc_handle_new(ipmi_mc_t *mc)
{
    int rv = 0;

    if (mc->provides_device_sdrs)
	rv = ipmi_sdr_fetch(mc->sdrs, mc_sdr_handler, mc);
    else
	sensors_reread(mc, 0, NULL);

    return rv;
}

/***********************************************************************
 *
 * MC ID handling
 *
 **********************************************************************/

ipmi_mcid_t
_ipmi_mc_convert_to_id(ipmi_mc_t *mc)
{
    ipmi_mcid_t val;

    CHECK_MC_LOCK(mc);

    val.domain_id = ipmi_domain_convert_to_id(mc->domain);
    val.mc_num = ipmi_mc_get_address(mc);
    val.channel = ipmi_mc_get_channel(mc);
    val.seq = mc->seq;
    return val;
}

typedef struct mc_ptr_info_s
{
    int              err;
    int              cmp_seq;
    ipmi_mcid_t      id;
    _ipmi_mc_ptr_cb  handler;
    void             *cb_data;
} mc_ptr_info_t;

static void
mc_ptr_cb(ipmi_domain_t *domain, void *cb_data)
{
    mc_ptr_info_t *info = cb_data;
    ipmi_addr_t   addr;
    unsigned int  addr_len;
    ipmi_mc_t     *mc;

    if (info->id.channel == IPMI_BMC_CHANNEL) {
	ipmi_system_interface_addr_t *si = (void *) &addr;

	si->addr_type = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
	si->channel = info->id.channel;
	si->lun = 0;
	addr_len = sizeof(*si);
    } else {
	ipmi_ipmb_addr_t *ipmb = (void *) &addr;

	ipmb->addr_type = IPMI_IPMB_ADDR_TYPE;
	ipmb->channel = info->id.channel;
	ipmb->slave_addr = info->id.mc_num;
	ipmb->lun = 0;
	addr_len = sizeof(*ipmb);
    }

    mc = _ipmi_find_mc_by_addr(domain, &addr, addr_len);
    if (mc) {
	if (info->cmp_seq && (mc->seq != info->id.seq))
	    return;

	info->err = 0;
	info->handler(mc, info->cb_data);
    }
}

int
_ipmi_mc_pointer_cb(ipmi_mcid_t id, _ipmi_mc_ptr_cb handler, void *cb_data)
{
    int           rv;
    mc_ptr_info_t info;

    info.err = EINVAL;
    info.id = id;
    info.handler = handler;
    info.cb_data = cb_data;
    info.cmp_seq = 1;
    rv = ipmi_domain_pointer_cb(id.domain_id, mc_ptr_cb, &info);
    if (!rv)
	rv = info.err;
    return rv;
}

int
_ipmi_mc_pointer_noseq_cb(ipmi_mcid_t     id,
			  _ipmi_mc_ptr_cb handler,
			  void            *cb_data)
{
    int           rv;
    mc_ptr_info_t info;

    info.err = EINVAL;
    info.id = id;
    info.handler = handler;
    info.cb_data = cb_data;
    info.cmp_seq = 0;
    rv = ipmi_domain_pointer_cb(id.domain_id, mc_ptr_cb, &info);
    if (!rv)
	rv = info.err;
    return rv;
}

int
_ipmi_cmp_mc_id_noseq(ipmi_mcid_t id1, ipmi_mcid_t id2)
{
    int d;

    d = ipmi_cmp_domain_id(id1.domain_id, id2.domain_id);
    if (d)
	return d;

    if (id1.mc_num > id2.mc_num)
	return 1;
    if (id1.mc_num < id2.mc_num)
	return -1;
    if (id1.channel > id2.channel)
	return 1;
    if (id1.channel < id2.channel)
	return -1;
    return 0;
}

int
_ipmi_cmp_mc_id(ipmi_mcid_t id1, ipmi_mcid_t id2)
{
    int d;

    d = _ipmi_cmp_mc_id_noseq(id1, id2);
    if (d)
	return d;

    if (id1.seq > id2.seq)
	return 1;
    if (id1.seq < id2.seq)
	return -1;
    return 0;
}

/***********************************************************************
 *
 * Handle sending commands and getting responses.
 *
 **********************************************************************/

static void
addr_rsp_handler(ipmi_domain_t *domain,
		 ipmi_addr_t   *addr,
		 unsigned int  addr_len,
		 ipmi_msg_t    *msg,
		 void          *rsp_data1,
		 void          *rsp_data2)
{
    ipmi_mc_response_handler_t rsp_handler = rsp_data2;
    ipmi_mc_t                  *mc;

    if (rsp_handler) {
	if (domain)
	    mc = _ipmi_find_mc_by_addr(domain, addr, addr_len);
	else
	    mc = NULL;
	rsp_handler(mc, msg, rsp_data1);
    }
}

int
ipmi_mc_send_command(ipmi_mc_t                  *mc,
		     unsigned int               lun,
		     ipmi_msg_t                 *msg,
		     ipmi_mc_response_handler_t rsp_handler,
		     void                       *rsp_data)
{
    int           rv;
    ipmi_addr_t   addr = mc->addr;
    ipmi_domain_t *domain;

    CHECK_MC_LOCK(mc);

    rv = ipmi_addr_set_lun(&addr, lun);
    if (rv)
	return rv;

    domain = ipmi_mc_get_domain(mc);

    rv = ipmi_send_command_addr(domain,
				&addr, mc->addr_len,
				msg,
				addr_rsp_handler,
				rsp_data,
				rsp_handler);
    return rv;
}

/***********************************************************************
 *
 * Handle global OEM callbacks for new MCs.
 *
 **********************************************************************/

typedef struct oem_handlers_s {
    unsigned int                 manufacturer_id;
    unsigned int                 product_id;
    ipmi_oem_mc_match_handler_cb handler;
    ipmi_oem_shutdown_handler_cb shutdown;
    void                         *cb_data;
} oem_handlers_t;
/* FIXME - do we need a lock?  Probably, add it. */
static ilist_t *oem_handlers;

int
ipmi_register_oem_handler(unsigned int                 manufacturer_id,
			  unsigned int                 product_id,
			  ipmi_oem_mc_match_handler_cb handler,
			  ipmi_oem_shutdown_handler_cb shutdown,
			  void                         *cb_data)
{
    oem_handlers_t *new_item;
    int            rv;

    /* This might be called before initialization, so be 100% sure.. */
    rv = _ipmi_mc_init();
    if (rv)
	return rv;

    new_item = ipmi_mem_alloc(sizeof(*new_item));
    if (!new_item)
	return ENOMEM;

    new_item->manufacturer_id = manufacturer_id;
    new_item->product_id = product_id;
    new_item->handler = handler;
    new_item->shutdown = shutdown;
    new_item->cb_data = cb_data;

    if (! ilist_add_tail(oem_handlers, new_item, NULL)) {
	ipmi_mem_free(new_item);
	return ENOMEM;
    }

    return 0;
}

static int
oem_handler_cmp(void *item, void *cb_data)
{
    oem_handlers_t *hndlr = item;
    ipmi_mc_t      *mc = cb_data;

    return ((hndlr->manufacturer_id == mc->manufacturer_id)
	    && (hndlr->product_id == mc->product_id));
}

int
ipmi_deregister_oem_handler(unsigned int manufacturer_id,
			    unsigned int product_id)
{
    oem_handlers_t *hndlr;
    ilist_iter_t   iter;

    ilist_init_iter(&iter, oem_handlers);
    ilist_unpositioned(&iter);
    hndlr = ilist_search_iter(&iter, oem_handler_cmp, NULL);
    if (hndlr) {
	ilist_delete(&iter);
	ipmi_mem_free(hndlr);
	return 0;
    }
    return ENOENT;
}

static int
check_oem_handlers(ipmi_mc_t *mc)
{
    oem_handlers_t *hndlr;

    hndlr = ilist_search(oem_handlers, oem_handler_cmp, mc);
    if (hndlr) {
	return hndlr->handler(mc, hndlr->cb_data);
    }
    return 0;
}

/***********************************************************************
 *
 * Handle the boatloads of information from a get device id.
 *
 **********************************************************************/

int
_ipmi_mc_get_device_id_data_from_rsp(ipmi_mc_t  *mc, ipmi_msg_t *rsp)
{
    unsigned char *rsp_data = rsp->data;

    if (rsp_data[0] != 0) {
	return IPMI_IPMI_ERR_VAL(rsp_data[0]);
    }

    if (rsp->data_len < 12) {
	return EINVAL;
    }

    mc->device_id = rsp_data[1];
    mc->device_revision = rsp_data[2] & 0xf;
    mc->provides_device_sdrs = (rsp_data[2] & 0x80) == 0x80;
    mc->device_available = (rsp_data[3] & 0x80) == 0x80;
    mc->major_fw_revision = rsp_data[3] & 0x7f;
    mc->minor_fw_revision = rsp_data[4];
    mc->major_version = rsp_data[5] & 0xf;
    mc->minor_version = (rsp_data[5] >> 4) & 0xf;
    mc->chassis_support = (rsp_data[6] & 0x80) == 0x80;
    mc->bridge_support = (rsp_data[6] & 0x40) == 0x40;
    mc->IPMB_event_generator_support = (rsp_data[6] & 0x20) == 0x20;
    mc->IPMB_event_receiver_support = (rsp_data[6] & 0x10) == 0x10;
    mc->FRU_inventory_support = (rsp_data[6] & 0x08) == 0x08;
    mc->SEL_device_support = (rsp_data[6] & 0x04) == 0x04;
    mc->SDR_repository_support = (rsp_data[6] & 0x02) == 0x02;
    mc->sensor_device_support = (rsp_data[6] & 0x01) == 0x01;
    mc->manufacturer_id = (rsp_data[7]
			     | (rsp_data[8] << 8)
			     | (rsp_data[9] << 16));
    mc->product_id = rsp_data[10] | (rsp_data[11] << 8);

    if (rsp->data_len < 16) {
	/* no aux revision. */
	memset(mc->aux_fw_revision, 0, 4);
    } else {
	memcpy(mc->aux_fw_revision, rsp_data + 12, 4);
    }

    /* Copy these to the version we use for comparison. */

    mc->real_device_id = mc->device_id;
    mc->real_device_revision = mc->device_revision;
    mc->real_provides_device_sdrs = mc->provides_device_sdrs;
    mc->real_device_available = mc->device_available;
    mc->real_chassis_support = mc->chassis_support;
    mc->real_bridge_support = mc->bridge_support;
    mc->real_IPMB_event_generator_support = mc->IPMB_event_generator_support;
    mc->real_IPMB_event_receiver_support = mc->IPMB_event_receiver_support;
    mc->real_FRU_inventory_support = mc->FRU_inventory_support;
    mc->real_SEL_device_support = mc->SEL_device_support;
    mc->real_SDR_repository_support = mc->SDR_repository_support;
    mc->real_sensor_device_support = mc->sensor_device_support;
    mc->real_major_fw_revision = mc->major_fw_revision;
    mc->real_minor_fw_revision = mc->minor_fw_revision;
    mc->real_major_version = mc->major_version;
    mc->real_minor_version = mc->minor_version;
    mc->real_manufacturer_id = mc->manufacturer_id;
    mc->real_product_id = mc->product_id;
    memcpy(mc->real_aux_fw_revision, mc->aux_fw_revision,
	   sizeof(mc->real_aux_fw_revision));

    return check_oem_handlers(mc);
}

int
_ipmi_mc_device_data_compares(ipmi_mc_t  *mc,
			      ipmi_msg_t *rsp)
{
    unsigned char *rsp_data = rsp->data;

    if (rsp->data_len < 12) {
	return EINVAL;
    }

    if (mc->real_device_id != rsp_data[1])
	return 0;

    if (mc->real_device_revision != (rsp_data[2] & 0xf))
	return 0;
    
    if (mc->real_provides_device_sdrs != ((rsp_data[2] & 0x80) == 0x80))
	return 0;

    if (mc->real_device_available != ((rsp_data[3] & 0x80) == 0x80))
	return 0;

    if (mc->real_major_fw_revision != (rsp_data[3] & 0x7f))
	return 0;

    if (mc->real_minor_fw_revision != (rsp_data[4]))
	return 0;

    if (mc->real_major_version != (rsp_data[5] & 0xf))
	return 0;

    if (mc->real_minor_version != ((rsp_data[5] >> 4) & 0xf))
	return 0;

    if (mc->real_chassis_support != ((rsp_data[6] & 0x80) == 0x80))
	return 0;

    if (mc->real_bridge_support != ((rsp_data[6] & 0x40) == 0x40))
	return 0;

    if (mc->real_IPMB_event_generator_support != ((rsp_data[6] & 0x20)==0x20))
	return 0;

    if (mc->real_IPMB_event_receiver_support != ((rsp_data[6] & 0x10) == 0x10))
	return 0;

    if (mc->real_FRU_inventory_support != ((rsp_data[6] & 0x08) == 0x08))
	return 0;

    if (mc->real_SEL_device_support != ((rsp_data[6] & 0x04) == 0x04))
	return 0;

    if (mc->real_SDR_repository_support != ((rsp_data[6] & 0x02) == 0x02))
	return 0;

    if (mc->real_sensor_device_support != ((rsp_data[6] & 0x01) == 0x01))
	return 0;

    if (mc->real_manufacturer_id != (rsp_data[7]
				     | (rsp_data[8] << 8)
				     | (rsp_data[9] << 16)))
	return 0;

    if (mc->real_product_id != (rsp_data[10] | (rsp_data[11] << 8)))
	return 0;

    if (rsp->data_len < 16) {
	/* no aux revision, it should be all zeros. */
	if ((mc->real_aux_fw_revision[0] != 0)
	    || (mc->real_aux_fw_revision[1] != 0)
	    || (mc->real_aux_fw_revision[2] != 0)
	    || (mc->real_aux_fw_revision[3] != 0))
	    return 0;
    } else {
	if (memcmp(mc->real_aux_fw_revision, rsp_data + 12, 4) != 0)
	    return 0;
    }

    /* Everything's the same. */
    return 1;
}

void
_ipmi_mc_get_sdr_sensors(ipmi_mc_t     *mc,
			 ipmi_sensor_t ***sensors,
			 unsigned int  *count)
{
    *sensors = mc->sensors_in_my_sdr;
    *count = mc->sensors_in_my_sdr_count;
}

void
_ipmi_mc_set_sdr_sensors(ipmi_mc_t     *mc,
			 ipmi_sensor_t **sensors,
			 unsigned int  count)
{
    mc->sensors_in_my_sdr = sensors;
    mc->sensors_in_my_sdr_count = count;
}

/***********************************************************************
 *
 * Get/set the information for an MC.
 *
 **********************************************************************/

int
ipmi_mc_provides_device_sdrs(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->provides_device_sdrs;
}

int
ipmi_mc_is_active(ipmi_mc_t *mc)
{
    return mc->active;
}

void
_ipmi_mc_set_active(ipmi_mc_t *mc, int val)
{
    mc->active = val;
}

void
ipmi_mc_set_provides_device_sdrs(ipmi_mc_t *mc, int val)
{
    CHECK_MC_LOCK(mc);
    mc->provides_device_sdrs = val;
}

int
ipmi_mc_device_available(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->device_available;
}

void
ipmi_mc_set_device_available(ipmi_mc_t *mc, int val)
{
    CHECK_MC_LOCK(mc);
    mc->device_available = val;
}

int
ipmi_mc_chassis_support(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->chassis_support;
}

void
ipmi_mc_set_chassis_support(ipmi_mc_t *mc, int val)
{
    CHECK_MC_LOCK(mc);
    mc->chassis_support = val;
}

int
ipmi_mc_bridge_support(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->bridge_support;
}

void
ipmi_mc_set_bridge_support(ipmi_mc_t *mc, int val)
{
    CHECK_MC_LOCK(mc);
    mc->bridge_support = val;
}

int
ipmi_mc_ipmb_event_generator_support(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->IPMB_event_generator_support;
}

void
ipmi_mc_set_ipmb_event_generator_support(ipmi_mc_t *mc, int val)
{
    CHECK_MC_LOCK(mc);
    mc->IPMB_event_generator_support = val;
}

int
ipmi_mc_ipmb_event_receiver_support(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->IPMB_event_receiver_support;
}

void
ipmi_mc_set_ipmb_event_receiver_support(ipmi_mc_t *mc, int val)
{
    CHECK_MC_LOCK(mc);
    mc->IPMB_event_receiver_support = val;
}

int
ipmi_mc_fru_inventory_support(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->FRU_inventory_support;
}

void
ipmi_mc_set_fru_inventory_support(ipmi_mc_t *mc, int val)
{
    CHECK_MC_LOCK(mc);
    mc->FRU_inventory_support = val;
}

int
ipmi_mc_sel_device_support(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->SEL_device_support;
}

void
ipmi_mc_set_sel_device_support(ipmi_mc_t *mc, int val)
{
    CHECK_MC_LOCK(mc);
    mc->SEL_device_support = val;
}

int
ipmi_mc_sdr_repository_support(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->SDR_repository_support;
}

void
ipmi_mc_set_sdr_repository_support(ipmi_mc_t *mc, int val)
{
    CHECK_MC_LOCK(mc);
    mc->SDR_repository_support = val;
}

int
ipmi_mc_sensor_device_support(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->sensor_device_support;
}

void
ipmi_mc_set_sensor_device_support(ipmi_mc_t *mc, int val)
{
    CHECK_MC_LOCK(mc);
    mc->sensor_device_support = val;
}

int
ipmi_mc_device_id(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->device_id;
}

int
ipmi_mc_device_revision(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->device_revision;
}

int
ipmi_mc_major_fw_revision(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->major_fw_revision;
}

int
ipmi_mc_minor_fw_revision(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->minor_fw_revision;
}

int
ipmi_mc_major_version(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->major_version;
}

int
ipmi_mc_minor_version(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->minor_version;
}

int
ipmi_mc_manufacturer_id(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->manufacturer_id;
}

int
ipmi_mc_product_id(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->product_id;
}

void
ipmi_mc_aux_fw_revision(ipmi_mc_t *mc, unsigned char val[])
{
    CHECK_MC_LOCK(mc);
    memcpy(val, mc->aux_fw_revision, sizeof(mc->aux_fw_revision));
}

void
ipmi_mc_set_oem_data(ipmi_mc_t *mc, void *data)
{
    CHECK_MC_LOCK(mc);
    mc->oem_data = data;
}

void *
ipmi_mc_get_oem_data(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->oem_data;
}

ipmi_sensor_info_t *
_ipmi_mc_get_sensors(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->sensors;
}

ipmi_control_info_t *
_ipmi_mc_get_controls(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->controls;
}

ipmi_sdr_info_t *
ipmi_mc_get_sdrs(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->sdrs;
}

unsigned int
ipmi_mc_get_address(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    if (mc->addr.addr_type == IPMI_IPMB_ADDR_TYPE) {
	ipmi_ipmb_addr_t *ipmb_addr = (ipmi_ipmb_addr_t *) &(mc->addr);
	return ipmb_addr->slave_addr;
    }

    /* Address is ignore for other types. */
    return 0;
}

void
ipmi_mc_get_ipmi_address(ipmi_mc_t    *mc,
			 ipmi_addr_t  *addr,
			 unsigned int *addr_len)
{
    CHECK_MC_LOCK(mc);

    memcpy(addr, &mc->addr, mc->addr_len);
    *addr_len = mc->addr_len;
}

unsigned int
ipmi_mc_get_channel(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->addr.channel;
}

ipmi_domain_t *ipmi_mc_get_domain(ipmi_mc_t *mc)
{
    return mc->domain;
}

int
_ipmi_mc_new_sensor(ipmi_mc_t     *mc,
		    ipmi_entity_t *ent,
		    ipmi_sensor_t *sensor,
		    void          *link)
{
    int rv = 0;

    CHECK_MC_LOCK(mc);

    ipmi_entity_lock(ent);
    if (mc->new_sensor_handler)
	rv = mc->new_sensor_handler(mc, ent, sensor, link,
				    mc->new_sensor_cb_data);
    ipmi_entity_unlock(ent);
    return rv;
}

int
ipmi_mc_set_oem_new_sensor_handler(ipmi_mc_t                 *mc,
				   ipmi_mc_oem_new_sensor_cb handler,
				   void                      *cb_data)
{
    CHECK_MC_LOCK(mc);
    mc->new_sensor_handler = handler;
    mc->new_sensor_cb_data = cb_data;
    return 0;
}

int
ipmi_mc_set_oem_removed_handler(ipmi_mc_t              *mc,
				ipmi_mc_oem_removed_cb handler,
				void                   *cb_data)
{
    CHECK_MC_LOCK(mc);

    mc->removed_mc_handler = handler;
    mc->removed_mc_cb_data = cb_data;
    return 0;
}

/***********************************************************************
 *
 * Global initialization and shutdown
 *
 **********************************************************************/

static int mc_initialized = 0;

int
_ipmi_mc_init(void)
{
    if (mc_initialized)
	return 0;

    oem_handlers = alloc_ilist();
    if (!oem_handlers)
	return ENOMEM;

    mc_initialized = 1;

    return 0;
}

void
_ipmi_mc_shutdown(void)
{
    if (mc_initialized) {
	oem_handlers_t *hndlr;
	ilist_iter_t   iter;

	/* Destroy the members of the OEM list. */
	ilist_init_iter(&iter, oem_handlers);
	while (ilist_first(&iter)) {
	    hndlr = ilist_get(&iter);
	    if (hndlr->shutdown)
		hndlr->shutdown(hndlr->cb_data);
	    ilist_delete(&iter);
	    ipmi_mem_free(hndlr);
	}

	free_ilist(oem_handlers);
	oem_handlers = NULL;
	mc_initialized = 0;
    }
}

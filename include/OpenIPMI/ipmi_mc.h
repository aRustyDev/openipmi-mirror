/*
 * ipmi_mc.h
 *
 * MontaVista IPMI interface for management controllers
 *
 * Author: MontaVista Software, Inc.
 *         Corey Minyard <minyard@mvista.com>
 *         source@mvista.com
 *
 * Copyright 2002 MontaVista Software Inc.
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

#ifndef _IPMI_MC_H
#define _IPMI_MC_H
#include <OpenIPMI/ipmi_types.h>
#include <OpenIPMI/os_handler.h>
#include <OpenIPMI/ipmi_entity.h>
#include <OpenIPMI/ipmi_sensor.h>
#include <OpenIPMI/ipmi_control.h>
#include <OpenIPMI/ipmi_sdr.h>
#include <OpenIPMI/ipmi_addr.h>

/* A response comes back in this format. */
typedef void (*ipmi_mc_response_handler_t)(ipmi_mc_t  *src,
					   ipmi_msg_t *msg,
					   void       *rsp_data);

/* Send the command in "msg" and register a handler to handle the
   response.  This will return without blocking; when the response
   comes back the handler will be called.  The handler may be NULL;
   then the response is ignored.  Note that if non-NULL the response
   handler will always be called; if no response is received in time
   the code will return a timeout response. rsp_data is passed to the
   response handler, it may contain anything the user likes.  Note
   that if the mc goes away between the time the command is sent and
   the response comes back, this callback WILL be called, but the MC
   value will be NULL.  You must handle that. */
int ipmi_mc_send_command(ipmi_mc_t                  *mc,
			 unsigned int               lun,
			 ipmi_msg_t                 *cmd,
			 ipmi_mc_response_handler_t rsp_handler,
			 void                       *rsp_data);

/* Basic information about a MC.  */
int ipmi_mc_provides_device_sdrs(ipmi_mc_t *mc);
int ipmi_mc_device_available(ipmi_mc_t *mc);
int ipmi_mc_chassis_support(ipmi_mc_t *mc);
int ipmi_mc_bridge_support(ipmi_mc_t *mc);
int ipmi_mc_ipmb_event_generator_support(ipmi_mc_t *mc);
int ipmi_mc_ipmb_event_receiver_support(ipmi_mc_t *mc);
int ipmi_mc_fru_inventory_support(ipmi_mc_t *mc);
int ipmi_mc_sel_device_support(ipmi_mc_t *mc);
int ipmi_mc_sdr_repository_support(ipmi_mc_t *mc);
int ipmi_mc_sensor_device_support(ipmi_mc_t *mc);
int ipmi_mc_device_id(ipmi_mc_t *mc);
int ipmi_mc_device_revision(ipmi_mc_t *mc);
int ipmi_mc_major_fw_revision(ipmi_mc_t *mc);
int ipmi_mc_minor_fw_revision(ipmi_mc_t *mc);
int ipmi_mc_major_version(ipmi_mc_t *mc);
int ipmi_mc_minor_version(ipmi_mc_t *mc);
int ipmi_mc_manufacturer_id(ipmi_mc_t *mc);
int ipmi_mc_product_id(ipmi_mc_t *mc);
void ipmi_mc_aux_fw_revision(ipmi_mc_t *mc, unsigned char val[]);

/* Some stupid systems don't have some settings right, this lets the
   OEM code fix it. */
void ipmi_mc_set_provides_device_sdrs(ipmi_mc_t *mc, int val);
void ipmi_mc_set_sel_device_support(ipmi_mc_t *mc, int val);
void ipmi_mc_set_sdr_repository_support(ipmi_mc_t *mc, int val);
void ipmi_mc_set_sensor_device_support(ipmi_mc_t *mc, int val);
void ipmi_mc_set_device_available(ipmi_mc_t *mc, int val);
void ipmi_mc_set_chassis_support(ipmi_mc_t *mc, int val);
void ipmi_mc_set_bridge_support(ipmi_mc_t *mc, int val);
void ipmi_mc_set_ipmb_event_generator_support(ipmi_mc_t *mc, int val);
void ipmi_mc_set_ipmb_event_receiver_support(ipmi_mc_t *mc, int val);
void ipmi_mc_set_fru_inventory_support(ipmi_mc_t *mc, int val);

/* Reread all the sensors for a given mc.  This will request the
   sensor SDRs for that mc (And only for that MC) and change the
   sensors as necessary. */
typedef void (*ipmi_mc_done_cb)(ipmi_mc_t *mc, int err, void *cb_data);
int ipmi_mc_reread_sensors(ipmi_mc_t       *mc,
			   ipmi_mc_done_cb done,
			   void            *done_data);

/* Check to see if the MC is operational in the system.  If this is
   false, then the MC was referred to by an SDR, but it doesn't really
   exist. */
int ipmi_mc_is_active(ipmi_mc_t *mc);
void _ipmi_mc_set_active(ipmi_mc_t *mc, int val);


/* Return the domain for the given MC. */
ipmi_domain_t *ipmi_mc_get_domain(ipmi_mc_t *mc);

/* Get the sensors that the given MC owns. */
ipmi_sensor_info_t *_ipmi_mc_get_sensors(ipmi_mc_t *mc);

/* Get the controls that the given MC owns. */
ipmi_control_info_t *_ipmi_mc_get_controls(ipmi_mc_t *mc);

/* Get the sensor SDRs for the given MC. */
ipmi_sdr_info_t *ipmi_mc_get_sdrs(ipmi_mc_t *mc);

/* Get the IPMI slave address of the given MC. */
unsigned ipmi_mc_get_address(ipmi_mc_t *mc);

/* Get the MC's full IPMI address. */
void ipmi_mc_get_ipmi_address(ipmi_mc_t    *mc,
			      ipmi_addr_t  *addr,
			      unsigned int *addr_len);

/* Get the channel for the given MC. */
unsigned ipmi_mc_get_channel(ipmi_mc_t *mc);

int _ipmi_create_mc(ipmi_domain_t *domain,
		    ipmi_addr_t   *addr,
		    unsigned int  addr_len,
		    ipmi_mc_t     **new_mc);

/* Destroy an MC. */
void _ipmi_cleanup_mc(ipmi_mc_t *mc);

#if 0
/* FIXME - need to handle this somehow. */
/* This should be called from OEM code for an SMI, ONLY WHEN THE NEW
   MC HANDLER IS CALLED, if the slave address of the SMI is not 0x20.
   This will allow the bmc t know it's own address, which is pretty
   important.  You pass in a function that the code will call (and
   pass in it's own function) when it wants the address. */
typedef void (*ipmi_mc_got_slave_addr_cb)(ipmi_mc_t    *bmc,
					  int          err,
					  unsigned int addr,
					  void         *cb_data);
typedef int (*ipmi_mc_slave_addr_fetch_cb)(
    ipmi_mc_t                 *bmc,
    ipmi_mc_got_slave_addr_cb handler,
    void                      *cb_data);
int ipmi_bmc_set_smi_slave_addr_fetcher(
    ipmi_mc_t                   *bmc,
    ipmi_mc_slave_addr_fetch_cb handler);
#endif

/* Return the timestamp that was fetched before the first SEL fetch.
   This is so that OEM code can properly ignore old events. */
unsigned long ipmi_mc_get_startup_SEL_time(ipmi_mc_t *bmc);

typedef void (*ipmi_mc_cb)(ipmi_mc_t *mc, int err, void *cb_data);

/* Some OEM boxes may have special SEL delete requirements, so we have
   a special hook to let the OEM code delete events on an MC with SEL
   support. */
typedef int (*ipmi_mc_del_event_cb)(ipmi_mc_t    *mc,
				    ipmi_event_t *event,
				    ipmi_mc_cb   done_handler,
				    void         *cb_data);
void ipmi_mc_set_del_event_handler(ipmi_mc_t            *mc,
				   ipmi_mc_del_event_cb handler);

typedef void (ipmi_mc_del_event_done_cb)(ipmi_mc_t *mc, int err, void *cb_data);
int _ipmi_mc_del_event(ipmi_mc_t                 *mc,
		       ipmi_event_t              *event, 
		       ipmi_mc_del_event_done_cb handler,
		       void                      *cb_data);

/* Check the event receiver for the MC. */
void _ipmi_mc_check_event_rcvr(ipmi_mc_t *mc);


int _ipmi_mc_init(void);
void _ipmi_mc_shutdown(void);


void _ipmi_mc_sel_event_add(ipmi_mc_t *mc, ipmi_event_t *event);
int _ipmi_mc_first_event(ipmi_mc_t *mc, ipmi_event_t *event);
int _ipmi_mc_last_event(ipmi_mc_t *mc, ipmi_event_t *event);
int _ipmi_mc_next_event(ipmi_mc_t *mc, ipmi_event_t *event);
int _ipmi_mc_prev_event(ipmi_mc_t *mc, ipmi_event_t *event);
int _ipmi_mc_sel_count(ipmi_mc_t *mc);
int _ipmi_mc_sel_entries_used(ipmi_mc_t *mc);
int _ipmi_mc_check_oem_event_handler(ipmi_mc_t *mc, ipmi_event_t *event);
int _ipmi_mc_check_sel_oem_event_handler(ipmi_mc_t *mc, ipmi_event_t *event);

/* Set and get the OEM data pointer in the mc. */
void ipmi_mc_set_oem_data(ipmi_mc_t *mc, void *data);
void *ipmi_mc_get_oem_data(ipmi_mc_t *mc);

/* Used by the sensor code to report a new sensor to the MC.  It
   should return 1 if the sensor code should not add the sensor to
   its database. */
int _ipmi_mc_new_sensor(ipmi_mc_t     *mc,
			ipmi_entity_t *ent,
			ipmi_sensor_t *sensor,
			void          *link);

/* This should be called with a new device id for an MC we don't have
   active in the system (it may be inactive). */
int _ipmi_mc_get_device_id_data_from_rsp(ipmi_mc_t *mc, ipmi_msg_t *rsp);

/* Compares the data in a get device id response (in rsp) with the
   data in the MC, returns true if they are the same and false if
   not.  Must be called with an error-free message. */
int _ipmi_mc_device_data_compares(ipmi_mc_t *mc, ipmi_msg_t *rsp);

/* Called when a new MC has been added to the system, to kick of
   processing it. */
int _ipmi_mc_handle_new(ipmi_mc_t *mc);

void _ipmi_mc_get_sdr_sensors(ipmi_mc_t     *mc,
			      ipmi_sensor_t ***sensors,
			      unsigned int  *count);
void _ipmi_mc_set_sdr_sensors(ipmi_mc_t     *mc,
			      ipmi_sensor_t **sensors,
			      unsigned int  count);

ipmi_mcid_t _ipmi_mc_convert_to_id(ipmi_mc_t *mc);
typedef void (*_ipmi_mc_ptr_cb)(ipmi_mc_t *mc, void *cb_data);
int _ipmi_mc_pointer_cb(ipmi_mcid_t   id,
		       _ipmi_mc_ptr_cb handler,
		       void           *cb_data);
int _ipmi_mc_pointer_noseq_cb(ipmi_mcid_t   id,
			      _ipmi_mc_ptr_cb handler,
			      void           *cb_data);
int _ipmi_cmp_mc_id(ipmi_mcid_t id1, ipmi_mcid_t id2);
int _ipmi_cmp_mc_id_noseq(ipmi_mcid_t id1, ipmi_mcid_t id2);

#endif /* _IPMI_MC_H */

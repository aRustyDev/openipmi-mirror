/*
 * ipmi_domain.h
 *
 * MontaVista IPMI interface for management domains
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

#ifndef _IPMI_DOMAIN_H
#define _IPMI_DOMAIN_H
#include <OpenIPMI/ipmi_types.h>
#include <OpenIPMI/os_handler.h>
#include <OpenIPMI/ipmi_entity.h>
#include <OpenIPMI/ipmi_sensor.h>
#include <OpenIPMI/ipmi_control.h>
#include <OpenIPMI/ipmi_sdr.h>
#include <OpenIPMI/ipmi_addr.h>

/* A response comes back in this format. */
typedef void (*ipmi_addr_response_handler_t)(ipmi_domain_t *domain,
					     ipmi_addr_t   *addr,
					     unsigned int  addr_len,
					     ipmi_msg_t    *msg,
					     void          *rsp_data1,
					     void          *rsp_data2);

/* Like ipmi_send_command, but sends it directly to the address
   specified, not to an MC. */
int
ipmi_send_command_addr(ipmi_domain_t                *domain,
		       ipmi_addr_t	            *addr,
		       unsigned int                 addr_len,
		       ipmi_msg_t                   *msg,
		       ipmi_addr_response_handler_t rsp_handler,
		       void                         *rsp_data1,
		       void                         *rsp_data2);


/* Iterate over all the mc's that the given domain represents. */
typedef void (*ipmi_domain_iterate_mcs_cb)(ipmi_domain_t *domain,
					   ipmi_mc_t     *mc,
					   void          *cb_data);
int ipmi_domain_iterate_mcs(ipmi_domain_t              *mc,
			    ipmi_domain_iterate_mcs_cb handler,
			    void                       *cb_data);
int ipmi_domain_iterate_mcs_rev(ipmi_domain_t              *domain,
				ipmi_domain_iterate_mcs_cb handler,
				void                       *cb_data);

/* Return the OS handler used by the mc. */
os_handler_t *ipmi_domain_get_os_hnd(ipmi_domain_t *domain);

/* Return the entity info for the given domain. */
ipmi_entity_info_t *ipmi_domain_get_entities(ipmi_domain_t *domain);

/* Rescan the entities for possible presence changes.  "force" causes
   a full rescan even if nothing on an entity has changed. */
int ipmi_detect_domain_presence_changes(ipmi_domain_t *domain, int force);

/* Should the BMC do a full bus scan at startup?  This is so OEM
   code can turn this function off.  The value is a boolean. */
int ipmi_domain_set_full_bus_scan(ipmi_domain_t *domain, int val);

int ipmi_domain_get_event_rcvr(ipmi_domain_t *domain);

/* Allocate an MC in the domain.  It doesn't add it to the domain's
   list, to allow the MC to be setup before that happens. */
int ipmi_create_mc(ipmi_domain_t *domain,
		   ipmi_addr_t   *addr,
		   unsigned int  addr_len,
		   ipmi_mc_t     **new_mc);

int _ipmi_remove_mc_from_domain(ipmi_domain_t *domain, ipmi_mc_t *mc);

/* Attempt to find the MC, and if it doesn't exist create it and
   return it. */
int _ipmi_find_or_create_mc_by_slave_addr(ipmi_domain_t *domain,
					  unsigned int  slave_addr,
					  ipmi_mc_t     **mc);

/* Find the MC with the given IPMI address, or return NULL if not
   found. */
ipmi_mc_t *_ipmi_find_mc_by_addr(ipmi_domain_t *domain,
				 ipmi_addr_t   *addr,
				 unsigned int  addr_len);

/* Return the SDRs for the given MC, or the main set of SDRs if the MC
   is NULL. */
void _ipmi_get_sdr_sensors(ipmi_domain_t *domain,
			   ipmi_mc_t     *mc,
			   ipmi_sensor_t ***sensors,
			   unsigned int  *count);

/* Set the SDRs for the given MC, or the main set of SDRs if the MC is
   NULL. */
void _ipmi_set_sdr_sensors(ipmi_domain_t *domain,
			   ipmi_mc_t     *mc,
			   ipmi_sensor_t **sensors,
			   unsigned int  count);

/* Add an MC to the list of MCs in the domain. */
int ipmi_add_mc_to_domain(ipmi_domain_t *domain, ipmi_mc_t *mc);

/* Remove an MC from the list of MCs in the domain. */
int ipmi_remove_mc_from_domain(ipmi_domain_t *domain, ipmi_mc_t *mc);

/* Call any OEM handlers for the given MC. */
int _ipmi_domain_check_oem_handlers(ipmi_domain_t *domain, ipmi_mc_t *mc);

/* Scan a set of addresses on the bmc for mcs.  This can be used by OEM
   code to add an MC if it senses that one has become present. */
void ipmi_start_ipmb_mc_scan(ipmi_domain_t  *domain,
	       		     int            channel,
	       		     unsigned int   start_addr,
			     unsigned int   end_addr,
                             ipmi_domain_cb done_handler,
			     void           *cb_data);

/* Add an IPMB address to a list of addresses to not scan.  This way,
   if you have weak puny devices in IPMB that will break if you do
   normal IPMB operations, you can have them be ignored. */
int ipmi_domain_add_ipmb_ignore(ipmi_domain_t *domain,
				unsigned char ipmb_addr);

/* If OEM code gets and event and it doesn't deliver it to the user,
   it should deliver it this way, that way it can be delivered to the
   user to be deleted. */
void ipmi_handle_unhandled_event(ipmi_domain_t *domain, ipmi_event_t *event);

/* Handle a new event from something, usually from an SEL. */
void _ipmi_domain_system_event_handler(ipmi_domain_t *domain,
				       ipmi_mc_t     *mc,
				       ipmi_event_t  *event);

/* Returns if the domain things it has a connection up. */
int ipmi_domain_con_up(ipmi_domain_t *domain);

/* Get the number of channels the domain supports. */
int ipmi_domain_get_num_channels(ipmi_domain_t *domain, int *val);

/* Get information about a channel by index.  The index is not
   necessarily the channel number, just an array index (up to the
   number of channels).  Get the channel number from the returned
   information. */
int ipmi_domain_get_channel(ipmi_domain_t    *domain,
			    int              index,
			    ipmi_chan_info_t *chan);


/* Initialize the domain code, called only once at init time. */
int _ipmi_domain_init(void);

/* Clean up the global domain memory. */
void _ipmi_domain_shutdown(void);

#endif /* _IPMI_DOMAIN_H */

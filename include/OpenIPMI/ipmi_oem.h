/*
 * ipmi_oem.h
 *
 * MontaVista IPMI interface, OEM handling
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

#ifndef IPMI_OEM_H
#define IPMI_OEM_H
/*
 * These calls allow OEM code to register for various events so they can
 * create their own entities, sensors, or modify existing ones as necessary.
 */

#include <OpenIPMI/ipmiif.h>

/* This registers an OEM handler.  If an MC is detected that has the
   given manufacturer id and product id, the handler will be
   called. If the IPMI system is shutdown while this is registered, then
   shutdown will be called (if it is not NULL). */
typedef int (*ipmi_oem_mc_match_handler_cb)(ipmi_mc_t *mc, void *cb_data);
typedef int (*ipmi_oem_shutdown_handler_cb)(void *cb_data);
int ipmi_register_oem_handler(unsigned int                 manufacturer_id,
			      unsigned int                 product_id,
			      ipmi_oem_mc_match_handler_cb handler,
			      ipmi_oem_shutdown_handler_cb shutdown,
			      void                         *cb_data);
int ipmi_deregister_oem_handler(unsigned int manufacturer_id,
				unsigned int product_id);

/* Used to report that an mc as been removed to the OEM handler.  The
   OEM handler may not refuse to allow the mc to be removed. This is
   so the OEM handler can free data if necessary.  This is registered
   against the MC itself, not the BMC, and must be called on each MC
   that needs it. */
typedef void (*ipmi_mc_oem_removed_cb)(ipmi_domain_t *domain,
				       ipmi_mc_t     *mc,
				       void          *cb_data);
int ipmi_mc_set_oem_removed_handler(ipmi_mc_t              *mc,
				    ipmi_mc_oem_removed_cb handler,
				    void                   *cb_data);

/* A new sensor has been added, the OEM handlers get first access at
   it.  This is called before it is added to the entity.  If this call
   returns true, the sensor will NOT be added to the entity, the OEM
   device is assumed to have taken over control of the sensor.  The
   OEM handler may also add it's own callback or register it's own
   data conversion handler for this sensor.  The link is a value
   allocated with ipmi_entity_alloc_sensor_link, if this returns
   false, the oem callback cannot use this value.  If this returns
   true, the oem callback will own the link and be responsible for
   freeing the link.  Setting the callback to NULL will disable it. */
typedef int (*ipmi_mc_oem_new_sensor_cb)(ipmi_mc_t     *mc,
					 ipmi_entity_t *ent,
					 ipmi_sensor_t *sensor,
					 void          *link,
					 void          *cb_data);
int ipmi_mc_set_oem_new_sensor_handler(ipmi_mc_t                 *mc,
				       ipmi_mc_oem_new_sensor_cb handler,
				       void                      *cb_data);

/* The handler should return 0 if it didn't handle the event, or 1
   if it did.  If the call handles the event, OpenIPMI will not deliver
   the event anywhere else. */
typedef int (*ipmi_oem_event_handler_cb)(ipmi_mc_t    *mc,
					 ipmi_event_t *log,
					 void         *cb_data);
/* Set a handler for when an event comes in from a specific MC.  This
   is so the code can handle custom messages or broken events. */
int ipmi_mc_set_oem_event_handler(ipmi_mc_t                 *mc,
				  ipmi_oem_event_handler_cb handler,
				  void                      *cb_data);
/* Set a handler for when an event comes from the SEL of a specific
   MC.  Note that the message may be from another MC, but it was
   stored in the given MC's SEL.  This handler gets first dibs at the
   message, and must parse the time itself.  This is so OEM code can
   handle broken or custom SELs. */
int ipmi_mc_set_sel_oem_event_handler(ipmi_mc_t                 *mc,
				      ipmi_oem_event_handler_cb handler,
				      void                      *cb_data);

#endif /* IPMI_OEM_H */

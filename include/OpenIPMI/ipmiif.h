/*
 * ipmiif.h
 *
 * MontaVista IPMI main interface
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

#ifndef __IPMIIF_H
#define __IPMIIF_H

/*
 * This is the main include file for dealing with IPMI.  It provides
 * an abstract interface to the IPMI system, so you don't have to deal
 * with all the nitty-gritty details of IPMI.  You only deal with
 * four things:
 *
 *  Domain - This is the main interface to the IPMI system.
 *  Entities - These are things that sensors monitor, they can be
 *             FRUs, or whatnot.
 *  Sensors - These are monitors for FRUs.
 *  Controls - These are output devices
 *
 * You don't have to deal with Management Controllers (MCs), IPMI
 * addressing, or anything like that.  This software will go out onto
 * the IPMB bus, detect all the MCs and entities present there, and
 * call you when it detects something.  It reads the SDR database and
 * detects all the entities and entity relationships.  It lets you add
 * entities and relationships to the local copies, and write the
 * information back into the database.
 *
 * You have to be careful with locking in this system.  The four things
 * you deal with all have two ways to get at them: An ID, and a pointer.
 * The ID is always valid, you can store that off on your own and use it
 * later.  The pointer is only valid inside a callback, the system is
 * free to change the pointers for a thing when no callbacks are active.
 *
 * To convert an ID to a pointer that you can work on, you have to go
 * through a callback.  These are provided for each type.  This is a
 * little inconvenient, but it's a lot faster than copying a lot of
 * data around all the time or re-validating an ID on every operation.
 * If a callback gives you a pointer to a sensor, entity, or domain, the
 * lock for that things will be held while you are in the callback.
 *
 * This interface is completely event-driven, meaning that a call will
 * never block.  Instead, if a call cannot complete inside the call
 * itself, you provide a "callback" that will be called when the
 * operation completes.  If you don't care about the results, you can
 * provide a NULL callback.  However, you will not receive any error
 * information about the operation; if it fails you will not know.
 * Note that if a function that you provide a callback returns an
 * error, the callback will NEVER be called.
 *
 * Callbacks are possible on things that have ceased to exist.  For
 * example, if you start an operation on a sensor and the sensor
 * ceases to exist during the operation, you will get an error
 * callback with a NULL sensor.  The same goes for controls, entities,
 * or anything else.
 *
 * You should NEVER block or exit in a callback.  Locks are held in
 * callbacks, so you will constipate the system if you block in
 * callbacks.  Just don't do it.
 */

#include <OpenIPMI/ipmi_types.h>
#include <OpenIPMI/ipmi_bits.h>
#include <OpenIPMI/os_handler.h>

/* This is how you convert a pointer to and ID and convert an ID to a
   pointer.  Pointers are ONLY valid in callbacks, the system is free
   to change the pointer value outside the callback.  So you should
   only store IDs.  IDs are good all the time, but you must go through
   the "pointer_cb" functions to get a usable pointer you can operate
   on.  This is how the locking works for this, inside the callback
   you will hold the locks so the item you are using will not change.
   It's kind of a pain, but it improves reliability.  This way, you
   cannot "forget" to release the lock for something. */
/* The comparisons below return -1 if id1<id2, 0 if id1==id2, and 1 if
   id1>id2. */
ipmi_domain_id_t ipmi_domain_convert_to_id(ipmi_domain_t *domain);
typedef void (*ipmi_domain_ptr_cb)(ipmi_domain_t *domain, void *cb_data);
int ipmi_domain_pointer_cb(ipmi_domain_id_t   id,
			   ipmi_domain_ptr_cb handler,
			   void               *cb_data);
int ipmi_cmp_domain_id(ipmi_domain_id_t id1, ipmi_domain_id_t id2);

ipmi_entity_id_t ipmi_entity_convert_to_id(ipmi_entity_t *ent);
typedef void (*ipmi_entity_ptr_cb)(ipmi_entity_t *entity, void *cb_data);
int ipmi_entity_pointer_cb(ipmi_entity_id_t   id,
			   ipmi_entity_ptr_cb handler,
			   void               *cb_data);

ipmi_sensor_id_t ipmi_sensor_convert_to_id(ipmi_sensor_t *sensor);
typedef void (*ipmi_sensor_ptr_cb)(ipmi_sensor_t *sensor, void *cb_data);
int ipmi_sensor_pointer_cb(ipmi_sensor_id_t   id,
			   ipmi_sensor_ptr_cb handler,
			   void               *cb_data);
int ipmi_cmp_sensor_id(ipmi_sensor_id_t id1, ipmi_sensor_id_t id2);

ipmi_control_id_t ipmi_control_convert_to_id(ipmi_control_t *control);
typedef void (*ipmi_control_ptr_cb)(ipmi_control_t *control, void *cb_data);
int ipmi_control_pointer_cb(ipmi_control_id_t   id,
			    ipmi_control_ptr_cb handler,
			    void                *cb_data);
int ipmi_cmp_control_id(ipmi_control_id_t id1, ipmi_control_id_t id2);

/* Callback used for generic domain reporting. */
typedef void (*ipmi_domain_cb)(ipmi_domain_t *domain, int err, void *cb_data);

typedef struct ipmi_domain_con_fail_s ipmi_domain_con_fail_t;

/* Add and remove a function to be called when the connection to the
   domain goes down or back up.  Being down does NOT mean the domain has
   been shutdown, it is still active, and OpenIPMI will continue to
   attempt to reconnect to the domain.  When the connection goes down,
   The "err" value in the callback will be non-zero to report the
   reason for the failure.  When the connection goes up, the "err"
   value will be zero reporting that the connection is now
   available. */
int ipmi_domain_add_con_fail_handler(ipmi_domain_t          *domain,
				     ipmi_domain_cb         handler,
				     void                   *cb_data,
				     ipmi_domain_con_fail_t **id);
void ipmi_domain_remove_con_fail_handler(ipmi_domain_t          *domain,
					 ipmi_domain_con_fail_t *id);

/* The domain has two timers, one for the SEL rescan interval and one for
   the IPMB bus rescan interval. */

/* The SEL rescan timer is the time between when the SEL will be
   checked for new events.  This timer is in seconds, and will
   currently default to 10 seconds.  You need to set this depending on
   how fast you need to know if events have come in. */
void ipmi_domain_set_sel_rescan_time(ipmi_domain_t *domain,
				     unsigned int  seconds);
unsigned int ipmi_domain_get_sel_rescan_time(ipmi_domain_t *domain);

/* The IPMB rescan timer is the time between scans of the IPMB bus to
   see if new MCs have appeared on the bus.  The timer is in seconds,
   and defaults to 600 seconds (10 minutes).  The setting of this
   timer depends on how fast you need to know if new devices have
   appeared, and if your system has proprietary extensions to detect
   insertion of devices more quickly.  */
void ipmi_domain_set_ipmb_rescan_time(ipmi_domain_t *domain,
				      unsigned int  seconds);
unsigned int ipmi_domain_get_ipmb_rescan_time(ipmi_domain_t *domain);

/* Events come in this format. */
typedef void (*ipmi_event_handler_cb)(ipmi_domain_t *domain,
				      ipmi_event_t  *event,
				      void          *event_data);

typedef struct ipmi_event_handler_id_s ipmi_event_handler_id_t;

/* Register a handler to receive events.  Multiple handlers may be
   registered, they will all receive all events.  The event_data will
   be passed in with every event received.  This will only catch
   events that are not sent to a sensor, so if you get a system
   software event or an event from a sensor the software doesn't know
   about, this handler will get it. */
int ipmi_register_for_events(ipmi_domain_t           *domain,
			     ipmi_event_handler_cb   handler,
			     void                    *event_data,
			     ipmi_event_handler_id_t **id);
/* Deregister an event handler. */
int ipmi_deregister_for_events(ipmi_domain_t           *domain,
			       ipmi_event_handler_id_t *id);

/* Globally enable or disable events on the mc. */
int ipmi_domain_enable_events(ipmi_domain_t *domain);
int ipmi_domain_disable_events(ipmi_domain_t *domain);

/* When you are done with an event, you should delete it.  This frees up
   the internal store for the event and removes it from the external
   system event event. */
/* Delete a specific event. */
int ipmi_domain_del_event(ipmi_domain_t  *domain,
			  ipmi_event_t   *event,
			  ipmi_domain_cb done_handler,
			  void           *cb_data);

/* You can also scan the current set of events stored in the system.
   They return an error if the SEL is empty, or if you try to go past
   the last or before the first event.  The first and last function
   return the event, the next and prev function take the current event in
   "event" and return the next or previous event in "event". */
int ipmi_domain_first_event(ipmi_domain_t *domain, ipmi_event_t *event);
int ipmi_domain_last_event(ipmi_domain_t *domain, ipmi_event_t *event);
int ipmi_domain_next_event(ipmi_domain_t *domain, ipmi_event_t *event);
int ipmi_domain_prev_event(ipmi_domain_t *domain, ipmi_event_t *event);

/* Return the number of non-deleted entries in the local copy of the
   SEL. */
int ipmi_domain_sel_count(ipmi_domain_t *domain,
			  unsigned int  *count);

/* Return the number of entries estimated to be used in the real SEL.
   If there are deleted event in the local copy of the SEL, they are
   not necessarily deleted from the real SEL, so this takes that into
   account. */
int ipmi_domain_sel_entries_used(ipmi_domain_t *domain,
				 unsigned int  *count);

/* Used in various operations to tell what has happened to a sensor,
   control, entity, or whatever. */
enum ipmi_update_e { IPMI_ADDED, IPMI_DELETED, IPMI_CHANGED };

/* A callback that will be called when entities are added to and
   removed from the domain, and when their presence changes. */
typedef void (*ipmi_domain_entity_cb)(enum ipmi_update_e op,
				      ipmi_domain_t      *domain,
				      ipmi_entity_t      *entity,
				      void               *cb_data);

/* Set the handler to be called when an entity is added or deleted. */
int ipmi_domain_set_entity_update_handler(ipmi_domain_t         *domain,
					  ipmi_domain_entity_cb handler,
					  void                  *cb_data);

/* Iterate over all the entities in the domain, calling the given
   function with each entity.  The entities will not change while this
   is happening. */
typedef void (*ipmi_entities_iterate_entity_cb)(ipmi_entity_t *entity,
						void          *cb_data);
int ipmi_domain_iterate_entities(ipmi_domain_t                   *domain,
				 ipmi_entities_iterate_entity_cb handler,
				 void                            *cb_data);

/* Store all the information I have locally into the SDR repository.
   This is a moderately dangerous operation, as it can wipe out your
   SDR repository if you are not careful. */
int ipmi_domain_store_entities(ipmi_domain_t  *domain,
			       ipmi_domain_cb done,
			       void           *cb_data);

/* For the given entity, iterate over all the children of the entity,
   calling the given handler with each child.  The children will not
   change while this is happening. */
typedef void (*ipmi_entity_iterate_child_cb)(ipmi_entity_t *ent,
					     ipmi_entity_t *child,
					     void          *cb_data);
void ipmi_entity_iterate_children(ipmi_entity_t                *ent,
				  ipmi_entity_iterate_child_cb handler,
				  void                         *cb_data);

/* Iterate over the parents of the given entitiy.
   FIXME - can an entity have more than one parent? */
typedef void (*ipmi_entity_iterate_parent_cb)(ipmi_entity_t *ent,
					      ipmi_entity_t *parent,
					      void          *cb_data);
void ipmi_entity_iterate_parents(ipmi_entity_t                 *ent,
				 ipmi_entity_iterate_parent_cb handler,
				 void                          *cb_data);

/* Iterate over all the sensors of an entity. */
typedef void (*ipmi_entity_iterate_sensor_cb)(ipmi_entity_t *ent,
					      ipmi_sensor_t *sensor,
					      void          *cb_data);
void ipmi_entity_iterate_sensors(ipmi_entity_t                 *ent,
				 ipmi_entity_iterate_sensor_cb handler,
				 void                          *cb_data);

/* Iterate over all the controls of an entity. */
typedef void (*ipmi_entity_iterate_control_cb)(ipmi_entity_t  *ent,
					       ipmi_control_t *control,
					       void           *cb_data);
void ipmi_entity_iterate_controls(ipmi_entity_t                  *ent,
				  ipmi_entity_iterate_control_cb handler,
				  void                           *cb_data);

/* Set the handle to monitor the presence of an entity.  Only one
   handler may be specified, add a NULL handler to remove the current
   handler.  If the presence change was due to a system event, then
   the event field will not be NULL and will point to the event that
   cause the presence change.  This is so the user can delete the event
   from the SEL. */
typedef void (*ipmi_entity_presence_cb)(ipmi_entity_t *entity,
					int           present,
					void          *cb_data,
					ipmi_event_t  *event);
int ipmi_entity_set_presence_handler(ipmi_entity_t           *ent,
				     ipmi_entity_presence_cb handler,
				     void                    *cb_data);

/* Get information about an entity.  Most of this is IPMI specific. */
ipmi_domain_t *ipmi_entity_get_domain(ipmi_entity_t *ent);
int ipmi_entity_get_access_address(ipmi_entity_t *ent);
int ipmi_entity_get_slave_address(ipmi_entity_t *ent);
int ipmi_entity_get_channel(ipmi_entity_t *ent);
int ipmi_entity_get_lun(ipmi_entity_t *ent);
int ipmi_entity_get_private_bus_id(ipmi_entity_t *ent);
int ipmi_entity_get_is_logical_fru(ipmi_entity_t *ent);
int ipmi_entity_get_is_fru(ipmi_entity_t *ent);
int ipmi_entity_get_entity_id(ipmi_entity_t *ent);
int ipmi_entity_get_entity_instance(ipmi_entity_t *ent);
int ipmi_entity_get_device_type(ipmi_entity_t *ent);
int ipmi_entity_get_device_modifier(ipmi_entity_t *ent);
int ipmi_entity_get_oem(ipmi_entity_t *ent);
int ipmi_entity_get_presense_sensor_always_there(ipmi_entity_t *ent);
int ipmi_entity_get_in_sdr_db(ipmi_entity_t *ent);
int ipmi_entity_get_is_child(ipmi_entity_t *ent);
int ipmi_entity_get_ACPI_system_power_notify_required(ipmi_entity_t *ent);
int ipmi_entity_get_ACPI_device_power_notify_required(ipmi_entity_t *ent);
int ipmi_entity_get_controller_logs_init_agent_errors(ipmi_entity_t *ent);
int ipmi_entity_get_log_init_agent_errors_accessing(ipmi_entity_t *ent);
int ipmi_entity_get_global_init(ipmi_entity_t *ent);
int ipmi_entity_get_chassis_device(ipmi_entity_t *ent);
int ipmi_entity_get_bridge(ipmi_entity_t *ent);
int ipmi_entity_get_IPMB_event_generator(ipmi_entity_t *ent);
int ipmi_entity_get_IPMB_event_receiver(ipmi_entity_t *ent);
int ipmi_entity_get_FRU_inventory_device(ipmi_entity_t *ent);
int ipmi_entity_get_SEL_device(ipmi_entity_t *ent);
int ipmi_entity_get_SDR_repository_device(ipmi_entity_t *ent);
int ipmi_entity_get_sensor_device(ipmi_entity_t *ent);
char *ipmi_entity_get_entity_id_string(ipmi_entity_t *ent);

/* The ID from the SDR. */
int ipmi_entity_get_id_length(ipmi_entity_t *ent);
void ipmi_entity_get_id(ipmi_entity_t *ent, char *id, int length);

/* Is the entity currently present? */
int ipmi_entity_is_present(ipmi_entity_t *ent);

/* Register a handler that will be called when a sensor that monitors
   this entity is added, deleted, or modified.  If you call this in
   the entity added callback for the domain, you are guaranteed to get
   this set before any sensors exist. */
typedef void (*ipmi_entity_sensor_cb)(enum ipmi_update_e op,
				      ipmi_entity_t      *ent,
				      ipmi_sensor_t      *sensor,
				      void               *cb_data);
int ipmi_entity_set_sensor_update_handler(ipmi_entity_t         *ent,
					  ipmi_entity_sensor_cb handler,
					  void                  *cb_data);

/* Register a handler that will be called when an control on
   this entity is added, deleted, or modified.  If you call this in
   the entity added callback for the domain, you are guaranteed to get
   this set before any sensors exist. */
typedef void (*ipmi_entity_control_cb)(enum ipmi_update_e op,
				       ipmi_entity_t      *ent,
				       ipmi_control_t     *control,
				       void               *cb_data);
int ipmi_entity_set_control_update_handler(ipmi_entity_t          *ent,
					   ipmi_entity_control_cb handler,
					   void                   *cb_data);

/* Handles events from the given sensor with the handler.  Only one
   handler may be registered against a sensor, if you call this again
   with a new handler, the old handler will be replaced.  Set the
   handler to NULL to disable it.  The dir variable tells if the
   threshold is being asserted or deasserted.  The high_low value
   tells if the value is going high or low, and the threshold value
   tells which threshold is being reported.  The value_present field
   tells whether the raw or converted values are present.  If the
   "event" field is not NULL, then the log provided is the log that
   caused this event to be generated; it is provided so you may delete
   the log from the SEL. */
enum ipmi_value_present_e { IPMI_NO_VALUES_PRESENT,
			    IPMI_RAW_VALUE_PRESENT,
			    IPMI_BOTH_VALUES_PRESENT };
typedef void (*ipmi_sensor_threshold_event_handler_cb)(
    ipmi_sensor_t               *sensor,
    enum ipmi_event_dir_e       dir,
    enum ipmi_thresh_e          threshold,
    enum ipmi_event_value_dir_e high_low,
    enum ipmi_value_present_e   value_present,
    unsigned int                raw_value,
    double                      value,
    void                        *cb_data,
    ipmi_event_t                *event);
int
ipmi_sensor_threshold_set_event_handler(
    ipmi_sensor_t                          *sensor,
    ipmi_sensor_threshold_event_handler_cb handler,
    void                                   *cb_data);

/* Register a handler for a discrete sensor.  Only one handler may be
   registered against a sensor, if you call this again with a new
   handler, the old handler will be replaced.  Set the handler to NULL
   to disable it.  When an event comes in from the sensor, the
   callback function will be called.  The "dir" variable tells if the
   state is being asserted or deasserted, the offset is the state that
   is being asserted or deasserted.  If the "event" field is not NULL,
   then the event provided is the event that caused this event to be
   generated; it is provided so you may delete the event from the SEL.
   Note that the offset, severity, and prev_severity values will be -1
   if not valid or present. */
typedef void (*ipmi_sensor_discrete_event_handler_cb)(
    ipmi_sensor_t         *sensor,
    enum ipmi_event_dir_e dir,
    int                   offset,
    int                   severity,
    int                   prev_severity,
    void                  *cb_data,
    ipmi_event_t          *event);
int
ipmi_sensor_discrete_set_event_handler(
    ipmi_sensor_t                         *sensor,
    ipmi_sensor_discrete_event_handler_cb handler,
    void                                  *cb_data);

/* The event state is which events are set and cleared for the given
   sensor.  Events are enumerated for threshold events and numbered
   for discrete events.  Use the provided functions to initialize,
   read, and modify an event state. */
typedef struct ipmi_event_state_s ipmi_event_state_t;

/* Return the size of an event state data structure, so you can
   allocate your own and copy them. */
unsigned int ipmi_event_state_size(void);
void ipmi_copy_event_state(ipmi_event_state_t *dest, ipmi_event_state_t *src);

/* Routines to init, clear, set, and query values in the event state. */
void ipmi_event_state_set_events_enabled(ipmi_event_state_t *events, int val);
int ipmi_event_state_get_events_enabled(ipmi_event_state_t *events);
void ipmi_event_state_set_scanning_enabled(ipmi_event_state_t *events,int val);
int ipmi_event_state_get_scanning_enabled(ipmi_event_state_t *events);
void ipmi_event_state_set_busy(ipmi_event_state_t *events, int val);
int ipmi_event_state_get_busy(ipmi_event_state_t *events);
void ipmi_event_state_set_enable_events(ipmi_event_state_t *events, int val);
void ipmi_event_state_init(ipmi_event_state_t *events);
void ipmi_threshold_event_clear(ipmi_event_state_t          *events,
				enum ipmi_thresh_e          type,
				enum ipmi_event_value_dir_e value_dir,
				enum ipmi_event_dir_e       dir);
void ipmi_threshold_event_set(ipmi_event_state_t          *events,
			      enum ipmi_thresh_e          type,
			      enum ipmi_event_value_dir_e value_dir,
			      enum ipmi_event_dir_e       dir);
int ipmi_is_threshold_event_set(ipmi_event_state_t          *events,
				enum ipmi_thresh_e          type,
				enum ipmi_event_value_dir_e value_dir,
				enum ipmi_event_dir_e       dir);
void ipmi_discrete_event_clear(ipmi_event_state_t    *events,
			       int                   event_offset,
			       enum ipmi_event_dir_e dir);
void ipmi_discrete_event_set(ipmi_event_state_t    *events,
			     int                   event_offset,
			     enum ipmi_event_dir_e dir);
int ipmi_is_discrete_event_set(ipmi_event_state_t    *events,
			       int                   event_offset,
			       enum ipmi_event_dir_e dir);

/* A generic callback for a lot of things. */
typedef void (*ipmi_sensor_done_cb)(ipmi_sensor_t *sensor,
				    int           err,
				    void          *cb_data);

/* Set the event enables for the given sensor. */
int ipmi_sensor_events_enable_set(ipmi_sensor_t         *sensor,
				  ipmi_event_state_t    *states,
				  ipmi_sensor_done_cb   done,
				  void                  *cb_data);

/* Get the event enables for the given sensor. */
typedef void (*ipmi_event_enables_get_cb)(ipmi_sensor_t      *sensor,
					  int                err,
					  ipmi_event_state_t *states,
					  void               *cb_data);
int ipmi_sensor_events_enable_get(ipmi_sensor_t             *sensor,
				  ipmi_event_enables_get_cb done,
				  void                      *cb_data);

/* Rearm the current sensor.  This will cause the sensor to resend
   it's current event state if it is out of range.  If
   ipmi_sensor_get_supports_auto_rearm() returns false and you receive
   an event, you have to rearm a sensor manually to get any that event
   from it.  If global_enable is set, all events are enable and the
   state is ignored (and may be NULL).  Otherwise, the events set in
   the state are enabled. */
int ipmi_sensor_rearm(ipmi_sensor_t       *sensor,
		      int                 global_enable,
		      ipmi_event_state_t  *state,
		      ipmi_sensor_done_cb done,
		      void                *cb_data);

/* Get the hysteresis values for the given sensor.
   FIXME - these are currently the raw values, how do I get the
   cooked values?  There doesn't seem to be an easy way to calculate them. */
typedef void (*ipmi_hysteresis_get_cb)(ipmi_sensor_t *sensor,
				       int           err,
				       unsigned int  positive_hysteresis,
				       unsigned int  negative_hysteresis,
				       void          *cb_data);
int ipmi_sensor_get_hysteresis(ipmi_sensor_t           *sensor,
			       ipmi_hysteresis_get_cb done,
			       void                   *cb_data);

/* Set the hysteresis values for the given sensor.
   FIXME - these are currently the raw values, how do I handle the
   cooked values?  There doesn't seem to be an easy way to calculate them. */
int ipmi_sensor_set_hysteresis(ipmi_sensor_t       *sensor,
			       unsigned int        positive_hysteresis,
			       unsigned int        negative_hysteresis,
			       ipmi_sensor_done_cb done,
			       void                *cb_data);

/* Get the LUN and sensor number for the sensor, as viewed from its
   management controller. */
int ipmi_sensor_get_num(ipmi_sensor_t *sensor,
			int           *lun,
			int           *num);

/* Strings for various values for a sensor.  We put them in here, and
   they will be the correct strings even for OEM values. */
char *ipmi_sensor_get_sensor_type_string(ipmi_sensor_t *sensor);
char *ipmi_sensor_get_event_reading_type_string(ipmi_sensor_t *sensor);
char *ipmi_sensor_get_rate_unit_string(ipmi_sensor_t *sensor);
char *ipmi_sensor_get_base_unit_string(ipmi_sensor_t *sensor);
char *ipmi_sensor_get_modifier_unit_string(ipmi_sensor_t *sensor);

/* This call is a little different from the other string calls.  For a
   discrete sensor, you can pass the offset into this call and it will
   return the string associated with the reading.  This way, OEM
   sensors can supply their own strings as necessary for the various
   offsets. */
char *ipmi_sensor_reading_name_string(ipmi_sensor_t *sensor, int offset);

/* Get the entity the sensor is hooked to. */
int ipmi_sensor_get_entity_id(ipmi_sensor_t *sensor);
int ipmi_sensor_get_entity_instance(ipmi_sensor_t *sensor);
ipmi_entity_t *ipmi_sensor_get_entity(ipmi_sensor_t *sensor);

/* Information about a sensor from it's SDR.  These are things that
   are specified by IPMI, see the spec for more details. */
int ipmi_sensor_get_sensor_init_scanning(ipmi_sensor_t *sensor);
int ipmi_sensor_get_sensor_init_events(ipmi_sensor_t *sensor);
int ipmi_sensor_get_sensor_init_thresholds(ipmi_sensor_t *sensor);
int ipmi_sensor_get_sensor_init_hysteresis(ipmi_sensor_t *sensor);
int ipmi_sensor_get_sensor_init_type(ipmi_sensor_t *sensor);
int ipmi_sensor_get_sensor_init_pu_events(ipmi_sensor_t *sensor);
int ipmi_sensor_get_sensor_init_pu_scanning(ipmi_sensor_t *sensor);
int ipmi_sensor_get_ignore_if_no_entity(ipmi_sensor_t *sensor);
int ipmi_sensor_get_supports_auto_rearm(ipmi_sensor_t *sensor);

/* Returns IPMI_THRESHOLD_ACCESS_SUPPORT_xxx */
int ipmi_sensor_get_threshold_access(ipmi_sensor_t *sensor);

/* Returns IPMI_HYSTERESIS_SUPPORT_xxx */
int ipmi_sensor_get_hysteresis_support(ipmi_sensor_t *sensor);

/* Returns IPMI_EVENT_SUPPORT_xxx */
int ipmi_sensor_get_event_support(ipmi_sensor_t *sensor);

/* Returns IPMI_SENSOR_TYPE_xxx */
int ipmi_sensor_get_sensor_type(ipmi_sensor_t *sensor);

/* Returns IPMI_EVENT_READING_TYPE_xxx */
int ipmi_sensor_get_event_reading_type(ipmi_sensor_t *sensor);

/* Returns if an assertion event is supported for this particular sensor. */
int ipmi_sensor_threshold_assertion_event_supported(
    ipmi_sensor_t               *sensor,
    enum ipmi_thresh_e          event,
    enum ipmi_event_value_dir_e dir,
    int                         *val);

/* Returns if a deassertion event is supported for this particular sensor. */
int ipmi_sensor_threshold_deassertion_event_supported(
    ipmi_sensor_t               *sensor,
    enum ipmi_thresh_e          event,
    enum ipmi_event_value_dir_e dir,
    int                         *val);

/* Returns if a specific threshold can be set. */
int ipmi_sensor_threshold_settable(ipmi_sensor_t      *sensor,
				   enum ipmi_thresh_e event,
				   int                *val);

/* Returns if a specific threshold can be read. */
int ipmi_sensor_threshold_readable(ipmi_sensor_t      *sensor,
				   enum ipmi_thresh_e event,
				   int                *val);

/* Returns if the assertion of a specific event can send an event */
int ipmi_sensor_discrete_assertion_event_supported(ipmi_sensor_t *sensor,
						   int           event,
						   int           *val);

/* Returns if the deassertion of a specific event can send an event */
int ipmi_sensor_discrete_deassertion_event_supported(ipmi_sensor_t *sensor,
						     int           event,
						     int           *val);

/* Returns if the specific event can be read (is supported). */
int ipmi_discrete_event_readable(ipmi_sensor_t *sensor,
				 int           event,
				 int           *val);

/* Returns IPMI_ANALOG_DATA_FORMAT_xxx */
int ipmi_sensor_get_analog_data_format(ipmi_sensor_t *sensor);

/* Returns IPMI_RATE_UNIT_xxx */
int ipmi_sensor_get_rate_unit(ipmi_sensor_t *sensor);

/* Returns IPMI_MODIFIER_UNIT_xxx */
int ipmi_sensor_get_modifier_unit_use(ipmi_sensor_t *sensor);

/* Returns if the value is a percentage. */
int ipmi_sensor_get_percentage(ipmi_sensor_t *sensor);

/* Returns IPMI_UNIT_TYPE_xxx */
int ipmi_sensor_get_base_unit(ipmi_sensor_t *sensor);

/* Returns IPMI_UNIT_TYPE_xxx */
int ipmi_sensor_get_modifier_unit(ipmi_sensor_t *sensor);

/* Sensor reading information from the SDR. */
int ipmi_sensor_get_tolerance(ipmi_sensor_t *sensor,
			      int           val,
			      double        *tolerance);
int ipmi_sensor_get_accuracy(ipmi_sensor_t *sensor, int val, double *accuracy);
int ipmi_sensor_get_normal_min_specified(ipmi_sensor_t *sensor);
int ipmi_sensor_get_normal_max_specified(ipmi_sensor_t *sensor);
int ipmi_sensor_get_nominal_reading_specified(ipmi_sensor_t *sensor);
int ipmi_sensor_get_nominal_reading(ipmi_sensor_t *sensor,
				    double *nominal_reading);
int ipmi_sensor_get_normal_max(ipmi_sensor_t *sensor, double *normal_max);
int ipmi_sensor_get_normal_min(ipmi_sensor_t *sensor, double *normal_min);
int ipmi_sensor_get_sensor_max(ipmi_sensor_t *sensor, double *sensor_max);
int ipmi_sensor_get_sensor_min(ipmi_sensor_t *sensor, double *sensor_min);

int ipmi_sensor_get_oem1(ipmi_sensor_t *sensor);

/* The ID string from the SDR. */
int ipmi_sensor_get_id_length(ipmi_sensor_t *sensor);
void ipmi_sensor_get_id(ipmi_sensor_t *sensor, char *id, int length);

/* Returns true if the sensor reports when an operator want to remove
   the hot-swappable entity from the system.  If this returns true,
   the offset will be set to the offset in the sensor of the hot-swap
   request value.  val_when_requesting will be set to the value (1 or
   0) that corresponds to the sensor requesting a hot-swap.  This
   should generally be a slot sensor (sensor type 21h). */
int ipmi_sensor_is_hot_swap_requester(ipmi_sensor_t *sensor,
				      unsigned int  *offset,
				      unsigned int  *val_when_requesting);


/* This is the implementation for a set of thresholds for a
   threshold-based sensor. */
typedef struct ipmi_thresholds_s ipmi_thresholds_t;

/* Return the size of a threshold data structure, so you can allocate
   your own and copy them. */
unsigned int ipmi_thresholds_size(void);
void ipmi_copy_thresholds(ipmi_thresholds_t *dest, ipmi_thresholds_t *src);

/* Clear out all the thresholds. */
int ipmi_thresholds_init(ipmi_thresholds_t *th);

/* Set a threshold and make it valid in the thresholds data structure.
   If sensor is non-null, it verifies that the given threshold can be
   set for the sensor. */
int ipmi_threshold_set(ipmi_thresholds_t  *th,
		       ipmi_sensor_t      *sensor,
		       enum ipmi_thresh_e threshold,
		       double             value);
/* Return the value of the threshold in the set of thresholds.
   Returns an error if the threshold is not set. */
int ipmi_threshold_get(ipmi_thresholds_t  *th,
		       enum ipmi_thresh_e threshold,
		       double             *value);

		       
/* Set the thresholds for the given sensor. */
int ipmi_thresholds_set(ipmi_sensor_t       *sensor,
			ipmi_thresholds_t   *thresholds,
			ipmi_sensor_done_cb done,
			void                *cb_data);

/* Fetch the thresholds from the given sensor. */
typedef void (*ipmi_thresh_get_cb)(ipmi_sensor_t     *sensor,
				   int               err,
				   ipmi_thresholds_t *th,
				   void              *cb_data);
int ipmi_thresholds_get(ipmi_sensor_t      *sensor,
			ipmi_thresh_get_cb done,
			void               *cb_data);

/* Discrete states, or threshold status.  This is the set of states or
   thresholds that the sensor has enabled event for, and the global
   event state of the sensor. */
typedef struct ipmi_states_s ipmi_states_t;

/* Get the size of ipmi_states_t, so you can allocate your own and
   copy them. */
unsigned int ipmi_states_size(void);
void ipmi_copy_states(ipmi_states_t *dest, ipmi_states_t *src);

/* Various global values in the states value.  See the IPMI "Get
   Sensor Readings" command in the IPMI spec for details on the
   meanings of these. */
int ipmi_is_event_messages_enabled(ipmi_states_t *states);
int ipmi_is_sensor_scanning_enabled(ipmi_states_t *states);
int ipmi_is_initial_update_in_progress(ipmi_states_t *states);

/* Use to tell if a discrete offset is set in the states. */
int ipmi_is_state_set(ipmi_states_t *states,
		      int           state_num);

/* Use to tell if a threshold is out of range in a threshold sensor. */
int ipmi_is_threshold_out_of_range(ipmi_states_t      *states,
				   enum ipmi_thresh_e thresh);

/* The following functions allow you to create and modify your own
   states structure. */
void ipmi_init_states(ipmi_states_t *states);
void ipmi_set_event_messages_enabled(ipmi_states_t *states, int val);
void ipmi_set_sensor_scanning_enabled(ipmi_states_t *states, int val);
void ipmi_set_initial_update_in_progress(ipmi_states_t *states, int val);
void ipmi_set_state(ipmi_states_t *states,
		    int           state_num,
		    int           val);
void ipmi_set_threshold_out_of_range(ipmi_states_t      *states,
				     enum ipmi_thresh_e thresh,
				     int                val);

/* Read the current value of the given threshold sensor.  It also
   returns the states of all the thresholds. */
typedef void (*ipmi_reading_done_cb)(ipmi_sensor_t             *sensor,
				     int                       err,
				     enum ipmi_value_present_e value_present,
				     unsigned int              raw_value,
				     double                    val,
				     ipmi_states_t             *states,
				     void                      *cb_data);
int ipmi_reading_get(ipmi_sensor_t        *sensor,
		     ipmi_reading_done_cb done,
		     void                 *cb_data);

/* Read the current value of the given threshold sensor, returning the
   set of states that are active. */
typedef void (*ipmi_states_read_cb)(ipmi_sensor_t *sensor,
				    int           err,
				    ipmi_states_t *states,
				    void          *cb_data);
int ipmi_states_get(ipmi_sensor_t       *sensor,
		    ipmi_states_read_cb done,
		    void                *cb_data);


/*
 * Controls are lights, relays, displays, alarms, or other things of
 * that nature.  Basically, output devices.  IPMI does not define
 * these, but they are pretty fundamental for system management.
 */
int ipmi_control_get_type(ipmi_control_t *control);
int ipmi_control_get_id_length(ipmi_control_t *control);
void ipmi_control_get_id(ipmi_control_t *control, char *id, int length);
int ipmi_control_get_entity_id(ipmi_control_t *control);
int ipmi_control_get_entity_instance(ipmi_control_t *control);
int ipmi_control_is_settable(ipmi_control_t *control);
int ipmi_control_is_readable(ipmi_control_t *control);
ipmi_entity_t *ipmi_control_get_entity(ipmi_control_t *control);
char *ipmi_control_get_type_string(ipmi_control_t *control);

/* Returns true if this control is a hot-swap indicator, meaning that
   is is used to indicate to the operator when it is save to remove a
   hot-swappable device. */
int ipmi_control_is_hot_swap_indicator(ipmi_control_t *control);

/* Get the number of values the control supports. */
int ipmi_control_get_num_vals(ipmi_control_t *control);


/* A general callback for control operations that don't received
   any data. */
typedef void (*ipmi_control_op_cb)(ipmi_control_t *control,
				   int            err,
				   void           *cb_data);

/* Set the setting of an control.  Note that an control may
 support more than one element, the array passed in to "val" must
 match the number of elements the control supports.  All the
 elements will be set simultaneously. */
int ipmi_control_set_val(ipmi_control_t     *control,
			 int                *val,
			 ipmi_control_op_cb handler,
			 void               *cb_data);

/* Get the setting of an control.  Like setting controls, this
   returns an array of values, one for each of the number of elements
   the control supports. */
typedef void (*ipmi_control_val_cb)(ipmi_control_t *control,
				    int            err,
				    int            *val,
				    void           *cb_data);
int ipmi_control_get_val(ipmi_control_t      *control,
			 ipmi_control_val_cb handler,
			 void                *cb_data);

/* For LIGHT types.  */

/* A light control may control one or more lights.  If a light
   control controls more than one light, the lights may not
   be set individually, they are controlled as a group, one set
   command will set them all. */

/* Get the number of settings the light supports. */
int ipmi_control_get_num_light_settings(ipmi_control_t *control,
					unsigned int   light);

/* This describes a setting for a light.  For each setting each light
   is defined to go through a number of transitions.  Each transition
   is described by a color, a time (in milliseconds) that the color is
   present.  For non-blinking lights, there will only be one transition.
   For blinking lights, there will be one or more transition.. */

/* Get the setting for the specific setting.  These return -1 for
   an invalid num. */
int ipmi_control_get_num_light_transitions(ipmi_control_t *control,
					   unsigned int   light,
					   unsigned int   setting);
int ipmi_control_get_light_color(ipmi_control_t *control,
				 unsigned int   light,
				 unsigned int   setting,
				 unsigned int   num);
int ipmi_control_get_light_color_time(ipmi_control_t *control,
				      unsigned int   light,
				      unsigned int   setting,
				      unsigned int   num);

/* RELAY types have no settings. */

/* ALARM types have no settings. */

/* CONTROL types are represented as arrays of unsigned data.
   Identifiers do not support multiple elements, and have their own
   setting function. */
typedef void (*ipmi_control_identifier_val_cb)(ipmi_control_t *control,
					       int            err,
					       unsigned char  *val,
					       int            length,
					       void           *cb_data);
int ipmi_control_identifier_get_val(ipmi_control_t                 *control,
				    ipmi_control_identifier_val_cb handler,
				    void                           *cb_data);
int ipmi_control_identifier_set_val(ipmi_control_t     *control,
				    unsigned char      *val,
				    int                length,
				    ipmi_control_op_cb handler,
				    void               *cb_data);
unsigned int ipmi_control_identifier_get_max_length(ipmi_control_t *control);


/* For DISPLAY types, which are string displays. Displays do not
   support multiple elements, and have their own setting function. */
/* Get the dimensions of the display device.  This assumes a square, which
   is usually (but maybe not always) a good assumption. */
void ipmi_control_get_display_dimensions(ipmi_control_t *control,
					 unsigned int   *columns,
					 unsigned int   *rows);

int ipmi_control_set_display_string(ipmi_control_t     *control,
				    unsigned int       start_row,
				    unsigned int       start_column,
				    char               *str,
				    unsigned int       len,
				    ipmi_control_op_cb handler,
				    void               *cb_data);
				
/* Fetch a string from the display. */
typedef void (*ipmi_control_str_cb)(ipmi_control_t *control,
				    int            err,
				    char           *str,
				    unsigned int   len,
				    void           *cb_data);
int ipmi_control_get_display_string(ipmi_control_t      *control,
				    unsigned int        start_row,
				    unsigned int        start_column,
				    unsigned int        len,
				    ipmi_control_str_cb handler,
				    void                *cb_data);

/* This must be called before calling any other IPMI functions.  It
   sets a mutex and mutex operations for the smi.  You must provide
   an OS handler to use for the system. */
int ipmi_init(os_handler_t *handler);

/* Create a new domain with the given IPMI connection.  The new domain
   is returned in the new_domain variable, the id for the connection
   fail handler is return in con_fail_id. */
int ipmi_init_domain(ipmi_con_t             *con,
		     ipmi_domain_cb         con_fail_handler,
		     void                   *con_fail_cb_data,
		     ipmi_domain_con_fail_t **con_fail_id,
		     ipmi_domain_id_t       *new_domain);


/* This will clean up all the memory associated with IPMI. */
void ipmi_shutdown(void);

void *ipmi_domain_get_user_data(ipmi_domain_t *domain);

/* Close an IPMI connection.  This will free all memory associated
   with the connections, any outstanding responses will be lost, etc.
   All slave MC's will be closed when this is closed. */
typedef void (*close_done_t)(void *cb_data);
int ipmi_close_connection(ipmi_domain_t *domain,
			  close_done_t  close_done,
			  void          *cb_data);

/* Extract a 32-bit integer from the data, IPMI (little-endian) style. */
unsigned int ipmi_get_uint32(unsigned char *data);

/* Extract a 16-bit integer from the data, IPMI (little-endian) style. */
unsigned int ipmi_get_uint16(unsigned char *data);

/* Add a 32-bit integer to the data, IPMI (little-endian) style. */
void ipmi_set_uint32(unsigned char *data, int val);

/* Add a 16-bit integer to the data, IPMI (little-endian) style. */
void ipmi_set_uint16(unsigned char *data, int val);

/* Fetch an IPMI device string as defined in section 37.14 of the IPMI
   version 1.5 manual.  The in_len is the number of input bytes in the
   string, including the type/length byte.  The max_out_len is the
   maximum number of characters to output, including the nil */
void ipmi_get_device_string(unsigned char *input,
			    int           in_len,
			    char          *output,
			    int           max_out_len);

/* Store an IPMI device string in the most compact form possible.
   input is the input string (nil terminated), output is where to
   place the output (including the type/length byte) and out_len is a
   pointer to the max size of output (including the type/length byte).
   Upon return, out_len will be set to the actual output length. */
void ipmi_set_device_string(char          *input,
			    unsigned char *output,
			    int           *out_len);

#endif /* __IPMIIF_H */

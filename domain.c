/*
 * domain.c
 *
 * MontaVista IPMI code for handling IPMI domains
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

/* Rescan the bus for MCs every 10 minutes by default. */
#define IPMI_RESCAN_BUS_INTERVAL 600

/* This is the number of device ID queries that an MC must not respond
   to in a row to be considered dead. */
#define MAX_MC_MISSED_RESPONSES 10

enum ipmi_con_state_e { DEAD = 0,
			QUERYING_DEVICE_ID,
			QUERYING_MAIN_SDRS,
			QUERYING_SENSOR_SDRS,
			QUERYING_CHANNEL_INFO,
			OPERATIONAL };

    /* This is a retry count for missed pings from an MC query. */
    int missed_responses;

/* Timer structure fo rescanning the bus. */
typedef struct bmc_rescan_bus_info_s
{
    int       cancelled;
    ipmi_mc_t *bmc;
} bmc_rescan_bus_info_t;

struct ipmi_bmc_con_fail_s
{
    ipmi_bmc_cb handler;
    void        *cb_data;
};

/* Used to keep a record of a bus scan. */
typedef struct mc_ipbm_scan_info_s mc_ipmb_scan_info_t;
struct mc_ipbm_scan_info_s
{
    ipmi_ipmb_addr_t addr;
    ipmi_mc_t        *bmc;
    ipmi_msg_t       msg;
    unsigned int     end_addr;
    ipmi_bmc_cb      done_handler;
    void             *cb_data;
    mc_ipmb_scan_info_t *next;
};

typedef struct ipmi_bmc_s
{
    /* The main set of SDRs on a BMC. */
    ipmi_sdr_info_t *main_sdrs;

    enum ipmi_con_state_e state;

    ipmi_chan_info_t chan[MAX_IPMI_USED_CHANNELS];
    unsigned char    msg_int_type;
    unsigned char    event_msg_int_type;

    /* This is the actual address of the BMC. */
    unsigned char bmc_slave_addr;
    ipmi_mc_slave_addr_fetch_cb slave_addr_fetcher;

    /* The sensors that came from the main SDR. */
    ipmi_sensor_t **sensors_in_my_sdr;
    unsigned int  sensors_in_my_sdr_count;

    ilist_t            *mc_list;
    ipmi_lock_t        *mc_list_lock;

    ipmi_event_handler_id_t  *event_handlers;
    ipmi_lock_t              *event_handlers_lock;
    ipmi_oem_event_handler_cb oem_event_handler;
    void                      *oem_event_cb_data;

    /* Are we in the middle of an MC bus scan? */
    int                scanning_bus;

    ipmi_entity_info_t *entities;
    ipmi_lock_t        *entities_lock;
    ipmi_bmc_entity_cb entity_handler;

    ipmi_ll_event_handler_id_t *ll_event_id;

    ipmi_con_t  *conn;

    ipmi_bmc_oem_new_entity_cb new_entity_handler;
    void                       *new_entity_cb_data;

    ipmi_bmc_oem_new_mc_cb     new_mc_handler;
    void                       *new_mc_cb_data;

    ipmi_oem_setup_finished_cb setup_finished_handler;
    void                       *setup_finished_cb_data;

    /* Should I do a full bus scan for devices on the bus? */
    int                        do_bus_scan;

    /* Timer for rescanning the bus periodically. */
    unsigned int          bus_scan_interval; /* seconds between scans */
    os_hnd_timer_id_t     *bus_scan_timer;
    bmc_rescan_bus_info_t *bus_scan_timer_info;

    /* This is a list of all the bus scans currently happening, so
       they can be properly freed. */
    mc_ipmb_scan_info_t *bus_scans_running;

    ilist_t *con_fail_handlers;

    /* A list of IPMB addresses to not scan. */
    ilist_t *ipmb_ignores;

    /* Is the low-level connection up? */
    int connection_up;

    ipmi_bmc_cb setup_done;
    void        *setup_done_cb_data;

} ipmi_bmc_t;

struct ipmi_event_handler_id_s
{
    ipmi_mc_t                  *mc;
    ipmi_event_handler_t       handler;
    void                       *event_data;

    ipmi_event_handler_id_t *next, *prev;
};

void
ipmi_bmc_set_ipmb_rescan_time(ipmi_mc_t *bmc, unsigned int seconds)
{
    CHECK_MC_LOCK(bmc);

    bmc->bmc->bus_scan_interval = seconds;
}

unsigned int
ipmi_bmc_get_ipmb_rescan_time(ipmi_mc_t *bmc)
{
    CHECK_MC_LOCK(bmc);

    return bmc->bmc->bus_scan_interval;
}

int
_ipmi_domain_init(void)
{
}

void
_ipmi_domain_shutdown(void)
{
}

/* A list of all the registered BMCs. */
static ipmi_mc_t *bmcs = NULL;

static void
add_known_bmc(ipmi_mc_t *bmc)
{
    bmc->prev_bmc = NULL;
    bmc->next_bmc = bmcs;
    if (bmcs)
	bmcs->prev_bmc = bmc;
    bmcs = bmc;
}

static void
remove_known_bmc(ipmi_mc_t *bmc)
{
    if (bmc->next_bmc)
	bmc->next_bmc->prev_bmc = bmc->prev_bmc;
    if (bmc->prev_bmc)
	bmc->prev_bmc->next_bmc = bmc->next_bmc;
    else
	bmcs = bmc->next_bmc;
}

/* Validate that the BMC and it's underlying connection is valid.
   This must be called with the read lock held. */
static int
ipmi_bmc_validate(ipmi_mc_t *bmc)
{
    ipmi_mc_t *c;

    c = bmcs;
    while (c != NULL) {
	if (c == bmc)
	    break;
    }
    if (c == NULL)
	return EINVAL;

    /* We do this check after we find the BMC in the list, because
       want to make sure the pointer is good before we do this. */
    if (!bmc->valid)
	return EINVAL;

    return __ipmi_validate(bmc->bmc->conn);
}

typedef struct mc_cmp_info_s
{
    ipmi_addr_t addr;
    int         addr_len;
} mc_cmp_info_t;

static
int mc_cmp(void *item, void *cb_data)
{
    ipmi_mc_t     *mc = item;
    mc_cmp_info_t *info = cb_data;

    return ipmi_addr_equal(&(mc->addr), mc->addr_len,
			   &(info->addr), info->addr_len);
}

static ipmi_mc_t *
find_mc_by_addr(ipmi_mc_t   *bmc,
		ipmi_addr_t *addr,
		int         addr_len)
{
    mc_cmp_info_t    info;

    /* Cheap hack to handle the BMC LUN. */
    if (addr->addr_type == IPMI_SYSTEM_INTERFACE_ADDR_TYPE) {
	return bmc;
    }
    if (addr->addr_type == IPMI_IPMB_ADDR_TYPE) {
	struct ipmi_ipmb_addr *ipmb_addr = (void *) addr;

	if (ipmb_addr->slave_addr == bmc->bmc->bmc_slave_addr) {
	    return bmc;
	}

	memcpy(&(info.addr), addr, addr_len);
	info.addr_len = addr_len;
	return ilist_search(bmc->bmc->mc_list, mc_cmp, &info);
    }
    return NULL;
}

static int
in_ipmb_ignores(ipmi_mc_t *bmc, unsigned char ipmb_addr)
{
    unsigned long addr;
    ilist_iter_t iter;

    ilist_init_iter(&iter, bmc->bmc->ipmb_ignores);
    ilist_unpositioned(&iter);
    while (ilist_next(&iter)) {
	addr = (unsigned long) ilist_get(&iter);
	if (addr == ipmb_addr)
	    return 1;
    }

    return 0;
}

int
ipmi_mc_find_or_create_mc_by_slave_addr(ipmi_mc_t    *bmc,
					unsigned int slave_addr,
					ipmi_mc_t    **new_mc)
{
    ipmi_mc_t        *mc;
    ipmi_ipmb_addr_t addr;
    int              rv;

    addr.addr_type = IPMI_IPMB_ADDR_TYPE;
    addr.channel = 0;
    addr.lun = 0;
    addr.slave_addr = slave_addr;

    if (slave_addr == bmc->bmc->bmc_slave_addr) {
	*new_mc = bmc;
	return 0;
    }

    mc = find_mc_by_addr(bmc, (ipmi_addr_t *) &addr, sizeof(addr));
    if (mc) {
	*new_mc = mc;
	return 0;
    }

    rv = ipmi_create_mc(bmc, (ipmi_addr_t *) &addr, sizeof(addr), &mc);
    if (rv)
	return rv;

    mc->active = 0;

    rv = ipmi_add_mc_to_bmc(bmc, mc);
    if (rv) {
	ipmi_cleanup_mc(mc);
	return rv;
    }

    *new_mc = mc;
    return 0;
}

int
ipmi_bmc_add_con_fail_handler(ipmi_mc_t           *bmc,
			      ipmi_bmc_cb         handler,
			      void                *cb_data,
			      ipmi_bmc_con_fail_t **id)
{
    ipmi_bmc_con_fail_t *new_id;

    if (bmc->bmc_mc != bmc)
	/* Not a BMC. */
	return EINVAL;

    new_id = ipmi_mem_alloc(sizeof(*new_id));
    if (!new_id)
	return ENOMEM;

    new_id->handler = handler;
    new_id->cb_data = cb_data;
    if (! ilist_add_tail(bmc->bmc->con_fail_handlers, new_id, NULL)) {
	ipmi_mem_free(new_id);
	return ENOMEM;
    }

    return 0;
}

int
ipmi_bmc_add_ipmb_ignore(ipmi_mc_t *bmc, unsigned char ipmb_addr)
{
    unsigned long addr = ipmb_addr;

    if (bmc->bmc_mc != bmc)
	/* Not a BMC. */
	return EINVAL;

    if (! ilist_add_tail(bmc->bmc->ipmb_ignores, (void *) addr, NULL))
	return ENOMEM;

    return 0;
}

void
ipmi_bmc_remove_con_fail_handler(ipmi_mc_t           *bmc,
				 ipmi_bmc_con_fail_t *id)
{
    ilist_iter_t iter;
    int          rv;

    if (bmc->bmc_mc != bmc)
	/* Not a BMC. */
	return;

    ilist_init_iter(&iter, bmc->bmc->con_fail_handlers);
    rv = ilist_first(&iter);
    while (rv) {
	if (ilist_get(&iter) == id) {
	    ilist_delete(&iter);
	    ipmi_mem_free(id);
	    break;
	}
	rv = ilist_next(&iter);
    }
}

typedef struct con_fail_info_s
{
    ipmi_mc_t *bmc;
    int       err;
} con_fail_info_t;

static void
iterate_con_fails(ilist_iter_t *iter, void *item, void *cb_data)
{
    con_fail_info_t     *info = cb_data;
    ipmi_bmc_con_fail_t *id = item;

    id->handler(info->bmc, info->err, id->cb_data);
}

static void
ll_con_failed(ipmi_con_t *ipmi,
	      int        err,
	      void       *cb_data)
{
    ipmi_mc_t       *bmc = cb_data;
    con_fail_info_t info = {bmc, err};
    int             rv;

    ipmi_read_lock();
    rv = ipmi_bmc_validate(bmc);
    if (rv)
	/* So the connection failed.  So what, there's no BMC. */
	goto out_unlock;

    if (err) {
	bmc->bmc->connection_up = 0;
    } else {
	bmc->bmc->connection_up = 1;
	/* When a connection comes back up, rescan the bus and do
           entity presence detection. */
	ipmi_lock(bmc->bmc->mc_list_lock);
	start_mc_scan(bmc);
	ipmi_detect_ents_presence_changes(bmc->bmc->entities, 1);
	ipmi_unlock(bmc->bmc->mc_list_lock);
    }

    ilist_iter(bmc->bmc->con_fail_handlers, iterate_con_fails, &info);

 out_unlock:
    ipmi_read_unlock();
}

static void
ll_rsp_handler(ipmi_con_t   *ipmi,
	       ipmi_addr_t  *addr,
	       unsigned int addr_len,
	       ipmi_msg_t   *msg,
	       void         *rsp_data,
	       void         *data2,
	       void         *data3)
{
    ipmi_response_handler_t rsp_handler = data2;
    ipmi_mc_t               *bmc = data3;
    ipmi_mc_t               *mc;
    int                     rv;

    if (rsp_handler) {
	ipmi_read_lock();
	rv = ipmi_bmc_validate(bmc);
	if (rv)
	    rsp_handler(NULL, msg, rsp_data);
	else {
	    ipmi_lock(bmc->bmc->mc_list_lock);
	    mc = find_mc_by_addr(bmc, addr, addr_len);
	    rsp_handler(mc, msg, rsp_data);
	    ipmi_unlock(bmc->bmc->mc_list_lock);
	}
	ipmi_read_unlock();
    }
}

int
ipmi_send_command_addr(ipmi_domain_t                *domain,
		       ipmi_addr_t		    *addr,
		       unsigned int                 addr_len,
		       ipmi_msg_t                   *msg,
		       ipmi_addr_response_handler_t rsp_handler,
		       void                         *rsp_data)
{
    int rv;

    if (bmc->bmc == NULL)
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    rv = bmc->bmc->conn->send_command(bmc->bmc->conn,
				      addr, addr_len,
				      msg,
				      ll_rsp_handler, rsp_data,
				      rsp_handler, bmc);
    return rv;
}

/* Must be called with event_lock held. */
static void
add_event_handler(ipmi_mc_t                *mc,
		  ipmi_event_handler_id_t  *event)
{
    event->mc = mc;

    event->next = mc->bmc->event_handlers;
    event->prev = NULL;
    if (mc->bmc->event_handlers)
	mc->bmc->event_handlers->prev = event;
    mc->bmc->event_handlers = event;
}

static int
remove_event_handler(ipmi_mc_t               *mc,
		     ipmi_event_handler_id_t *event)
{
    ipmi_event_handler_id_t *ev;

    ev = mc->bmc->event_handlers;
    while (ev != NULL) {
	if (ev == event)
	    break;
	ev = ev->next;
    }

    if (!ev)
	return EINVAL;

    if (event->next)
	event->next->prev = event->prev;
    if (event->prev)
	event->prev->next = event->next;
    else
	mc->bmc->event_handlers = event->next;

    ipmi_mem_free(event);

    return 0;
}

typedef struct event_sensor_info_s
{
    int          handled;
    int          err;
    ipmi_event_t *event;
} event_sensor_info_t;

void
event_sensor_cb(ipmi_sensor_t *sensor, void *cb_data)
{
    event_sensor_info_t *info = cb_data;

    /* It's an event for a specific sensor, and the sensor exists. */
    info->err = ipmi_sensor_event(sensor, info->event);
}

int
ipmi_bmc_set_oem_event_handler(ipmi_mc_t                 *bmc,
			       ipmi_oem_event_handler_cb handler,
			       void                      *cb_data)
{
    if (bmc->bmc == NULL)
	return EINVAL;

    bmc->bmc->oem_event_handler = handler;
    bmc->bmc->oem_event_cb_data = cb_data;
    return 0;
}

int
ipmi_mc_set_oem_event_handler(ipmi_mc_t                 *mc,
			      ipmi_oem_event_handler_cb handler,
			      void                      *cb_data)
{
    mc->oem_event_handler = handler;
    mc->oem_event_handler_cb_data = cb_data;
    return 0;
}

void
mc_event_cb(ipmi_mc_t *mc, void *cb_data)
{
    event_sensor_info_t *info = cb_data;

    if (mc->oem_event_handler)
	info->handled = mc->oem_event_handler(mc,
					      info->event,
					      mc->oem_event_handler_cb_data);
}

void
ipmi_handle_unhandled_event(ipmi_mc_t *mc, ipmi_event_t *event)
{
    ipmi_event_handler_id_t *l;

    ipmi_lock(mc->bmc_mc->bmc->event_handlers_lock);
    l = mc->bmc_mc->bmc->event_handlers;
    while (l) {
	l->handler(mc, event, l->event_data);
	l = l->next;
    }
    ipmi_unlock(mc->bmc_mc->bmc->event_handlers_lock);
}

static void
system_event_handler(ipmi_mc_t    *mc,
		     ipmi_event_t *event)
{
    int                 rv = 1;
    ipmi_sensor_id_t    id;
    event_sensor_info_t info;
    unsigned long       timestamp;

    if (DEBUG_EVENTS) {
	ipmi_log(IPMI_LOG_DEBUG,
		 "Event recid mc (%d 0x%x):%4.4x type:%2.2x:"
		 " %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x"
		 " %2.2x %2.2x %2.2x %2.2x %2.2x",
		 event->mc_id.channel, event->mc_id.mc_num,
		 event->record_id, event->type,
		 event->data[0], event->data[1], event->data[2],
		 event->data[3], event->data[4], event->data[5],
		 event->data[6], event->data[7], event->data[8],
		 event->data[9], event->data[10], event->data[11],
		 event->data[12]);
    }

    /* Let the OEM handler have a go at it first.  Note that OEM
       handlers must look at the time themselves. */
    if (mc->bmc_mc->bmc->oem_event_handler) {
	if (mc->bmc_mc->bmc->oem_event_handler(
	    mc,
	    event,
	    mc->bmc_mc->bmc->oem_event_cb_data))
	    return;
    }

    timestamp = ipmi_get_uint32(&(event->data[0]));

    /* It's a system event record from an MC, and the timestamp is
       later than our startup timestamp. */
    if ((event->type == 0x02)
	&& ((event->data[4] & 0x01) == 0)
	&& (timestamp >= mc->startup_SEL_time))
    {
	ipmi_mc_id_t mc_id;

	info.handled = 0;
	info.err = 0;
	info.event = event;

	/* See if the MC has an OEM handler for this. */
	mc_id.bmc = mc->bmc_mc;
	if (event->data[6] == 0x03) {
	    mc_id.channel = 0;
	} else {
	    mc_id.channel = event->data[5] >> 4;
	}
	mc_id.mc_num = event->data[4];
	ipmi_mc_pointer_noseq_cb(mc_id, mc_event_cb, &info);

	if (info.handled) {
	    rv = 0;
	} else {
	    /* The OEM code didn't handle it. */
	    id.mc_id.bmc = mc->bmc_mc;
	    if (event->data[6] == 0x03)
		id.mc_id.channel = 0;
	    else
		id.mc_id.channel = event->data[5] >> 4;
	    id.mc_id.mc_num = event->data[4];
	    id.lun = event->data[5] & 0x3;
	    id.sensor_num = event->data[8];

	    rv = ipmi_sensor_pointer_noseq_cb(id, event_sensor_cb, &info);
	    if (!rv)
		rv = info.err;
	}
    }

    /* It's an event from system software, or the info couldn't be found. */
    if (rv)
	ipmi_handle_unhandled_event(mc, event);
}

/* Got a new event in the system event log that we didn't have before. */
static void
mc_sel_new_event_handler(ipmi_sel_info_t *sel,
			 ipmi_event_t    *event,
			 void            *cb_data)
{
    system_event_handler(cb_data, event);
}

static void
mc_rescan_event_handler(ipmi_mc_t *bmc, ipmi_mc_t *mc, void *cb_data)
{
    if (mc->SEL_device_support)
	ipmi_sel_get(mc->sel, NULL, NULL);
}

int
ipmi_bmc_rescan_events(ipmi_mc_t *bmc)
{
    if (bmc->bmc == NULL)
	/* It's not the BMC. */
	return EINVAL;

    if (bmc->SEL_device_support)
	ipmi_sel_get(bmc->sel, NULL, NULL);

    return ipmi_bmc_iterate_mcs(bmc, mc_rescan_event_handler, NULL);
}

static void
ll_event_handler(ipmi_con_t   *ipmi,
		 ipmi_addr_t  *addr,
		 unsigned int addr_len,
		 ipmi_msg_t   *event,
		 void         *event_data,
		 void         *data2)
{
    ipmi_event_t devent;
    ipmi_mc_t    *bmc = data2;

    /* Events coming in through the event handler are always from the
       BMC. */
    devent.mc_id = ipmi_mc_convert_to_id(bmc);
    devent.record_id = ipmi_get_uint16(event->data);
    devent.type = event->data[2];
    memcpy(devent.data, event+3, IPMI_MAX_SEL_DATA);

    /* Add it to the system event log. */
    ipmi_sel_event_add(bmc->sel, &devent);

    /* Call the handler on it. */
    system_event_handler(bmc, &devent);
}

int
ipmi_register_for_events(ipmi_mc_t               *bmc,
			 ipmi_event_handler_t    handler,
			 void                    *event_data,
			 ipmi_event_handler_id_t **id)
{
    ipmi_event_handler_id_t *elem;

    /* Make sure it's an SMI mc. */
    if (bmc->bmc_mc != bmc)
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    elem = ipmi_mem_alloc(sizeof(*elem));
    if (!elem)
	return ENOMEM;
    elem->handler = handler;
    elem->event_data = event_data;

    ipmi_lock(bmc->bmc->event_handlers_lock);
    add_event_handler(bmc, elem);
    ipmi_unlock(bmc->bmc->event_handlers_lock);

    *id = elem;

    return 0;
}

int
ipmi_deregister_for_events(ipmi_mc_t               *bmc,
			   ipmi_event_handler_id_t *id)
{
    int        rv;

    /* Make sure it's an SMI mc. */
    if (bmc->bmc_mc != bmc)
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    ipmi_lock(bmc->bmc->event_handlers_lock);
    rv = remove_event_handler(bmc, id);
    ipmi_unlock(bmc->bmc->event_handlers_lock);

    return rv;
}

int
ipmi_bmc_disable_events(ipmi_mc_t *bmc)
{
    int rv;

    /* Make sure it's an SMI mc. */
    if (bmc->bmc_mc != bmc)
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    if (! bmc->bmc->ll_event_id)
	return EINVAL;

    rv = bmc->bmc->conn->deregister_for_events(bmc->bmc->conn,
					       bmc->bmc->ll_event_id);
    if (!rv)
	bmc->bmc->ll_event_id = NULL;
    return rv;
}

int
ipmi_bmc_enable_events(ipmi_mc_t *bmc)
{
    int rv;

    /* Make sure it's an SMI mc. */
    if (bmc->bmc_mc != bmc)
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    if (bmc->bmc->ll_event_id)
	return EINVAL;

    rv = bmc->bmc->conn->register_for_events(bmc->bmc->conn,
					     ll_event_handler, NULL, bmc,
					     &(bmc->bmc->ll_event_id));
    return rv;
}

/* Closing a connection is subtle because of locks.  We schedule it to
   be done in a timer callback, that way we can handle all the locks
   as part of the close. */
typedef struct close_info_s
{
    close_done_t close_done;
    void         *cb_data;
    ipmi_mc_t    *bmc;
} close_info_t;

static void
real_close_connection(void *cb_data, os_hnd_timer_id_t *id)
{
    close_info_t *info = cb_data;
    ipmi_mc_t    *bmc = info->bmc;
    ipmi_con_t   *ipmi;

    bmc->bmc->conn->os_hnd->free_timer(bmc->bmc->conn->os_hnd, id);

    ipmi_write_lock();

    remove_known_bmc(bmc);

    ipmi = bmc->bmc->conn;

    ipmi_cleanup_mc(bmc);

    ipmi->close_connection(ipmi);

    ipmi_write_unlock();

    if (info->close_done)
	info->close_done(info->cb_data);
    ipmi_mem_free(info);
}

int
ipmi_close_connection(ipmi_mc_t    *bmc,
		      close_done_t close_done,
		      void         *cb_data)
{
    int               rv;
    close_info_t      *close_info = NULL;
    os_hnd_timer_id_t *timer = NULL;
    struct timeval    timeout;

    if (bmc->bmc_mc != bmc)
	/* You can only shut down BMCs. */
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    close_info = ipmi_mem_alloc(sizeof(*close_info));
    if (!close_info)
	return ENOMEM;

    rv = bmc->bmc->conn->os_hnd->alloc_timer(bmc->bmc->conn->os_hnd, &timer);
    if (rv)
	goto out;

    if ((rv = ipmi_bmc_validate(bmc)))
	goto out;

    close_info->bmc = bmc;
    close_info->close_done = close_done;
    close_info->cb_data = cb_data;

    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    rv = bmc->bmc->conn->os_hnd->start_timer(bmc->bmc->conn->os_hnd,
					     timer,
					     &timeout,
					     real_close_connection,
					     close_info);
 out:
    if (rv) {
	if (close_info)
	    ipmi_mem_free(close_info);
	if (timer)
	    bmc->bmc->conn->os_hnd->free_timer(bmc->bmc->conn->os_hnd, timer);
    } else {
	bmc->valid = 0;
    }
    return rv;
}

void
domain_cleanup()
{
    if (mc->bmc) {
	/* It's a BMC, clean up all the bmc-specific information. */

	/* Delete the sensors from the main SDR repository. */
	if (mc->bmc->sensors_in_my_sdr) {
	    for (i=0; i<mc->bmc->sensors_in_my_sdr_count; i++) {
		if (mc->bmc->sensors_in_my_sdr[i])
		    ipmi_sensor_destroy(mc->bmc->sensors_in_my_sdr[i]);
	    }
	    ipmi_mem_free(mc->bmc->sensors_in_my_sdr);
	}

	/* We cleanup the MCs twice.  Some MCs may not be destroyed
           (but only left inactive) in the first pass due to
           references form other MCs SDR repositories.  The second
           pass will get them all. */
	ilist_iter(mc->bmc->mc_list, iterate_cleanup_mc, NULL);
	ilist_iter(mc->bmc->mc_list, iterate_cleanup_mc, NULL);

	/* Destroy the main SDR repository, if it exists. */
	if (mc->bmc->main_sdrs)
	    ipmi_sdr_info_destroy(mc->bmc->main_sdrs, NULL, NULL);

	if (mc->bmc->bus_scan_timer_info) {
	    mc->bmc->bus_scan_timer_info->cancelled = 1;
	    rv = mc->bmc->conn->os_hnd->stop_timer(mc->bmc->conn->os_hnd,
						   mc->bmc->bus_scan_timer);
	    if (!rv) {
		/* If we can stop the timer, free it and it's info.
                   If we can't stop the timer, that means that the
                   code is currently in the timer handler, so we let
                   the "cancelled" value do this for us. */
		mc->bmc->conn->os_hnd->free_timer(mc->bmc->conn->os_hnd,
						  mc->bmc->bus_scan_timer);
		ipmi_mem_free(mc->bmc->bus_scan_timer_info);
	    }
	}

	ipmi_lock(mc->bmc->event_handlers_lock);
	while (mc->bmc->event_handlers)
	    remove_event_handler(mc, mc->bmc->event_handlers);
	ipmi_unlock(mc->bmc->event_handlers_lock);

	if (mc->bmc->mc_list)
	    free_ilist(mc->bmc->mc_list);
	if (mc->bmc->con_fail_handlers) {
	    ilist_iter_t iter;
	    void         *data;
	    ilist_init_iter(&iter, mc->bmc->con_fail_handlers);
	    while (ilist_first(&iter)) {
		data = ilist_get(&iter);
		ilist_delete(&iter);
		ipmi_mem_free(data);
	    }
	    free_ilist(mc->bmc->con_fail_handlers);
	}
	if (mc->bmc->ipmb_ignores) {
	    ilist_iter_t iter;
	    ilist_init_iter(&iter, mc->bmc->ipmb_ignores);
	    while (ilist_first(&iter)) {
		ilist_delete(&iter);
	    }
	    free_ilist(mc->bmc->ipmb_ignores);
	}
	if (mc->bmc->bus_scans_running) {
	    mc_ipmb_scan_info_t *item;
	    while (mc->bmc->bus_scans_running) {
		item = mc->bmc->bus_scans_running;
		mc->bmc->bus_scans_running = item->next;
		ipmi_mem_free(item);
	    }
	}
	if (mc->bmc->mc_list_lock)
	    ipmi_destroy_lock(mc->bmc->mc_list_lock);
	if (mc->bmc->event_handlers_lock)
	    ipmi_destroy_lock(mc->bmc->event_handlers_lock);
	if (mc->bmc->ll_event_id)
	    mc->bmc->conn->deregister_for_events(mc->bmc->conn,
						 mc->bmc->ll_event_id);

	/* Remove all the connection fail handlers. */
	mc->bmc->conn->set_con_fail_handler(mc->bmc->conn, NULL,  NULL);

	/* When cleaning up a BMC, we always destroy these. */
	if (mc->sdrs)
	    ipmi_sdr_info_destroy(mc->sdrs, NULL, NULL);
	if (mc->sel)
	    ipmi_sel_destroy(mc->sel, NULL, NULL);
	if (mc->sensors)
	    ipmi_sensors_destroy(mc->sensors);
	if (mc->controls)
	    ipmi_controls_destroy(mc->controls);

	/* Destroy the entities last, since sensors and controls may
           refer to them. */
	if (mc->bmc->entities)
	    ipmi_entity_info_destroy(mc->bmc->entities);
	if (mc->bmc->entities_lock)
	    ipmi_destroy_lock(mc->bmc->entities_lock);

	ipmi_mem_free(mc->bmc);
	ipmi_mem_free(mc);
    }
}

int
ipmi_create_mc(ipmi_mc_t    *bmc,
	       ipmi_addr_t  *addr,
	       unsigned int addr_len,
	       ipmi_mc_t    **new_mc)
{
    ipmi_mc_t *mc;
    int       rv = 0;

    if (addr_len > sizeof(ipmi_addr_t))
	return EINVAL;

    mc = ipmi_mem_alloc(sizeof(*mc));
    if (!mc)
	return ENOMEM;
    memset(mc, 0, sizeof(*mc));

    mc->bmc_mc = bmc;

    mc->seq = ipmi_get_seq();

    mc->active = 1;

    mc->bmc = NULL;
    mc->sensors = NULL;
    mc->sensors_in_my_sdr = NULL;
    mc->sensors_in_my_sdr_count = 0;
    mc->controls = NULL;
    mc->new_sensor_handler = NULL;
    mc->removed_mc_handler = NULL;
    mc->sel = NULL;
    mc->sel_timer_info = NULL;

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
    /* When we get new logs, handle them. */
    ipmi_sel_set_new_event_handler(mc->sel,
				   mc_sel_new_event_handler,
				   mc);

 out_err:
    if (rv) {
	ipmi_cleanup_mc(mc);
    }
    else
	*new_mc = mc;

    return rv;
}

static void
check_event_rcvr(ipmi_mc_t *bmc, ipmi_mc_t *mc, void *cb_data)
{
    if (mc->SEL_device_support) {
	unsigned int *addr = cb_data;
	*addr = ipmi_addr_get_slave_addr(&mc->addr);
    }
}

/* Find a valid event receiver in the system. */
static unsigned int
find_event_rcvr(ipmi_mc_t *bmc)
{
    unsigned int addr = 0;

    if (bmc->SEL_device_support) {
	return bmc->bmc->bmc_slave_addr;
    }
    ipmi_bmc_iterate_mcs(bmc, check_event_rcvr, &addr);
    return addr;
}

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
    ipmi_send_command(mc, 0, &msg, set_event_rcvr_done, NULL);
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
    ipmi_send_command(mc, 0, &msg, get_event_rcvr_done,
		      (void *) (unsigned long) addr);
    /* No care about return values, if this fails it will be done
       again later. */
}

static void
do_event_rcvr(ipmi_mc_t *mc)
{
    if (mc && mc->IPMB_event_generator_support) {
	/* We have an MC that is live (or still live) and generates
	   events, make sure the event receiver is set properly. */
	unsigned int event_rcvr = find_event_rcvr(mc->bmc_mc);

	if (event_rcvr) {
	    send_get_event_rcvr(mc, event_rcvr);
	}
    }
}

int
ipmi_add_mc_to_bmc(ipmi_mc_t *bmc, ipmi_mc_t *mc)
{
    int rv;

    CHECK_MC_LOCK(bmc);

    ipmi_lock(bmc->bmc->mc_list_lock);
    rv = !ilist_add_tail(bmc->bmc->mc_list, mc, NULL);
    if (!rv)
	mc->in_bmc_list = 1;

    if (bmc->bmc->new_mc_handler)
	bmc->bmc->new_mc_handler(bmc, mc, bmc->bmc->new_mc_cb_data);

    ipmi_unlock(bmc->bmc->mc_list_lock);

    return rv;
}

static void
add_bus_scans_running(ipmi_mc_t *bmc, mc_ipmb_scan_info_t *info)
{
    info->next = bmc->bmc->bus_scans_running;
    bmc->bmc->bus_scans_running = info;
}

static void
remove_bus_scans_running(ipmi_mc_t *bmc, mc_ipmb_scan_info_t *info)
{
    mc_ipmb_scan_info_t *i2;

    i2 = bmc->bmc->bus_scans_running;
    if (i2 == info)
	bmc->bmc->bus_scans_running = info->next;
    else
	while (i2->next != NULL) {
	    if (i2->next == info) {
		i2->next = info->next;
		break;
	    }
	    i2 = i2->next;
	}
}

static void devid_bc_rsp_handler(ipmi_con_t   *ipmi,
				 ipmi_addr_t  *addr,
				 unsigned int addr_len,
				 ipmi_msg_t   *msg,
				 void         *rsp_data,
				 void         *data2,
				 void         *data3)
{
    mc_ipmb_scan_info_t *info = rsp_data;
    int                 rv;
    ipmi_mc_t           *mc;
    int                 created_here = 0;


    ipmi_read_lock();
    rv = ipmi_bmc_validate(info->bmc);
    if (rv) {
	ipmi_log(IPMI_LOG_INFO,
		 "BMC went away while scanning for MCs");
	ipmi_read_unlock();
	return;
    }

    ipmi_lock(info->bmc->bmc->mc_list_lock);
    /* Found one, start the discovery process on it. */
    mc = find_mc_by_addr(info->bmc, addr, addr_len);
    if (msg->data[0] == 0) {
	if (mc)
	    mc->missed_responses = 0;
	if (mc && mc->active && !mc_device_data_compares(mc, msg)) {
	    /* The MC was replaced with a new one, so clear the old
               one and add a new one. */
	    ipmi_cleanup_mc(mc);
	    mc = NULL;
	}
	if (!mc || !mc->active) {
	    /* It doesn't already exist, or it's inactive, so add
               it. */
	    if (!mc) {
		/* If it's not there, then add it.  If it's just not
                   active, reuse the same data. */
		rv = ipmi_create_mc(info->bmc, addr, addr_len, &mc);
		if (rv) {
		    /* Out of memory, just give up for now. */
		    remove_bus_scans_running(info->bmc, info);
		    ipmi_mem_free(info);
		    ipmi_unlock(info->bmc->bmc->mc_list_lock);
		    goto out;
		}

		rv = ipmi_sdr_info_alloc(mc, 0, 1, &(mc->sdrs));
		if (!rv)
		    rv = ipmi_add_mc_to_bmc(mc->bmc_mc, mc);
		if (rv) {
		    ipmi_cleanup_mc(mc);
		    goto next_addr;
		}
	    }
	    rv = get_device_id_data_from_rsp(mc, msg);
	    if (rv) {
		/* If we couldn't handle the device data, just leave
                   it inactive. */
		mc->active = 0;
		goto next_addr;
	    }

	    if (!rv) {
		created_here = 1;
		if (mc->provides_device_sdrs)
		    rv = ipmi_sdr_fetch(mc->sdrs, mc_sdr_handler, mc);
		else
		    sensors_reread(mc, 0, NULL);
	    }
	    if (rv)
		ipmi_cleanup_mc(mc);
	}
    } else if (mc && mc->active) {
	/* Didn't get a response.  Maybe the MC has gone away? */
	mc->missed_responses++;
	if (mc->missed_responses >= MAX_MC_MISSED_RESPONSES) {
	    ipmi_cleanup_mc(mc);
	    goto next_addr;
	} else {
	    /* Try again right now. */
	    ipmi_unlock(info->bmc->bmc->mc_list_lock);
	    goto retry_addr;
	}
    }

    /* If we didn't create the MC above, then check the event
       receiver.  If the MC was created above, then setting the event
       receiver will be done after the SDRs are read. */
    if (!created_here)
	do_event_rcvr(mc);

 next_addr:
    ipmi_unlock(info->bmc->bmc->mc_list_lock);

 next_addr_nolock:
    if (info->addr.slave_addr == info->end_addr) {
	/* We've hit the end, we can quit now. */
	if (info->done_handler)
	    info->done_handler(info->bmc, 0, info->cb_data);
	remove_bus_scans_running(info->bmc, info);
	ipmi_mem_free(info);
	goto out;
    }
    info->addr.slave_addr += 2;
    if ((info->addr.slave_addr == info->bmc->bmc->bmc_slave_addr)
	|| (in_ipmb_ignores(info->bmc, info->addr.slave_addr)))
    {
	/* We don't scan the BMC, that would be scary.  We also check
           the ignores list. */
	goto next_addr_nolock;
    }

 retry_addr:
    rv = info->bmc->bmc->conn->send_command(info->bmc->bmc->conn,
					    (ipmi_addr_t *) &(info->addr),
					    sizeof(info->addr),
					    &(info->msg),
					    devid_bc_rsp_handler,
					    info, NULL, NULL);
    while ((rv) && (info->addr.slave_addr < info->end_addr)) {
	info->addr.slave_addr += 2;
	rv = info->bmc->bmc->conn->send_command(info->bmc->bmc->conn,
						(ipmi_addr_t *) &(info->addr),
						sizeof(info->addr),
						&(info->msg),
						devid_bc_rsp_handler,
						info, NULL, NULL);
    }

    if (rv) {
	remove_bus_scans_running(info->bmc, info);
	ipmi_mem_free(info);
    }
 out:
    ipmi_read_unlock();
}

void
ipmi_start_ipmb_mc_scan(ipmi_mc_t    *bmc,
	       		int          channel,
	       		unsigned int start_addr,
			unsigned int end_addr,
			ipmi_bmc_cb  done_handler,
			void         *cb_data)
{
    mc_ipmb_scan_info_t *info;
    int                 rv;

    CHECK_MC_LOCK(bmc);

    info = ipmi_mem_alloc(sizeof(*info));
    if (!info)
	return;

    info->bmc = bmc;
    info->addr.addr_type = IPMI_IPMB_BROADCAST_ADDR_TYPE;
    info->addr.channel = channel;
    info->addr.slave_addr = start_addr;
    info->addr.lun = 0;
    info->msg.netfn = IPMI_APP_NETFN;
    info->msg.cmd = IPMI_GET_DEVICE_ID_CMD;
    info->msg.data = NULL;
    info->msg.data_len = 0;
    info->end_addr = end_addr;
    info->done_handler = done_handler;
    info->cb_data = cb_data;
    rv = bmc->bmc->conn->send_command(bmc->bmc->conn,
				      (ipmi_addr_t *) &(info->addr),
				      sizeof(info->addr),
				      &(info->msg),
				      devid_bc_rsp_handler,
				      info, NULL, NULL);
    while ((rv) && (info->addr.slave_addr < end_addr)) {
	info->addr.slave_addr += 2;
	rv = bmc->bmc->conn->send_command(bmc->bmc->conn,
					  (ipmi_addr_t *) &(info->addr),
					  sizeof(info->addr),
					  &(info->msg),
					  devid_bc_rsp_handler,
					  info, NULL, NULL);
    }

    if (rv)
	ipmi_mem_free(info);
    else
	add_bus_scans_running(bmc, info);
}

static void
mc_scan_done(ipmi_mc_t *bmc, int err, void *cb_data)
{
    bmc->bmc->scanning_bus = 0;
}

static void
start_mc_scan(ipmi_mc_t *bmc)
{
    int i;

    if (!bmc->bmc->do_bus_scan)
	return;

    if (bmc->bmc->scanning_bus)
	return;

    bmc->bmc->scanning_bus = 1;

    for (i=0; i<MAX_IPMI_USED_CHANNELS; i++) {
	if (bmc->bmc->chan[i].medium == 1) /* IPMB */
	    ipmi_start_ipmb_mc_scan(bmc, i, 0x10, 0xf0, mc_scan_done, NULL);
    }
}

static void
bmc_rescan_bus(void *cb_data, os_hnd_timer_id_t *id)
{
    struct timeval        timeout;
    bmc_rescan_bus_info_t *info = cb_data;
    ipmi_mc_t             *bmc = info->bmc;

    if (info->cancelled) {
	ipmi_mem_free(info);
	return;
    }

    /* Only operate if we know the connection is up. */
    if (bmc->bmc->connection_up) {
	/* Rescan all the presence sensors to make sure they are valid. */
	ipmi_detect_bmc_presence_changes(bmc, 1);

	ipmi_lock(bmc->bmc->mc_list_lock);
	start_mc_scan(bmc);
	ipmi_unlock(bmc->bmc->mc_list_lock);
    }

    timeout.tv_sec = bmc->bmc->bus_scan_interval;
    timeout.tv_usec = 0;
    bmc->bmc->conn->os_hnd->start_timer(bmc->bmc->conn->os_hnd,
					id,
					&timeout,
					bmc_rescan_bus,
					info);
}

static void
set_operational(ipmi_mc_t *bmc)
{
    struct timeval        timeout;
    bmc_rescan_bus_info_t *info;
    int                   rv;

    /* Report this before we start scanning for entities and
       sensors so the user can register a callback handler for
       those. */
    bmc->bmc->state = OPERATIONAL;
    if (bmc->bmc->setup_done)
	bmc->bmc->setup_done(bmc, 0, bmc->bmc->setup_done_cb_data);

    /* Call the OEM setup finish if it is registered. */
    if (bmc->bmc->setup_finished_handler)
	bmc->bmc->setup_finished_handler(bmc,
					 bmc->bmc->setup_finished_cb_data);

    /* Start an SDR scan. */
    ipmi_entity_scan_sdrs(bmc->bmc->entities, bmc->bmc->main_sdrs);
    ipmi_sensor_handle_sdrs(bmc, NULL, bmc->bmc->main_sdrs);

    /* Scan all the sensors and call sensors_reread() when done. */
    if (bmc->provides_device_sdrs)
	ipmi_mc_reread_sensors(bmc, sensors_reread, NULL);
    else
	sensors_reread(bmc, 0, NULL);

    start_mc_scan(bmc);

    /* Start the timer to rescan the bus periodically. */
    info = ipmi_mem_alloc(sizeof(*info));
    if (!info)
	rv = ENOMEM;
    else {
	info->bmc = bmc;
	info->cancelled = 0;
        timeout.tv_sec = bmc->bmc->bus_scan_interval;
        timeout.tv_usec = 0;
        rv = bmc->bmc->conn->os_hnd->alloc_timer(bmc->bmc->conn->os_hnd,
						 &(bmc->bmc->bus_scan_timer));
	if (!rv) {
	    rv = bmc->bmc->conn->os_hnd->start_timer(bmc->bmc->conn->os_hnd,
						     bmc->bmc->bus_scan_timer,
						     &timeout,
						     bmc_rescan_bus,
						     info);
	    if (rv)
		bmc->bmc->conn->os_hnd->free_timer(bmc->bmc->conn->os_hnd,
						   bmc->bmc->bus_scan_timer);
	}
    }
    if (rv) {
	ipmi_log(IPMI_LOG_SEVERE,
		 "Unable to start the bus scan timer."
		 " The bus will not be scanned periodically.");
    } else {
	bmc->bmc->bus_scan_timer_info = info;
    }
}

static int
setup_bmc(ipmi_con_t   *ipmi,
	  ipmi_addr_t  *mc_addr,
	  int          mc_addr_len,
	  ipmi_mc_t    **new_mc)
{
    ipmi_mc_t *mc;
    int       rv;

    if (mc_addr_len > sizeof(ipmi_addr_t))
	return EINVAL;

    mc = ipmi_mem_alloc(sizeof(*mc));
    if (!mc)
	return ENOMEM;
    memset(mc, 0, sizeof(*mc));

    mc->bmc_mc = mc;
    mc->valid = 1;
    mc->active = 1;

    mc->seq = ipmi_get_seq();
    mc->bmc = NULL;
    mc->sensors = NULL;
    mc->sensors_in_my_sdr = NULL;
    mc->sensors_in_my_sdr_count = 0;
    mc->controls = NULL;
    mc->new_sensor_handler = NULL;
    mc->removed_mc_handler = NULL;
    mc->sel = NULL;

    memcpy(&(mc->addr), mc_addr, mc_addr_len);
    mc->addr_len = mc_addr_len;
    mc->sdrs = NULL;

    mc->bmc = ipmi_mem_alloc(sizeof(*(mc->bmc)));
    if (! (mc->bmc)) {
	rv = ENOMEM;
	goto out_err;
    }
    memset(mc->bmc, 0, sizeof(*(mc->bmc)));

    mc->bmc->bmc_slave_addr = 0x20; /* Assume 0x20 until told otherwise */
    mc->bmc->slave_addr_fetcher = NULL;

    mc->bmc->conn = ipmi;

    /* Create the locks before anything else. */
    mc->bmc->mc_list_lock = NULL;
    mc->bmc->entities_lock = NULL;
    mc->bmc->event_handlers_lock = NULL;

    /* Set the default timer intervals. */
    mc->bmc->sel_scan_interval = IPMI_SEL_QUERY_INTERVAL;
    mc->bmc->bus_scan_interval = IPMI_RESCAN_BUS_INTERVAL;

    rv = ipmi_create_lock(mc, &mc->bmc->mc_list_lock);
    if (rv)
	goto out_err;
    /* Lock this first thing. */
    ipmi_lock(mc->bmc->mc_list_lock);

    rv = ipmi_create_lock(mc, &mc->bmc->entities_lock);
    if (rv)
	goto out_err;

    rv = ipmi_create_lock(mc, &mc->bmc->event_handlers_lock);
    if (rv)
	goto out_err;

    mc->bmc->main_sdrs = NULL;
    mc->bmc->scanning_bus = 0;
    mc->bmc->event_handlers = NULL;
    mc->bmc->oem_event_handler = NULL;
    mc->bmc->mc_list = NULL;
    mc->bmc->entities = NULL;
    mc->bmc->entity_handler = NULL;
    mc->bmc->new_entity_handler = NULL;
    mc->bmc->new_mc_handler = NULL;
    mc->bmc->setup_finished_handler = NULL;
    mc->bmc->do_bus_scan = 1;

    mc->bmc->connection_up = 1;
    mc->bmc->conn->set_con_fail_handler(mc->bmc->conn, ll_con_failed, mc);

    mc->bmc->mc_list = alloc_ilist();
    if (! mc->bmc->mc_list) {
	rv = ENOMEM;
	goto out_err;
    }

    mc->bmc->con_fail_handlers = alloc_ilist();
    if (! mc->bmc->con_fail_handlers) {
	rv = ENOMEM;
	goto out_err;
    }

    mc->bmc->ipmb_ignores = alloc_ilist();
    if (! mc->bmc->ipmb_ignores) {
	rv = ENOMEM;
	goto out_err;
    }

    mc->bmc->bus_scans_running = NULL;

    rv = ipmi_entity_info_alloc(mc, &(mc->bmc->entities));
    if (rv)
	goto out_err;

    rv = ipmi_sensors_alloc(mc, &(mc->sensors));
    if (rv)
	goto out_err;

    rv = ipmi_controls_alloc(mc, &(mc->controls));
    if (rv)
	goto out_err;

    memset(mc->bmc->chan, 0, sizeof(mc->bmc->chan));

    rv = ipmi_sel_alloc(mc, 0, &(mc->sel));
    if (rv)
	goto out_err;
    /* When we get new logs, handle them. */
    ipmi_sel_set_new_event_handler(mc->sel,
				   mc_sel_new_event_handler,
				   mc);

 out_err:
    if (mc->bmc->mc_list_lock)
	ipmi_unlock(mc->bmc->mc_list_lock);

    if (rv) {
	ipmi_cleanup_mc(mc);
    }
    else
	*new_mc = mc;

    return rv;
}

typedef struct init_con_info_s
{
    ipmi_bmc_cb handler;
    void        *cb_data;
} init_con_info_t;

static void
ipmi_init_con(ipmi_con_t   *ipmi,
	      int          err,
	      ipmi_addr_t  *mc_addr,
	      int          mc_addr_len,
	      void         *cb_data)
{
    init_con_info_t *info = cb_data;
    ipmi_msg_t      cmd_msg;
    int             rv = 0;
    ipmi_mc_t       *mc;

    if (err) {
	info->handler(NULL, err, info->cb_data);
	goto out;
    }

    rv = setup_bmc(ipmi, mc_addr, mc_addr_len, &mc);
    if (rv) {
	ipmi->close_connection(ipmi);
	info->handler(NULL, rv, info->cb_data);
	goto out;
    }

    mc->bmc->setup_done = info->handler;
    mc->bmc->setup_done_cb_data = info->cb_data;

    add_known_bmc(mc);

    ipmi_write_lock();
    ipmi_lock(mc->bmc_mc->bmc->mc_list_lock);
    ipmi_write_unlock();

    cmd_msg.netfn = IPMI_APP_NETFN;
    cmd_msg.cmd = IPMI_GET_DEVICE_ID_CMD;
    cmd_msg.data_len = 0;

    rv = ipmi_send_command(mc, 0, &cmd_msg, dev_id_rsp_handler, NULL);
    if (rv)
	goto close_and_quit;

    mc->bmc->state = QUERYING_DEVICE_ID;

 close_and_quit:
    if (rv) {
	ipmi_close_connection(mc, NULL, NULL);
	info->handler(NULL, rv, info->cb_data);
    }

    ipmi_unlock(mc->bmc_mc->bmc->mc_list_lock);

 out:
    ipmi_mem_free(info);
}

int
ipmi_init_bmc(ipmi_con_t  *con,
	      ipmi_bmc_cb handler,
	      void        *cb_data)
{
    int             rv;
    init_con_info_t *info;

    info = ipmi_mem_alloc(sizeof(*info));
    if (!info)
	return ENOMEM;

    info->handler = handler;
    info->cb_data = cb_data;
    rv = con->start_con(con, ipmi_init_con, info);
    if (rv)
	ipmi_mem_free(info);
    return rv;
}

int
ipmi_detect_bmc_presence_changes(ipmi_mc_t *mc, int force)
{
    int rv;
    CHECK_MC_LOCK(mc);
    
    ipmi_mc_entity_lock(mc);
    rv = ipmi_detect_ents_presence_changes(mc->bmc_mc->bmc->entities, force);
    ipmi_mc_entity_unlock(mc);
    return rv;
}

os_handler_t *
ipmi_mc_get_os_hnd(ipmi_mc_t *mc)
{
    if (mc->bmc_mc->bmc->mc_list_lock)
	CHECK_MC_LOCK(mc);
    return mc->bmc_mc->bmc->conn->os_hnd;
}

ipmi_entity_info_t *
ipmi_mc_get_entities(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    return mc->bmc_mc->bmc->entities;
}

void
ipmi_mc_entity_lock(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    ipmi_lock(mc->bmc_mc->bmc->entities_lock);
}

void
ipmi_mc_entity_unlock(ipmi_mc_t *mc)
{
    CHECK_MC_LOCK(mc);
    ipmi_unlock(mc->bmc_mc->bmc->entities_lock);
}

void
ipmi_mc_get_sdr_sensors(ipmi_mc_t     *bmc,
			ipmi_mc_t     *mc,
			ipmi_sensor_t ***sensors,
			unsigned int  *count)
{
    if (mc) {
	CHECK_MC_LOCK(mc);
	*sensors = mc->sensors_in_my_sdr;
	*count = mc->sensors_in_my_sdr_count;
    } else {
	CHECK_MC_LOCK(bmc);
	*sensors = bmc->bmc->sensors_in_my_sdr;
	*count = bmc->bmc->sensors_in_my_sdr_count;
    }
}

void
ipmi_mc_set_sdr_sensors(ipmi_mc_t     *bmc,
			ipmi_mc_t     *mc,
			ipmi_sensor_t **sensors,
			unsigned int  count)
{
    if (mc) {
	CHECK_MC_LOCK(mc);
	mc->sensors_in_my_sdr = sensors;
	mc->sensors_in_my_sdr_count = count;
    } else {
	CHECK_MC_LOCK(bmc);
	bmc->bmc->sensors_in_my_sdr = sensors;
	bmc->bmc->sensors_in_my_sdr_count = count;
    }
}

int
ipmi_bmc_set_entity_update_handler(ipmi_mc_t          *bmc,
				   ipmi_bmc_entity_cb handler,
				   void               *cb_data)
{
    CHECK_MC_LOCK(bmc);

    /* Make sure it's an SMI mc. */
    if (bmc->bmc_mc != bmc)
	return EINVAL;

    return ipmi_entity_set_update_handler(bmc->bmc->entities,
					  handler,
					  cb_data);
}

int
ipmi_bmc_iterate_entities(ipmi_mc_t                       *bmc,
			  ipmi_entities_iterate_entity_cb handler,
			  void                            *cb_data)
{
    CHECK_MC_LOCK(bmc);
    ipmi_mc_entity_lock(bmc);
    ipmi_entities_iterate_entities(bmc->bmc->entities, handler, cb_data);
    ipmi_mc_entity_unlock(bmc);
    return 0;
}

typedef struct iterate_mc_info_s
{
    ipmi_mc_t               *bmc;
    ipmi_bmc_iterate_mcs_cb handler;
    void                    *cb_data;
} iterate_mc_info_t;

static void
iterate_mcs_handler(ilist_iter_t *iter, void *item, void *cb_data)
{
    iterate_mc_info_t *info = cb_data;
    info->handler(info->bmc, item, info->cb_data);
}

int
ipmi_bmc_iterate_mcs(ipmi_mc_t               *bmc,
		     ipmi_bmc_iterate_mcs_cb handler,
		     void                    *cb_data)
{
    iterate_mc_info_t info = { bmc, handler, cb_data };

    if (bmc->bmc == NULL)
	/* Not a BMC */
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    ipmi_lock(bmc->bmc->mc_list_lock);
    ilist_iter(bmc->bmc->mc_list, iterate_mcs_handler, &info);
    ipmi_unlock(bmc->bmc->mc_list_lock);
    return 0;
}

static int
ipmi_bmc_iterate_mcs_rev(ipmi_mc_t               *bmc,
			 ipmi_bmc_iterate_mcs_cb handler,
			 void                    *cb_data)
{
    iterate_mc_info_t info = { bmc, handler, cb_data };

    if (bmc->bmc == NULL)
	/* Not a BMC */
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    ipmi_lock(bmc->bmc->mc_list_lock);
    ilist_iter_rev(bmc->bmc->mc_list, iterate_mcs_handler, &info);
    ipmi_unlock(bmc->bmc->mc_list_lock);
    return 0;
}

ipmi_mc_id_t
ipmi_mc_convert_to_id(ipmi_mc_t *mc)
{
    ipmi_mc_id_t val;

    CHECK_MC_LOCK(mc);

    val.bmc = mc->bmc_mc;
    val.channel = mc->addr.channel;
    val.seq = mc->seq;
    if (mc->addr.addr_type == IPMI_SYSTEM_INTERFACE_ADDR_TYPE) {
	/* The BMC address is always zero. */
	val.mc_num = 0;
    } else {
	ipmi_ipmb_addr_t *ipmb = (ipmi_ipmb_addr_t *) &(mc->addr);
	val.mc_num = ipmb->slave_addr;
    }
    return val;
}

int
ipmi_mc_pointer_cb(ipmi_mc_id_t id, ipmi_mc_cb handler, void *cb_data)
{
    int       rv;
    ipmi_mc_t *mc;

    ipmi_read_lock();
    rv = ipmi_bmc_validate(id.bmc);
    if (rv)
	goto out_unlock;
    ipmi_lock(id.bmc->bmc->mc_list_lock);
    if (id.mc_num == 0) {
	mc = id.bmc;
    } else {
	ipmi_ipmb_addr_t ipmb = {IPMI_IPMB_ADDR_TYPE, id.channel,
				 id.mc_num, 0};

	mc = find_mc_by_addr(id.bmc, (ipmi_addr_t *) &ipmb, sizeof(ipmb));
	if (!mc)
	    rv = EINVAL;
    }
    if (!rv) {
	if (mc->seq != id.seq)
	    rv = EINVAL;
    }
    if (!rv)
	handler(mc, cb_data);
    ipmi_unlock(id.bmc->bmc->mc_list_lock);
 out_unlock:
    ipmi_read_unlock();

    return rv;
}

int
ipmi_mc_pointer_noseq_cb(ipmi_mc_id_t id, ipmi_mc_cb handler, void *cb_data)
{
    int       rv;
    ipmi_mc_t *mc;

    ipmi_read_lock();
    rv = ipmi_bmc_validate(id.bmc);
    if (rv)
	goto out_unlock;
    ipmi_lock(id.bmc->bmc->mc_list_lock);
    if (id.mc_num == 0) {
	mc = id.bmc;
    } else {
	ipmi_ipmb_addr_t ipmb = {IPMI_IPMB_ADDR_TYPE, id.channel,
				 id.mc_num, 0};

	mc = find_mc_by_addr(id.bmc, (ipmi_addr_t *) &ipmb, sizeof(ipmb));
	if (!mc)
	    rv = EINVAL;
    }
    if (!rv)
	handler(mc, cb_data);
    ipmi_unlock(id.bmc->bmc->mc_list_lock);
 out_unlock:
    ipmi_read_unlock();

    return rv;
}

int
ipmi_cmp_mc_id(ipmi_mc_id_t id1, ipmi_mc_id_t id2)
{
    if (id1.bmc > id2.bmc)
	return 1;
    if (id1.bmc < id2.bmc)
	return -1;
    if (id1.mc_num > id2.mc_num)
	return 1;
    if (id1.mc_num < id2.mc_num)
	return -1;
    if (id1.channel > id2.channel)
	return 1;
    if (id1.channel < id2.channel)
	return -1;
    if (id1.seq > id2.seq)
	return 1;
    if (id1.seq < id2.seq)
	return -1;
    return 0;
}

typedef struct sdrs_saved_info_s
{
    ipmi_mc_t   *bmc;
    ipmi_bmc_cb done;
    void        *cb_data;
} sdrs_saved_info_t;

static void
sdrs_saved(ipmi_sdr_info_t *sdrs, int err, void *cb_data)
{
    sdrs_saved_info_t *info = cb_data;

    info->done(info->bmc, err, info->cb_data);
    ipmi_mem_free(info);
}

int
ipmi_bmc_store_entities(ipmi_mc_t *bmc, ipmi_bmc_cb done, void *cb_data)
{
    int               rv;
    ipmi_sdr_info_t   *stored_sdrs;
    sdrs_saved_info_t *info;

    /* Make sure it's the BMC. */
    if (bmc->bmc_mc != bmc)
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    info = ipmi_mem_alloc(sizeof(*info));
    if (!info)
	return ENOMEM;

    /* Create an SDR repository to store. */
    rv = ipmi_sdr_info_alloc(bmc, 0, 0, &stored_sdrs);
    if (rv) {
	ipmi_mem_free(info);
	return rv;
    }

    /* Now store a channel SDR if we are less than 1.5. */
    if ((bmc->major_version <= 1) && (bmc->minor_version < 5)) {
	ipmi_sdr_t sdr;
	int        i;
	
	sdr.major_version = bmc->major_version;
	sdr.minor_version = bmc->minor_version;
	sdr.type = 0x14; /*  */
	sdr.length = 11;
	for (i=0; i<8; i++) {
	    /* FIXME - what about the LUN and transmit support? */
	    if (bmc->bmc->chan[i].protocol) {
		sdr.data[i] = (bmc->bmc->chan[i].protocol
			       | (bmc->bmc->chan[i].xmit_support << 7)
			       | (bmc->bmc->chan[i].recv_lun << 4));
	    } else {
		sdr.data[i] = 0;
	    }
	}
	sdr.data[8] = bmc->bmc->msg_int_type;
	sdr.data[9] = bmc->bmc->event_msg_int_type;
	sdr.data[10] = 0;

	rv = ipmi_sdr_add(stored_sdrs, &sdr);
	if (rv)
	    goto out_err;
    }

    rv = ipmi_entity_append_to_sdrs(bmc->bmc->entities, stored_sdrs);
    if (rv)
	goto out_err;

    info->bmc = bmc;
    info->done = done;
    info->cb_data = cb_data;
    rv = ipmi_sdr_save(stored_sdrs, sdrs_saved, info);

 out_err:
    if (rv)
	ipmi_mem_free(info);
    ipmi_sdr_info_destroy(stored_sdrs, NULL, NULL);
    return rv;
}

void
ipmi_bmc_oem_new_entity(ipmi_mc_t *bmc, ipmi_entity_t *ent)
{
    CHECK_MC_LOCK(bmc);

    ipmi_entity_lock(ent);
    if (bmc->bmc->new_entity_handler)
	bmc->bmc->new_entity_handler(bmc, ent,
				     bmc->bmc->new_entity_cb_data);
    ipmi_entity_unlock(ent);
}

int
ipmi_bmc_set_oem_new_entity_handler(ipmi_mc_t                  *bmc,
				    ipmi_bmc_oem_new_entity_cb handler,
				    void                       *cb_data)
{
    /* Make sure it's an SMI mc. */
    if (bmc->bmc_mc != bmc)
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    bmc->bmc->new_entity_handler = handler;
    bmc->bmc->new_entity_cb_data = cb_data;
    return 0;
}

int
ipmi_bmc_set_oem_new_mc_handler(ipmi_mc_t              *bmc,
				ipmi_bmc_oem_new_mc_cb handler,
				void                   *cb_data)
{
    /* Make sure it's an SMI mc. */
    if (bmc->bmc_mc != bmc)
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    bmc->bmc->new_mc_handler = handler;
    bmc->bmc->new_mc_cb_data = cb_data;
    return 0;
}

int
ipmi_bmc_set_oem_setup_finished_handler(ipmi_mc_t                  *bmc,
					ipmi_oem_setup_finished_cb handler,
					void                       *cb_data)
{
    /* Make sure it's an SMI mc. */
    if (bmc->bmc_mc != bmc)
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    bmc->bmc->setup_finished_handler = handler;
    bmc->bmc->setup_finished_cb_data = cb_data;
    return 0;
}

int
ipmi_bmc_set_full_bus_scan(ipmi_mc_t *bmc, int val)
{
    /* Make sure it's an SMI mc. */
    if (bmc->bmc_mc != bmc)
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    bmc->bmc->do_bus_scan = val;
    return 0;
}

typedef struct sel_op_done_info_s
{
    ipmi_mc_t   *mc;
    ipmi_bmc_cb done;
    void        *cb_data;
} sel_op_done_info_t;

static void
sel_op_done(ipmi_sel_info_t *sel,
	    void            *cb_data,
	    int             err)
{
    sel_op_done_info_t *info = cb_data;

    /* No need to lock, the BMC should already be locked. */
    if (info->done)
        info->done(info->mc->bmc_mc, err, info->cb_data);
    ipmi_mem_free(info);
}

typedef struct del_event_info_s
{
    ipmi_event_t *event;
    ipmi_bmc_cb  done_handler;
    void         *cb_data;
    int          rv;
} del_event_info_t;

static void
del_event_handler(ipmi_mc_t *mc, void *cb_data)
{
    del_event_info_t   *info = cb_data;
    sel_op_done_info_t *sel_info;

    if (!mc->SEL_device_support) {
	info->rv = EINVAL;
	return;
    }

    /* If we have an OEM handler, call it instead. */
    if (mc->sel_del_event_handler) {
	info->rv = mc->sel_del_event_handler(mc,
					     info->event,
					     info->done_handler,
					     info->cb_data);
	return;
    }

    sel_info = ipmi_mem_alloc(sizeof(*sel_info));
    if (!sel_info) {
	info->rv = ENOMEM;
	return;
    }

    sel_info->mc = mc;
    sel_info->done = info->done_handler;
    sel_info->cb_data = cb_data;

    info->rv = ipmi_sel_del_event(mc->sel, info->event, sel_op_done, sel_info);
    if (info->rv)
	ipmi_mem_free(sel_info);
}

int
ipmi_bmc_del_event(ipmi_mc_t    *bmc,
		   ipmi_event_t *event,
		   ipmi_bmc_cb  done_handler,
		   void         *cb_data)
{
    int              rv;
    del_event_info_t info;

    CHECK_MC_LOCK(bmc);

    info.event = event;
    info.done_handler = done_handler;
    info.cb_data = cb_data;
    info.rv = 0;
    rv = ipmi_mc_pointer_cb(event->mc_id, del_event_handler, &info);
    if (rv)
	return rv;
    else
	return info.rv;
}

typedef struct next_event_handler_info_s
{
    int          rv;
    ipmi_event_t *event;
    int          found_curr_mc;
    int          do_prev; /* If going backwards, this will be 1. */
} next_event_handler_info_t;

static void
next_event_handler(ipmi_mc_t *bmc, ipmi_mc_t *mc, void *cb_data)
{
    next_event_handler_info_t *info = cb_data;
    ipmi_mc_id_t              mc_id = ipmi_mc_convert_to_id(mc);

    if (!info->rv)
	/* We've found an event already, just return. */
	return;

    if (info->do_prev) {
	if (info->found_curr_mc)
	    /* We've found the MC that had the event, but it didn't have
	       any more events.  Look for last events now. */
	    info->rv = ipmi_sel_get_last_event(mc->sel, info->event);
	else if (ipmi_cmp_mc_id(info->event->mc_id, mc_id) == 0) {
	    info->found_curr_mc = 1;
	    info->rv = ipmi_sel_get_prev_event(mc->sel, info->event);
	}
    } else {
	if (info->found_curr_mc)
	    /* We've found the MC that had the event, but it didn't have
	       any more events.  Look for first events now. */
	    info->rv = ipmi_sel_get_first_event(mc->sel, info->event);
	else if (ipmi_cmp_mc_id(info->event->mc_id, mc_id) == 0) {
	    info->found_curr_mc = 1;
	    info->rv = ipmi_sel_get_next_event(mc->sel, info->event);
	}
    }
}

int
ipmi_bmc_first_event(ipmi_mc_t *bmc, ipmi_event_t *event)
{
    int                       rv;
    next_event_handler_info_t info;

    if (!bmc->bmc)
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    rv = ipmi_sel_get_first_event(bmc->sel, event);
    if (rv) {
	info.rv = ENODEV;
	info.event = event;
	info.found_curr_mc = 1;
	info.do_prev = 0;
	rv = ipmi_bmc_iterate_mcs(bmc, next_event_handler, &info);
	if (!rv)
	    rv = info.rv;
    }

    return rv;
}

int
ipmi_bmc_last_event(ipmi_mc_t *bmc, ipmi_event_t *event)
{
    int                       rv;
    next_event_handler_info_t info;

    if (!bmc->bmc)
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    info.rv = ENODEV;
    info.event = event;
    info.found_curr_mc = 1;
    info.do_prev = 1;
    rv = ipmi_bmc_iterate_mcs(bmc, next_event_handler, &info);
    if (!rv)
	rv = info.rv;
    if (rv)
	rv = ipmi_sel_get_last_event(bmc->sel, event);

    return rv;
}

int
ipmi_bmc_next_event(ipmi_mc_t *bmc, ipmi_event_t *event)
{
    int                       rv;
    next_event_handler_info_t info;
    ipmi_mc_id_t              mc_id = ipmi_mc_convert_to_id(bmc);

    if (!bmc->bmc)
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    rv = ENODEV;
    if (ipmi_cmp_mc_id(event->mc_id, mc_id) == 0)
	/* If the event is from the BMC, try the next event in the BMC. */
	rv = ipmi_sel_get_next_event(bmc->sel, event);
    if (rv) {
	info.rv = ENODEV;
	info.event = event;
	info.found_curr_mc = 1;
	info.do_prev = 0;
	rv = ipmi_bmc_iterate_mcs(bmc, next_event_handler, &info);
	if (!rv)
	    rv = info.rv;
    }

    return rv;
}

int
ipmi_bmc_prev_event(ipmi_mc_t *bmc, ipmi_event_t *event)
{
    int                       rv;
    next_event_handler_info_t info;
    ipmi_mc_id_t              mc_id = ipmi_mc_convert_to_id(bmc);

    if (!bmc->bmc)
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    rv = ENODEV;
    if (ipmi_cmp_mc_id(event->mc_id, mc_id) == 0) {
	/* If the event is from the BMC, try the prev event in the BMC. */
	rv = ipmi_sel_get_prev_event(bmc->sel, event);
    } else {
	info.rv = ENODEV;
	info.event = event;
	info.found_curr_mc = 1;
	info.do_prev = 1;
	rv = ipmi_bmc_iterate_mcs_rev(bmc, next_event_handler, &info);
	if (!rv)
	    rv = info.rv;
	if (rv)
	    /* If we weren't on the bmc SEL but didn't find anything
               else, then we try the last on in the BMC sel. */
	    rv = ipmi_sel_get_last_event(bmc->sel, event);
    }

    return rv;
}

static void
sel_count_handler(ipmi_mc_t *bmc, ipmi_mc_t *mc, void *cb_data)
{
    int *count = cb_data;
    int nc = 0;

    ipmi_get_sel_count(mc->sel, &nc);
    *count += nc;
}

int
ipmi_bmc_sel_count(ipmi_mc_t    *bmc,
		   unsigned int *count)
{
    if (!bmc->bmc)
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    *count = 0;
    ipmi_get_sel_count(bmc->sel, count);
    ipmi_bmc_iterate_mcs(bmc, sel_count_handler, count);
    return 0;
}

static void
sel_entries_used_handler(ipmi_mc_t *bmc, ipmi_mc_t *mc, void *cb_data)
{
    int *count = cb_data;
    int nc = 0;

    ipmi_get_sel_entries_used(mc->sel, &nc);
    *count += nc;
}

int ipmi_bmc_sel_entries_used(ipmi_mc_t    *bmc,
			      unsigned int *count)
{
    if (!bmc->bmc)
	return EINVAL;

    CHECK_MC_LOCK(bmc);

    *count = 0;
    ipmi_get_sel_entries_used(bmc->sel, count);
    ipmi_bmc_iterate_mcs(bmc, sel_entries_used_handler, count);
    return 0;
}

#ifdef IPMI_CHECK_LOCKS
void
__ipmi_check_mc_lock(ipmi_mc_t *mc)
{
    ipmi_check_lock(mc->bmc_mc->bmc->mc_list_lock,
		    "MC not locked when it should have been");
}

void
__ipmi_check_mc_entity_lock(ipmi_mc_t *mc)
{
    ipmi_check_lock(mc->bmc_mc->bmc->entities_lock,
		    "Entity not locked when it should have been");
}
#endif

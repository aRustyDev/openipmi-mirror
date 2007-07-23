/*
 * conn.c
 *
 * MontaVista IPMI support for connections
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

#include <errno.h>

#include <OpenIPMI/ipmi_conn.h>

#include <OpenIPMI/internal/locked_list.h>
#include <OpenIPMI/internal/ipmi_oem.h>
#include <OpenIPMI/internal/ipmi_int.h>

/***********************************************************************
 *
 * Handle global OEM callbacks for new MCs.
 *
 **********************************************************************/

typedef struct oem_conn_handlers_s {
    unsigned int             manufacturer_id;
    unsigned int             product_id;
    ipmi_oem_conn_handler_cb handler;
    void                     *cb_data;
} oem_conn_handlers_t;

ipmi_lock_t *oem_conn_handlers_lock = NULL;
static locked_list_t *oem_conn_handlers = NULL;

static int
oem_conn_handler_clean(void *cb_data, void *data1, void *data2)
{
    locked_list_remove(oem_conn_handlers, data1, data2);
    ipmi_mem_free(data1);
    return LOCKED_LIST_ITER_CONTINUE;
}

static void
cleanup_oem_conn_handlers(void)
{
    ipmi_lock(oem_conn_handlers_lock);
    locked_list_iterate(oem_conn_handlers, oem_conn_handler_clean, NULL);
    ipmi_unlock(oem_conn_handlers_lock);
}

int
ipmi_register_oem_conn_handler(unsigned int             manufacturer_id,
			       unsigned int             product_id,
			       ipmi_oem_conn_handler_cb handler,
			       void                     *cb_data)
{
    oem_conn_handlers_t *new_item;
    int                 rv;

    /* This might be called before initialization, so be 100% sure.. */
    rv = _ipmi_conn_init(ipmi_get_global_os_handler());
    if (rv)
	return rv;

    new_item = ipmi_mem_alloc(sizeof(*new_item));
    if (!new_item)
	return ENOMEM;

    new_item->manufacturer_id = manufacturer_id;
    new_item->product_id = product_id;
    new_item->handler = handler;
    new_item->cb_data = cb_data;

    if (locked_list_add(oem_conn_handlers, new_item, NULL))
	return 0;
    else {
	ipmi_mem_free(new_item);
	return ENOMEM;
    }

    return 0;
}

static int
oem_conn_handler_rm(void *cb_data, void *data1, void *data2)
{
    oem_conn_handlers_t *hndlr = data1;
    oem_conn_handlers_t *cmp = cb_data;

    if ((hndlr->manufacturer_id == cmp->manufacturer_id)
	&& (hndlr->product_id == cmp->product_id))
    {
	int *found = cmp->cb_data;

	*found = 1;
	locked_list_remove(oem_conn_handlers, data1, data2);
	ipmi_mem_free(data1);
	return LOCKED_LIST_ITER_STOP;
    } else 
	return LOCKED_LIST_ITER_CONTINUE;
}

int
ipmi_deregister_oem_conn_handler(unsigned int manufacturer_id,
				 unsigned int product_id)
{
    oem_conn_handlers_t tmp;
    int                 found = 0;

    tmp.manufacturer_id = manufacturer_id;
    tmp.product_id = product_id;
    tmp.cb_data = &found;
    ipmi_lock(oem_conn_handlers_lock);
    locked_list_iterate(oem_conn_handlers, oem_conn_handler_rm, &tmp);
    ipmi_unlock(oem_conn_handlers_lock);

    if (!found)
	return ENOENT;
    return 0;
}

static int
oem_conn_handler_cmp(void *cb_data, void *data1, void *data2)
{
    oem_conn_handlers_t      *hndlr = data1;
    oem_conn_handlers_t      *cmp = cb_data;
    ipmi_oem_conn_handler_cb handler;
    void                     *rcb_data;
    ipmi_con_t               *conn;
    int                      rv = EINVAL;

    if ((hndlr->manufacturer_id == cmp->manufacturer_id)
	&& (hndlr->product_id == cmp->product_id))
    {
	handler = hndlr->handler;
	rcb_data = hndlr->cb_data;
	conn = cmp->cb_data;
	ipmi_unlock(oem_conn_handlers_lock);
	rv = handler(conn, rcb_data);
	ipmi_lock(oem_conn_handlers_lock);
    }

    if (!rv)
	return LOCKED_LIST_ITER_STOP;
    else 
	return LOCKED_LIST_ITER_CONTINUE;
}

int
ipmi_check_oem_conn_handlers(ipmi_con_t   *conn,
			     unsigned int manufacturer_id,
			     unsigned int product_id)
{
    oem_conn_handlers_t tmp;

    tmp.manufacturer_id = manufacturer_id;
    tmp.product_id = product_id;
    tmp.cb_data = conn;
    ipmi_lock(oem_conn_handlers_lock);
    locked_list_iterate(oem_conn_handlers, oem_conn_handler_cmp, &tmp);
    ipmi_unlock(oem_conn_handlers_lock);
    return 0;
}

/***********************************************************************
 *
 * Handle global OEM callbacks new connections.
 *
 **********************************************************************/

static locked_list_t *oem_handlers;

int
ipmi_register_conn_oem_check(ipmi_conn_oem_check check,
			     void                *cb_data)
{
    if (locked_list_add(oem_handlers, check, cb_data))
	return 0;
    else
	return ENOMEM;
}

int
ipmi_deregister_conn_oem_check(ipmi_conn_oem_check check,
			       void                *cb_data)
{
    if (locked_list_remove(oem_handlers, check, cb_data))
	return 0;
    else
	return EINVAL;
}

typedef struct conn_check_oem_s
{
    ipmi_con_t               *conn;
    volatile unsigned int    count;
    ipmi_lock_t              *lock;
    ipmi_conn_oem_check_done done;
    void                     *cb_data;
} conn_check_oem_t;

static void
conn_oem_check_done(ipmi_con_t *conn,
		    void       *cb_data)
{
    conn_check_oem_t *check = cb_data;
    int              done = 0;

    ipmi_lock(check->lock);
    check->count--;
    if (check->count == 0)
	done = 1;
    ipmi_unlock(check->lock);

    if (done) {
	ipmi_destroy_lock(check->lock);
	check->done(conn, check->cb_data);
	ipmi_mem_free(check);
    }
}

static int
conn_handler_call(void *cb_data, void *ihandler, void *data2)
{
    conn_check_oem_t    *check = cb_data;
    ipmi_conn_oem_check check_cb = ihandler;
    int                 rv;

    ipmi_lock(check->lock);
    check->count++;
    ipmi_unlock(check->lock);
    rv = check_cb(check->conn, data2, conn_oem_check_done, check);
    if (rv) {
	ipmi_lock(check->lock);
	check->count--;
	ipmi_unlock(check->lock);
    }
    return LOCKED_LIST_ITER_CONTINUE;
}

int
ipmi_conn_check_oem_handlers(ipmi_con_t               *conn,
			     ipmi_conn_oem_check_done done,
			     void                     *cb_data)
{
    conn_check_oem_t *check;
    int              rv;
    unsigned int     count = 0;

    check = ipmi_mem_alloc(sizeof(*check));
    if (!check)
	return ENOMEM;

    rv = ipmi_create_lock_os_hnd(conn->os_hnd, &check->lock);
    if (rv)
	return rv;
    check->count = 1;
    check->conn = conn;
    check->done = done;
    check->cb_data = cb_data;

    locked_list_iterate(oem_handlers, conn_handler_call, check);

    ipmi_lock(check->lock);
    count = check->count;
    ipmi_unlock(check->lock);

    /* Say that this function is done with the check. */
    conn_oem_check_done(conn, check);

    return 0;
}

/***********************************************************************
 *
 * Init/shutdown
 *
 **********************************************************************/
int
_ipmi_conn_init(os_handler_t *os_hnd)
{
    int rv;

    if (!oem_conn_handlers_lock) {
	rv = ipmi_create_global_lock(&oem_conn_handlers_lock);
	if (rv)
	    return rv;
    }

    if (!oem_conn_handlers) {
	oem_conn_handlers = locked_list_alloc(os_hnd);
	if (!oem_conn_handlers)
	    return ENOMEM;
    }
    if (!oem_handlers) {
	oem_handlers = locked_list_alloc(os_hnd);
	if (!oem_handlers)
	    return ENOMEM;
    }
    return 0;
}

void
_ipmi_conn_shutdown(void)
{
    if (oem_conn_handlers) {
	cleanup_oem_conn_handlers();
	locked_list_destroy(oem_conn_handlers);
	oem_conn_handlers = NULL;
    }

    if (oem_handlers) {
	locked_list_destroy(oem_handlers);
	oem_handlers = NULL;
    }

    if (oem_conn_handlers_lock) {
	ipmi_destroy_lock(oem_conn_handlers_lock);
	oem_conn_handlers_lock = NULL;
    }
}

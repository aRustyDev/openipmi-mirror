/*
 * posix_os.c
 *
 * POSIX OS-handlers for OpenIPMI
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

#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <OpenIPMI/os_handler.h>
#include <OpenIPMI/selector.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include <OpenIPMI/ipmi_int.h>
#include <OpenIPMI/ipmi_posix.h>

struct os_hnd_fd_id_s
{
    int             fd;
    void            *cb_data;
    os_data_ready_t data_ready;
    os_handler_t    *handler;
    os_fd_data_freed_t freed;
};

static void
fd_handler(int fd, void *data)
{
    os_hnd_fd_id_t *fd_data = (os_hnd_fd_id_t *) data;
    void            *cb_data;
    os_handler_t    *handler;

    handler = fd_data->handler;
    cb_data = fd_data->cb_data;
    fd_data->data_ready(fd, fd_data->cb_data, fd_data);
}

static void
free_fd_data(int fd, void *data)
{
    os_hnd_fd_id_t *fd_data = data;

    if (fd_data->freed)
        fd_data->freed(fd, fd_data->cb_data);
    ipmi_mem_free(data);
}

static int
add_fd(os_handler_t       *handler,
       int                fd,
       os_data_ready_t    data_ready,
       void               *cb_data,
       os_fd_data_freed_t freed,
       os_hnd_fd_id_t     **id)
{
    os_hnd_fd_id_t *fd_data;
    int            rv;
    selector_t     *posix_sel = handler->internal_data;


    fd_data = ipmi_mem_alloc(sizeof(*fd_data));
    if (!fd_data)
	return ENOMEM;

    fd_data->fd = fd;
    fd_data->cb_data = cb_data;
    fd_data->data_ready = data_ready;
    fd_data->handler = handler;
    fd_data->freed = freed;
    sel_set_fd_write_handler(posix_sel, fd, SEL_FD_HANDLER_DISABLED);
    sel_set_fd_except_handler(posix_sel, fd, SEL_FD_HANDLER_DISABLED);
    rv = sel_set_fd_handlers(posix_sel, fd, fd_data, fd_handler, NULL, NULL,
			     free_fd_data);
    if (rv) {
	ipmi_mem_free(fd_data);
	return rv;
    }
    sel_set_fd_read_handler(posix_sel, fd, SEL_FD_HANDLER_ENABLED);

    *id = fd_data;
    return 0;
}

static int
remove_fd(os_handler_t *handler, os_hnd_fd_id_t *fd_data)
{
    selector_t *posix_sel = handler->internal_data;

    sel_set_fd_read_handler(posix_sel, fd_data->fd, SEL_FD_HANDLER_DISABLED);
    sel_clear_fd_handlers(posix_sel, fd_data->fd);
    /* fd_data gets freed in the free_fd_data callback registered at
       set time. */
    return 0;
}

struct os_hnd_timer_id_s
{
    void           *cb_data;
    os_timed_out_t timed_out;
    sel_timer_t    *timer;
    int            running;
    os_handler_t   *handler;
};

static void
timer_handler(selector_t  *sel,
	      sel_timer_t *timer,
	      void        *data)
{
    os_hnd_timer_id_t *timer_data = (os_hnd_timer_id_t *) data;
    /* Make a copy of this, because the handler may delete the timer
       data. */
    void              *cb_data;
    os_timed_out_t    timed_out;

    timed_out = timer_data->timed_out;
    cb_data = timer_data->cb_data;
    timer_data->running = 0;
    timed_out(cb_data, timer_data);
}

static int
start_timer(os_handler_t      *handler, 
	    os_hnd_timer_id_t *id,
	    struct timeval    *timeout,
	    os_timed_out_t    timed_out,
	    void              *cb_data)
{
    struct timeval    now;

    if (id->running)
	return EBUSY;

    id->running = 1;
    id->cb_data = cb_data;
    id->timed_out = timed_out;

    gettimeofday(&now, NULL);
    now.tv_sec += timeout->tv_sec;
    now.tv_usec += timeout->tv_usec;
    while (now.tv_usec >= 1000000) {
	now.tv_usec -= 1000000;
	now.tv_sec += 1;
    }

    return sel_start_timer(id->timer, &now);
}

static int
stop_timer(os_handler_t *handler, os_hnd_timer_id_t *timer_data)
{
    return sel_stop_timer(timer_data->timer);
}

static int
alloc_timer(os_handler_t      *handler, 
	    os_hnd_timer_id_t **id)
{
    os_hnd_timer_id_t *timer_data;
    int               rv;
    selector_t        *posix_sel = handler->internal_data;

    timer_data = ipmi_mem_alloc(sizeof(*timer_data));
    if (!timer_data)
	return ENOMEM;

    timer_data->running = 0;
    timer_data->timed_out = NULL;
    timer_data->handler = handler;

    rv = sel_alloc_timer(posix_sel, timer_handler, timer_data,
			 &(timer_data->timer));
    if (rv) {
	ipmi_mem_free(timer_data);
	return rv;
    }

    *id = timer_data;
    return 0;
}

static int
free_timer(os_handler_t *handler, os_hnd_timer_id_t *timer_data)
{
    sel_free_timer(timer_data->timer);
    ipmi_mem_free(timer_data);
    return 0;
}

static int
get_random(os_handler_t *handler, void *data, unsigned int len)
{
    int fd = open("/dev/urandom", O_RDONLY);
    int rv;

    if (fd == -1)
	return errno;

    rv = read(fd, data, len);

    close(fd);
    return rv;
}

static void
sposix_log(os_handler_t         *handler,
	   enum ipmi_log_type_e log_type,
	   char                 *format,
	   ...)
{
    va_list ap;

    va_start(ap, format);
    posix_vlog(format, log_type, ap);
    va_end(ap);
}

static void
sposix_vlog(os_handler_t         *handler,
	    enum ipmi_log_type_e log_type,
	    char                 *format,
	    va_list              ap)
{
    posix_vlog(format, log_type, ap);
}

static os_handler_t ipmi_posix_os_handler =
{
    .add_fd_to_wait_for = add_fd,
    .remove_fd_to_wait_for = remove_fd,
    .start_timer = start_timer,
    .stop_timer = stop_timer,
    .alloc_timer = alloc_timer,
    .free_timer = free_timer,
    .get_random = get_random,
    .log = sposix_log,
    .vlog = sposix_vlog,
};

os_handler_t *
ipmi_posix_get_os_handler(void)
{
    os_handler_t *rv;

    rv = ipmi_mem_alloc(sizeof(*rv));
    if (!rv)
	return NULL;

    memcpy(rv, &ipmi_posix_os_handler, sizeof(*rv));
    return rv;
}

void
ipmi_posix_os_handler_set_sel(os_handler_t *os_hnd, selector_t *sel)
{
    os_hnd->internal_data = sel;
}

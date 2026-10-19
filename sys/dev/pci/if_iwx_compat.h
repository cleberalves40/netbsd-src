/*	$NetBSD$	*/

/*
 * Porting layer for the OpenBSD iwx(4) driver.
 *
 * The driver uses a few OpenBSD kernel primitives that NetBSD does not
 * have under the same names: timeouts, task queues, reference counts and
 * mbuf lists.  Each is mapped onto the closest NetBSD facility here so the
 * driver body changes as little as possible.  Nothing in this file is
 * visible outside the iwx driver.
 */

#ifndef _DEV_PCI_IF_IWX_COMPAT_H_
#define _DEV_PCI_IF_IWX_COMPAT_H_

#include <sys/callout.h>
#include <sys/workqueue.h>
#include <sys/kmem.h>
#include <sys/atomic.h>
#include <sys/time.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/systm.h>
#include <sys/param.h>
#include <machine/limits.h>
#include <net/if.h>

/*
 * timeout(9) on top of callout(9).  Times are in ticks of the given
 * unit; each call converts to ticks with mstohz().
 */
struct timeout {
	callout_t	to_c;
};

static inline void
timeout_set(struct timeout *to, void (*fn)(void *), void *arg)
{
	callout_init(&to->to_c, CALLOUT_MPSAFE);
	callout_setfunc(&to->to_c, fn, arg);
}

static inline void
timeout_add_msec(struct timeout *to, int ms)
{
	callout_schedule(&to->to_c, mstohz(ms));
}

static inline void
timeout_add_usec(struct timeout *to, int us)
{
	callout_schedule(&to->to_c, mstohz((us + 999) / 1000));
}

static inline void
timeout_add_sec(struct timeout *to, int sec)
{
	callout_schedule(&to->to_c, mstohz(sec * 1000));
}

static inline int
timeout_del(struct timeout *to)
{
	return callout_stop(&to->to_c);
}

static inline int
timeout_pending(struct timeout *to)
{
	return callout_pending(&to->to_c);
}

/*
 * task(9) on top of workqueue(9).  A task is queued at most once; a
 * second task_add() while it is queued is a no-op, as in OpenBSD.
 */
struct taskq {
	struct workqueue	*tq_wq;
};

struct task {
	struct work		 t_wk;	/* must be first */
	void			(*t_fn)(void *);
	void			 *t_arg;
	volatile unsigned int	 t_pending;
};

static inline void
iwx_task_run(struct work *wk, void *arg __unused)
{
	struct task *t = (struct task *)wk;

	/* clear first so the handler may queue the task again */
	atomic_swap_uint(&t->t_pending, 0);
	(*t->t_fn)(t->t_arg);
}

static inline void
task_set(struct task *t, void (*fn)(void *), void *arg)
{
	t->t_fn = fn;
	t->t_arg = arg;
	t->t_pending = 0;
}

static inline int
task_add(struct taskq *tq, struct task *t)
{
	if (atomic_cas_uint(&t->t_pending, 0, 1) != 0)
		return 0;
	workqueue_enqueue(tq->tq_wq, &t->t_wk, NULL);
	return 1;
}

/*
 * NetBSD cannot cancel a queued work item.  task_del() therefore waits
 * for the item to finish, so the handler still runs once; OpenBSD would
 * have dropped it.  Callers only use this on detach and stop paths.
 */
static inline int
task_del(struct taskq *tq, struct task *t)
{
	if (t->t_pending == 0)
		return 0;
	workqueue_wait(tq->tq_wq, &t->t_wk);
	return 1;
}

static inline int
task_pending(struct task *t)
{
	return t->t_pending != 0;
}

static inline struct taskq *
taskq_create(const char *name, int nthreads, int ipl, int flags)
{
	struct taskq *tq;

	tq = kmem_zalloc(sizeof(*tq), KM_SLEEP);
	if (workqueue_create(&tq->tq_wq, name, iwx_task_run, NULL,
	    PRI_NONE, ipl, WQ_MPSAFE) != 0) {
		kmem_free(tq, sizeof(*tq));
		return NULL;
	}
	return tq;
}

/* 12-bit 802.11 sequence numbers: nonzero when a comes before b. */
#define SEQ_LT(a, b)	((((a) - (b)) & 0xfff) >= 2048)

/* refcnt(9) with polling finalize; the driver calls these from a few
 * detach paths only. */
struct refcnt {
	unsigned int	r_refs;
};

static inline void
refcnt_init(struct refcnt *r)
{
	r->r_refs = 1;
}

static inline void
refcnt_take(struct refcnt *r)
{
	atomic_inc_uint(&r->r_refs);
}

static inline int
refcnt_rele(struct refcnt *r)
{
	return atomic_dec_uint_nv(&r->r_refs) == 0;
}

#define refcnt_rele_wake(r)	((void)refcnt_rele(r))

static inline void
refcnt_finalize(struct refcnt *r, const char *wmesg)
{
	(void)refcnt_rele(r);
	while (r->r_refs != 0)
		kpause(wmesg, false, 1, NULL);
}

/*
 * mbuf_list: an mbuf chain linked through m_nextpkt, as in OpenBSD.
 */
struct mbuf_list {
	struct mbuf	*ml_head;
	struct mbuf	*ml_tail;
	unsigned int	 ml_len;
};

#define MBUF_LIST_INITIALIZER()	{ NULL, NULL, 0 }

static inline void
ml_init(struct mbuf_list *ml)
{
	ml->ml_head = ml->ml_tail = NULL;
	ml->ml_len = 0;
}

static inline void
ml_enqueue(struct mbuf_list *ml, struct mbuf *m)
{
	m->m_nextpkt = NULL;
	if (ml->ml_tail != NULL)
		ml->ml_tail->m_nextpkt = m;
	else
		ml->ml_head = m;
	ml->ml_tail = m;
	ml->ml_len++;
}

static inline struct mbuf *
ml_dequeue(struct mbuf_list *ml)
{
	struct mbuf *m = ml->ml_head;

	if (m != NULL) {
		ml->ml_head = m->m_nextpkt;
		if (ml->ml_head == NULL)
			ml->ml_tail = NULL;
		m->m_nextpkt = NULL;
		ml->ml_len--;
	}
	return m;
}

static inline int
ml_empty(const struct mbuf_list *ml)
{
	return ml->ml_len == 0;
}

static inline void
ml_purge(struct mbuf_list *ml)
{
	struct mbuf *m;

	while ((m = ml_dequeue(ml)) != NULL)
		m_freem(m);
}

/* Hand a batch of received frames to the stack. */
static inline void
if_input_ml(struct ifnet *ifp, struct mbuf_list *ml)
{
	struct mbuf *m;

	while ((m = ml_dequeue(ml)) != NULL)
		if_percpuq_enqueue(ifp->if_percpuq, m);
}

/* OpenBSD allocator names; the NetBSD free() takes no size. */
#define iwx_free(p, type, size)	free((p), (type))

static inline void *
mallocarray(size_t n, size_t size, struct malloc_type *type, int flags)
{
	if (n != 0 && size > SIZE_MAX / n)
		return NULL;
	return malloc(n * size, type, flags);
}

#endif /* _DEV_PCI_IF_IWX_COMPAT_H_ */

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
#include <net/if_ether.h>
#include <sys/endian.h>
#include <sys/stdint.h>
#include <dev/firmload.h>
#include <dev/pci/pcivar.h>

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

/* ------------------------------------------------------------------
 * Kernel primitives with different names or semantics on NetBSD.
 * ------------------------------------------------------------------ */

/* nitems(9) and the OpenBSD byte-order names. */
#define nitems(a)		(sizeof(a) / sizeof((a)[0]))
#define letoh16(x)		le16toh(x)
#define letoh32(x)		le32toh(x)
#define letoh64(x)		le64toh(x)

/*
 * OpenBSD asserts the IPL at run time; NetBSD has no equivalent in
 * this driver's paths, and the driver takes its locks at IPL_NET anyway.
 */
#define splassert(ipl)		((void)0)

/*
 * malloc(9) on NetBSD with M_WAITOK never returns NULL, so the
 * OpenBSD "can fail" flag has no effect; the NULL checks stay valid.
 */
#define M_CANFAIL		0

/*
 * BUS_DMA_64BIT asks for a tag that can address above 4 GB.  On amd64
 * the PCI DMA tags are already 64-bit capable, so no flag is needed.
 */
#define BUS_DMA_64BIT		0
#define BUS_DMA_ZERO		0

/* tsleep_nsec(9): nanosecond timeouts; INFSLP means no timeout. */
#define INFSLP			(~(uint64_t)0)
#define SEC_TO_NSEC(s)		((uint64_t)(s) * 1000000000ULL)

static inline int
tsleep_nsec(void *ident, int prio, const char *wmesg, uint64_t nsec)
{
	int ticks;

	if (nsec == INFSLP)
		ticks = 0;
	else {
		ticks = mstohz((nsec + 999999) / 1000000);
		if (ticks == 0)
			ticks = 1;
	}
	return tsleep(ident, prio, wmesg, ticks);
}

/* The all-zero link-layer address, as in OpenBSD's etheranyaddr. */
static u_int8_t etheranyaddr[ETHER_ADDR_LEN];

/*
 * The OpenBSD system taskq.  It is created on first use and shared by
 * the driver.  Racing first callers both create one; the loser frees
 * its copy.
 */
static struct taskq *iwx_systq_tq;

static inline struct taskq *
iwx_get_systq(void)
{
	struct taskq *tq, *old;

	tq = iwx_systq_tq;
	if (tq != NULL)
		return tq;
	tq = taskq_create("iwxsys", 1, IPL_NONE, 0);
	if (tq == NULL)
		return NULL;
	old = atomic_cas_ptr(&iwx_systq_tq, NULL, tq);
	if (old != NULL) {
		workqueue_destroy(tq->tq_wq);
		kmem_free(tq, sizeof(*tq));
		return old;
	}
	return tq;
}

#define systq			(iwx_get_systq())

/* pci_matchbyid(9): match an attach against a vendor/product table. */
struct pci_matchid {
	u_int16_t	pm_vid;
	u_int16_t	pm_pid;
};

static inline int
pci_matchbyid(const struct pci_attach_args *pa,
    const struct pci_matchid *tbl, int n)
{
	int i;

	for (i = 0; i < n; i++)
		if (PCI_VENDOR(pa->pa_id) == tbl[i].pm_vid &&
		    PCI_PRODUCT(pa->pa_id) == tbl[i].pm_pid)
			return 1;
	return 0;
}

/*
 * loadfirmware(9): read a named image into a fresh buffer.  The caller
 * releases it with iwx_free(buf, M_DEVBUF, size).  Images come from the
 * if_iwx firmware set, as the OpenBSD driver loads them.
 */
static inline int
loadfirmware(const char *name, u_char **buf, size_t *size)
{
	firmware_handle_t fwh;
	off_t fsize;
	u_char *p;
	int err;

	*buf = NULL;
	*size = 0;

	err = firmware_open("if_iwx", name, &fwh);
	if (err)
		return err;
	fsize = firmware_get_size(fwh);
	if (fsize <= 0) {
		firmware_close(fwh);
		return EINVAL;
	}
	p = malloc(fsize, M_DEVBUF, M_WAITOK);
	err = firmware_read(fwh, 0, p, fsize);
	firmware_close(fwh);
	if (err) {
		iwx_free(p, M_DEVBUF, fsize);
		return err;
	}
	*buf = p;
	*size = fsize;
	return 0;
}

/* Names the OpenBSD driver uses that NetBSD spells differently. */
#define PCI_PCIE_LCSR			PCIE_LCSR
#define PCI_PCIE_LCSR_ASPM_L0S		PCIE_LCSR_ASPM_L0S
#define PCI_PCIE_DCSR2			PCIE_DCSR2
#define PCI_PCIE_DCSR2_LTREN		PCIE_DCSR2_LTR_MEC
#define EDCA_AC_BE			WME_AC_BE
#define EDCA_AC_BK			WME_AC_BK
#define EDCA_AC_VI			WME_AC_VI
#define EDCA_AC_VO			WME_AC_VO
#define EDCA_NUM_AC			WME_NUM_AC

/* NetBSD's malloc(9) macro discards the malloc type. */
#define M_DEVBUF			((void *)0)

/* Position of the most significant set bit, 0 for zero (as ffs()). */
static inline int
fls(unsigned int x)
{
	int b = 0;

	while (x != 0) {
		x >>= 1;
		b++;
	}
	return b;
}

static inline int
flsl(unsigned long x)
{
	int b = 0;

	while (x != 0) {
		x >>= 1;
		b++;
	}
	return b;
}

#endif /* _DEV_PCI_IF_IWX_COMPAT_H_ */

// SPDX-License-Identifier: GPL-2.0
/*
 *  linux/mm/page_io.c
 *
 *  Copyright (C) 1991, 1992, 1993, 1994  Linus Torvalds
 *
 *  Swap reorganised 29.12.95, 
 *  Asynchronous swapping added 30.12.95. Stephen Tweedie
 *  Removed race in async swapping. 14.4.1996. Bruno Haible
 *  Add swap of shared pages through the page cache. 20.2.1998. Stephen Tweedie
 *  Always use brw_page, life becomes simpler. 12 May 1998 Eric Biederman
 */

#include <linux/mm.h>
#include <linux/kernel_stat.h>
#include <linux/gfp.h>
#include <linux/pagemap.h>
#include <linux/swap.h>
#include <linux/bio.h>
#include <linux/swapops.h>
#include <linux/writeback.h>
#include <linux/blkdev.h>
#include <linux/psi.h>
#include <linux/uio.h>
#include <linux/sched/task.h>
#include <linux/delayacct.h>
#include <linux/zswap.h>
#include <linux/kfifo.h>
#include <linux/kthread.h>
#include <linux/sched/mm.h>
#include <linux/sysctl.h>
#include "internal.h"
#include "swap.h"

#undef CREATE_TRACE_POINTS
#include <trace/hooks/mm.h>
#include <trace/hooks/vmscan.h>

static void __end_swap_bio_write(struct bio *bio)
{
	struct folio *folio = bio_first_folio_all(bio);

	if (bio->bi_status) {
		/*
		 * We failed to write the page out to swap-space.
		 * Re-dirty the page in order to avoid it being reclaimed.
		 * Also print a dire warning that things will go BAD (tm)
		 * very quickly.
		 *
		 * Also clear PG_reclaim to avoid folio_rotate_reclaimable()
		 */
		folio_mark_dirty(folio);
		pr_alert_ratelimited("Write-error on swap-device (%u:%u:%llu)\n",
				     MAJOR(bio_dev(bio)), MINOR(bio_dev(bio)),
				     (unsigned long long)bio->bi_iter.bi_sector);
		folio_clear_reclaim(folio);
	}
	folio_end_writeback(folio);
}

static void end_swap_bio_write(struct bio *bio)
{
	__end_swap_bio_write(bio);
	bio_put(bio);
}

static void __end_swap_bio_read(struct bio *bio)
{
	struct folio *folio = bio_first_folio_all(bio);

	if (bio->bi_status) {
		pr_alert_ratelimited("Read-error on swap-device (%u:%u:%llu)\n",
				     MAJOR(bio_dev(bio)), MINOR(bio_dev(bio)),
				     (unsigned long long)bio->bi_iter.bi_sector);
	} else {
		folio_mark_uptodate(folio);
	}
	folio_unlock(folio);
}

static void end_swap_bio_read(struct bio *bio)
{
	__end_swap_bio_read(bio);
	bio_put(bio);
}

int generic_swapfile_activate(struct swap_info_struct *sis,
				struct file *swap_file,
				sector_t *span)
{
	struct address_space *mapping = swap_file->f_mapping;
	struct inode *inode = mapping->host;
	unsigned blocks_per_page;
	unsigned long page_no;
	unsigned blkbits;
	sector_t probe_block;
	sector_t last_block;
	sector_t lowest_block = -1;
	sector_t highest_block = 0;
	int nr_extents = 0;
	int ret;

	blkbits = inode->i_blkbits;
	blocks_per_page = PAGE_SIZE >> blkbits;

	/*
	 * Map all the blocks into the extent tree.  This code doesn't try
	 * to be very smart.
	 */
	probe_block = 0;
	page_no = 0;
	last_block = i_size_read(inode) >> blkbits;
	while ((probe_block + blocks_per_page) <= last_block &&
			page_no < sis->max) {
		unsigned block_in_page;
		sector_t first_block;

		cond_resched();

		first_block = probe_block;
		ret = bmap(inode, &first_block);
		if (ret || !first_block)
			goto bad_bmap;

		/*
		 * It must be PAGE_SIZE aligned on-disk
		 */
		if (first_block & (blocks_per_page - 1)) {
			probe_block++;
			goto reprobe;
		}

		for (block_in_page = 1; block_in_page < blocks_per_page;
					block_in_page++) {
			sector_t block;

			block = probe_block + block_in_page;
			ret = bmap(inode, &block);
			if (ret || !block)
				goto bad_bmap;

			if (block != first_block + block_in_page) {
				/* Discontiguity */
				probe_block++;
				goto reprobe;
			}
		}

		first_block >>= (PAGE_SHIFT - blkbits);
		if (page_no) {	/* exclude the header page */
			if (first_block < lowest_block)
				lowest_block = first_block;
			if (first_block > highest_block)
				highest_block = first_block;
		}

		/*
		 * We found a PAGE_SIZE-length, PAGE_SIZE-aligned run of blocks
		 */
		ret = add_swap_extent(sis, page_no, 1, first_block);
		if (ret < 0)
			goto out;
		nr_extents += ret;
		page_no++;
		probe_block += blocks_per_page;
reprobe:
		continue;
	}
	ret = nr_extents;
	*span = 1 + highest_block - lowest_block;
	if (page_no == 0)
		page_no = 1;	/* force Empty message */
	sis->max = page_no;
	sis->pages = page_no - 1;
out:
	return ret;
bad_bmap:
	pr_err("swapon: swapfile has holes\n");
	ret = -EINVAL;
	goto out;
}

static bool is_folio_zero_filled(struct folio *folio)
{
	unsigned int pos, last_pos;
	unsigned long *data;
	unsigned int i;

	last_pos = PAGE_SIZE / sizeof(*data) - 1;
	for (i = 0; i < folio_nr_pages(folio); i++) {
		data = kmap_local_folio(folio, i * PAGE_SIZE);
		/*
		 * Check last word first, incase the page is zero-filled at
		 * the start and has non-zero data at the end, which is common
		 * in real-world workloads.
		 */
		if (data[last_pos]) {
			kunmap_local(data);
			return false;
		}
		for (pos = 0; pos < last_pos; pos++) {
			if (data[pos]) {
				kunmap_local(data);
				return false;
			}
		}
		kunmap_local(data);
	}

	return true;
}

static void swap_zeromap_folio_set(struct folio *folio)
{
	struct obj_cgroup *objcg = get_obj_cgroup_from_folio(folio);
	struct swap_info_struct *sis = __swap_entry_to_info(folio->swap);
	int nr_pages = folio_nr_pages(folio);
	swp_entry_t entry;
	unsigned int i;

	for (i = 0; i < folio_nr_pages(folio); i++) {
		entry = page_swap_entry(folio_page(folio, i));
		set_bit(swp_offset(entry), sis->zeromap);
	}

	count_vm_events(SWPOUT_ZERO, nr_pages);
	if (objcg) {
		count_objcg_events(objcg, SWPOUT_ZERO, nr_pages);
		obj_cgroup_put(objcg);
	}
}

static void swap_zeromap_folio_clear(struct folio *folio)
{
	struct swap_info_struct *sis = __swap_entry_to_info(folio->swap);
	swp_entry_t entry;
	unsigned int i;

	for (i = 0; i < folio_nr_pages(folio); i++) {
		entry = page_swap_entry(folio_page(folio, i));
		clear_bit(swp_offset(entry), sis->zeromap);
	}
}

/*
 * k618 port kcompressd -- vm.kcompressd: kswapd hands the anon folios it writes to zram (a device
 * that compresses inside the write call: __SWP_WRITE_SYNCHRONOUS_IO) to per-node kcompressd
 * threads instead of compressing them itself, and goes on scanning (the kcompressd proposal
 * posted to linux-mm in 2025, as on the phone's 5.15 kernel). The folio stays
 * locked until a thread has written it, as during kswapd's own write, so a fault on it, swapoff
 * and anything else that wants it waits on the folio lock as before; with the queue (256 folios
 * a node) full, kswapd writes it itself. The value is how many threads per node may work (0:
 * off, at most 4): the first takes every folio, each further one joins while the backlog is
 * another quarter of the queue deep. Direct reclaim and every other writer, shmem and large
 * folios write in place as before. The threads are a reclaim context like kswapd (PF_MEMALLOC:
 * may use the reserves, never reclaim themselves; their time counts as memory stall for PSI, as
 * kswapd's own does), are not freezable, so that a queued folio never stays locked across a
 * suspend, and follow vm.reclaim_cpus (mm/vmscan.c) each time they wake. kswapd frees what they
 * wrote: back on the LRU, a written folio goes to the cold end; still in kswapd's batch, it gets
 * a second pass (kcompressd_written() in mm/vmscan.c).
 *   vm.kcompressd_pages  folios the threads wrote; vm.kcompressd_freed  folios the second pass
 *                        freed (statistics: not exact)
 */
int vm_kcompressd = 2;
unsigned long kcompressd_freed;
static unsigned long kcompressd_pages;

#define KCOMPRESSD_QUEUE	256	/* folios a node */
#define KCOMPRESSD_THREADS	4	/* a node: the most vm.kcompressd allows */

struct kcompressd {
	spinlock_t lock;		/* the producer: kswapd */
	int nid;
	struct kcompressd_worker {
		struct kcompressd *kc;
		wait_queue_head_t wait;
	} worker[KCOMPRESSD_THREADS];
	DECLARE_KFIFO(fifo, struct folio *, KCOMPRESSD_QUEUE);
};

static struct kcompressd *kcompressd_of[MAX_NUMNODES] __read_mostly;

static bool kcompressd_queue(struct folio *folio)
{
	int threads = min(READ_ONCE(vm_kcompressd), KCOMPRESSD_THREADS), i;
	unsigned long flags;
	struct kcompressd *kc;
	unsigned int queued;

	if (threads <= 0 || !current_is_kswapd())
		return false;
	/* anon folios of one page, to a device that compresses in the write call (zram) */
	flags = data_race(__swap_entry_to_info(folio->swap)->flags);
	if (!folio_test_anon(folio) || folio_test_large(folio) ||
	    (flags & (SWP_FS_OPS | __SWP_WRITE_SYNCHRONOUS_IO)) != __SWP_WRITE_SYNCHRONOUS_IO)
		return false;
	kc = READ_ONCE(kcompressd_of[folio_nid(folio)]);
	if (!kc)
		return false;
	folio_get(folio);
	spin_lock(&kc->lock);
	queued = kfifo_in(&kc->fifo, &folio, 1) ? kfifo_len(&kc->fifo) : 0;
	spin_unlock(&kc->lock);
	if (!queued) {
		folio_put(folio);
		return false;	/* the queue is full: kswapd writes it itself */
	}
	/* the first thread for any folio, one more for each further quarter of the queue */
	threads = min_t(int, threads, 1 + (queued - 1) / (KCOMPRESSD_QUEUE / KCOMPRESSD_THREADS));
	for (i = 0; i < threads; i++)
		if (wq_has_sleeper(&kc->worker[i].wait))
			wake_up_interruptible(&kc->worker[i].wait);
	return true;
}

static void kcompressd_write(struct folio *folio)
{
	/* as at the start of swap_writeout(): the owner may have gone while it was queued */
	if (folio_free_swap(folio)) {
		trace_android_vh_shrink_folio_lock_owner_clear(folio);
		folio_unlock(folio);
	} else {
		__swap_writepage(folio, NULL);
		kcompressd_pages++;
		/*
		 * kswapd met it still locked and kept it. Back on the LRU by now, it goes to the
		 * cold end, where the next scan frees it, as kswapd would have right away had it
		 * written the folio itself; still off the LRU, kswapd's second pass frees it
		 * (kcompressd_written() in mm/vmscan.c).
		 */
		folio_rotate_reclaimable(folio);
	}
	folio_put(folio);
}

static int kcompressd(void *arg)
{
	struct kcompressd_worker *w = arg;
	struct kcompressd *kc = w->kc;
	unsigned long pflags;
	struct folio *folio;

	/*
	 * As kswapd: may use the reserves, never reclaims itself. Not freezable: a queued (locked)
	 * folio is always written, also while the system suspends.
	 */
	current->flags |= PF_MEMALLOC;
	while (!kthread_should_stop()) {
		const struct cpumask *node = cpumask_of_node(kc->nid);

		wait_event_interruptible(w->wait, !kfifo_is_empty(&kc->fifo) ||
					 kthread_should_stop());
		/* the node's reclaim CPUs (vm.reclaim_cpus, mm/vmscan.c) */
		reclaim_cpus_follow(cpumask_empty(node) ? cpu_online_mask : node);
		psi_memstall_enter(&pflags);
		__fs_reclaim_acquire(_THIS_IP_);
		while (kfifo_out_spinlocked_noirqsave(&kc->fifo, &folio, 1, &kc->lock)) {
			kcompressd_write(folio);
			cond_resched();
		}
		__fs_reclaim_release(_THIS_IP_);
		psi_memstall_leave(&pflags);
	}
	return 0;
}

static const struct ctl_table kcompressd_sysctls[] = {
	{
		.procname	= "kcompressd",
		.data		= &vm_kcompressd,
		.maxlen		= sizeof(vm_kcompressd),
		.mode		= 0644,
		.proc_handler	= proc_dointvec_minmax,
		.extra1		= SYSCTL_ZERO,
		.extra2		= SYSCTL_FOUR,
	},
	{
		.procname	= "kcompressd_pages",
		.data		= &kcompressd_pages,
		.maxlen		= sizeof(kcompressd_pages),
		.mode		= 0444,
		.proc_handler	= proc_doulongvec_minmax,
	},
	{
		.procname	= "kcompressd_freed",
		.data		= &kcompressd_freed,
		.maxlen		= sizeof(kcompressd_freed),
		.mode		= 0444,
		.proc_handler	= proc_doulongvec_minmax,
	},
};

static int __init kcompressd_init(void)
{
	int nid, i;

	for_each_node_state(nid, N_MEMORY) {
		struct kcompressd *kc = kzalloc_node(sizeof(*kc), GFP_KERNEL, nid);

		if (!kc) {
			pr_err("kcompressd%d: no memory, kswapd compresses in place\n", nid);
			continue;
		}
		spin_lock_init(&kc->lock);
		INIT_KFIFO(kc->fifo);
		kc->nid = nid;
		/* every queue first: kswapd may wake one whose thread did not start (a no-op) */
		for (i = 0; i < KCOMPRESSD_THREADS; i++) {
			kc->worker[i].kc = kc;
			init_waitqueue_head(&kc->worker[i].wait);
		}
		for (i = 0; i < KCOMPRESSD_THREADS; i++) {
			struct task_struct *t;

			t = i ? kthread_run(kcompressd, &kc->worker[i], "kcompressd%d:%d", nid, i) :
				kthread_run(kcompressd, &kc->worker[i], "kcompressd%d", nid);
			if (IS_ERR(t)) {
				pr_err("kcompressd%d: worker %d not started (%ld)\n", nid, i,
				       PTR_ERR(t));
				break;
			}
		}
		if (i)
			WRITE_ONCE(kcompressd_of[nid], kc);
		else
			kfree(kc);
	}
	register_sysctl_init("vm", kcompressd_sysctls);
	return 0;
}
late_initcall(kcompressd_init);

/*
 * We may have stale swap cache pages in memory: notice
 * them here and get rid of the unnecessary final write.
 */
int swap_writeout(struct folio *folio, struct swap_iocb **swap_plug)
{
	int ret = 0;

	if (folio_free_swap(folio))
		goto out_unlock;

	/*
	 * Arch code may have to preserve more data than just the page
	 * contents, e.g. memory tags.
	 */
	ret = arch_prepare_to_swap(folio);
	if (ret) {
		folio_mark_dirty(folio);
		goto out_unlock;
	}

	/*
	 * Use a bitmap (zeromap) to avoid doing IO for zero-filled pages.
	 * The bits in zeromap are protected by the locked swapcache folio
	 * and atomic updates are used to protect against read-modify-write
	 * corruption due to other zero swap entries seeing concurrent updates.
	 */
	if (is_folio_zero_filled(folio)) {
		swap_zeromap_folio_set(folio);
		goto out_unlock;
	}

	/*
	 * Clear bits this folio occupies in the zeromap to prevent zero data
	 * being read in from any previous zero writes that occupied the same
	 * swap entries.
	 */
	swap_zeromap_folio_clear(folio);

	if (zswap_store(folio)) {
		count_mthp_stat(folio_order(folio), MTHP_STAT_ZSWPOUT);
		goto out_unlock;
	}
	if (!mem_cgroup_zswap_writeback_enabled(folio_memcg(folio))) {
		folio_mark_dirty(folio);
		return AOP_WRITEPAGE_ACTIVATE;
	}

	if (kcompressd_queue(folio))	/* k618 port kcompressd */
		return 0;	/* still locked: a kcompressd thread writes it */
	__swap_writepage(folio, swap_plug);
	return 0;
out_unlock:
	trace_android_vh_shrink_folio_lock_owner_clear(folio);
	folio_unlock(folio);
	return ret;
}

static inline void count_swpout_vm_event(struct folio *folio)
{
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	if (unlikely(folio_test_pmd_mappable(folio))) {
		count_memcg_folio_events(folio, THP_SWPOUT, 1);
		count_vm_event(THP_SWPOUT);
	}
#endif
	count_mthp_stat(folio_order(folio), MTHP_STAT_SWPOUT);
	count_memcg_folio_events(folio, PSWPOUT, folio_nr_pages(folio));
	count_vm_events(PSWPOUT, folio_nr_pages(folio));
}

#if defined(CONFIG_MEMCG) && defined(CONFIG_BLK_CGROUP)
static void bio_associate_blkg_from_page(struct bio *bio, struct folio *folio)
{
	struct cgroup_subsys_state *css;
	struct mem_cgroup *memcg;

	memcg = folio_memcg(folio);
	if (!memcg)
		return;

	rcu_read_lock();
	css = cgroup_e_css(memcg->css.cgroup, &io_cgrp_subsys);
	bio_associate_blkg_from_css(bio, css);
	rcu_read_unlock();
}
#else
#define bio_associate_blkg_from_page(bio, folio)		do { } while (0)
#endif /* CONFIG_MEMCG && CONFIG_BLK_CGROUP */

struct swap_iocb {
	struct kiocb		iocb;
	struct bio_vec		bvec[SWAP_CLUSTER_MAX];
	int			pages;
	int			len;
};
static mempool_t *sio_pool;

int sio_pool_init(void)
{
	if (!sio_pool) {
		mempool_t *pool = mempool_create_kmalloc_pool(
			SWAP_CLUSTER_MAX, sizeof(struct swap_iocb));
		if (cmpxchg(&sio_pool, NULL, pool))
			mempool_destroy(pool);
	}
	if (!sio_pool)
		return -ENOMEM;
	return 0;
}

static void sio_write_complete(struct kiocb *iocb, long ret)
{
	struct swap_iocb *sio = container_of(iocb, struct swap_iocb, iocb);
	struct page *page = sio->bvec[0].bv_page;
	int p;

	if (ret != sio->len) {
		/*
		 * In the case of swap-over-nfs, this can be a
		 * temporary failure if the system has limited
		 * memory for allocating transmit buffers.
		 * Mark the page dirty and avoid
		 * folio_rotate_reclaimable but rate-limit the
		 * messages.
		 */
		pr_err_ratelimited("Write error %ld on dio swapfile (%llu)\n",
				   ret, swap_dev_pos(page_swap_entry(page)));
		for (p = 0; p < sio->pages; p++) {
			page = sio->bvec[p].bv_page;
			set_page_dirty(page);
			ClearPageReclaim(page);
		}
	}

	for (p = 0; p < sio->pages; p++)
		end_page_writeback(sio->bvec[p].bv_page);

	mempool_free(sio, sio_pool);
}

static void swap_writepage_fs(struct folio *folio, struct swap_iocb **swap_plug)
{
	struct swap_iocb *sio = swap_plug ? *swap_plug : NULL;
	struct swap_info_struct *sis = __swap_entry_to_info(folio->swap);
	struct file *swap_file = sis->swap_file;
	loff_t pos = swap_dev_pos(folio->swap);

	count_swpout_vm_event(folio);
	folio_start_writeback(folio);
	trace_android_vh_shrink_folio_lock_owner_clear(folio);
	folio_unlock(folio);
	if (sio) {
		if (sio->iocb.ki_filp != swap_file ||
		    sio->iocb.ki_pos + sio->len != pos) {
			swap_write_unplug(sio);
			sio = NULL;
		}
	}
	if (!sio) {
		sio = mempool_alloc(sio_pool, GFP_NOIO);
		init_sync_kiocb(&sio->iocb, swap_file);
		sio->iocb.ki_complete = sio_write_complete;
		sio->iocb.ki_pos = pos;
		sio->pages = 0;
		sio->len = 0;
	}
	bvec_set_folio(&sio->bvec[sio->pages], folio, folio_size(folio), 0);
	sio->len += folio_size(folio);
	sio->pages += 1;
	if (sio->pages == ARRAY_SIZE(sio->bvec) || !swap_plug) {
		swap_write_unplug(sio);
		sio = NULL;
	}
	if (swap_plug)
		*swap_plug = sio;
}

static void swap_writepage_bdev_sync(struct folio *folio,
		struct swap_info_struct *sis)
{
	struct bio_vec bv;
	struct bio bio;

	bio_init(&bio, sis->bdev, &bv, 1, REQ_OP_WRITE | REQ_SWAP);
	bio.bi_iter.bi_sector = swap_folio_sector(folio);
	bio_add_folio_nofail(&bio, folio, folio_size(folio), 0);

	bio_associate_blkg_from_page(&bio, folio);
	count_swpout_vm_event(folio);

	folio_start_writeback(folio);
	trace_android_vh_shrink_folio_lock_owner_clear(folio);
	folio_unlock(folio);

	submit_bio_wait(&bio);
	__end_swap_bio_write(&bio);
}

static void swap_writepage_bdev_async(struct folio *folio,
		struct swap_info_struct *sis)
{
	struct bio *bio;

	bio = bio_alloc(sis->bdev, 1, REQ_OP_WRITE | REQ_SWAP, GFP_NOIO);
	bio->bi_iter.bi_sector = swap_folio_sector(folio);
	bio->bi_end_io = end_swap_bio_write;
	bio_add_folio_nofail(bio, folio, folio_size(folio), 0);

	bio_associate_blkg_from_page(bio, folio);
	count_swpout_vm_event(folio);
	folio_start_writeback(folio);
	trace_android_vh_shrink_folio_lock_owner_clear(folio);
	folio_unlock(folio);
	submit_bio(bio);
}

void __swap_writepage(struct folio *folio, struct swap_iocb **swap_plug)
{
	struct swap_info_struct *sis = __swap_entry_to_info(folio->swap);
	unsigned long sis_flags = 0;
	unsigned long swap_writepage_start;

	VM_BUG_ON_FOLIO(!folio_test_swapcache(folio), folio);
	trace_android_vh_swap_writepage_start(&swap_writepage_start);
	/*
	 * ->flags can be updated non-atomicially (scan_swap_map_slots),
	 * but that will never affect SWP_FS_OPS, so the data_race
	 * is safe.
	 */
	sis_flags = data_race(sis->flags);
	trace_android_vh_swap_writepage(&sis_flags, &folio->page);
	if (sis_flags & SWP_FS_OPS)
		swap_writepage_fs(folio, swap_plug);
	/*
	 * ->flags can be updated non-atomicially (scan_swap_map_slots),
	 * but that will never affect __SWP_WRITE_SYNCHRONOUS_IO, so the data_race
	 * is safe.
	 */
	else if (sis_flags & __SWP_WRITE_SYNCHRONOUS_IO)
		swap_writepage_bdev_sync(folio, sis);
	else
		swap_writepage_bdev_async(folio, sis);

	trace_android_vh_swap_writepage_end(&folio->page, swap_writepage_start);
}

void swap_write_unplug(struct swap_iocb *sio)
{
	struct iov_iter from;
	struct address_space *mapping = sio->iocb.ki_filp->f_mapping;
	int ret;

	iov_iter_bvec(&from, ITER_SOURCE, sio->bvec, sio->pages, sio->len);
	ret = mapping->a_ops->swap_rw(&sio->iocb, &from);
	if (ret != -EIOCBQUEUED)
		sio_write_complete(&sio->iocb, ret);
}

static void sio_read_complete(struct kiocb *iocb, long ret)
{
	struct swap_iocb *sio = container_of(iocb, struct swap_iocb, iocb);
	int p;

	if (ret == sio->len) {
		for (p = 0; p < sio->pages; p++) {
			struct folio *folio = page_folio(sio->bvec[p].bv_page);

			count_mthp_stat(folio_order(folio), MTHP_STAT_SWPIN);
			count_memcg_folio_events(folio, PSWPIN, folio_nr_pages(folio));
			folio_mark_uptodate(folio);
			folio_unlock(folio);
		}
		count_vm_events(PSWPIN, sio->pages);
	} else {
		for (p = 0; p < sio->pages; p++) {
			struct folio *folio = page_folio(sio->bvec[p].bv_page);

			folio_unlock(folio);
		}
		pr_alert_ratelimited("Read-error on swap-device\n");
	}
	mempool_free(sio, sio_pool);
}

static bool swap_read_folio_zeromap(struct folio *folio)
{
	int nr_pages = folio_nr_pages(folio);
	struct obj_cgroup *objcg;
	bool is_zeromap;

	/*
	 * Swapping in a large folio that is partially in the zeromap is not
	 * currently handled. Return true without marking the folio uptodate so
	 * that an IO error is emitted (e.g. do_swap_page() will sigbus).
	 */
	if (WARN_ON_ONCE(swap_zeromap_batch(folio->swap, nr_pages,
			&is_zeromap) != nr_pages))
		return true;

	if (!is_zeromap)
		return false;

	objcg = get_obj_cgroup_from_folio(folio);
	count_vm_events(SWPIN_ZERO, nr_pages);
	if (objcg) {
		count_objcg_events(objcg, SWPIN_ZERO, nr_pages);
		obj_cgroup_put(objcg);
	}

	folio_zero_range(folio, 0, folio_size(folio));
	folio_mark_uptodate(folio);
	return true;
}

static void swap_read_folio_fs(struct folio *folio, struct swap_iocb **plug)
{
	struct swap_info_struct *sis = __swap_entry_to_info(folio->swap);
	struct swap_iocb *sio = NULL;
	loff_t pos = swap_dev_pos(folio->swap);

	if (plug)
		sio = *plug;
	if (sio) {
		if (sio->iocb.ki_filp != sis->swap_file ||
		    sio->iocb.ki_pos + sio->len != pos) {
			swap_read_unplug(sio);
			sio = NULL;
		}
	}
	if (!sio) {
		sio = mempool_alloc(sio_pool, GFP_KERNEL);
		init_sync_kiocb(&sio->iocb, sis->swap_file);
		sio->iocb.ki_pos = pos;
		sio->iocb.ki_complete = sio_read_complete;
		sio->pages = 0;
		sio->len = 0;
	}
	bvec_set_folio(&sio->bvec[sio->pages], folio, folio_size(folio), 0);
	sio->len += folio_size(folio);
	sio->pages += 1;
	if (sio->pages == ARRAY_SIZE(sio->bvec) || !plug) {
		swap_read_unplug(sio);
		sio = NULL;
	}
	if (plug)
		*plug = sio;
}

static void swap_read_folio_bdev_sync(struct folio *folio,
		struct swap_info_struct *sis)
{
	struct bio_vec bv;
	struct bio bio;
	bool read = false;

	trace_android_rvh_swap_read_folio_bdev_sync(sis->bdev,
		swap_folio_sector(folio) + get_start_sect(sis->bdev),
		&folio->page, &read);
	if (read) {
		count_mthp_stat(folio_order(folio), MTHP_STAT_SWPIN);
		count_memcg_folio_events(folio, PSWPIN, folio_nr_pages(folio));
		count_vm_events(PSWPIN, folio_nr_pages(folio));
		return;
	}

	bio_init(&bio, sis->bdev, &bv, 1, REQ_OP_READ);
	bio.bi_iter.bi_sector = swap_folio_sector(folio);
	bio_add_folio_nofail(&bio, folio, folio_size(folio), 0);
	/*
	 * Keep this task valid during swap readpage because the oom killer may
	 * attempt to access it in the page fault retry time check.
	 */
	get_task_struct(current);
	count_mthp_stat(folio_order(folio), MTHP_STAT_SWPIN);
	count_memcg_folio_events(folio, PSWPIN, folio_nr_pages(folio));
	count_vm_events(PSWPIN, folio_nr_pages(folio));
	submit_bio_wait(&bio);
	trace_android_rvh_swap_bio_charge(&bio);
	__end_swap_bio_read(&bio);
	put_task_struct(current);
}

static void swap_read_folio_bdev_async(struct folio *folio,
		struct swap_info_struct *sis)
{
	struct bio *bio;

	bio = bio_alloc(sis->bdev, 1, REQ_OP_READ, GFP_KERNEL);
	bio->bi_iter.bi_sector = swap_folio_sector(folio);
	bio->bi_end_io = end_swap_bio_read;
	bio_add_folio_nofail(bio, folio, folio_size(folio), 0);
	count_mthp_stat(folio_order(folio), MTHP_STAT_SWPIN);
	count_memcg_folio_events(folio, PSWPIN, folio_nr_pages(folio));
	count_vm_events(PSWPIN, folio_nr_pages(folio));
	submit_bio(bio);
}

void swap_read_folio(struct folio *folio, struct swap_iocb **plug)
{
	struct swap_info_struct *sis = __swap_entry_to_info(folio->swap);
	bool synchronous = sis->flags & SWP_SYNCHRONOUS_IO;
	bool workingset = folio_test_workingset(folio);
	unsigned long pflags;
	bool in_thrashing;
	unsigned long swapin_start;

	VM_BUG_ON_FOLIO(!folio_test_swapcache(folio) && !synchronous, folio);
	VM_BUG_ON_FOLIO(!folio_test_locked(folio), folio);
	VM_BUG_ON_FOLIO(folio_test_uptodate(folio), folio);
	trace_android_vh_swapin_start(&swapin_start);

	/*
	 * Count submission time as memory stall and delay. When the device
	 * is congested, or the submitting cgroup IO-throttled, submission
	 * can be a significant part of overall IO time.
	 */
	if (workingset) {
		delayacct_thrashing_start(&in_thrashing);
		psi_memstall_enter(&pflags);
	}
	delayacct_swapin_start();

	if (swap_read_folio_zeromap(folio)) {
		folio_unlock(folio);
		goto finish;
	}

	if (zswap_load(folio) != -ENOENT)
		goto finish;

	/* We have to read from slower devices. Increase zswap protection. */
	zswap_folio_swapin(folio);

	if (data_race(sis->flags & SWP_FS_OPS)) {
		swap_read_folio_fs(folio, plug);
	} else if (synchronous) {
		swap_read_folio_bdev_sync(folio, sis);
	} else {
		swap_read_folio_bdev_async(folio, sis);
	}

finish:
	if (workingset) {
		delayacct_thrashing_end(&in_thrashing);
		psi_memstall_leave(&pflags);
	}
	delayacct_swapin_end();
	trace_android_vh_swapin_end(folio, swapin_start);
}

void __swap_read_unplug(struct swap_iocb *sio)
{
	struct iov_iter from;
	struct address_space *mapping = sio->iocb.ki_filp->f_mapping;
	int ret;

	iov_iter_bvec(&from, ITER_DEST, sio->bvec, sio->pages, sio->len);
	ret = mapping->a_ops->swap_rw(&sio->iocb, &from);
	if (ret != -EIOCBQUEUED)
		sio_read_complete(&sio->iocb, ret);
}

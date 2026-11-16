/* k618: the 5.15 calls this driver uses that 6.x renamed or dropped */
#ifndef K618_RPC_COMPAT_H
#define K618_RPC_COMPAT_H
#include <linux/firmware/qcom/qcom_scm.h>

/*
 * hyp_assign_phys() was Qualcomm's wrapper around the SCM memory-assign call; 6.x has the call
 * itself, taking the source list as a bitmap and the destinations as {vmid, perm} pairs.
 */
static inline int k618_hyp_assign_phys(phys_addr_t addr, u64 size, u32 *source_vm_list,
		int source_nelems, int *dest_vmids, int *dest_perms, int dest_nelems)
{
	struct qcom_scm_vmperm dst[8];
	u64 src = 0;
	int i;

	if (dest_nelems > (int)ARRAY_SIZE(dst))
		return -EINVAL;
	for (i = 0; i < source_nelems; i++)
		src |= BIT_ULL(source_vm_list[i]);
	for (i = 0; i < dest_nelems; i++) {
		dst[i].vmid = dest_vmids[i];
		dst[i].perm = dest_perms[i];
	}
	return qcom_scm_assign_mem(addr, size, &src, dst, dest_nelems);
}
#define hyp_assign_phys(a, s, svl, sn, dv, dp, dn) \
	k618_hyp_assign_phys((a), (s), (svl), (sn), (dv), (dp), (dn))

/*
 * Qualcomm's own DMA attributes, which 5.15 carried in the core dma-mapping.h and 6.x does not.
 * The driver passes this one for the buffers it wants kept in the system cache only; without the
 * attribute the mapping is simply the ordinary cached one.
 */
#ifndef DMA_ATTR_SYS_CACHE_ONLY
#define DMA_ATTR_SYS_CACHE_ONLY		0
#endif

/*
 * iommu_dma_enable_best_fit_algo() was Qualcomm's 5.15 addition to the DMA-IOMMU allocator (pick
 * the tightest free IOVA range instead of the first). 6.x has no such knob, and the allocator's
 * own choice is what every other driver here lives with.
 */
static inline int k618_iommu_best_fit_noop(struct device *dev)
{
	return 0;
}
#define iommu_dma_enable_best_fit_algo(dev) k618_iommu_best_fit_noop(dev)

/* Device compatibility. */
#include <linux/ratelimit.h>
static DEFINE_RATELIMIT_STATE(k618_rpc_rs, 5 * HZ, 10);

#endif /* K618_RPC_COMPAT_H */

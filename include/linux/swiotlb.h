#ifndef __LINUX_SWIOTLB_H
#define __LINUX_SWIOTLB_H

#include <linux/types.h>

struct device;
struct dma_attrs;
struct scatterlist;

extern int swiotlb_force;

/*
 * Maximum allowable number of contiguous slabs to map,
 * must be a power of 2.  What is the appropriate value ?
 * The complexity of {map,unmap}_single is linearly dependent on this value.
 */
#define IO_TLB_SEGSIZE	128

/*
 * log of the size of each IO TLB slab.  The number of slabs is command line
 * controllable.
 */
#define IO_TLB_SHIFT 11

/* swiotlb-core.c */
extern void swiotlb_init(int verbose);
#ifdef CONFIG_SWIOTLB
extern void __init swiotlb_free(void);
#else
static inline void swiotlb_free(void) { }
#endif
extern void swiotlb_print_info(void);

/* swiotlb-core.c: Internal book-keeping functions.
 * Must be linked against the library to take advantage of them.*/
#ifdef CONFIG_SWIOTLB
/*
 * Enumeration for sync targets
 */
enum dma_sync_target {
	SYNC_FOR_CPU = 0,
	SYNC_FOR_DEVICE = 1,
};
extern char *io_tlb_start;
extern char *io_tlb_end;
extern unsigned long io_tlb_nslabs;
extern void *io_tlb_overflow_buffer;
extern unsigned long io_tlb_overflow;
extern int is_swiotlb_buffer(phys_addr_t paddr);
extern void swiotlb_bounce(phys_addr_t phys, char *dma_addr, size_t size,
			   enum dma_data_direction dir);
extern void *do_map_single(struct device *hwdev, phys_addr_t phys,
			    unsigned long start_dma_addr, size_t size, int dir);

extern void do_unmap_single(struct device *hwdev, char *dma_addr, size_t size,
			     int dir);

extern void do_sync_single(struct device *hwdev, char *dma_addr, size_t size,
			   int dir, int target);
extern void swiotlb_full(struct device *dev, size_t size, int dir, int do_panic);
extern void __init swiotlb_init_early(size_t default_size, int verbose);
#endif

/* swiotlb.c: dma_ops functions. */
extern void
*swiotlb_alloc_coherent(struct device *hwdev, size_t size,
			dma_addr_t *dma_handle, gfp_t flags);

extern void
swiotlb_free_coherent(struct device *hwdev, size_t size,
		      void *vaddr, dma_addr_t dma_handle);

extern dma_addr_t swiotlb_map_page(struct device *dev, struct page *page,
				   unsigned long offset, size_t size,
				   enum dma_data_direction dir,
				   struct dma_attrs *attrs);
extern void swiotlb_unmap_page(struct device *hwdev, dma_addr_t dev_addr,
			       size_t size, enum dma_data_direction dir,
			       struct dma_attrs *attrs);

extern int
swiotlb_map_sg(struct device *hwdev, struct scatterlist *sg, int nents,
	       int direction);

extern void
swiotlb_unmap_sg(struct device *hwdev, struct scatterlist *sg, int nents,
		 int direction);

extern int
swiotlb_map_sg_attrs(struct device *hwdev, struct scatterlist *sgl, int nelems,
		     enum dma_data_direction dir, struct dma_attrs *attrs);

extern void
swiotlb_unmap_sg_attrs(struct device *hwdev, struct scatterlist *sgl,
		       int nelems, enum dma_data_direction dir,
		       struct dma_attrs *attrs);

extern void
swiotlb_sync_single_for_cpu(struct device *hwdev, dma_addr_t dev_addr,
			    size_t size, enum dma_data_direction dir);

extern void
swiotlb_sync_sg_for_cpu(struct device *hwdev, struct scatterlist *sg,
			int nelems, enum dma_data_direction dir);

extern void
swiotlb_sync_single_for_device(struct device *hwdev, dma_addr_t dev_addr,
			       size_t size, enum dma_data_direction dir);

extern void
swiotlb_sync_sg_for_device(struct device *hwdev, struct scatterlist *sg,
			   int nelems, enum dma_data_direction dir);

extern void
swiotlb_sync_single_range_for_cpu(struct device *hwdev, dma_addr_t dev_addr,
				  unsigned long offset, size_t size,
				  enum dma_data_direction dir);

extern void
swiotlb_sync_single_range_for_device(struct device *hwdev, dma_addr_t dev_addr,
				     unsigned long offset, size_t size,
				     enum dma_data_direction dir);

extern int
swiotlb_dma_mapping_error(struct device *hwdev, dma_addr_t dma_addr);

extern int
swiotlb_dma_supported(struct device *hwdev, u64 mask);

/* swiotlb-xen.c: dma_ops functions. */
extern void xen_swiotlb_init(int verbose);
extern void
*xen_swiotlb_alloc_coherent(struct device *hwdev, size_t size,
			dma_addr_t *dma_handle, gfp_t flags);

extern void
xen_swiotlb_free_coherent(struct device *hwdev, size_t size,
		      void *vaddr, dma_addr_t dma_handle);

extern dma_addr_t xen_swiotlb_map_page(struct device *dev, struct page *page,
				   unsigned long offset, size_t size,
				   enum dma_data_direction dir,
				   struct dma_attrs *attrs);
extern void xen_swiotlb_unmap_page(struct device *hwdev, dma_addr_t dev_addr,
			       size_t size, enum dma_data_direction dir,
			       struct dma_attrs *attrs);

extern int
xen_swiotlb_map_sg(struct device *hwdev, struct scatterlist *sg, int nents,
	       int direction);

extern void
xen_swiotlb_unmap_sg(struct device *hwdev, struct scatterlist *sg, int nents,
		 int direction);

extern int
xen_swiotlb_map_sg_attrs(struct device *hwdev, struct scatterlist *sgl,
			 int nelems, enum dma_data_direction dir,
			 struct dma_attrs *attrs);

extern void
xen_swiotlb_unmap_sg_attrs(struct device *hwdev, struct scatterlist *sgl,
		       int nelems, enum dma_data_direction dir,
		       struct dma_attrs *attrs);

extern void
xen_swiotlb_sync_single_for_cpu(struct device *hwdev, dma_addr_t dev_addr,
			    size_t size, enum dma_data_direction dir);

extern void
xen_swiotlb_sync_sg_for_cpu(struct device *hwdev, struct scatterlist *sg,
			int nelems, enum dma_data_direction dir);

extern void
xen_swiotlb_sync_single_for_device(struct device *hwdev, dma_addr_t dev_addr,
			       size_t size, enum dma_data_direction dir);

extern void
xen_swiotlb_sync_sg_for_device(struct device *hwdev, struct scatterlist *sg,
			   int nelems, enum dma_data_direction dir);

extern void
xen_swiotlb_sync_single_range_for_cpu(struct device *hwdev, dma_addr_t dev_addr,
				  unsigned long offset, size_t size,
				  enum dma_data_direction dir);

extern void
xen_swiotlb_sync_single_range_for_device(struct device *hwdev,
					 dma_addr_t dev_addr,
					 unsigned long offset, size_t size,
					 enum dma_data_direction dir);

extern int
xen_swiotlb_dma_mapping_error(struct device *hwdev, dma_addr_t dma_addr);

extern int
xen_swiotlb_dma_supported(struct device *hwdev, u64 mask);


#endif /* __LINUX_SWIOTLB_H */

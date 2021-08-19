#ifndef _LINUX_APPLE_RTKIT_H_
#define _LINUX_APPLE_RTKIT_H_

#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/mailbox_client.h>

/*
 * APPLE_RTKIT_SHMEM_OWNER_LINUX - shared memory buffers are allocated and
 *                                 managed by Linux. ops->shmem_alloc and
 *                                 ops->shmem_free can be used to override
 *                                 dma_alloc/free_coherent.
 * APPLE_RTKIT_SHMEM_OWNER_RTKIT - shared memory buffers are allocated and
 *                                 managed by RTKit. ops->shmem_map and
 *                                 ops->shmem_unmap must be defined.
 * APPLE_RTKIT_RECV_ATOMIC       - ops->recv_message will be called from
 *                                 atomic / interrupt context.
 */
#define APPLE_RTKIT_SHMEM_OWNER_LINUX BIT(0)
#define APPLE_RTKIT_SHMEM_OWNER_RTKIT BIT(1)
#define APPLE_RTKIT_RECV_ATOMIC BIT(2)

/*
 * Struct to represent implementation-specific RTKit operations.
 *
 * @flags:        Combination of flags defined above. Exactly one of
 *                APPLE_RTKIT_SHMEM_OWNER_RTKIT or APPLE_RTKIT_SHMEM_OWNER_LINUX
 *                must be set.
 * @recv_message: Function called when a message from RTKit is recevied
 *                on a non-system endpoint. Called from a worker thread unless
 *                APPLE_RTKIT_RECV_ATOMIC is set.
 * @shmem_map:    Used with APPLE_RTKIT_SHMEM_OWNER_RTKIT to map an
 *                addressed returned by the co-processor into the kernel.
 * @shmem_unmap:  Used with APPLE_RTKIT_SHMEM_OWNER_RTKIT to unmap a previous
 *                mapping created with shmem_map again.
 * @shmem_alloc:  Used with APPLE_RTKIT_SHMEM_OWNER_LINUX to allocate a shared
 *                memory buffer for the co-processor. If not specified
 *                dma_alloc_coherent is used.
 * @shmem_free:   Used with APPLE_RTKIT_SHMEM_OWNER_LINUX to free a shared
 *                memory buffer previously allocated with shmem_alloc. If not
 *                specified dma_free_coherent is used.
 */
struct apple_rtkit_ops {
	unsigned int flags;
	void (*recv_message)(void *cookie, u8 endpoint, u64 message);
	void __iomem *(*shmem_map)(void *cookie, dma_addr_t addr, size_t len);
	void (*shmem_unmap)(void *cookie, void __iomem *ptr, dma_addr_t addr,
			    size_t len);
	void *(*shmem_alloc)(void *cookie, size_t size, dma_addr_t *dma_handle,
			     gfp_t flag);
	void (*shmem_free)(void *cookie, size_t size, void *cpu_addr,
			   dma_addr_t *dma_handle);
};

struct apple_rtkit;

#if CONFIG_APPLE_RTKIT

/*
 * Initializes the internal state required to handle RTKit. This
 * should usually be called within _probe.
 *
 * @dev: Pointer to the device node this coprocessor is assocated with
 * @cookie: opaque cookie passed to all functions defined in rtkit_ops
 * @resource: resource containing the CPU_CONTROL register
 * @mbox_name: mailbox name used to communicate with the co-processor
 * @mbox_idx: mailbox index to be used if mbox_name is NULL
 * @ops: pointer to rtkit_ops to be used for this co-processor
 */
struct apple_rtkit *apple_rtkit_init(struct device *dev, void *cookie,
				     struct resource *res,
				     const char *mbox_name,
				     int mbox_idx,
				     const struct apple_rtkit_ops *ops);

void apple_rtkit_free(struct apple_rtkit *rtk);

/*
 * Turns on the co-processor and initialize the RTKit system endpoints.
 * Has to be called before any messages can be sent or recevied and will return
 * immediately. apple_rtkit_boot can be used afterwards to wait for the boot
 * process to complete.
 */
int apple_rtkit_boot(struct apple_rtkit *rtk);

/*
 * Same as rtkit_boot but waits until the processor has booted successfully.
 * Can be called after apple_rtkit_boot to wait for the boot process to finish.
 */
int apple_rtkit_boot_wait(struct apple_rtkit *rtk, unsigned long timeout);

/*
 * Puts the co-processor into hibernation mode.
 * The processor loses almost all state and cannot be used anymore after this
 * call. All shared memory buffers will be freed.
 */
int apple_rtkit_hibernate(struct apple_rtkit *rtk);

/*
 * Starts an endpoint. Must be called after boot but before any messages can be
 * sent or received from that endpoint.
 */
int apple_rtkit_start_ep(struct apple_rtkit *rtk, u8 endpoint);

/*
 * Send a message to the given endpoint.
 */
int apple_rtkit_send_message(struct apple_rtkit *rtk, u8 ep, u64 message);

#else

static inline struct apple_rtkit *
apple_rtkit_init(struct device *dev, void *cookie, struct resource *res,
		 const char *mbox_name, const struct apple_rtkit_ops *ops)
{
	return ERR_PTR(-ENODEV);
}

static inline void apple_rtkit_free(struct apple_rtkit *rtk)
{
}

static inline int apple_rtkit_boot(struct apple_rtkit *rtk)
{
	return -ENODEV;
}

static inline int apple_rtkit_boot_wait(struct apple_rtkit *rtk)
{
	return -ENODEV;
}

static inline int apple_rtkit_hibernate(struct apple_rtkit *rtk)
{
	return -ENODEV;
}

static inline int apple_rtkit_start_ep(struct apple_rtkit *rtk, u8 endpoint)
{
	return -ENODEV;
}

static inline int apple_rtkit_send_message(struct apple_rtkit *rtk, u8 ep,
					   u64 message)
{
	return -ENODEV;
}

#endif

#endif

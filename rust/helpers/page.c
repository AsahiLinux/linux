// SPDX-License-Identifier: GPL-2.0

#include <asm/io.h>
#include <linux/gfp.h>
#include <linux/highmem.h>
#include <linux/mm.h>

__rust_helper struct page *rust_helper_alloc_pages(gfp_t gfp_mask,
						   unsigned int order)
{
	return alloc_pages(gfp_mask, order);
}

__rust_helper void *rust_helper_kmap_local_page(struct page *page)
{
	return kmap_local_page(page);
}

__rust_helper void rust_helper_kunmap_local(const void *addr)
{
	kunmap_local(addr);
}

#ifndef NODE_NOT_IN_PAGE_FLAGS
__rust_helper int rust_helper_page_to_nid(const struct page *page)
{
	return page_to_nid(page);
}
#endif

struct page *rust_helper_phys_to_page(phys_addr_t phys)
{
	return phys_to_page(phys);
}

phys_addr_t rust_helper_page_to_phys(struct page *page)
{
	return page_to_phys(page);
}

unsigned long rust_helper_phys_to_pfn(phys_addr_t phys)
{
	return __phys_to_pfn(phys);
}

struct page *rust_helper_pfn_to_page(unsigned long pfn)
{
	return pfn_to_page(pfn);
}

bool rust_helper_pfn_valid(unsigned long pfn)
{
	return pfn_valid(pfn);
}

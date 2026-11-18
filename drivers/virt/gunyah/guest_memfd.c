// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define pr_fmt(fmt) "gunyah_guest_mem: " fmt

#include <linux/anon_inodes.h>
#include <linux/types.h>
#include <linux/falloc.h>
#include <linux/file.h>
#include <linux/migrate.h>
#include <linux/pagemap.h>
#include <linux/set_memory.h>

#include <uapi/linux/gunyah.h>

#include "vm_mgr.h"

/**
 * struct gunyah_gmem_inode - Per-inode state shared by all guestmem bindings
 * @flags: Guest-memory creation flags
 * @bindings: List of linked &struct gunyah_gmem_binding objects
 */
struct gunyah_gmem_inode {
	unsigned long flags;
	struct list_head bindings;
};

static inline struct gunyah_gmem_inode *
gunyah_gmem_inode(struct inode *inode)
{
	return inode->i_private;
}

struct gunyah_gmem_binding {
	struct gunyah_vm_binding base;
	u64 gfn;
	struct gunyah_vm *ghvm;

	pgoff_t i_off;
	struct file *file;
	struct list_head i_entry;

	u32 flags;
	unsigned long nr;
};

static inline pgoff_t gunyah_gfn_to_off(struct gunyah_gmem_binding *b, u64 gfn)
{
	return gfn - b->gfn + b->i_off;
}

static inline u64 gunyah_off_to_gfn(struct gunyah_gmem_binding *b, pgoff_t off)
{
	return off - b->i_off + b->gfn;
}

static inline bool gunyah_guest_mem_is_lend(struct gunyah_vm *ghvm, u32 flags)
{
	u8 access = flags & GUNYAH_MEM_ACCESS_MASK;

	if (access == GUNYAH_MEM_FORCE_LEND)
		return true;
	else if (access == GUNYAH_MEM_FORCE_SHARE)
		return false;

	/* RM requires all VMs to be protected (isolated) */
	return true;
}

static struct folio *gunyah_gmem_get_huge_folio(struct inode *inode,
						pgoff_t index)
{
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	unsigned long huge_index = round_down(index, HPAGE_PMD_NR);
	unsigned long flags = (unsigned long)inode->i_private;
	struct address_space *mapping = inode->i_mapping;
	gfp_t gfp = mapping_gfp_mask(mapping);
	struct folio *folio;

	if (!(flags & GHMF_ALLOW_HUGEPAGE))
		return NULL;

	if (filemap_range_has_page(mapping, huge_index << PAGE_SHIFT,
				   (huge_index + HPAGE_PMD_NR - 1)
					   << PAGE_SHIFT))
		return NULL;

	folio = filemap_alloc_folio(gfp, HPAGE_PMD_ORDER, NULL);
	if (!folio)
		return NULL;

	if (filemap_add_folio(mapping, folio, huge_index, gfp)) {
		folio_put(folio);
		return NULL;
	}

	return folio;
#else
	return NULL;
#endif
}

static struct folio *gunyah_gmem_get_folio(struct inode *inode, pgoff_t index)
{
	struct folio *folio;

	folio = gunyah_gmem_get_huge_folio(inode, index);
	if (!folio) {
		folio = filemap_grab_folio(inode->i_mapping, index);
		if (IS_ERR_OR_NULL(folio))
			return NULL;
	}

	/*
	 * Use the up-to-date flag to track whether or not the memory has been
	 * zeroed before being handed off to the guest.  There is no backing
	 * storage for the memory, so the folio will remain up-to-date until
	 * it's removed.
	 */
	if (!folio_test_uptodate(folio)) {
		unsigned long nr_pages = folio_nr_pages(folio);
		unsigned long i;

		for (i = 0; i < nr_pages; i++)
			clear_highpage(folio_page(folio, i));

		folio_mark_uptodate(folio);
	}

	/*
	 * Ignore accessed, referenced, and dirty flags.  The memory is
	 * unevictable and there is no storage to write back to.
	 */
	return folio;
}

/**
 * gunyah_gmem_launder_folio() - Tries to unmap one folio from virtual machine(s)
 * @folio: The folio to unmap
 *
 * Returns - 0 if the folio has been reclaimed from any virtual machine(s) that
 *           folio was mapped into.
 */
static int gunyah_gmem_launder_folio(struct folio *folio)
{
	struct address_space *const mapping = folio->mapping;
	struct gunyah_gmem_binding *b;
	pgoff_t index = folio->index;
	int ret = 0;
	u64 gfn;

	filemap_invalidate_lock_shared(mapping);
	list_for_each_entry(b, &gunyah_gmem_inode(mapping->host)->bindings,
			    i_entry) {
		/* if the mapping doesn't cover this folio: skip */
		if (index < b->i_off || index >= b->i_off + b->nr)
			continue;

		gfn = gunyah_off_to_gfn(b, index);
		ret = gunyah_vm_reclaim_folio(b->ghvm, gfn, folio);
		if (WARN_RATELIMIT(ret, "failed to reclaim gfn: %08llx %d\n",
				   gfn, ret))
			break;
	}
	filemap_invalidate_unlock_shared(mapping);

	return ret;
}

static vm_fault_t gunyah_gmem_host_fault(struct vm_fault *vmf)
{
	struct folio *folio;

	folio = gunyah_gmem_get_folio(file_inode(vmf->vma->vm_file),
				      vmf->pgoff);
	if (!folio)
		return VM_FAULT_SIGBUS;

	/* If the folio is lent to a VM, try to reclaim it */
	if (folio_test_private(folio) && gunyah_gmem_launder_folio(folio)) {
		folio_unlock(folio);
		folio_put(folio);
		return VM_FAULT_SIGBUS;
	}
	/* gunyah_gmem_launder_folio should clear the private bit if it returns 0 */
	if (WARN_ON_ONCE(folio_test_private(folio))) {
		folio_unlock(folio);
		folio_put(folio);
		return VM_FAULT_SIGBUS;
	}

	vmf->page = folio_file_page(folio, vmf->pgoff);

	return VM_FAULT_LOCKED;
}

static const struct vm_operations_struct gunyah_gmem_vm_ops = {
	.fault = gunyah_gmem_host_fault,
};

static int gunyah_gmem_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct address_space *const mapping = file->f_mapping;
	struct gunyah_gmem_binding *b;
	pgoff_t end_off, start_off, binding_end;
	int ret = 0;
	u64 gfn, nr;

	/* No support for private mappings to avoid COW.  */
	if ((vma->vm_flags & (VM_SHARED | VM_MAYSHARE)) !=
	    (VM_SHARED | VM_MAYSHARE)) {
		return -EINVAL;
	}

	filemap_invalidate_lock_shared(mapping);
	/**
	 * userspace can only mmap if the folios covered by the requested
	 * offset are not lent to the guest
	 */
	list_for_each_entry(b, &gunyah_gmem_inode(mapping->host)->bindings,
			    i_entry) {
		if (!gunyah_guest_mem_is_lend(b->ghvm, b->flags))
			continue;

		/* Reclaim only the intersection with this binding. */
		binding_end = b->i_off + b->nr;
		start_off = max(vma->vm_pgoff, b->i_off);
		end_off = min(vma->vm_pgoff + vma_pages(vma), binding_end);
		if (start_off >= end_off)
			continue;

		gfn = gunyah_off_to_gfn(b, start_off);
		nr = end_off - start_off;
		ret = gunyah_vm_reclaim_range(b->ghvm, gfn, nr - 1);
		if (ret)
			break;
	}
	filemap_invalidate_unlock_shared(mapping);

	if (!ret) {
		file_accessed(file);
		vma->vm_ops = &gunyah_gmem_vm_ops;
	}

	return ret;
}

/**
 * gunyah_gmem_punch_hole() - try to reclaim a range of pages
 * @inode: guest memfd inode
 * @offset: Offset into memfd to start reclaim
 * @len: length to reclaim
 *
 * Will try to unmap from virtual machines any folios covered by
 * [offset, offset+len]. If unmapped, then tries to free those folios
 *
 * Returns - error code if any folio in the range couldn't be freed.
 */
static long gunyah_gmem_punch_hole(struct inode *inode, loff_t offset,
				   loff_t len)
{
	pgoff_t start = offset >> PAGE_SHIFT;
	pgoff_t end = (offset + len - 1) >> PAGE_SHIFT;

	return invalidate_inode_pages2_range(inode->i_mapping, start, end);
}

static long gunyah_gmem_allocate(struct inode *inode, loff_t offset, loff_t len)
{
	struct address_space *mapping = inode->i_mapping;
	pgoff_t start, index, end;
	int r;

	filemap_invalidate_lock_shared(mapping);

	start = offset >> PAGE_SHIFT;
	end = (offset + len) >> PAGE_SHIFT;

	r = 0;
	for (index = start; index < end;) {
		struct folio *folio;

		if (signal_pending(current)) {
			r = -EINTR;
			break;
		}

		folio = gunyah_gmem_get_folio(inode, index);
		if (!folio) {
			r = -ENOMEM;
			break;
		}

		index = folio_next_index(folio);

		folio_unlock(folio);
		folio_put(folio);

		/* 64-bit only, wrapping the index should be impossible. */
		if (WARN_ON_ONCE(!index))
			break;

		cond_resched();
	}

	filemap_invalidate_unlock_shared(mapping);

	return r;
}

static long gunyah_gmem_fallocate(struct file *file, int mode, loff_t offset,
				  loff_t len)
{
	loff_t end;
	long ret;

	if (!(mode & FALLOC_FL_KEEP_SIZE))
		return -EOPNOTSUPP;

	if (mode & ~(FALLOC_FL_KEEP_SIZE | FALLOC_FL_PUNCH_HOLE |
		     FALLOC_FL_ZERO_RANGE))
		return -EOPNOTSUPP;

	if (!len || !PAGE_ALIGNED(offset) || !PAGE_ALIGNED(len) ||
	    check_add_overflow(offset, len, &end) ||
	    end > i_size_read(file_inode(file)))
		return -EINVAL;

	if (mode & FALLOC_FL_PUNCH_HOLE)
		ret = gunyah_gmem_punch_hole(file_inode(file), offset, len);
	else
		ret = gunyah_gmem_allocate(file_inode(file), offset, len);

	if (!ret)
		file_modified(file);
	return ret;
}

static int gunyah_gmem_release(struct inode *inode, struct file *file)
{
	struct gunyah_gmem_inode *gmi = gunyah_gmem_inode(inode);

	if (WARN_ON_ONCE(!list_empty(&gmi->bindings)))
		return 0;

	kfree(gmi);
	inode->i_private = NULL;

	return 0;
}

static const struct file_operations gunyah_gmem_fops = {
	.owner = THIS_MODULE,
	.llseek = generic_file_llseek,
	.mmap = gunyah_gmem_mmap,
	.open = generic_file_open,
	.fallocate = gunyah_gmem_fallocate,
	.release = gunyah_gmem_release,
};

static bool gunyah_gmem_release_folio(struct folio *folio, gfp_t gfp_flags)
{
	/* should return true if released; launder folio returns 0 if freed */
	return !gunyah_gmem_launder_folio(folio);
}

static int gunyah_gmem_remove_folio(struct address_space *mapping,
				    struct folio *folio)
{
	if (mapping != folio->mapping)
		return -EINVAL;

	return gunyah_gmem_launder_folio(folio);
}

static const struct address_space_operations gunyah_gmem_aops = {
	.dirty_folio = noop_dirty_folio,
	.release_folio = gunyah_gmem_release_folio,
	.launder_folio = gunyah_gmem_launder_folio,
	.error_remove_folio = gunyah_gmem_remove_folio,
};

int gunyah_guest_mem_create(struct gunyah_create_mem_args *args)
{
	const char *anon_name = "[gh-gmem]";
	unsigned long fd_flags = 0;
	struct gunyah_gmem_inode *gmi;
	struct inode *inode;
	struct file *file;
	int fd, err;

	if (!args->size || !PAGE_ALIGNED(args->size) ||
	    !IS_ALIGNED(args->size, PAGE_SIZE))
		return -EINVAL;

	if (args->flags & ~(GHMF_CLOEXEC | GHMF_ALLOW_HUGEPAGE))
		return -EINVAL;

	if (memchr_inv(args->reserved, 0, sizeof(args->reserved)))
		return -EINVAL;

	if (args->flags & GHMF_CLOEXEC)
		fd_flags |= O_CLOEXEC;

	fd = get_unused_fd_flags(fd_flags);
	if (fd < 0)
		return fd;

	file = anon_inode_create_getfile(anon_name, &gunyah_gmem_fops,
					 NULL, O_RDWR, NULL);
	if (IS_ERR(file)) {
		err = PTR_ERR(file);
		goto err_fd;
	}

	file->f_flags |= O_LARGEFILE;

	inode = file->f_inode;

	WARN_ON(file->f_mapping != inode->i_mapping);

	gmi = kzalloc_obj(*gmi);
	if (!gmi) {
		err = -ENOMEM;
		goto err_file;
	}

	gmi->flags = args->flags;
	INIT_LIST_HEAD(&gmi->bindings);

	inode->i_private = gmi;
	inode->i_mapping->a_ops = &gunyah_gmem_aops;
	inode->i_mode |= S_IFREG;
	inode->i_size = args->size;

	mapping_set_gfp_mask(inode->i_mapping, GFP_HIGHUSER);
	mapping_set_large_folios(inode->i_mapping);
	mapping_set_unevictable(inode->i_mapping);
	mapping_set_release_always(inode->i_mapping);

	WARN_ON_ONCE(!mapping_unevictable(inode->i_mapping));

	fd_install(fd, file);

	return fd;

err_file:
	fput(file);

err_fd:
	put_unused_fd(fd);

	return err;
}

void gunyah_gmem_remove_binding(struct gunyah_gmem_binding *b)
{
	struct gunyah_vm *ghvm = b->ghvm;

	mtree_erase(&ghvm->bindings, b->gfn);
	list_del(&b->i_entry);
	fput(b->file);
	kfree(b);
}

static inline unsigned long gunyah_gmem_page_mask(struct file *file)
{
	struct gunyah_gmem_inode *gmi = gunyah_gmem_inode(file_inode(file));

	if (gmi->flags & GHMF_ALLOW_HUGEPAGE) {
#if IS_ENABLED(CONFIG_TRANSPARENT_HUGEPAGE)
		return HPAGE_PMD_MASK;
#else
		return ULONG_MAX;
#endif
	}

	return PAGE_MASK;
}

static int gunyah_gmem_init_binding(struct gunyah_vm *ghvm, struct file *file,
				    struct gunyah_map_mem_args *args,
				    struct gunyah_gmem_binding *binding)
{
	const unsigned long page_mask = ~gunyah_gmem_page_mask(file);
	u64 end;

	if (args->flags & ~(GUNYAH_MEM_ALLOW_RWX | GUNYAH_MEM_ACCESS_MASK))
		return -EINVAL;

	if (!args->size || (args->guest_addr & page_mask) ||
	    (args->offset & page_mask) || (args->size & page_mask))
		return -EINVAL;

	if (check_add_overflow(args->offset, args->size, &end) ||
	    end > i_size_read(file_inode(file)))
		return -EOVERFLOW;

	if (check_add_overflow(args->guest_addr, args->size, &end))
		return -EOVERFLOW;

	binding->gfn = gunyah_gpa_to_gfn(args->guest_addr);
	binding->ghvm = ghvm;
	binding->i_off = args->offset >> PAGE_SHIFT;
	binding->file = file;
	binding->flags = args->flags;
	binding->nr = args->size >> PAGE_SHIFT;

	binding->base.mem_type = VM_MEM_GUEST_MEM;
	binding->base.gmem = binding;
	binding->base.share_type = gunyah_guest_mem_is_lend(ghvm, args->flags) ?
					   VM_MEM_LEND : VM_MEM_SHARE;
	binding->base.guest_phys_addr = args->guest_addr;
	binding->base.size = args->size;
	binding->base.flags = args->flags;
	binding->base.label = 0;

	return 0;
}

static int gunyah_gmem_trim_binding(struct gunyah_gmem_binding *b,
				    unsigned long start_delta,
				    unsigned long remove_pages,
				    unsigned long end_delta)
{
	struct gunyah_gmem_binding *tail = NULL;
	struct gunyah_vm *ghvm = b->ghvm;
	u64 old_gfn = b->gfn;
	unsigned long old_nr = b->nr;
	int ret;

	ret = gunyah_vm_reclaim_range(ghvm, b->gfn + start_delta,
				      remove_pages - 1);
	if (ret)
		return ret;

	down_write(&ghvm->bindings_lock);
	if (!start_delta && !end_delta) {
		gunyah_gmem_remove_binding(b);
		goto unlock;
	}

	mtree_erase(&ghvm->bindings, old_gfn);
	if (start_delta && end_delta) {
		tail = kzalloc_obj(*tail);
		if (!tail) {
			ret = -ENOMEM;
			goto restore;
		}
		*tail = *b;
		tail->file = get_file(b->file);
		tail->gfn = old_gfn + start_delta + remove_pages;
		tail->i_off = b->i_off + start_delta + remove_pages;
		tail->nr = end_delta;
		tail->base.gmem = tail;
		tail->base.guest_phys_addr = gunyah_gfn_to_gpa(tail->gfn);
		tail->base.size = gunyah_gfn_to_gpa(tail->nr);
		tail->base.vm_parcel = NULL;
		list_add(&tail->i_entry,
			 &gunyah_gmem_inode(file_inode(tail->file))->bindings);

		b->nr = start_delta;
		ret = mtree_insert_range(&ghvm->bindings, tail->gfn,
					 tail->gfn + tail->nr - 1, &tail->base,
					 GFP_KERNEL);
		if (ret) {
			list_del(&tail->i_entry);
			fput(tail->file);
			kfree(tail);
			goto restore;
		}
	} else if (start_delta) {
		b->nr = start_delta;
	} else {
		b->gfn += old_nr - end_delta;
		b->i_off += old_nr - end_delta;
		b->nr = end_delta;
	}
	b->base.guest_phys_addr = gunyah_gfn_to_gpa(b->gfn);
	b->base.size = gunyah_gfn_to_gpa(b->nr);
	b->base.vm_parcel = NULL;

	ret = mtree_insert_range(&ghvm->bindings, b->gfn,
				 b->gfn + b->nr - 1, &b->base, GFP_KERNEL);
	if (ret) {
		if (tail) {
			mtree_erase(&ghvm->bindings, tail->gfn);
			list_del(&tail->i_entry);
			fput(tail->file);
			kfree(tail);
		}
		goto restore;
	}
	goto unlock;

restore:
	b->gfn = old_gfn;
	b->nr = old_nr;
	b->base.guest_phys_addr = gunyah_gfn_to_gpa(b->gfn);
	b->base.size = gunyah_gfn_to_gpa(b->nr);
	WARN_ON(mtree_insert_range(&ghvm->bindings, b->gfn,
				   b->gfn + b->nr - 1, &b->base, GFP_KERNEL));
unlock:
	up_write(&ghvm->bindings_lock);
	return ret;
}

static int gunyah_gmem_remove_mapping(struct gunyah_vm *ghvm, struct file *file,
				      struct gunyah_map_mem_args *args)
{
	struct inode *inode = file_inode(file);
	struct gunyah_gmem_binding *b = NULL;
	struct gunyah_gmem_binding remove = {};
	unsigned long start_delta, end_delta;
	int ret;

	ret = gunyah_gmem_init_binding(ghvm, file, args, &remove);
	if (ret)
		return ret;

	ret = -ENOENT;
	filemap_invalidate_lock(inode->i_mapping);
	list_for_each_entry(b, &gunyah_gmem_inode(inode)->bindings, i_entry) {
		u64 remove_end, binding_end;

		if (b->ghvm != ghvm || b->file != file ||
		    b->base.share_type != remove.base.share_type ||
		    b->flags != remove.flags)
			continue;
		if (check_add_overflow(remove.gfn, (u64)remove.nr, &remove_end) ||
		    check_add_overflow(b->gfn, (u64)b->nr, &binding_end))
			break;
		if (b->gfn > remove.gfn || binding_end < remove_end)
			continue;

		start_delta = remove.gfn - b->gfn;
		if (remove.i_off - b->i_off != start_delta)
			break;
		end_delta = binding_end - remove_end;
		ret = gunyah_gmem_trim_binding(b, start_delta, remove.nr,
					       end_delta);
		break;
	}

	filemap_invalidate_unlock(inode->i_mapping);
	return ret;
}

static bool gunyah_gmem_binding_overlaps(struct gunyah_gmem_binding *a,
					 struct gunyah_gmem_binding *b)
{
	if (WARN_ON_ONCE(a->file != b->file))
		return true;
	if (a->i_off + a->nr <= b->i_off)
		return false;
	if (a->i_off >= b->i_off + b->nr)
		return false;
	return true;
}

static int gunyah_gmem_add_mapping(struct gunyah_vm *ghvm, struct file *file,
				   struct gunyah_map_mem_args *args)
{
	struct gunyah_gmem_binding *b, *tmp = NULL;
	struct inode *inode = file_inode(file);
	int ret;

	b = kzalloc_obj(*b);
	if (!b)
		return -ENOMEM;

	ret = gunyah_gmem_init_binding(ghvm, file, args, b);
	if (ret)
		goto free_binding;

	if (gunyah_guest_mem_is_lend(ghvm, b->flags) && !can_set_direct_map()) {
		pr_warn_once("Cannot lend guest_memfd memory without rodata=full\n");
		ret = -EINVAL;
		goto free_binding;
	}

	INIT_LIST_HEAD(&b->i_entry);
	filemap_invalidate_lock(inode->i_mapping);
	list_for_each_entry(tmp, &gunyah_gmem_inode(inode)->bindings, i_entry) {
		if (gunyah_gmem_binding_overlaps(b, tmp)) {
			ret = -EEXIST;
			goto unlock;
		}
	}

	down_write(&ghvm->bindings_lock);
	ret = mtree_insert_range(&ghvm->bindings, b->gfn,
				 b->gfn + b->nr - 1, &b->base, GFP_KERNEL);
	if (!ret)
		list_add(&b->i_entry, &gunyah_gmem_inode(inode)->bindings);
	up_write(&ghvm->bindings_lock);
	if (ret)
		goto unlock;

	filemap_invalidate_unlock(inode->i_mapping);
	return 0;

unlock:
	filemap_invalidate_unlock(inode->i_mapping);
free_binding:
	kfree(b);
	return ret;
}

int gunyah_gmem_modify_mapping(struct gunyah_vm *ghvm,
			       struct gunyah_map_mem_args *args)
{
	u8 access = args->flags & GUNYAH_MEM_ACCESS_MASK;
	struct file *file;
	int ret = -EINVAL;

	if (current->mm != ghvm->mm_s)
		return -EPERM;

	guard(rwsem_read)(&ghvm->status_lock);
	if (ghvm->vm_status != GUNYAH_RM_VM_STATUS_NO_STATE)
		return -EBUSY;

	file = fget(args->guest_mem_fd);
	if (!file)
		return -EBADF;

	if (file->f_op != &gunyah_gmem_fops)
		goto err_file;

	if (args->flags & ~(GUNYAH_MEM_ALLOW_RWX | GUNYAH_MEM_UNMAP |
			     GUNYAH_MEM_ACCESS_MASK))
		goto err_file;

	if (!(args->flags & GUNYAH_MEM_ALLOW_RWX))
		goto err_file;

	if (access != GUNYAH_MEM_DEFAULT_ACCESS &&
	    access != GUNYAH_MEM_FORCE_LEND &&
	    access != GUNYAH_MEM_FORCE_SHARE)
		goto err_file;

	if (!PAGE_ALIGNED(args->guest_addr) || !PAGE_ALIGNED(args->offset) ||
	    !PAGE_ALIGNED(args->size))
		goto err_file;

	if (args->flags & GUNYAH_MEM_UNMAP) {
		args->flags &= ~GUNYAH_MEM_UNMAP;
		ret = gunyah_gmem_remove_mapping(ghvm, file, args);
		fput(file);
		return ret;
	}

	ret = gunyah_gmem_add_mapping(ghvm, file, args);
err_file:
	if (ret)
		fput(file);
	return ret;
}

int gunyah_gmem_share_parcel(struct gunyah_vm *ghvm,
			     struct gunyah_vm_parcel *vm_parcel,
			     struct gunyah_vm_binding *binding,
			     u64 *gfn, u64 *nr)
{
	struct gunyah_rm_mem_parcel *parcel = &vm_parcel->parcel;
	struct gunyah_gmem_binding *b = binding->gmem;
	struct folio *folio;
	unsigned long nr_entries, i, j, start, end;
	bool lend;
	int ret;

	if (!*nr || !b || b->base.vm_parcel)
		return -EINVAL;

	if (*gfn < b->gfn)
		return -EINVAL;
	start = *gfn - b->gfn;
	if (start >= b->nr || *nr > b->nr || start > b->nr - *nr)
		return -EINVAL;

	parcel->mem_handle = GUNYAH_MEM_HANDLE_INVAL;
	end = start + *nr;
	lend = gunyah_guest_mem_is_lend(ghvm, b->flags);

	nr_entries = 0;
	for (i = start + b->i_off; i < end + b->i_off;) {
		folio = gunyah_gmem_get_folio(file_inode(b->file), i);
		if (!folio) {
			ret = -ENOMEM;
			goto out_unlocked;
		}

		if (lend) {
			if (!gunyah_folio_lend_safe(folio)) {
				folio_unlock(folio);
				folio_put(folio);
				ret = -EPERM;
				goto out_unlocked;
			}
			folio_set_private(folio);
		}

		nr_entries++;
		i = folio_next_index(folio);
	}
	end = i - b->i_off;

	parcel->mem_entries =
		kcalloc(nr_entries, sizeof(*parcel->mem_entries),
			GFP_KERNEL_ACCOUNT);
	if (!parcel->mem_entries) {
		ret = -ENOMEM;
		goto out;
	}

	j = 0;
	for (i = start + b->i_off; i < end + b->i_off; j++) {
		folio = filemap_get_folio(file_inode(b->file)->i_mapping, i);
		if (WARN_ON(IS_ERR(folio))) {
			ret = PTR_ERR(folio);
			i = end + b->i_off;
			goto out;
		}

		parcel->mem_entries[j].size = cpu_to_le64(folio_size(folio));
		parcel->mem_entries[j].phys_addr =
			cpu_to_le64(PFN_PHYS(folio_pfn(folio)));
		i = folio_next_index(folio);
		folio_put(folio);
	}
	if (WARN_ON(j != nr_entries)) {
		ret = -EINVAL;
		goto out;
	}
	parcel->n_mem_entries = nr_entries;

	folio = filemap_get_folio(file_inode(b->file)->i_mapping,
				  start + b->i_off);
	if (IS_ERR(folio)) {
		ret = PTR_ERR(folio);
		goto free_entries;
	}
	*gfn = folio->index - b->i_off + b->gfn;
	*nr = end - (folio->index - b->i_off);
	folio_put(folio);

	ret = gunyah_rm_mem_share(ghvm->rm, parcel);
	if (ret)
		goto out;

	vm_parcel->start = *gfn;
	vm_parcel->pages = *nr;
	binding->vm_parcel = vm_parcel;
	goto out;

free_entries:
	kfree(parcel->mem_entries);
	parcel->mem_entries = NULL;
	parcel->n_mem_entries = 0;
out:
	for (j = start + b->i_off; j < i;) {
		folio = filemap_get_folio(file_inode(b->file)->i_mapping, j);
		if (WARN_ON(IS_ERR(folio)))
			break;
		j = folio_next_index(folio);
		folio_unlock(folio);
		if (ret) {
			if (folio_test_private(folio)) {
				gunyah_folio_host_reclaim(folio);
				folio_clear_private(folio);
			}
			folio_put(folio);
		}
		folio_put(folio);
	}
out_unlocked:
	return ret;
}

int gunyah_gmem_reclaim_parcel(struct gunyah_vm *ghvm,
			       struct gunyah_vm_parcel *vm_parcel,
			       struct gunyah_vm_binding *binding)
{
	struct gunyah_rm_mem_parcel *parcel = &vm_parcel->parcel;
	struct gunyah_rm_mem_entry *entry;
	struct folio *folio;
	u64 remaining;
	u64 i;
	int ret;

	if (!gunyah_rm_mem_reclaim_needed(parcel))
		return 0;

	ret = gunyah_rm_mem_reclaim(ghvm->rm, parcel);
	if (ret) {
		dev_err(ghvm->parent, "Failed to reclaim guest_memfd parcel: %d\n",
			ret);
		return ret;
	}

	for (i = 0; i < parcel->n_mem_entries; i++) {
		entry = &parcel->mem_entries[i];
		folio = pfn_folio(PHYS_PFN(le64_to_cpu(entry->phys_addr)));
		remaining = le64_to_cpu(entry->size);
		while (remaining) {
			if (folio_test_private(folio))
				gunyah_folio_host_reclaim(folio);
			folio_clear_private(folio);
			folio_put(folio);
			remaining -= folio_size(folio);
			folio = folio_next(folio);
		}
	}

	parcel->mem_handle = GUNYAH_MEM_HANDLE_INVAL;
	kfree(parcel->mem_entries);
	parcel->mem_entries = NULL;
	parcel->n_mem_entries = 0;
	vm_parcel->start = 0;
	vm_parcel->pages = 0;
	binding->vm_parcel = NULL;
	return 0;
}

int gunyah_gmem_demand_page(struct gunyah_vm *ghvm,
			    struct gunyah_vm_binding *binding,
			    u64 gpa, bool write)
{
	struct gunyah_gmem_binding *b = binding->gmem;
	unsigned long gfn = gunyah_gpa_to_gfn(gpa);
	struct folio *folio;
	bool retained = false;
	int ret;

	if (!b)
		return -ENOENT;
	if (gfn < b->gfn || gfn - b->gfn >= b->nr)
		return -ENOENT;
	if (write && !(binding->flags & GUNYAH_MEM_ALLOW_WRITE))
		return -EPERM;

	folio = gunyah_gmem_get_folio(file_inode(b->file),
				      gunyah_gfn_to_off(b, gfn));
	if (!folio) {
		pr_err_ratelimited("Failed to obtain memory for guest addr %016llx\n",
				   gpa);
		return -ENOMEM;
	}

	gfn = gunyah_off_to_gfn(b, folio->index);
	filemap_invalidate_lock_shared(b->file->f_mapping);
	ret = gunyah_vm_provide_folio(ghvm, folio, gfn,
				      !gunyah_guest_mem_is_lend(ghvm, b->flags),
				      !!(binding->flags & GUNYAH_MEM_ALLOW_WRITE),
				      true, &retained);
	filemap_invalidate_unlock_shared(b->file->f_mapping);
	folio_unlock(folio);
	if (!retained)
		folio_put(folio);
	if (ret && ret != -EAGAIN)
		pr_err_ratelimited("Failed to provide folio for guest addr: %016llx: %d\n",
				   gpa, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(gunyah_gmem_demand_page);

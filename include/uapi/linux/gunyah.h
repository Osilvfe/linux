/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _UAPI_LINUX_GUNYAH_H
#define _UAPI_LINUX_GUNYAH_H

/*
 * Userspace interface for /dev/gunyah - gunyah based virtual machine
 */

#include <linux/types.h>
#include <linux/ioctl.h>

#define GUNYAH_IOCTL_TYPE 'G'

/*
 * ioctls for /dev/gunyah fds:
 */
#define GUNYAH_CREATE_VM _IO(GUNYAH_IOCTL_TYPE, 0x0) /* Returns a Gunyah VM fd */

enum gunyah_mem_flags {
	GHMF_CLOEXEC		= 1UL << 0,
	GHMF_ALLOW_HUGEPAGE	= 1UL << 1,
};

/**
 * struct gunyah_create_mem_args - Description of guest memory to create
 * @flags: See &enum gunyah_mem_flags.
 * @size: Size of the guest memory file in bytes; page aligned.
 * @reserved: Reserved for future extension and must be zero.
 */
struct gunyah_create_mem_args {
	__u64 flags;
	__u64 size;
	__u64 reserved[6];
};

#define GUNYAH_CREATE_GUEST_MEM			\
	_IOW(GUNYAH_IOCTL_TYPE, 0x8, \
	     struct gunyah_create_mem_args)

/**
 * enum gunyah_device_resource_type - Resource descriptor types reported by RM
 * @GUNYAH_DEVICE_RESOURCE_MMIO: MMIO aperture
 * @GUNYAH_DEVICE_RESOURCE_IRQ: Physical interrupt
 * @GUNYAH_DEVICE_RESOURCE_IOMMU_ENDPOINT: IOMMU stream range
 * @GUNYAH_DEVICE_RESOURCE_MSI_ENDPOINT: MSI endpoint range
 * @GUNYAH_DEVICE_RESOURCE_PCIE_FUNCTION: PCIe requester function
 */
enum gunyah_device_resource_type {
	GUNYAH_DEVICE_RESOURCE_MMIO = 1,
	GUNYAH_DEVICE_RESOURCE_IRQ,
	GUNYAH_DEVICE_RESOURCE_IOMMU_ENDPOINT,
	GUNYAH_DEVICE_RESOURCE_MSI_ENDPOINT,
	GUNYAH_DEVICE_RESOURCE_PCIE_FUNCTION,
};

/**
 * struct gunyah_device_resource - RM physical device resource descriptor
 * @type: One of &enum gunyah_device_resource_type
 * @data: Type-specific data in RM wire format
 *
 * For MMIO, data[0] is size, data[1] is the low 32 bits of the physical
 * address, and data[2] is the high 32 bits.  For IRQ, data[0] is the physical
 * interrupt number.  IOMMU and MSI endpoints use data[0] as the interface
 * handle, data[1] as the first endpoint ID, and data[2] as the endpoint count.
 * PCIe functions use data[0] as the requester ID and data[1] as the root
 * complex handle.
 */
struct gunyah_device_resource {
	__u32 type;
	__u32 data[3];
};

#define GUNYAH_DEVICE_HANDLE_INVALID	0xffffffffU

/**
 * struct gunyah_device_find_args - Find an RM-owned physical device
 * @resource: Any resource descriptor identifying the device
 * @handle: Returned RM device handle
 * @flags: Reserved; must be zero
 */
struct gunyah_device_find_args {
	struct gunyah_device_resource resource;
	__u32 handle;
	__u32 flags;
};

#define GUNYAH_FIND_DEVICE	\
	_IOWR(GUNYAH_IOCTL_TYPE, 0xb, struct gunyah_device_find_args)

/**
 * enum gunyah_device_get_resources_flags - Flags for resource enumeration
 * @GUNYAH_DEVICE_GET_RESOURCES_PHYSICAL: Return resources before VM remapping
 */
enum gunyah_device_get_resources_flags {
	GUNYAH_DEVICE_GET_RESOURCES_PHYSICAL = 1UL << 0,
};

/**
 * struct gunyah_device_get_resources_args - Describe a device's resources
 * @handle: Device handle returned by &GUNYAH_FIND_DEVICE
 * @flags: See &enum gunyah_device_get_resources_flags
 * @count: On input, capacity of @resources; on output, required entries.
 *         A null @resources pointer performs a count-only query.
 * @resources: Pointer to an array of &struct gunyah_device_resource
 */
struct gunyah_device_get_resources_args {
	__u32 handle;
	__u32 flags;
	__u32 count;
	__u32 reserved;
	__u64 resources;
};

#define GUNYAH_GET_DEVICE_RESOURCES	\
	_IOWR(GUNYAH_IOCTL_TYPE, 0xc, \
	      struct gunyah_device_get_resources_args)

/**
 * enum gunyah_device_accept_flags - Accept a device offered to this VM
 * @GUNYAH_DEVICE_ACCEPT_RESET: Reset the device before attaching it
 * @GUNYAH_DEVICE_ACCEPT_BIND: Bind the device to @bus_handle
 */
enum gunyah_device_accept_flags {
	GUNYAH_DEVICE_ACCEPT_RESET = 1UL << 0,
	GUNYAH_DEVICE_ACCEPT_BIND = 1UL << 1,
};

/**
 * struct gunyah_device_accept_args - Accept a lent or donated device
 * @handle: Device handle returned by &GUNYAH_FIND_DEVICE in the borrower VM
 * @flags: See &enum gunyah_device_accept_flags
 * @bus_handle: Destination bus when &GUNYAH_DEVICE_ACCEPT_BIND is set
 * @reserved: Reserved for future extension and must be zero
 */
struct gunyah_device_accept_args {
	__u32 handle;
	__u32 flags;
	__u32 bus_handle;
	__u32 reserved;
};

#define GUNYAH_DEVICE_ACCEPT	\
	_IOW(GUNYAH_IOCTL_TYPE, 0xd, struct gunyah_device_accept_args)

/**
 * enum gunyah_device_release_flags - Release a borrowed device
 * @GUNYAH_DEVICE_RELEASE_RESET: Reset the device while releasing it
 */
enum gunyah_device_release_flags {
	GUNYAH_DEVICE_RELEASE_RESET = 1UL << 0,
};

/**
 * struct gunyah_device_release_args - Release a borrowed device
 * @handle: Device handle returned by &GUNYAH_FIND_DEVICE in the borrower VM
 * @flags: See &enum gunyah_device_release_flags
 */
struct gunyah_device_release_args {
	__u32 handle;
	__u32 flags;
};

#define GUNYAH_DEVICE_RELEASE	\
	_IOW(GUNYAH_IOCTL_TYPE, 0xe, struct gunyah_device_release_args)

/*
 * ioctls for gunyah-vm fds (returned by GUNYAH_CREATE_VM)
 */

/**
 * struct gunyah_vm_dtb_config - Set the location of the VM's devicetree blob
 * @guest_phys_addr: Address of the VM's devicetree in guest memory.
 * @size: Maximum size of the devicetree including space for overlays.
 *        Resource manager applies an overlay to the DTB and dtb_size should
 *        include room for the overlay. A page of memory is typicaly plenty.
 */
struct gunyah_vm_dtb_config {
	__u64 guest_phys_addr;
	__u64 size;
};
#define GUNYAH_VM_SET_DTB_CONFIG	_IOW(GUNYAH_IOCTL_TYPE, 0x2, struct gunyah_vm_dtb_config)

#define GUNYAH_VM_START		_IO(GUNYAH_IOCTL_TYPE, 0x3)

/**
 * enum gunyah_fn_type - Valid types of Gunyah VM functions
 * @GUNYAH_FN_VCPU: create a vCPU instance to control a vCPU
 *              &struct gunyah_fn_desc.arg is a pointer to &struct gunyah_fn_vcpu_arg
 *              Return: file descriptor to manipulate the vcpu.
 * @GUNYAH_FN_IRQFD: register eventfd to assert a Gunyah doorbell
 *               &struct gunyah_fn_desc.arg is a pointer to &struct gunyah_fn_irqfd_arg
 * @GUNYAH_FN_IOEVENTFD: register ioeventfd to trigger when VM faults on parameter
 *                   &struct gunyah_fn_desc.arg is a pointer to &struct gunyah_fn_ioeventfd_arg
 */
enum gunyah_fn_type {
	GUNYAH_FN_VCPU = 1,
	GUNYAH_FN_IRQFD,
	GUNYAH_FN_IOEVENTFD,
};

#define GUNYAH_FN_MAX_ARG_SIZE		256

/**
 * struct gunyah_fn_vcpu_arg - Arguments to create a vCPU.
 * @id: vcpu id
 *
 * Create this function with &GUNYAH_VM_ADD_FUNCTION using type &GUNYAH_FN_VCPU.
 *
 * The vcpu type will register with the VM Manager to expect to control
 * vCPU number `vcpu_id`. It returns a file descriptor allowing interaction with
 * the vCPU. See the Gunyah vCPU API description sections for interacting with
 * the Gunyah vCPU file descriptors.
 */
struct gunyah_fn_vcpu_arg {
	__u32 id;
};

/**
 * enum gunyah_irqfd_flags - flags for use in gunyah_fn_irqfd_arg
 * @GUNYAH_IRQFD_FLAGS_LEVEL: make the interrupt operate like a level triggered
 *                        interrupt on guest side. Triggering IRQFD before
 *                        guest handles the interrupt causes interrupt to
 *                        stay asserted.
 */
enum gunyah_irqfd_flags {
	GUNYAH_IRQFD_FLAGS_LEVEL		= 1UL << 0,
};

/**
 * struct gunyah_fn_irqfd_arg - Arguments to create an irqfd function.
 *
 * Create this function with &GUNYAH_VM_ADD_FUNCTION using type &GUNYAH_FN_IRQFD.
 *
 * Allows setting an eventfd to directly trigger a guest interrupt.
 * irqfd.fd specifies the file descriptor to use as the eventfd.
 * irqfd.label corresponds to the doorbell label used in the guest VM's devicetree.
 *
 * @fd: an eventfd which when written to will raise a doorbell
 * @label: Label of the doorbell created on the guest VM
 * @flags: see &enum gunyah_irqfd_flags
 * @padding: padding bytes
 */
struct gunyah_fn_irqfd_arg {
	__u32 fd;
	__u32 label;
	__u32 flags;
	__u32 padding;
};

/**
 * enum gunyah_ioeventfd_flags - flags for use in gunyah_fn_ioeventfd_arg
 * @GUNYAH_IOEVENTFD_FLAGS_DATAMATCH: the event will be signaled only if the
 *                                written value to the registered address is
 *                                equal to &struct gunyah_fn_ioeventfd_arg.datamatch
 */
enum gunyah_ioeventfd_flags {
	GUNYAH_IOEVENTFD_FLAGS_DATAMATCH	= 1UL << 0,
};

/**
 * struct gunyah_fn_ioeventfd_arg - Arguments to create an ioeventfd function
 * @datamatch: data used when GUNYAH_IOEVENTFD_DATAMATCH is set
 * @addr: Address in guest memory
 * @len: Length of access
 * @fd: When ioeventfd is matched, this eventfd is written
 * @flags: See &enum gunyah_ioeventfd_flags
 * @padding: padding bytes
 *
 * Create this function with &GUNYAH_VM_ADD_FUNCTION using type &GUNYAH_FN_IOEVENTFD.
 *
 * Attaches an ioeventfd to a legal mmio address within the guest. A guest write
 * in the registered address will signal the provided event instead of triggering
 * an exit on the GUNYAH_VCPU_RUN ioctl.
 */
struct gunyah_fn_ioeventfd_arg {
	__u64 datamatch;
	__u64 addr;        /* legal mmio address */
	__u32 len;         /* 1, 2, 4, or 8 bytes; or 0 to ignore length */
	__s32 fd;
	__u32 flags;
	__u32 padding;
};

/**
 * struct gunyah_fn_desc - Arguments to create a VM function
 * @type: Type of the function. See &enum gunyah_fn_type.
 * @arg_size: Size of argument to pass to the function. arg_size <= GUNYAH_FN_MAX_ARG_SIZE
 * @arg: Pointer to argument given to the function. See &enum gunyah_fn_type for expected
 *       arguments for a function type.
 */
struct gunyah_fn_desc {
	__u32 type;
	__u32 arg_size;
	__u64 arg;
};

#define GUNYAH_VM_ADD_FUNCTION	_IOW(GUNYAH_IOCTL_TYPE, 0x4, struct gunyah_fn_desc)
#define GUNYAH_VM_REMOVE_FUNCTION	_IOW(GUNYAH_IOCTL_TYPE, 0x7, struct gunyah_fn_desc)

/**
 * enum gunyah_device_assignment_flags - Ownership transition for a device
 * @GUNYAH_DEVICE_ASSIGN_DONATE: Donate ownership instead of lending it.  The
 *                               device is not returned to the host VM when
 *                               the target VM exits.
 * @GUNYAH_DEVICE_ASSIGN_RESET: Reset the device while transferring it
 */
enum gunyah_device_assignment_flags {
	GUNYAH_DEVICE_ASSIGN_DONATE = 1UL << 0,
	GUNYAH_DEVICE_ASSIGN_RESET = 1UL << 1,
};

/**
 * struct gunyah_device_assignment - Assign a physical device at VM start
 * @handle: Device handle returned by &GUNYAH_FIND_DEVICE
 * @flags: See &enum gunyah_device_assignment_flags
 */
struct gunyah_device_assignment {
	__u32 handle;
	__u32 flags;
};

#define GUNYAH_VM_ASSIGN_DEVICE		\
	_IOW(GUNYAH_IOCTL_TYPE, 0xb, struct gunyah_device_assignment)
#define GUNYAH_VM_UNASSIGN_DEVICE	\
	_IOW(GUNYAH_IOCTL_TYPE, 0xc, struct gunyah_device_assignment)

/**
 * enum gunyah_map_flags- Possible flags on &struct gunyah_map_mem_args
 * @GUNYAH_MEM_DEFAULT_SHARE: Use default host access for the VM type
 * @GUNYAH_MEM_FORCE_LEND: Force unmapping the memory once the guest starts to use
 * @GUNYAH_MEM_FORCE_SHARE: Allow host to continue accessing memory when guest starts to use
 * @GUNYAH_MEM_ALLOW_READ: Allow guest to read memory
 * @GUNYAH_MEM_ALLOW_WRITE: Allow guest to write to the memory
 * @GUNYAH_MEM_ALLOW_EXEC: Allow guest to execute instructions in the memory
 */
enum gunyah_map_flags {
	GUNYAH_MEM_ALLOW_READ = 1UL << 0,
	GUNYAH_MEM_ALLOW_WRITE = 1UL << 1,
	GUNYAH_MEM_ALLOW_EXEC = 1UL << 2,
	GUNYAH_MEM_ALLOW_RWX =
		(GUNYAH_MEM_ALLOW_READ | GUNYAH_MEM_ALLOW_WRITE | GUNYAH_MEM_ALLOW_EXEC),
	GUNYAH_MEM_DEFAULT_ACCESS = 0x00,
	GUNYAH_MEM_FORCE_LEND = 0x10,
	GUNYAH_MEM_FORCE_SHARE = 0x20,
#define GUNYAH_MEM_ACCESS_MASK 0x70

	GUNYAH_MEM_UNMAP = 1UL << 8,
};

/**
 * struct gunyah_map_mem_args - Description to provide guest memory into a VM
 * @guest_addr: Location in guest address space to place the memory
 * @flags: See &enum gunyah_map_flags.
 * @guest_mem_fd: File descriptor created by GUNYAH_CREATE_GUEST_MEM
 * @offset: Offset into the guest memory file
 */
struct gunyah_map_mem_args {
	__u64 guest_addr;
	__u32 flags;
	__u32 guest_mem_fd;
	__u64 offset;
	__u64 size;
};

#define GUNYAH_VM_MAP_MEM _IOW(GUNYAH_IOCTL_TYPE, 0x9, struct gunyah_map_mem_args)

enum gunyah_vm_boot_context_reg {
	REG_SET_X		= 0,
	REG_SET_PC		= 1,
	REG_SET_SP		= 2,
};

#define GUNYAH_VM_BOOT_CONTEXT_REG_SHIFT	8
#define GUNYAH_VM_BOOT_CONTEXT_REG(reg, idx) (((reg & 0xff) << GUNYAH_VM_BOOT_CONTEXT_REG_SHIFT) |\
					      (idx & 0xff))

/**
 * struct gunyah_vm_boot_context - Set an initial register for the VM
 * @reg: Register to set. See GUNYAH_VM_BOOT_CONTEXT_REG_* macros
 * @reserved: reserved for alignment
 * @value: value to fill in the register
 */
struct gunyah_vm_boot_context {
	__u32 reg;
	__u32 reserved;
	__u64 value;
};
#define GUNYAH_VM_SET_BOOT_CONTEXT	_IOW(GUNYAH_IOCTL_TYPE, 0xa, struct gunyah_vm_boot_context)

/*
 * ioctls for vCPU fds
 */

/**
 * enum gunyah_vm_status - Stores status reason why VM is not runnable (exited).
 * @GUNYAH_VM_STATUS_LOAD_FAILED: VM didn't start because it couldn't be loaded.
 * @GUNYAH_VM_STATUS_EXITED: VM requested shutdown/reboot.
 *                       Use &struct gunyah_vm_exit_info.reason for further details.
 * @GUNYAH_VM_STATUS_CRASHED: VM state is unknown and has crashed.
 */
enum gunyah_vm_status {
	GUNYAH_VM_STATUS_LOAD_FAILED	= 1,
	GUNYAH_VM_STATUS_EXITED		= 2,
	GUNYAH_VM_STATUS_CRASHED		= 3,
};

/*
 * Gunyah presently sends max 4 bytes of exit_reason.
 * If that changes, this macro can be safely increased without breaking
 * userspace so long as struct gunyah_vcpu_run < PAGE_SIZE.
 */
#define GUNYAH_VM_MAX_EXIT_REASON_SIZE	8u

/**
 * struct gunyah_vm_exit_info - Reason for VM exit as reported by Gunyah
 * See Gunyah documentation for values.
 * @type: Describes how VM exited
 * @padding: padding bytes
 * @reason_size: Number of bytes valid for `reason`
 * @reason: See Gunyah documentation for interpretation. Note: these values are
 *          not interpreted by Linux and need to be converted from little-endian
 *          as applicable.
 */
struct gunyah_vm_exit_info {
	__u16 type;
	__u16 padding;
	__u32 reason_size;
	__u8 reason[GUNYAH_VM_MAX_EXIT_REASON_SIZE];
};

/**
 * enum gunyah_vcpu_exit - Stores reason why &GUNYAH_VCPU_RUN ioctl recently exited with status 0
 * @GUNYAH_VCPU_EXIT_UNKNOWN: Not used, status != 0
 * @GUNYAH_VCPU_EXIT_MMIO: vCPU performed a read or write that could not be handled
 *                     by hypervisor or Linux. Use @struct gunyah_vcpu_run.mmio for
 *                     details of the read/write.
 * @GUNYAH_VCPU_EXIT_STATUS: vCPU not able to run because the VM has exited.
 *                       Use @struct gunyah_vcpu_run.status for why VM has exited.
 * @GUNYAH_VCPU_EXIT_PAGE_FAULT: vCPU tried to execute an instruction at an address
 *                               for which memory hasn't been provided. Use
 *                               @struct gunyah_vcpu_run.page_fault for details.
 */
enum gunyah_vcpu_exit {
	GUNYAH_VCPU_EXIT_UNKNOWN,
	GUNYAH_VCPU_EXIT_MMIO,
	GUNYAH_VCPU_EXIT_STATUS,
	GUNYAH_VCPU_EXIT_PAGE_FAULT,
};

/**
 * enum gunyah_vcpu_resume_action - Provide resume action after an MMIO or page fault
 * @GUNYAH_VCPU_RESUME_HANDLED: The mmio or page fault has been handled, continue
 *                              normal operation of vCPU
 * @GUNYAH_VCPU_RESUME_FAULT: The mmio or page fault could not be satisfied and
 *                            inject the original fault back to the guest.
 * @GUNYAH_VCPU_RESUME_RETRY: Retry the faulting instruction. Perhaps you added
 *                            memory binding to satisfy the request.
 */
enum gunyah_vcpu_resume_action {
	GUNYAH_VCPU_RESUME_HANDLED = 0,
	GUNYAH_VCPU_RESUME_FAULT,
	GUNYAH_VCPU_RESUME_RETRY,
};

/**
 * struct gunyah_vcpu_run - Application code obtains a pointer to the gunyah_vcpu_run
 *                      structure by mmap()ing a vcpu fd.
 * @immediate_exit: polled when scheduling the vcpu. If set, immediately returns -EINTR.
 * @padding: padding bytes
 * @exit_reason: Set when GUNYAH_VCPU_RUN returns successfully and gives reason why
 *               GUNYAH_VCPU_RUN has stopped running the vCPU. See &enum gunyah_vcpu_exit.
 * @mmio: Used when exit_reason == GUNYAH_VCPU_EXIT_MMIO
 *        The guest has faulted on an memory-mapped I/O that
 *        couldn't be satisfied by gunyah.
 * @mmio.phys_addr: Address guest tried to access
 * @mmio.data: the value that was written if `is_write == 1`. Filled by
 *        user for reads (`is_write == 0`).
 * @mmio.len: Length of write. Only the first `len` bytes of `data`
 *       are considered by Gunyah.
 * @mmio.is_write: 1 if VM tried to perform a write, 0 for a read
 * @mmio.resume_action: See &enum gunyah_vcpu_resume_action
 * @status: Used when exit_reason == GUNYAH_VCPU_EXIT_STATUS.
 *          The guest VM is no longer runnable. This struct informs why.
 * @status.status: See &enum gunyah_vm_status for possible values
 * @status.exit_info: Used when status == GUNYAH_VM_STATUS_EXITED
 * @page_fault: Used when EXIT_REASON == GUNYAH_VCPU_EXIT_PAGE_FAULT
 *              The guest has faulted on a region that can only be provided
 *              by mapping memory at phys_addr.
 * @page_fault.phys_addr: Address guest tried to access.
 * @page_fault.attempt: Error code why Linux wasn't able to handle fault itself
 *                      Typically, if no memory was mapped: -ENOENT,
 *                      If permission bits weren't what the VM wanted: -EPERM
 * @page_fault.resume_action: See &enum gunyah_vcpu_resume_action
 */
struct gunyah_vcpu_run {
	/* in */
	__u8 immediate_exit;
	__u8 padding[7];

	/* out */
	__u32 exit_reason;

	union {
		struct {
			__u64 phys_addr;
			__u8  data[8];
			__u32 len;
			__u8  is_write;
			__u8  resume_action;
		} mmio;

		struct {
			enum gunyah_vm_status status;
			struct gunyah_vm_exit_info exit_info;
		} status;

		struct {
			__u64 phys_addr;
			__s32 attempt;
			__u8  resume_action;
		} page_fault;
	};
};

#define GUNYAH_VCPU_RUN		_IO(GUNYAH_IOCTL_TYPE, 0x5)
#define GUNYAH_VCPU_MMAP_SIZE	_IO(GUNYAH_IOCTL_TYPE, 0x6)

/**
 * struct gunyah_userspace_memory_region - Userspace memory descripion for GH_VM_SET_USER_MEM_REGION
 * @label: Identifer to the region which is unique to the VM.
 * @flags: Flags for memory parcel behavior. See &enum gh_mem_flags.
 * @guest_phys_addr: Location of the memory region in guest's memory space (page-aligned)
 * @memory_size: Size of the region (page-aligned)
 * @userspace_addr: Location of the memory region in caller (userspace)'s memory
 *
 * See Documentation/virt/gunyah/vm-manager.rst for further details.
 */
struct gunyah_userspace_memory_region {
	__u32 label;
	__u32 flags;
	__u64 guest_phys_addr;
	__u64 memory_size;
	__u64 userspace_addr;
};

#define GH_VM_SET_USER_MEM_REGION	_IOW(GUNYAH_IOCTL_TYPE, 0x1, \
						struct gunyah_userspace_memory_region)
#define GH_ANDROID_IOCTL_TYPE		'A'

#define GH_VM_ANDROID_LEND_USER_MEM	_IOW(GH_ANDROID_IOCTL_TYPE, 0x11, \
						struct gunyah_userspace_memory_region)

struct gunyah_vm_firmware_config {
	__u64 guest_phys_addr;
	__u64 size;
};

#define GH_VM_ANDROID_SET_FW_CONFIG	_IOW(GH_ANDROID_IOCTL_TYPE, 0x12, \
						struct gunyah_vm_firmware_config)

struct gunyah_address_range {
	__u64 guest_phys_addr;
	__u64 size;
};

#define GH_VM_RECLAIM_REGION		_IOW(GH_ANDROID_IOCTL_TYPE, 0x13, \
						struct gunyah_address_range)


/**
 * Obtain an fd for the CMA backed file.
 */
#define GH_ANDROID_CREATE_CMA_MEM_FD _IO(GH_ANDROID_IOCTL_TYPE, 0x14)

/**
 * struct gunyah_map_cma_mem_args - Description to provide CMA based guest memory into a VM
 * @guest_addr: Location in guest address space to place the memory
 * @flags: See &enum gunyah_map_flags.
 * @guest_mem_fd: File descriptor created by GH_ANDROID_CREATE_CMA_MEM_FD
 * @offset: Offset into the guest memory file
 * @size: Size of the region to be mapped.
 */
struct gunyah_map_cma_mem_args {
	__u32 label;
	__u64 guest_addr;
	__u32 flags;
	__u32 guest_mem_fd;
	__u64 offset;
	__u64 size;
};

#define GH_VM_ANDROID_MAP_CMA_MEM _IOW(GH_ANDROID_IOCTL_TYPE, 0x15, struct gunyah_map_cma_mem_args)

/**
 * enum gunyah_auth_type - Valid types of authentication supported by Gunyah
 */
enum gunyah_auth_type {
	GUNYAH_QCOM_TRUSTED_VM_TYPE = 1,
};

/**
 * struct gunyah_qtvm_auth_arg - Argument to QTVM auth vm manager
 * @vm_id: The VM ID that Gunyah associates with the QTVM.
 * @peripheral_id: Also called pas_id which firmware needs for authenticating the QTVM.
 * @guest_phys_addr: Address where the VM image should be mapped to.
 * @size: Size of the VM image
 */
struct gunyah_qtvm_auth_arg {
	__u16 vm_id;
	__u32 peripheral_id;
	__u64 guest_phys_addr;
	__u64 size;
};

/**
 * struct gunyah_auth_desc - Arguments to match with an auth vm manager
 * @type: Type of the authentication mechanism. See &enum gunyah_auth_type.
 * @arg_size: Size of argument to pass to the auth vm mgr. arg_size <= GUNYAH_FN_MAX_ARG_SIZE
 * @arg: Pointer to argument given to the auth vm mgr. See &enum gunyah_auth_type for expected
 *       arguments for a function type.
 */
struct gunyah_auth_desc {
	__u32 type;
	__u32 arg_size;
	__u64 arg;
};
#define GH_VM_ANDROID_SET_AUTH_TYPE		_IOW(GH_ANDROID_IOCTL_TYPE, 0x16, \
							struct gunyah_auth_desc)

/**
 * GH_VM_CHECK_EXTENSION - Query whether the kernel supports a given capability.
 *
 * Issued on a VM fd. Pass a GH_CAP_* constant as the ioctl arg.
 * Returns 1 if the capability is supported, 0 if not.
 *
 * On kernels that do not recognise this ioctl, -ENOTTY is returned, which
 * userspace should also treat as "not supported".
 *
 * Example: ioctl(vm_fd, GH_VM_CHECK_EXTENSION, GH_CAP_VCPU_TIMER_WAKEUP)
 */
#define GH_VM_CHECK_EXTENSION		_IO(GH_ANDROID_IOCTL_TYPE, 0x17)

/*
 * Capability IDs for use with GH_VM_CHECK_EXTENSION.
 *
 * GH_CAP_VCPU_TIMER_WAKEUP: kernel handles GUNYAH_VCPU_STATE_EXPECTS_WAKEUP_OR_TIMEOUT,
 *     arming a per-vCPU hrtimer for the hypervisor-provided deadline.
 * GH_CAP_DEMAND_PAGING: the resident resource manager supports VM_SET_DEMAND_PAGING.
 * GH_CAP_DEVICE_PASSTHROUGH: the RM device manager and kernel physical-device
 *     UAPI are available.
 */
#define GH_CAP_VCPU_TIMER_WAKEUP	0
#define GH_CAP_DEMAND_PAGING		1
#define GH_CAP_DEVICE_PASSTHROUGH	2
#define GH_CAP_VM_REGISTRY		3
#define GH_CAP_VM_TIME_BASE		4
#define GH_CAP_IRQ_LENDING		5
#define GH_CAP_VM_CRASH_MSG		6
#define GH_CAP_VM_CONSOLE		7

/*
 * Host-side Resource Manager services
 *
 * These ioctls are issued on the /dev/gunyah file descriptor and reach RM
 * services that do not belong to a single VM: the VM registry, the interrupt
 * lending service and the host identity query.
 */
enum gunyah_vm_id_type {
	GUNYAH_VM_ID_URI	= 0,
	GUNYAH_VM_ID_GUID	= 1,
	GUNYAH_VM_ID_NAME	= 2,
};

/**
 * struct gunyah_vm_lookup_args - Look up a VM in the RM registry
 * @id_type: See &enum gunyah_vm_id_type
 * @key_len: Length of the key in bytes (ignored for %GUNYAH_VM_ID_GUID)
 * @key: User pointer to the key
 * @vmid: Out: first matching VM identifier
 * @flags: Reserved, must be 0
 *
 * Returns -ENOENT when no VM matches.
 */
struct gunyah_vm_lookup_args {
	__u32 id_type;
	__u32 key_len;
	__u64 key;
	__u16 vmid;
	__u16 reserved;
	__u32 flags;
};

#define GH_SERVICE_VM_LOOKUP	_IOWR(GH_ANDROID_IOCTL_TYPE, 0x20, \
				      struct gunyah_vm_lookup_args)

/**
 * struct gunyah_vm_id_args - Read a VM's RM identifier blob
 * @vmid: In: VM to query, 0 for the caller
 * @num_entries: Out: number of identifier entries in the blob
 * @size: In: capacity of @buf, out: bytes written
 * @buf: User pointer receiving the RM identifier TLV blob
 * @flags: Reserved, must be 0
 *
 * The blob is RM-defined: a sequence of
 * &struct gunyah_vm_id_entry records.
 */
struct gunyah_vm_id_args {
	__u16 vmid;
	__u16 num_entries;
	__u32 size;
	__u64 buf;
	__u32 flags;
	__u32 reserved;
};

/** struct gunyah_vm_id_entry - One entry of the identifier blob */
struct gunyah_vm_id_entry {
	__u8 type;
	__u8 reserved;
	__u16 len;
	__u8 data[];
};

#define GH_SERVICE_VM_GET_ID	_IOWR(GH_ANDROID_IOCTL_TYPE, 0x21, \
				      struct gunyah_vm_id_args)

/**
 * struct gunyah_vm_crash_msg_args - Read a VM's crash message
 * @vmid: VM to query
 * @msg_len: In: capacity of @msg, out: message length
 * @msg: User pointer receiving the crash message
 *
 * Returns -ENOSPC (with @msg_len set) when the message does not fit.
 */
struct gunyah_vm_crash_msg_args {
	__u16 vmid;
	__u16 msg_len;
	__u32 reserved;
	__u64 msg;
};

#define GH_SERVICE_VM_CRASH_MSG	_IOWR(GH_ANDROID_IOCTL_TYPE, 0x22, \
				      struct gunyah_vm_crash_msg_args)

/**
 * struct gunyah_irq_lend_args - Lend an interrupt to another VM
 * @borrower: VM that receives the interrupt
 * @virq: Interrupt number in the caller's VM
 * @label: Caller-defined label echoed back in notifications
 * @handle: Out: handle used to release or reclaim the interrupt
 * @flags: Reserved, must be 0
 */
struct gunyah_irq_lend_args {
	__u16 borrower;
	__u16 reserved;
	__u32 virq;
	__u32 label;
	__u32 handle;
	__u32 flags;
};

#define GH_SERVICE_IRQ_LEND	_IOWR(GH_ANDROID_IOCTL_TYPE, 0x23, \
				      struct gunyah_irq_lend_args)

/** struct gunyah_irq_handle_args - Address an interrupt by RM handle */
struct gunyah_irq_handle_args {
	__u32 handle;
	__u32 flags;
};

#define GH_SERVICE_IRQ_RELEASE	_IOW(GH_ANDROID_IOCTL_TYPE, 0x24, \
				     struct gunyah_irq_handle_args)
#define GH_SERVICE_IRQ_RECLAIM	_IOW(GH_ANDROID_IOCTL_TYPE, 0x25, \
				     struct gunyah_irq_handle_args)

/**
 * struct gunyah_irq_accept_args - Accept an interrupt lent to the caller
 * @handle: Handle from the corresponding lend notification
 * @virq: In: destination interrupt or %GUNYAH_VIRQ_AUTO, out: assigned number
 * @flags: Reserved, must be 0
 */
struct gunyah_irq_accept_args {
	__u32 handle;
	__u32 virq;
	__u32 flags;
	__u32 reserved;
};

#define GUNYAH_VIRQ_AUTO	0xffffffff
#define GH_SERVICE_IRQ_ACCEPT	_IOWR(GH_ANDROID_IOCTL_TYPE, 0x26, \
				      struct gunyah_irq_accept_args)

/** Interrupt lending event types reported by %GH_SERVICE_IRQ_GET_EVENT */
#define GUNYAH_IRQ_EVENT_LENT		1
#define GUNYAH_IRQ_EVENT_RELEASED	2
#define GUNYAH_IRQ_EVENT_ACCEPTED	3

/** struct gunyah_irq_event - One queued interrupt lending event */
struct gunyah_irq_event {
	__u32 type;
	__u32 handle;
	__u32 label;
	__u16 vmid;
	__u16 reserved;
	__u32 flags;
};

#define GH_SERVICE_IRQ_GET_EVENT _IOR(GH_ANDROID_IOCTL_TYPE, 0x27, \
				      struct gunyah_irq_event)

#define GH_SERVICE_HOST_GET_TYPE _IOR(GH_ANDROID_IOCTL_TYPE, 0x28, __u32)

/**
 * struct gunyah_vm_time_base_args - Set the guest's virtual RTC reference
 * @time_base: Guest time base in nanoseconds
 * @sys_timer_ref: Host system timer reference matching @time_base
 *
 * Issued on a VM fd. Before %GUNYAH_VM_START the value is remembered and
 * applied by the kernel when the VM reaches the RM INIT state; afterwards the
 * call is forwarded to RM directly and needs the VM to be in INIT or READY.
 * The RM rejects the call with -EPERM when the VM has no vRTC device.
 */
struct gunyah_vm_time_base_args {
	__u64 time_base;
	__u64 sys_timer_ref;
};

#define GH_VM_SET_TIME_BASE	_IOW(GH_ANDROID_IOCTL_TYPE, 0x30, \
				     struct gunyah_vm_time_base_args)

/**
 * struct gunyah_vm_console_write_args - Bytes written to a guest console
 * @buf: Userspace buffer holding the data
 * @len: Number of bytes in @buf
 * @flags: Must be zero
 *
 * Issued on a VM fd after %GH_VM_CONSOLE_OPEN. The kernel forwards the bytes
 * to the resource manager, which delivers them to the guest console input.
 */
struct gunyah_vm_console_write_args {
	__u64 buf;
	__u32 len;
	__u32 flags;
};

/**
 * struct gunyah_vm_console_read_args - Guest console output
 * @buf: Userspace buffer that receives the bytes
 * @len: Size of @buf in bytes
 * @flags: Must be zero
 *
 * Issued on a VM fd after %GH_VM_CONSOLE_OPEN. Returns the number of bytes
 * copied, or 0 once the console is closed. Blocks until data arrives; when
 * the fd is opened with O_NONBLOCK an empty buffer returns -EAGAIN.
 */
struct gunyah_vm_console_read_args {
	__u64 buf;
	__u32 len;
	__u32 flags;
};

/*
 * Guest console control. The console must be opened before the RM forwards
 * characters, and closed to release the console session in the RM.
 */
#define GH_VM_CONSOLE_OPEN	_IO(GH_ANDROID_IOCTL_TYPE, 0x31)
#define GH_VM_CONSOLE_CLOSE	_IO(GH_ANDROID_IOCTL_TYPE, 0x32)
#define GH_VM_CONSOLE_WRITE	_IOW(GH_ANDROID_IOCTL_TYPE, 0x33, \
				     struct gunyah_vm_console_write_args)
#define GH_VM_CONSOLE_READ	_IOWR(GH_ANDROID_IOCTL_TYPE, 0x34, \
				      struct gunyah_vm_console_read_args)
#define GH_VM_CONSOLE_FLUSH	_IO(GH_ANDROID_IOCTL_TYPE, 0x35)

#endif

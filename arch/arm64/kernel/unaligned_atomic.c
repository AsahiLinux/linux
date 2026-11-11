// SPDX-License-Identifier: GPL-2.0-only

#include <linux/compiler.h>
#include <linux/errno.h>
#include <linux/semaphore.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/uaccess.h>

#include <asm/asm-extable.h>
#include <asm/alternative-macros.h>
#include <asm/cacheflush.h>
#include <asm/exception.h>
#include <asm/ptrace.h>
#include <asm/traps.h>

#define __LOAD_ACQUIRE_RCPC(sfx, regs...) \
  ALTERNATIVE("ldar" #sfx "\t" #regs,     \
              ".arch_extension rcpc\n"    \
              "ldapr" #sfx "\t" #regs,    \
              ARM64_HAS_LDAPR)

struct fault_info {
  int error;
  u64 addr;
  u64 size;
};

#define CASPAL_MASK 0xBFE0FC00
#define CASPAL_INST 0x0860FC00

#define CASAL_MASK 0x3FE0FC00
#define CASAL_INST 0x08E0FC00

#define ATOMIC_MEM_MASK 0x3B200C00
#define ATOMIC_MEM_INST 0x38200000

#define RCPC2_MASK 0x3FE00C00
#define LDAPUR_INST 0x19400000
#define STLUR_INST 0x19000000

#define LDAXR_MASK 0x3FFFFC00

#define LDAXP_MASK 0xBFFF8000
#define LDAXP_INST 0x887F8000

#define LDAR_INST 0x08DFFC00
#define LDAPR_INST 0x38BFC000
#define STLR_INST 0x089FFC00

// Load/store register (register offset) (Rm encoded as xzr)
#define LDSTREGISTER_MASK 0b00111111111111111111110000000000
#define LDR_INST 0b00111000011111110110100000000000
#define STR_INST 0b00111000001111110110100000000000

#define LDSTUNSCALED_MASK 0b00111011111000000000110000000000
#define LDUR_INST 0b00111000010000000000000000000000
#define STUR_INST 0b00111000000000000000000000000000

#define NOP_INST 0xD503201F

// Load ops are 4 bits
// Acquire and release bits are independent on the instruction
#define ATOMIC_ADD_OP 0b0000
#define ATOMIC_CLR_OP 0b0001
#define ATOMIC_EOR_OP 0b0010
#define ATOMIC_SET_OP 0b0011
#define ATOMIC_SWAP_OP 0b1000

#define REGISTER_MASK 0b11111
#define RD_OFFSET 0
#define RN_OFFSET 5
#define RM_OFFSET 16

#define DMB (0b11010101000000110011000010111111 | 0b101100000000) // Inner shareable all

#define DMB_LD (0b11010101000000110011000010111111 | 0b110100000000) // Inner shareable load

#define GetRdReg(Instr) ((Instr >> RD_OFFSET) & REGISTER_MASK)
#define GetRnReg(Instr) ((Instr >> RN_OFFSET) & REGISTER_MASK)
#define GetRmReg(Instr) ((Instr >> RM_OFFSET) & REGISTER_MASK)
#define GetReg(GPRs, Reg) (Reg == 31 ? 0 : GPRs[Reg])
#define SetReg(GPRs, Reg, Val) do { if (Reg != 31) GPRs[Reg] = Val; } while (0)

static DEFINE_SEMAPHORE(buslock_sem, 1);

static struct fault_info LoadAcquire128(u64 Addr, u128* Result) {
  u64 OrigAddr = Addr;
  Addr = (u64)__uaccess_mask_ptr((void __user*)Addr);

  if (!access_ok((void __user*)OrigAddr, 16)) {
    return (struct fault_info) {.error = -EFAULT, .addr = OrigAddr, .size = 16};
  }

  u64 Lower;
  u64 Upper;
  int ret = 0;

  uaccess_enable_privileged();
  asm volatile("1: ldaxp %[ResultLower], %[ResultUpper], [%[Addr]]\n"
               "   clrex\n"
               "2:\n"
               _ASM_EXTABLE_UACCESS_ERR(1b, 2b, %w[ret])
               : [ResultLower] "=r"(Lower), [ResultUpper] "=r"(Upper), [ret] "+r"(ret)
               : [Addr] "r"(Addr)
               : "memory");
  uaccess_disable_privileged();

  if (ret) {
    return (struct fault_info) {.error = ret, .addr = OrigAddr, .size = 16};
  }

  *Result = (u128)Upper << 64 | Lower;
  return (struct fault_info) {0};
}

static struct fault_info Load128(u64 Addr, u128* Result) {
  u64 OrigAddr = Addr;
  Addr = (u64)__uaccess_mask_ptr((void __user*)Addr);

  if (!access_ok((void __user*)OrigAddr, 16)) {
    return (struct fault_info) {.error = -EFAULT, .addr = OrigAddr, .size = 16};
  }

  u64 Lower;
  u64 Upper;
  u32 Tmp;
  int ret = 0;

  uaccess_enable_privileged();
  asm volatile("1: ldaxp %[ResultLower], %[ResultUpper], [%[Addr]]\n"
               "   stlxp %w[Tmp], %[ResultLower], %[ResultUpper], [%[Addr]]\n"
               "   cbnz %w[Tmp], 1b\n"
               "2:\n"
               _ASM_EXTABLE_UACCESS_ERR(1b, 2b, %w[ret])
               : [Tmp] "=&r"(Tmp), [ResultLower] "=&r"(Lower), [ResultUpper] "=&r"(Upper), [ret] "+r"(ret)
               : [Addr] "r"(Addr)
               : "memory");
  uaccess_disable_privileged();

  if (ret) {
    return (struct fault_info) {.error = ret, .addr = OrigAddr, .size = 16};
  }

  *Result = (u128)Upper << 64 | Lower;
  return (struct fault_info) {0};
}

static struct fault_info StoreCAS128(u128* Expected, u128 Val, u64 Addr) {
  u64 OrigAddr = Addr;
  Addr = (u64)__uaccess_mask_ptr((void __user*)Addr);

  if (!access_ok((void __user*)OrigAddr, 16)) {
    return (struct fault_info) {.error = -EFAULT, .addr = OrigAddr, .size = 16};
  }

  // Decompose the 128-bit expected and new values into 64-bit halves
  // for use in general-purpose registers.
  u64 ExpectedLower = (u64)(*Expected);
  u64 ExpectedUpper = (u64)(*Expected >> 64);
  const u64 ValLower = (u64)Val;
  const u64 ValUpper = (u64)(Val >> 64);

  // Registers to hold the result of the store-exclusive and the value read from memory.
  u32 Tmp;
  u64 ReadLower;
  u64 ReadUpper;
  int ret = 0;

  uaccess_enable_privileged();
  asm volatile(
    "1: ldaxp %[ReadLower], %[ReadUpper], [%[Addr]]\n"
    "   cmp   %[ReadLower], %[ExpectedLower]\n"
    "   b.ne  3f\n"
    "   cmp   %[ReadUpper], %[ExpectedUpper]\n"
    "   b.ne  3f\n"
    "2: stlxp %w[Tmp], %[ValLower], %[ValUpper], [%[Addr]]\n"
    "   cbnz  %w[Tmp], 1b\n"
    "3:\n"
    _ASM_EXTABLE_UACCESS_ERR(1b, 3b, %w[ret])
    _ASM_EXTABLE_UACCESS_ERR(2b, 3b, %w[ret])
    : [Tmp] "=&r"(Tmp), [ReadLower] "=&r"(ReadLower), [ReadUpper] "=&r"(ReadUpper), [ret] "+r"(ret)
    : [Addr] "r"(Addr), [ExpectedLower] "r"(ExpectedLower), [ExpectedUpper] "r"(ExpectedUpper), [ValLower] "r"(ValLower), [ValUpper] "r"(ValUpper)
    : "memory", "cc");
  uaccess_disable_privileged();

  if (ret) {
    return (struct fault_info) {.error = ret, .addr = OrigAddr, .size = 16};
  }

  if (ReadLower != ExpectedLower || ReadUpper != ExpectedUpper) {
    u128 ActualVal = (u128)ReadUpper << 64 | ReadLower;
    *Expected = ActualVal;
    return (struct fault_info) {.error = -EAGAIN}; // CAS failed, but no fault
  }

  return (struct fault_info) {0}; // CAS succeeded
}

static struct fault_info LoadAcquire64(u64 Addr, u64* Result) {
  u64 OrigAddr = Addr;
  Addr = (u64)__uaccess_mask_ptr((void __user*)Addr);

  if (!access_ok((void __user*)OrigAddr, 8)) {
    return (struct fault_info) {.error = -EFAULT, .addr = OrigAddr, .size = 8};
  }

  int ret = 0;

  uaccess_enable_privileged();
  asm volatile("1: " __LOAD_ACQUIRE_RCPC(, %[Result], [%[Addr]]) "\n"
               "2:\n"
               _ASM_EXTABLE_UACCESS_ERR(1b, 2b, %w[ret])
               : [Result] "=r"(*Result), [ret] "+r"(ret)
               : [Addr] "r"(Addr)
               : "memory");
  uaccess_disable_privileged();

  return (struct fault_info) {.error = ret, .addr = OrigAddr, .size = 8};
}

static struct fault_info Load64(u64 Addr, u64* Result) {
  u64 OrigAddr = Addr;
  Addr = (u64)__uaccess_mask_ptr((void __user*)Addr);

  if (!access_ok((void __user*)OrigAddr, 8)) {
    return (struct fault_info) {.error = -EFAULT, .addr = OrigAddr, .size = 8};
  }

  int ret = 0;

  uaccess_enable_privileged();
  asm volatile("1: ldar %[Result], [%[Addr]]\n"
               "2:\n"
               _ASM_EXTABLE_UACCESS_ERR(1b, 2b, %w[ret])
               : [Result] "=r"(*Result), [ret] "+r"(ret)
               : [Addr] "r"(Addr)
               : "memory");
  uaccess_disable_privileged();

  return (struct fault_info) {.error = ret, .addr = OrigAddr, .size = 8};
}

static struct fault_info StoreCAS64(u64* Expected, u64 Val, u64 Addr) {
  u64 OrigAddr = Addr;
  Addr = (u64)__uaccess_mask_ptr((void __user*)Addr);

  if (!access_ok((void __user*)OrigAddr, 8)) {
    return (struct fault_info) {.error = -EFAULT, .addr = OrigAddr, .size = 8};
  }

  u64 tmp, oldval;
  int ret = 0;

  uaccess_enable_privileged();
  asm volatile("1: ldxr %[oldval], [%[Addr]]\n"
               "   cmp  %[oldval], %[expected]\n"
               "   b.ne 3f\n"
               "2: stlxr %w[tmp], %[val], [%[Addr]]\n"
               "   cbnz %w[tmp], 1b\n"
               "3:\n"
               _ASM_EXTABLE_UACCESS_ERR(1b, 3b, %w[ret])
               _ASM_EXTABLE_UACCESS_ERR(2b, 3b, %w[ret])
               : [tmp] "=&r"(tmp), [oldval] "=&r"(oldval), [ret] "+r"(ret)
               : [Addr] "r"(Addr), [expected] "r"(*Expected), [val] "r"(Val)
               : "memory", "cc");
  uaccess_disable_privileged();

  if (ret) {
    return (struct fault_info) {.error = ret, .addr = OrigAddr, .size = 8};
  }

  if (oldval != *Expected) {
    *Expected = oldval;
    return (struct fault_info) {.error = -EAGAIN}; // CAS failed, but no fault
  }

  return (struct fault_info) {0}; // CAS succeeded
}

static struct fault_info LoadAcquire32(u64 Addr, u32* Result) {
  u64 OrigAddr = Addr;
  Addr = (u64)__uaccess_mask_ptr((void __user*)Addr);

  if (!access_ok((void __user*)OrigAddr, 4)) {
    return (struct fault_info) {.error = -EFAULT, .addr = OrigAddr, .size = 4};
  }

  int ret = 0;

  uaccess_enable_privileged();
  asm volatile("1: " __LOAD_ACQUIRE_RCPC(, %w[Result], [%[Addr]]) "\n"
               "2:\n"
               _ASM_EXTABLE_UACCESS_ERR(1b, 2b, %w[ret])
               : [Result] "=r"(*Result), [ret] "+r"(ret)
               : [Addr] "r"(Addr)
               : "memory");
  uaccess_disable_privileged();

  return (struct fault_info) {.error = ret, .addr = OrigAddr, .size = 4};
}

static struct fault_info Load32(u64 Addr, u32* Result) {
  u64 OrigAddr = Addr;
  Addr = (u64)__uaccess_mask_ptr((void __user*)Addr);

  if (!access_ok((void __user*)OrigAddr, 4)) {
    return (struct fault_info) {.error = -EFAULT, .addr = OrigAddr, .size = 4};
  }

  int ret = 0;

  uaccess_enable_privileged();
  asm volatile("1: ldar %w[Result], [%[Addr]]\n"
               "2:\n"
               _ASM_EXTABLE_UACCESS_ERR(1b, 2b, %w[ret])
               : [Result] "=r"(*Result), [ret] "+r"(ret)
               : [Addr] "r"(Addr)
               : "memory");
  uaccess_disable_privileged();

  return (struct fault_info) {.error = ret, .addr = OrigAddr, .size = 4};
}

static struct fault_info StoreRelease32(u32 Val, u64 Addr) {
  u64 OrigAddr = Addr;
  Addr = (u64)__uaccess_mask_ptr((void __user*)Addr);

  if (!access_ok((void __user*)OrigAddr, 4)) {
    return (struct fault_info) {.error = -EFAULT, .addr = OrigAddr, .size = 4};
  }

  int ret = 0;

  uaccess_enable_privileged();
  asm volatile("1: stlr %w[Val], [%[Addr]]\n"
               "2:\n"
               _ASM_EXTABLE_UACCESS_ERR(1b, 2b, %w[ret])
               : [ret] "+r"(ret)
               : [Val] "r"(Val), [Addr] "r"(Addr)
               : "memory");
  uaccess_disable_privileged();

  return (struct fault_info) {.error = ret, .addr = OrigAddr, .size = 4};
}

static struct fault_info StoreCAS32(u32* Expected, u32 Val, u64 Addr) {
  u64 OrigAddr = Addr;
  Addr = (u64)__uaccess_mask_ptr((void __user*)Addr);

  if (!access_ok((void __user*)OrigAddr, 4)) {
    return (struct fault_info) {.error = -EFAULT, .addr = OrigAddr, .size = 4};
  }

  u32 tmp, oldval;
  int ret = 0;

  uaccess_enable_privileged();
  asm volatile("1: ldxr %w[oldval], [%[Addr]]\n"
               "   cmp  %w[oldval], %w[expected]\n"
               "   b.ne 3f\n"
               "2: stlxr %w[tmp], %w[val], [%[Addr]]\n"
               "   cbnz %w[tmp], 1b\n"
               "3:\n"
               _ASM_EXTABLE_UACCESS_ERR(1b, 3b, %w[ret])
               _ASM_EXTABLE_UACCESS_ERR(2b, 3b, %w[ret])
               : [tmp] "=&r"(tmp), [oldval] "=&r"(oldval), [ret] "+r"(ret)
               : [Addr] "r"(Addr), [expected] "r"(*Expected), [val] "r"(Val)
               : "memory", "cc");
  uaccess_disable_privileged();

  if (ret) {
    return (struct fault_info) {.error = ret, .addr = OrigAddr, .size = 4};
  }

  if (oldval != *Expected) {
    *Expected = oldval;
    return (struct fault_info) {.error = -EAGAIN}; // CAS failed, but no fault
  }

  return (struct fault_info) {0}; // CAS succeeded
}

static struct fault_info LoadAcquire8(u64 Addr, u8* Result) {
  u64 OrigAddr = Addr;
  Addr = (u64)__uaccess_mask_ptr((void __user*)Addr);

  if (!access_ok((void __user*)OrigAddr, 1)) {
    return (struct fault_info) {.error = -EFAULT, .addr = OrigAddr, .size = 1};
  }

  int ret = 0;

  uaccess_enable_privileged();
  asm volatile("1: " __LOAD_ACQUIRE_RCPC(b, %w[Result], [%[Addr]]) "\n"
               "2:\n"
               _ASM_EXTABLE_UACCESS_ERR(1b, 2b, %w[ret])
               : [Result] "=r"(*Result), [ret] "+r"(ret)
               : [Addr] "r"(Addr)
               : "memory");
  uaccess_disable_privileged();

  return (struct fault_info) {.error = ret, .addr = OrigAddr, .size = 1};
}

static struct fault_info StoreCAS8(u8* Expected, u8 Val, u64 Addr) {
  u64 OrigAddr = Addr;
  Addr = (u64)__uaccess_mask_ptr((void __user*)Addr);

  if (!access_ok((void __user*)OrigAddr, 1)) {
    return (struct fault_info) {.error = -EFAULT, .addr = OrigAddr, .size = 1};
  }

  u32 tmp, oldval;
  int ret = 0;

  uaccess_enable_privileged();
  asm volatile("1: ldxrb %w[oldval], [%[Addr]]\n"
               "   cmp   %w[oldval], %w[expected]\n"
               "   b.ne  3f\n"
               "2: stlxrb %w[tmp], %w[val], [%[Addr]]\n"
               "   cbnz  %w[tmp], 1b\n"
               "3:\n"
               _ASM_EXTABLE_UACCESS_ERR(1b, 3b, %w[ret])
               _ASM_EXTABLE_UACCESS_ERR(2b, 3b, %w[ret])
               : [tmp] "=&r"(tmp), [oldval] "=&r"(oldval), [ret] "+r"(ret)
               : [Addr] "r"(Addr), [expected] "r"(*Expected), [val] "r"(Val)
               : "memory", "cc");
  uaccess_disable_privileged();

  if (ret) {
    return (struct fault_info) {.error = ret, .addr = OrigAddr, .size = 1};
  }

  if ((u8)oldval != *Expected) {
    *Expected = (u8)oldval;
    return (struct fault_info) {.error = -EAGAIN}; // CAS failed, but no fault
  }

  return (struct fault_info) {0}; // CAS succeeded
}

static struct fault_info DoLoad16(u64 Addr, u16* Result) {
  struct fault_info fi = {0};
  u64 AlignmentMask = 0b1111;
  if ((Addr & AlignmentMask) == 15) {
    // Address crosses over 16byte or 64byte threshold
    // Needs two loads
    u64 AddrUpper = Addr + 1;
    u8 ActualUpper = 0;
    u8 ActualLower = 0;
    // Careful ordering here
    fi = LoadAcquire8(AddrUpper, &ActualUpper);
    if (fi.error) {
      return fi;
    }
    fi = LoadAcquire8(Addr, &ActualLower);
    if (fi.error) {
      return fi;
    }

    *Result = (u16)ActualUpper << 8 | ActualLower;
    return fi;
  } else {
    AlignmentMask = 0b111;
    if ((Addr & AlignmentMask) == 7) {
      // Crosses 8byte boundary
      // Needs 128bit load
      // Fits within a 16byte region
      u64 Alignment = Addr & 0b1111;
      Addr &= ~0b1111ULL;

      u128 TmpResult;
      fi = LoadAcquire128(Addr, &TmpResult);
      if (fi.error) {
        return fi;
      }

      // Zexts the result
      *Result = TmpResult >> (Alignment * 8);
      return fi;
    } else {
      AlignmentMask = 0b11;
      if ((Addr & AlignmentMask) == 3) {
        // Crosses 4byte boundary
        // Needs 64bit Load
        u64 Alignment = Addr & AlignmentMask;
        Addr &= ~AlignmentMask;

        u64 TmpResult;
        fi = LoadAcquire64(Addr, &TmpResult);
        if (fi.error) {
          return fi;
        }

        // Zexts the result
        *Result = TmpResult >> (Alignment * 8);
        return fi;
      } else {
        // Fits within 4byte boundary
        // Only needs 32bit Load
        // Only alignment offset will be 1 here
        u64 Alignment = Addr & AlignmentMask;
        Addr &= ~AlignmentMask;

        u32 TmpResult;
        fi = LoadAcquire32(Addr, &TmpResult);
        if (fi.error) {
          return fi;
        }

        // Zexts the result
        *Result = TmpResult >> (Alignment * 8);
        return fi;
      }
    }
  }
}

static struct fault_info DoLoad32(u64 Addr, u32* Result) {
  u64 AlignmentMask = 0b1111;
  struct fault_info fi = {0};

  if ((Addr & AlignmentMask) > 12) {
    // Address crosses over 16byte threshold
    // Needs dual 32bit load
    u64 Alignment = Addr & 0b11;
    Addr &= ~0b11ULL;

    u64 AddrUpper = Addr + 4;

    // Careful ordering here
    u32 ActualUpper, ActualLower;
    fi = LoadAcquire32(AddrUpper, &ActualUpper);
    if (fi.error) {
      return fi;
    }
    fi = LoadAcquire32(Addr, &ActualLower);
    if (fi.error) {
      return fi;
    }

    u64 TmpResult = (u64)ActualUpper << 32 | ActualLower;
    *Result = TmpResult >> (Alignment * 8);
    return fi;
  } else {
    AlignmentMask = 0b111;
    if ((Addr & AlignmentMask) >= 5) {
      // Crosses 8byte boundary
      // Needs 128bit load
      // Fits within a 16byte region
      u64 Alignment = Addr & 0b1111;
      Addr &= ~0b1111ULL;

      u128 TmpResult;
      fi = LoadAcquire128(Addr, &TmpResult);
      if (fi.error) {
        return fi;
      }

      *Result = TmpResult >> (Alignment * 8);
      return fi;
    } else {
      // Fits within 8byte boundary
      // Only needs 64bit CAS
      // Alignments can be [1,5)
      u64 Alignment = Addr & AlignmentMask;
      Addr &= ~AlignmentMask;

      u64 TmpResult;
      fi = LoadAcquire64(Addr, &TmpResult);
      if (fi.error) {
        return fi;
      }

      *Result = TmpResult >> (Alignment * 8);
      return fi;
    }
  }
}

static struct fault_info DoLoad64(u64 Addr, u64* Result) {
  struct fault_info fi = {0};
  u64 AlignmentMask = 0b1111;
  if ((Addr & AlignmentMask) > 8) {
    u64 Alignment = Addr & 0b111;
    Addr &= ~0b111ULL;
    u64 AddrUpper = Addr + 8;

    // Crosses a 16byte boundary
    // Needs two 8 byte loads
    u64 ActualUpper = 0;
    u64 ActualLower = 0;
    // Careful ordering here
    fi = LoadAcquire64(AddrUpper, &ActualUpper);
    if (fi.error) {
      return fi;
    }
    fi = LoadAcquire64(Addr, &ActualLower);
    if (fi.error) {
      return fi;
    }

    u128 TmpResult = (u128)ActualUpper << 64 | ActualLower;
    *Result = TmpResult >> (Alignment * 8);
    return fi;
  } else {
    // Fits within a 16byte region
    u64 Alignment = Addr & AlignmentMask;
    Addr &= ~AlignmentMask;
    u128 TmpResult;
    fi = LoadAcquire128(Addr, &TmpResult);
    if (fi.error) {
      return fi;
    }
    *Result = TmpResult >> (Alignment * 8);
    return fi;
  }
}

static struct fault_info RunCASPAL(u64* GPRs, u32 Size, u32 DesiredReg1, u32 DesiredReg2, u32 ExpectedReg1, u32 ExpectedReg2,
                                   u32 AddressReg) {

  bool SplitLockMutexLocked = false;
  struct fault_info fi = {0};
  if (Size == 0) {
    // 32bit
    u64 Addr = GetReg(GPRs, AddressReg);

    u32 DesiredLower = GetReg(GPRs, DesiredReg1);
    u32 DesiredUpper = GetReg(GPRs, DesiredReg2);

    u32 ExpectedLower = GetReg(GPRs, ExpectedReg1);
    u32 ExpectedUpper = GetReg(GPRs, ExpectedReg2);

    // Cross-cacheline CAS doesnt work on ARM
    // It isnt even guaranteed to work on x86
    // Intel will do a "split lock" which locks the full bus
    // AMD will tear instead
    // Both cross-cacheline and cross 16byte both need dual CAS loops that can tear
    // ARMv8.4 LSE2 solves all atomic issues except cross-cacheline

    // Check for Split lock across a cacheline
    if ((Addr & 63) > 56) {
      if (test_thread_flag(TIF_UNALIGN_ATOMIC_STRICT_SPLIT_LOCKS)) {
        down(&buslock_sem);
        SplitLockMutexLocked = true;
      }
    }

    u64 AlignmentMask = 0b1111;
    if ((Addr & AlignmentMask) > 8) {
      if (test_thread_flag(TIF_UNALIGN_ATOMIC_STRICT_SPLIT_LOCKS) && !SplitLockMutexLocked) {
        down(&buslock_sem);
        SplitLockMutexLocked = true;
      }

      u64 Alignment = Addr & 0b111;
      Addr &= ~0b111ULL;
      u64 AddrUpper = Addr + 8;

      // Crosses a 16byte boundary
      // Need to do 256bit atomic, but since that doesnt exist we need to do a dual CAS loop
      u128 Mask = ~0ULL;
      Mask <<= Alignment * 8;
      u128 NegMask = ~Mask;
      u128 TmpExpected = 0;
      u128 TmpDesired = 0;

      u128 Desired = DesiredUpper;
      Desired <<= 32;
      Desired |= DesiredLower;
      Desired <<= Alignment * 8;

      u128 Expected = ExpectedUpper;
      Expected <<= 32;
      Expected |= ExpectedLower;
      Expected <<= Alignment * 8;

      while (1) {
        u64 LoadOrderUpperVal, LoadOrderLowerVal;
        fi = LoadAcquire64(AddrUpper, &LoadOrderUpperVal);
        if (fi.error) {
          goto unlock_fault;
        }
        fi = LoadAcquire64(Addr, &LoadOrderLowerVal);
        if (fi.error) {
          goto unlock_fault;
        }
        u128 LoadOrderUpper = (u128)LoadOrderUpperVal << 64;
        u128 TmpActual = LoadOrderUpper | LoadOrderLowerVal;

        // Set up expected
        TmpExpected = TmpActual;
        TmpExpected &= NegMask;
        TmpExpected |= Expected;

        // Set up desired
        TmpDesired = TmpExpected;
        TmpDesired &= NegMask;
        TmpDesired |= Desired;

        u64 TmpExpectedLower = TmpExpected;
        u64 TmpExpectedUpper = TmpExpected >> 64;

        u64 TmpDesiredLower = TmpDesired;
        u64 TmpDesiredUpper = TmpDesired >> 64;

        if (TmpExpected == TmpActual) {
          fi = StoreCAS64(&TmpExpectedUpper, TmpDesiredUpper, AddrUpper);
          if (fi.error && fi.error != -EAGAIN) {
            goto unlock_fault;
          }
          if (fi.error == 0) { // CAS succeeded
            fi = StoreCAS64(&TmpExpectedLower, TmpDesiredLower, Addr);
            if (fi.error && fi.error != -EAGAIN) {
              goto unlock_fault;
            }
            if (fi.error == 0) { // CAS succeeded
              // Stored successfully
              goto unlock_success;
            }
          }

          TmpExpected = TmpExpectedUpper;
          TmpExpected <<= 64;
          TmpExpected |= TmpExpectedLower;
        } else {
          // Mismatch up front
          TmpExpected = TmpActual;
        }

        // Not successful
        // Now we need to check the results to see if we need to try again
        u128 FailedResultOurBits = TmpExpected & Mask;
        u128 FailedResultNotOurBits = TmpExpected & NegMask;

        u128 FailedDesiredOurBits = TmpDesired & Mask;
        u128 FailedDesiredNotOurBits = TmpDesired & NegMask;
        if ((FailedResultNotOurBits ^ FailedDesiredNotOurBits) != 0) {
          // If the bits changed that werent part of our regular CAS then we need to try again
          continue;
        }
        if ((FailedResultOurBits ^ FailedDesiredOurBits) != 0) {
          // If the bits changed that we were wanting to change then we have failed and can return
          // We need to extract the bits and return them in EXPECTED
          u64 FailedResult = FailedResultOurBits >> (Alignment * 8);
          SetReg(GPRs, ExpectedReg1, FailedResult & ~0U);
          SetReg(GPRs, ExpectedReg2, FailedResult >> 32);
          goto unlock_success;
        }

        // This happens in the case that between Load and CAS that something has store our desired in to the memory location
        // This means our CAS fails because what we wanted to store was already stored
        u64 FailedResult = FailedResultOurBits >> (Alignment * 8);
        SetReg(GPRs, ExpectedReg1, FailedResult & ~0U);
        SetReg(GPRs, ExpectedReg2, FailedResult >> 32);
        goto unlock_success;
      }
    } else {
      // Fits within a 16byte region
      u64 Alignment = Addr & 0b1111;
      Addr &= ~0b1111ULL;

      u128 Mask = ~0ULL;
      Mask <<= Alignment * 8;
      u128 NegMask = ~Mask;
      u128 TmpExpected = 0;
      u128 TmpDesired = 0;

      u128 Desired = (u64)DesiredUpper << 32 | DesiredLower;
      Desired <<= Alignment * 8;

      u128 Expected = (u64)ExpectedUpper << 32 | ExpectedLower;
      Expected <<= Alignment * 8;

      while (1) {
        fi = Load128(Addr, &TmpExpected);
        if (fi.error) {
          goto unlock_fault;
        }

        // Set up expected
        TmpExpected &= NegMask;
        TmpExpected |= Expected;

        // Set up desired
        TmpDesired = TmpExpected;
        TmpDesired &= NegMask;
        TmpDesired |= Desired;

        fi = StoreCAS128(&TmpExpected, TmpDesired, Addr);
        if (fi.error && fi.error != -EAGAIN) {
          goto unlock_fault; // Fault
        }
        if (fi.error == 0) {
          // Successful, so we are done
          goto unlock_success;
        } else {
          // Not successful
          // Now we need to check the results to see if we need to try again
          u128 FailedResultOurBits = TmpExpected & Mask;
          u128 FailedResultNotOurBits = TmpExpected & NegMask;

          u128 FailedDesiredNotOurBits = TmpDesired & NegMask;
          if ((FailedResultNotOurBits ^ FailedDesiredNotOurBits) != 0) {
            // If the bits changed that werent part of our regular CAS then we need to try again
            continue;
          }

          // This happens in the case that between Load and CAS that something has store our desired in to the memory location
          // This means our CAS fails because what we wanted to store was already stored
          u64 FailedResult = FailedResultOurBits >> (Alignment * 8);
          SetReg(GPRs, ExpectedReg1, FailedResult & ~0U);
          SetReg(GPRs, ExpectedReg2, FailedResult >> 32);
          goto unlock_success;
        }
      }
    }
  } else {
    fi.error = -ENOSYS;
    goto unlock_fault;
  }

unlock_success:
  fi = (struct fault_info) {0};
unlock_fault:
  if (SplitLockMutexLocked) {
    up(&buslock_sem);
  }
  return fi;
}

static struct fault_info HandleCASPAL(u32 Instr, u64* GPRs) {
  u32 Size = (Instr >> 30) & 1;

  u32 DesiredReg1 = Instr & 0b11111;
  u32 DesiredReg2 = DesiredReg1 + 1;
  u32 ExpectedReg1 = (Instr >> 16) & 0b11111;
  u32 ExpectedReg2 = ExpectedReg1 + 1;
  u32 AddressReg = (Instr >> 5) & 0b11111;

  return RunCASPAL(GPRs, Size, DesiredReg1, DesiredReg2, ExpectedReg1, ExpectedReg2, AddressReg);
}

typedef u16 (*CASExpectedFn16)(u16 Src, u16 Expected);
typedef u16 (*CASDesiredFn16)(u16 Src, u16 Desired);

static struct fault_info DoCAS16(bool Retry, u16 DesiredSrc, u16 ExpectedSrc, u64 Addr, CASExpectedFn16 ExpectedFunction,
                                 CASDesiredFn16 DesiredFunction, u16* Result) {
  bool SplitLockMutexLocked = false;
  struct fault_info fi;

  if ((Addr & 63) == 63) {
    if (test_thread_flag(TIF_UNALIGN_ATOMIC_STRICT_SPLIT_LOCKS)) {
      down(&buslock_sem);
      SplitLockMutexLocked = true;
    }
  }

  // 16 bit
  u64 AlignmentMask = 0b1111;
  if ((Addr & AlignmentMask) == 15) {
    if (test_thread_flag(TIF_UNALIGN_ATOMIC_STRICT_SPLIT_LOCKS) && !SplitLockMutexLocked) {
      down(&buslock_sem);
      SplitLockMutexLocked = true;
    }

    // Address crosses over 16byte or 64byte threshold
    // Need a dual 8bit CAS loop
    u64 AddrUpper = Addr + 1;

    while (1) {
      u8 ActualUpper = 0;
      u8 ActualLower = 0;
      // Careful ordering here
      fi = LoadAcquire8(AddrUpper, &ActualUpper);
      if (fi.error) {
        goto unlock_fault;
      }
      fi = LoadAcquire8(Addr, &ActualLower);
      if (fi.error) {
        goto unlock_fault;
      }

      u16 Actual = ((u16)ActualUpper << 8) | ActualLower;

      u16 Desired = DesiredFunction(Actual, DesiredSrc);
      u8 DesiredLower = Desired;
      u8 DesiredUpper = Desired >> 8;

      u16 Expected = ExpectedFunction(Actual, ExpectedSrc);
      u8 ExpectedLower = Expected;
      u8 ExpectedUpper = Expected >> 8;

      bool Tear = false;
      if (ActualUpper == ExpectedUpper && ActualLower == ExpectedLower) {
        fi = StoreCAS8(&ExpectedUpper, DesiredUpper, AddrUpper);
        if (fi.error && fi.error != -EAGAIN) {
          goto unlock_fault;
        }
        if (fi.error == 0) { // CAS succeeded
          fi = StoreCAS8(&ExpectedLower, DesiredLower, Addr);
          if (fi.error && fi.error != -EAGAIN) {
            goto unlock_fault;
          }
          if (fi.error == 0) { // CAS succeeded
            // Stored successfully
            *Result = Expected;
            goto unlock;
          } else {
            // CAS managed to tear, we cant really solve this
            // Continue down the path to let the guest know values werent expected
            Tear = true;
          }
        }

        ActualLower = ExpectedLower;
      }

      // If the bits changed that we were wanting to change then we have failed and can return
      // We need to extract the bits and return them in EXPECTED
      u16 FailedResult = ActualUpper;
      FailedResult <<= 8;
      FailedResult |= ActualLower;

      if (Retry) {
        if (Tear) {
          // If we are retrying and tearing then we cant do anything here
          // XXX: Resolve with TME
          *Result = FailedResult;
          goto unlock;
        } else {
          // We can retry safely
        }
      } else {
        // Without Retry (CAS) then we have failed regardless of tear
        // CAS failed but handled successfully
        *Result = FailedResult;
        goto unlock;
      }
    }
  } else {
    AlignmentMask = 0b111;
    if ((Addr & AlignmentMask) == 7) {
      // Crosses 8byte boundary
      // Needs 128bit CAS
      // Fits within a 16byte region
      u64 Alignment = Addr & 0b1111;
      Addr &= ~0b1111ULL;

      u128 Mask = 0xFFFF;
      Mask <<= Alignment * 8;
      u128 NegMask = ~Mask;
      u128 TmpExpected = 0;
      u128 TmpDesired = 0;

      while (1) {
        fi = Load128(Addr, &TmpExpected);
        if (fi.error) {
          goto unlock_fault;
        }

        u128 Desired = DesiredFunction(TmpExpected >> (Alignment * 8), DesiredSrc);
        Desired <<= Alignment * 8;

        u128 Expected = ExpectedFunction(TmpExpected >> (Alignment * 8), ExpectedSrc);
        Expected <<= Alignment * 8;

        // Set up expected
        TmpExpected &= NegMask;
        TmpExpected |= Expected;

        // Set up desired
        TmpDesired = TmpExpected;
        TmpDesired &= NegMask;
        TmpDesired |= Desired;

        fi = StoreCAS128(&TmpExpected, TmpDesired, Addr);
        if (fi.error && fi.error != -EAGAIN) {
          goto unlock_fault;
        }
        if (fi.error == 0) { // CAS succeeded
          // Successful, so we are done
          *Result = Expected >> (Alignment * 8);
          goto unlock;
        } else {
          if (Retry) {
            // If we failed but we have enabled retry then just retry without checking results
            // CAS cant retry but atomic memory ops need to retry until passing
            continue;
          }
          // Not successful
          // Now we need to check the results to see if we need to try again
          u128 FailedResultOurBits = TmpExpected & Mask;
          u128 FailedResultNotOurBits = TmpExpected & NegMask;

          u128 FailedDesiredNotOurBits = TmpDesired & NegMask;
          if ((FailedResultNotOurBits ^ FailedDesiredNotOurBits) != 0) {
            // If the bits changed that werent part of our regular CAS then we need to try again
            continue;
          }

          // This happens in the case that between Load and CAS that something has store our desired in to the memory location
          // This means our CAS fails because what we wanted to store was already stored
          u16 FailedResult = FailedResultOurBits >> (Alignment * 8);
          // CAS failed but handled successfully
          *Result = FailedResult;
          goto unlock;
        }
      }
    } else {
      AlignmentMask = 0b11;
      if ((Addr & AlignmentMask) == 3) {
        // Crosses 4byte boundary
        // Needs 64bit CAS
        u64 Alignment = Addr & AlignmentMask;
        Addr &= ~AlignmentMask;

        u64 Mask = 0xFFFF;
        Mask <<= Alignment * 8;

        u64 NegMask = ~Mask;

        u64 TmpExpected = 0;
        u64 TmpDesired = 0;

        while (1) {
          fi = Load64(Addr, &TmpExpected);
          if (fi.error) {
            goto unlock_fault;
          }

          u64 Desired = DesiredFunction(TmpExpected >> (Alignment * 8), DesiredSrc);
          Desired <<= Alignment * 8;

          u64 Expected = ExpectedFunction(TmpExpected >> (Alignment * 8), ExpectedSrc);
          Expected <<= Alignment * 8;

          // Set up expected
          TmpExpected &= NegMask;
          TmpExpected |= Expected;

          // Set up desired
          TmpDesired = TmpExpected;
          TmpDesired &= NegMask;
          TmpDesired |= Desired;

          fi = StoreCAS64(&TmpExpected, TmpDesired, Addr);
          if (fi.error && fi.error != -EAGAIN) {
            goto unlock_fault;
          }
          if (fi.error == 0) { // CAS succeeded
            // Successful, so we are done
            *Result = Expected >> (Alignment * 8);
            goto unlock;
          } else {
            if (Retry) {
              // If we failed but we have enabled retry then just retry without checking results
              // CAS cant retry but atomic memory ops need to retry until passing
              continue;
            }
            // Not successful
            // Now we need to check the results to see if we can try again
            u64 FailedResultOurBits = TmpExpected & Mask;
            u64 FailedResultNotOurBits = TmpExpected & NegMask;

            u64 FailedDesiredNotOurBits = TmpDesired & NegMask;

            if ((FailedResultNotOurBits ^ FailedDesiredNotOurBits) != 0) {
              // If the bits changed that werent part of our regular CAS then we need to try again
              continue;
            }

            // This happens in the case that between Load and CAS that something has store our desired in to the memory location
            // This means our CAS fails because what we wanted to store was already stored
            u16 FailedResult = FailedResultOurBits >> (Alignment * 8);
            // CAS failed but handled successfully
            *Result = FailedResult;
            goto unlock;
          }
        }
      } else {
        // Fits within 4byte boundary
        // Only needs 32bit CAS
        // Only alignment offset will be 1 here
        u64 Alignment = Addr & AlignmentMask;
        Addr &= ~AlignmentMask;

        u32 Mask = 0xFFFF;
        Mask <<= Alignment * 8;

        u32 NegMask = ~Mask;

        u32 TmpExpected = 0;
        u32 TmpDesired = 0;

        while (1) {
          fi = Load32(Addr, &TmpExpected);
          if (fi.error) {
            goto unlock_fault;
          }

          u32 Desired = DesiredFunction(TmpExpected >> (Alignment * 8), DesiredSrc);
          Desired <<= Alignment * 8;

          u32 Expected = ExpectedFunction(TmpExpected >> (Alignment * 8), ExpectedSrc);
          Expected <<= Alignment * 8;

          // Set up expected
          TmpExpected &= NegMask;
          TmpExpected |= Expected;

          // Set up desired
          TmpDesired = TmpExpected;
          TmpDesired &= NegMask;
          TmpDesired |= Desired;

          fi = StoreCAS32(&TmpExpected, TmpDesired, Addr);
          if (fi.error && fi.error != -EAGAIN) {
            goto unlock_fault;
          }
          if (fi.error == 0) { // CAS succeeded
            // Successful, so we are done
            *Result = Expected >> (Alignment * 8);
            goto unlock;
          } else {
            if (Retry) {
              // If we failed but we have enabled retry then just retry without checking results
              // CAS cant retry but atomic memory ops need to retry until passing
              continue;
            }
            // Not successful
            // Now we need to check the results to see if we can try again
            u32 FailedResultOurBits = TmpExpected & Mask;
            u32 FailedResultNotOurBits = TmpExpected & NegMask;

            u32 FailedDesiredNotOurBits = TmpDesired & NegMask;

            if ((FailedResultNotOurBits ^ FailedDesiredNotOurBits) != 0) {
              // If the bits changed that werent part of our regular CAS then we need to try again
              continue;
            }

            // This happens in the case that between Load and CAS that something has store our desired in to the memory location
            // This means our CAS fails because what we wanted to store was already stored
            u16 FailedResult = FailedResultOurBits >> (Alignment * 8);
            // CAS failed but handled successfully
            *Result = FailedResult;
            goto unlock;
          }
        }
      }
    }
  }

unlock:
  fi = (struct fault_info) {0};
unlock_fault:
  if (SplitLockMutexLocked) {
    up(&buslock_sem);
  }
  return fi;
}


typedef u32 (*CASExpectedFn32)(u32 Src, u32 Expected);
typedef u32 (*CASDesiredFn32)(u32 Src, u32 Desired);

static struct fault_info DoCAS32(bool Retry, u32 DesiredSrc, u32 ExpectedSrc, u64 Addr, CASExpectedFn32 ExpectedFunction,
                                 CASDesiredFn32 DesiredFunction, u32* Result) {
  bool SplitLockMutexLocked = false;
  struct fault_info fi;

  if ((Addr & 63) > 60) {
    if (test_thread_flag(TIF_UNALIGN_ATOMIC_STRICT_SPLIT_LOCKS)) {
      down(&buslock_sem);
      SplitLockMutexLocked = true;
    }
  }

  // 32 bit
  u64 AlignmentMask = 0b1111;
  if ((Addr & AlignmentMask) > 12) {
    if (test_thread_flag(TIF_UNALIGN_ATOMIC_STRICT_SPLIT_LOCKS) && !SplitLockMutexLocked) {
      down(&buslock_sem);
      SplitLockMutexLocked = true;
    }

    // Address crosses over 16byte threshold
    // Needs dual 4 byte CAS loop
    u64 Alignment = Addr & 0b11;
    Addr &= ~0b11;

    u64 AddrUpper = Addr + 4;

    u64 Mask = ~0U;
    Mask <<= Alignment * 8;
    u64 NegMask = ~Mask;

    // Careful ordering here
    while (1) {
      u32 LoadOrderUpper = 0;
      u32 LoadOrderLower = 0;

      fi = LoadAcquire32(AddrUpper, &LoadOrderUpper);
      if (fi.error) {
        goto unlock_fault;
      }
      fi = LoadAcquire32(Addr, &LoadOrderLower);
      if (fi.error) {
        goto unlock_fault;
      }
      u64 TmpActual = ((u64)LoadOrderUpper << 32) | LoadOrderLower;

      u64 Desired = DesiredFunction(TmpActual >> (Alignment * 8), DesiredSrc);
      u64 Expected = ExpectedFunction(TmpActual >> (Alignment * 8), ExpectedSrc);

      u64 TmpExpected = TmpActual;
      TmpExpected &= NegMask;
      TmpExpected |= Expected << (Alignment * 8);

      u64 TmpDesired = TmpExpected;
      TmpDesired &= NegMask;
      TmpDesired |= Desired << (Alignment * 8);

      bool Tear = false;
      if (TmpExpected == TmpActual) {
        u32 TmpExpectedLower = TmpExpected;
        u32 TmpExpectedUpper = TmpExpected >> 32;

        u32 TmpDesiredLower = TmpDesired;
        u32 TmpDesiredUpper = TmpDesired >> 32;

        fi = StoreCAS32(&TmpExpectedUpper, TmpDesiredUpper, AddrUpper);
        if (fi.error && fi.error != -EAGAIN) {
          goto unlock_fault;
        }
        if (fi.error == 0) { // CAS succeeded
          fi = StoreCAS32(&TmpExpectedLower, TmpDesiredLower, Addr);
          if (fi.error && fi.error != -EAGAIN) {
            goto unlock_fault;
          }
          if (fi.error == 0) { // CAS succeeded
            // Stored successfully
            *Result = Expected;
            goto unlock;
          } else {
            // CAS managed to tear, we cant really solve this
            // Continue down the path to let the guest know values werent expected
            Tear = true;
          }
        }

        TmpExpected = TmpExpectedUpper;
        TmpExpected <<= 32;
        TmpExpected |= TmpExpectedLower;
      } else {
        // Mismatch up front
        TmpExpected = TmpActual;
      }

      // Not successful
      // Now we need to check the results to see if we need to try again
      u64 FailedResultOurBits = TmpExpected & Mask;
      u64 FailedResultNotOurBits = TmpExpected & NegMask;

      u64 FailedDesiredNotOurBits = TmpDesired & NegMask;
      if ((FailedResultNotOurBits ^ FailedDesiredNotOurBits) != 0) {
        // If the bits changed that werent part of our regular CAS then we need to try again
        continue;
      }

      // This happens in the case that between Load and CAS that something has store our desired in to the memory location
      // This means our CAS fails because what we wanted to store was already stored
      u32 FailedResult = FailedResultOurBits >> (Alignment * 8);

      if (Retry) {
        if (Tear) {
          // If we are retrying and tearing then we cant do anything here
          // XXX: Resolve with TME
          *Result = FailedResult;
          goto unlock;
        } else {
          // We can retry safely
        }
      } else {
        // Without Retry (CAS) then we have failed regardless of tear
        // CAS failed but handled successfully
        *Result = FailedResult;
        goto unlock;
      }
    }
  } else {
    AlignmentMask = 0b111;
    if ((Addr & AlignmentMask) >= 5) {
      // Crosses 8byte boundary
      // Needs 128bit CAS
      // Fits within a 16byte region
      u64 Alignment = Addr & 0b1111;
      Addr &= ~0b1111ULL;

      u128 Mask = ~0U;
      Mask <<= Alignment * 8;
      u128 NegMask = ~Mask;
      u128 TmpExpected = 0;
      u128 TmpDesired = 0;

      while (1) {
        u128 TmpActual = 0;
        fi = Load128(Addr, &TmpActual);
        if (fi.error) {
          goto unlock_fault;
        }

        u128 Desired = DesiredFunction(TmpActual >> (Alignment * 8), DesiredSrc);
        u128 Expected = ExpectedFunction(TmpActual >> (Alignment * 8), ExpectedSrc);

        // Set up expected
        TmpExpected = TmpActual;
        TmpExpected &= NegMask;
        TmpExpected |= Expected << (Alignment * 8);

        // Set up desired
        TmpDesired = TmpExpected;
        TmpDesired &= NegMask;
        TmpDesired |= Desired << (Alignment * 8);

        fi = StoreCAS128(&TmpExpected, TmpDesired, Addr);
        if (fi.error && fi.error != -EAGAIN) {
          goto unlock_fault;
        }
        if (fi.error == 0) { // CAS succeeded
          // Stored successfully
          *Result = Expected;
          goto unlock;
        } else {
          if (Retry) {
            // If we failed but we have enabled retry then just retry without checking results
            // CAS cant retry but atomic memory ops need to retry until passing
            continue;
          }

          // Not successful
          // Now we need to check the results to see if we need to try again
          u128 FailedResultOurBits = TmpExpected & Mask;
          u128 FailedResultNotOurBits = TmpExpected & NegMask;

          u128 FailedDesiredNotOurBits = TmpDesired & NegMask;
          if ((FailedResultNotOurBits ^ FailedDesiredNotOurBits) != 0) {
            // If the bits changed that werent part of our regular CAS then we need to try again
            continue;
          }

          // This happens in the case that between Load and CAS that something has store our desired in to the memory location
          // This means our CAS fails because what we wanted to store was already stored
          u32 FailedResult = FailedResultOurBits >> (Alignment * 8);
          // CAS failed but handled successfully
          *Result = FailedResult;
          goto unlock;
        }
      }
    } else {
      // Fits within 8byte boundary
      // Only needs 64bit CAS
      // Alignments can be [1,5)
      u64 Alignment = Addr & AlignmentMask;
      Addr &= ~AlignmentMask;

      u64 Mask = ~0U;
      Mask <<= Alignment * 8;

      u64 NegMask = ~Mask;

      u64 TmpExpected = 0;
      u64 TmpDesired = 0;

      while (1) {
        u64 TmpActual = 0;
        fi = Load64(Addr, &TmpActual);
        if (fi.error) {
          goto unlock_fault;
        }


        u64 Desired = DesiredFunction(TmpActual >> (Alignment * 8), DesiredSrc);
        u64 Expected = ExpectedFunction(TmpActual >> (Alignment * 8), ExpectedSrc);

        // Set up expected
        TmpExpected = TmpActual;
        TmpExpected &= NegMask;
        TmpExpected |= Expected << (Alignment * 8);

        // Set up desired
        TmpDesired = TmpExpected;
        TmpDesired &= NegMask;
        TmpDesired |= Desired << (Alignment * 8);

        fi = StoreCAS64(&TmpExpected, TmpDesired, Addr);
        if (fi.error && fi.error != -EAGAIN) {
          goto unlock_fault;
        }
        if (fi.error == 0) { // CAS succeeded
          // Stored successfully
          *Result = Expected;
          goto unlock;
        } else {
          if (Retry) {
            // If we failed but we have enabled retry then just retry without checking results
            // CAS cant retry but atomic memory ops need to retry until passing
            continue;
          }

          // Not successful
          // Now we need to check the results to see if we can try again
          u64 FailedResultOurBits = TmpExpected & Mask;
          u64 FailedResultNotOurBits = TmpExpected & NegMask;

          u64 FailedDesiredNotOurBits = TmpDesired & NegMask;

          if ((FailedResultNotOurBits ^ FailedDesiredNotOurBits) != 0) {
            // If the bits changed that werent part of our regular CAS then we need to try again
            continue;
          }

          // This happens in the case that between Load and CAS that something has store our desired in to the memory location
          // This means our CAS fails because what we wanted to store was already stored
          u32 FailedResult = FailedResultOurBits >> (Alignment * 8);
          // CAS failed but handled successfully
          *Result = FailedResult;
          goto unlock;
        }
      }
    }
  }

unlock:
  fi = (struct fault_info) {0};
unlock_fault:
  if (SplitLockMutexLocked) {
    up(&buslock_sem);
  }
  return fi;
}


typedef u64 (*CASExpectedFn64)(u64 Src, u64 Expected);
typedef u64 (*CASDesiredFn64)(u64 Src, u64 Desired);

static struct fault_info DoCAS64(bool Retry, u64 DesiredSrc, u64 ExpectedSrc, u64 Addr, CASExpectedFn64 ExpectedFunction,
                                 CASDesiredFn64 DesiredFunction, u64* Result) {
  bool SplitLockMutexLocked = false;
  struct fault_info fi;

  if ((Addr & 63) > 56) {
    if (test_thread_flag(TIF_UNALIGN_ATOMIC_STRICT_SPLIT_LOCKS)) {
      down(&buslock_sem);
      SplitLockMutexLocked = true;
    }
  }

  // 64bit
  u64 AlignmentMask = 0b1111;
  if ((Addr & AlignmentMask) > 8) {
    if (test_thread_flag(TIF_UNALIGN_ATOMIC_STRICT_SPLIT_LOCKS) && !SplitLockMutexLocked) {
      down(&buslock_sem);
      SplitLockMutexLocked = true;
    }

    u64 Alignment = Addr & 0b111;
    Addr &= ~0b111ULL;
    u64 AddrUpper = Addr + 8;

    // Crosses a 16byte boundary
    // Need to do 256bit atomic, but since that doesnt exist we need to do a dual CAS loop
    u128 Mask = ~0ULL;
    Mask <<= Alignment * 8;
    u128 NegMask = ~Mask;
    u128 TmpExpected = 0;
    u128 TmpDesired = 0;

    while (1) {
      u64 LoadOrderUpper = 0;
      u64 LoadOrderLower = 0;
      fi = LoadAcquire64(AddrUpper, &LoadOrderUpper);
      if (fi.error) {
        goto unlock_fault;
      }
      fi = LoadAcquire64(Addr, &LoadOrderLower);
      if (fi.error) {
        goto unlock_fault;
      }
      u128 TmpActual = (u128)LoadOrderUpper << 64 | LoadOrderLower;

      u128 Desired = DesiredFunction(TmpActual >> (Alignment * 8), DesiredSrc);
      u128 Expected = ExpectedFunction(TmpActual >> (Alignment * 8), ExpectedSrc);

      // Set up expected
      TmpExpected = TmpActual;
      TmpExpected &= NegMask;
      TmpExpected |= Expected << (Alignment * 8);

      // Set up desired
      TmpDesired = TmpExpected;
      TmpDesired &= NegMask;
      TmpDesired |= Desired << (Alignment * 8);

      u64 TmpExpectedLower = TmpExpected;
      u64 TmpExpectedUpper = TmpExpected >> 64;

      u64 TmpDesiredLower = TmpDesired;
      u64 TmpDesiredUpper = TmpDesired >> 64;

      bool Tear = false;
      if (TmpExpected == TmpActual) {
        fi = StoreCAS64(&TmpExpectedUpper, TmpDesiredUpper, AddrUpper);
        if (fi.error && fi.error != -EAGAIN) {
          goto unlock_fault;
        }
        if (fi.error == 0) { // CAS succeeded
          fi = StoreCAS64(&TmpExpectedLower, TmpDesiredLower, Addr);
          if (fi.error && fi.error != -EAGAIN) {
            goto unlock_fault;
          }
          if (fi.error == 0) { // CAS succeeded
            // Stored successfully
            *Result = Expected;
            goto unlock;
          } else {
            // CAS managed to tear, we cant really solve this
            // Continue down the path to let the guest know values werent expected
            Tear = true;
          }
        }

        TmpExpected = TmpExpectedUpper;
        TmpExpected <<= 64;
        TmpExpected |= TmpExpectedLower;
      } else {
        // Mismatch up front
        TmpExpected = TmpActual;
      }

      // Not successful
      // Now we need to check the results to see if we need to try again
      u128 FailedResultOurBits = TmpExpected & Mask;
      u128 FailedResultNotOurBits = TmpExpected & NegMask;

      u128 FailedDesiredNotOurBits = TmpDesired & NegMask;
      if ((FailedResultNotOurBits ^ FailedDesiredNotOurBits) != 0) {
        // If the bits changed that werent part of our regular CAS then we need to try again
        continue;
      }

      // This happens in the case that between Load and CAS that something has store our desired in to the memory location
      // This means our CAS fails because what we wanted to store was already stored
      u64 FailedResult = FailedResultOurBits >> (Alignment * 8);

      if (Retry) {
        if (Tear) {
          // If we are retrying and tearing then we cant do anything here
          // XXX: Resolve with TME
          *Result = FailedResult;
          goto unlock;
        } else {
          // We can retry safely
        }
      } else {
        // Without Retry (CAS) then we have failed regardless of tear
        // CAS failed but handled successfully
        *Result = FailedResult;
        goto unlock;
      }
    }
  } else {
    // Fits within a 16byte region
    u64 Alignment = Addr & AlignmentMask;
    Addr &= ~AlignmentMask;

    u128 Mask = ~0ULL;
    Mask <<= Alignment * 8;
    u128 NegMask = ~Mask;
    u128 TmpExpected = 0;
    u128 TmpDesired = 0;

    while (1) {
      u128 TmpActual;
      fi = Load128(Addr, &TmpActual);
      if (fi.error) {
        goto unlock_fault;
      }

      u128 Desired = DesiredFunction(TmpActual >> (Alignment * 8), DesiredSrc);
      u128 Expected = ExpectedFunction(TmpActual >> (Alignment * 8), ExpectedSrc);

      // Set up expected
      TmpExpected = TmpActual;
      TmpExpected &= NegMask;
      TmpExpected |= Expected << (Alignment * 8);

      // Set up desired
      TmpDesired = TmpExpected;
      TmpDesired &= NegMask;
      TmpDesired |= Desired << (Alignment * 8);

      fi = StoreCAS128(&TmpExpected, TmpDesired, Addr);
      if (fi.error && fi.error != -EAGAIN) {
        goto unlock_fault;
      }
      if (fi.error == 0) { // CAS succeeded
        // Stored successfully
        *Result = Expected;
        goto unlock;
      } else {
        if (Retry) {
          // If we failed but we have enabled retry then just retry without checking results
          // CAS cant retry but atomic memory ops need to retry until passing
          continue;
        }

        // Not successful
        // Now we need to check the results to see if we need to try again
        u128 FailedResultOurBits = TmpExpected & Mask;
        u128 FailedResultNotOurBits = TmpExpected & NegMask;

        u128 FailedDesiredNotOurBits = TmpDesired & NegMask;
        if ((FailedResultNotOurBits ^ FailedDesiredNotOurBits) != 0) {
          // If the bits changed that werent part of our regular CAS then we need to try again
          continue;
        }

        // This happens in the case that between Load and CAS that something has store our desired in to the memory location
        // This means our CAS fails because what we wanted to store was already stored
        u64 FailedResult = FailedResultOurBits >> (Alignment * 8);
        // CAS failed but handled successfully
        *Result = FailedResult;
        goto unlock;
      }
    }
  }

unlock:
  fi = (struct fault_info) {0};
unlock_fault:
  if (SplitLockMutexLocked) {
    up(&buslock_sem);
  }
  return fi;
}

static u16 CASSwap16(u16 Src, u16 ExpectedDesired) {
  return ExpectedDesired;
}

static u32 CASSwap32(u32 Src, u32 ExpectedDesired) {
  return ExpectedDesired;
}

static u64 CASSwap64(u64 Src, u64 ExpectedDesired) {
  return ExpectedDesired;
}

static struct fault_info RunCASAL(u64* GPRs, u32 Size, u32 DesiredReg, u32 ExpectedReg, u32 AddressReg) {
  u64 Addr = GetReg(GPRs, AddressReg);
  struct fault_info fi = {0};

  // Cross-cacheline CAS doesnt work on ARM
  // It isnt even guaranteed to work on x86
  // Intel will do a "split lock" which locks the full bus
  // AMD will tear instead
  // Both cross-cacheline and cross 16byte both need dual CAS loops that can tear
  // ARMv8.4 LSE2 solves all atomic issues except cross-cacheline
  // ARMs TME extension solves the cross-cacheline problem

  // 8bit cant be unaligned
  // Only need to handle 16, 32, 64
  if (Size == 2) {
    u16 Res = 0;
    fi = DoCAS16(false, GetReg(GPRs, DesiredReg), GetReg(GPRs, ExpectedReg), Addr, CASSwap16, CASSwap16, &Res);

    // Regardless of pass or fail
    // We set the result register if it isnt a zero register
    if (!fi.error) {
      SetReg(GPRs, ExpectedReg, Res);
    }
  } else if (Size == 4) {
    u32 Res = 0;
    fi = DoCAS32(false, GetReg(GPRs, DesiredReg), GetReg(GPRs, ExpectedReg), Addr, CASSwap32, CASSwap32, &Res);

    // Regardless of pass or fail
    // We set the result register if it isnt a zero register
    if (!fi.error) {
      SetReg(GPRs, ExpectedReg, Res);
    }
  } else if (Size == 8) {
    u64 Res = 0;
    fi = DoCAS64(false, GetReg(GPRs, DesiredReg), GetReg(GPRs, ExpectedReg), Addr, CASSwap64, CASSwap64, &Res);
    if (fi.error) {
      return fi;
    }

    // Regardless of pass or fail
    // We set the result register if it isnt a zero register
    if (!fi.error) {
      SetReg(GPRs, ExpectedReg, Res);
    }
  } else {
    fi.error = -ENOSYS;
  }

  return fi;
}

static struct fault_info HandleCASAL(u64* GPRs, u32 Instr) {
  u32 Size = 1 << (Instr >> 30);

  u32 DesiredReg = Instr & 0b11111;
  u32 ExpectedReg = (Instr >> 16) & 0b11111;
  u32 AddressReg = (Instr >> 5) & 0b11111;
  return RunCASAL(GPRs, Size, DesiredReg, ExpectedReg, AddressReg);
}

static u16 CASNOPExpected16(u16 SrcVal, u16 /*Desired*/) {
  return SrcVal;
}

static u16 CASADDDesired16(u16 SrcVal, u16 Desired) {
  return SrcVal + Desired;
}

static u16 CASBICDesired16(u16 SrcVal, u16 Desired) {
  return SrcVal & ~Desired;
}

static u16 CASORDesired16(u16 SrcVal, u16 Desired) {
  return SrcVal | Desired;
}

static u16 CASEORDesired16(u16 SrcVal, u16 Desired) {
  return SrcVal ^ Desired;
}

static u16 CASSWAPDesired16(u16 /*SrcVal*/, u16 Desired) {
  return Desired;
}

static u32 CASNOPExpected32(u32 SrcVal, u32 /*Desired*/) {
  return SrcVal;
}

static u32 CASADDDesired32(u32 SrcVal, u32 Desired) {
  return SrcVal + Desired;
}

static u32 CASBICDesired32(u32 SrcVal, u32 Desired) {
  return SrcVal & ~Desired;
}

static u32 CASORDesired32(u32 SrcVal, u32 Desired) {
  return SrcVal | Desired;
}

static u32 CASEORDesired32(u32 SrcVal, u32 Desired) {
  return SrcVal ^ Desired;
}

static u32 CASSWAPDesired32(u32 /*SrcVal*/, u32 Desired) {
  return Desired;
}

static u64 CASNOPExpected64(u64 SrcVal, u64 /*Desired*/) {
  return SrcVal;
}

static u64 CASADDDesired64(u64 SrcVal, u64 Desired) {
  return SrcVal + Desired;
}

static u64 CASBICDesired64(u64 SrcVal, u64 Desired) {
  return SrcVal & ~Desired;
}

static u64 CASORDesired64(u64 SrcVal, u64 Desired) {
  return SrcVal | Desired;
}

static u64 CASEORDesired64(u64 SrcVal, u64 Desired) {
  return SrcVal ^ Desired;
}

static u64 CASSWAPDesired64(u64 /*SrcVal*/, u64 Desired) {
  return Desired;
}

static struct fault_info HandleAtomicMemOp(u32 Instr, u64* GPRs) {
  u32 Size = 1 << (Instr >> 30);
  u32 ResultReg = Instr & 0b11111;
  u32 SourceReg = (Instr >> 16) & 0b11111;
  u32 AddressReg = (Instr >> 5) & 0b11111;

  u64 Addr = GetReg(GPRs, AddressReg);

  u8 Op = (Instr >> 12) & 0xF;
  struct fault_info fi = {0};

  if (Size == 2) {
    CASDesiredFn16 DesiredFunction = NULL;

    switch (Op) {
    case ATOMIC_ADD_OP: DesiredFunction = CASADDDesired16; break;
    case ATOMIC_CLR_OP: DesiredFunction = CASBICDesired16; break;
    case ATOMIC_EOR_OP: DesiredFunction = CASEORDesired16; break;
    case ATOMIC_SET_OP: DesiredFunction = CASORDesired16; break;
    case ATOMIC_SWAP_OP: DesiredFunction = CASSWAPDesired16; break;
    default: pr_warn("Unhandled JIT SIGBUS Atomic mem op 0x%02x\n", Op); return (struct fault_info) {.error = -ENOSYS};
    }

    u16 Res = 0;
    fi = DoCAS16(true, GetReg(GPRs, SourceReg),
                 0, // Unused
                 Addr, CASNOPExpected16, DesiredFunction, &Res);
    // If we passed and our destination register is not zero
    // Then we need to update the result register with what was in memory
    if (!fi.error) {
      SetReg(GPRs, ResultReg, Res);
    }
  } else if (Size == 4) {
    CASDesiredFn32 DesiredFunction = NULL;

    switch (Op) {
    case ATOMIC_ADD_OP: DesiredFunction = CASADDDesired32; break;
    case ATOMIC_CLR_OP: DesiredFunction = CASBICDesired32; break;
    case ATOMIC_EOR_OP: DesiredFunction = CASEORDesired32; break;
    case ATOMIC_SET_OP: DesiredFunction = CASORDesired32; break;
    case ATOMIC_SWAP_OP: DesiredFunction = CASSWAPDesired32; break;
    default: pr_warn("Unhandled JIT SIGBUS Atomic mem op 0x%02x\n", Op); return (struct fault_info) {.error = -ENOSYS};
    }

    u32 Res = 0;
    fi = DoCAS32(true, GetReg(GPRs, SourceReg),
                 0, // Unused
                 Addr, CASNOPExpected32, DesiredFunction, &Res);

    // If we passed and our destination register is not zero
    // Then we need to update the result register with what was in memory
    if (!fi.error) {
      SetReg(GPRs, ResultReg, Res);
    }
  } else if (Size == 8) {
    CASDesiredFn64 DesiredFunction = NULL;

    switch (Op) {
    case ATOMIC_ADD_OP: DesiredFunction = CASADDDesired64; break;
    case ATOMIC_CLR_OP: DesiredFunction = CASBICDesired64; break;
    case ATOMIC_EOR_OP: DesiredFunction = CASEORDesired64; break;
    case ATOMIC_SET_OP: DesiredFunction = CASORDesired64; break;
    case ATOMIC_SWAP_OP: DesiredFunction = CASSWAPDesired64; break;
    default: pr_warn("Unhandled JIT SIGBUS Atomic mem op 0x%02x\n", Op); return (struct fault_info) {.error = -ENOSYS};
    }

    u64 Res = 0;
    fi = DoCAS64(true, GetReg(GPRs, SourceReg),
                 0, // Unused
                 Addr, CASNOPExpected64, DesiredFunction, &Res);

    // If we passed and our destination register is not zero
    // Then we need to update the result register with what was in memory
    if (!fi.error) {
      SetReg(GPRs, ResultReg, Res);
    }
  } else {
    fi.error = -ENOSYS;
  }

  return fi;
}

static struct fault_info HandleAtomicLoad(u32 Instr, u64* GPRs, s64 Offset) {
  u32 Size = 1 << (Instr >> 30);

  u32 ResultReg = Instr & 0b11111;
  u32 AddressReg = (Instr >> 5) & 0b11111;

  u64 Addr = GetReg(GPRs, AddressReg) + Offset;
  struct fault_info fi = {0};

  if (Size == 2) {
    u16 Res;
    fi = DoLoad16(Addr, &Res);
    // We set the result register if it isnt a zero register
    if (!fi.error) {
      SetReg(GPRs, ResultReg, Res);
    }
  } else if (Size == 4) {
    u32 Res;
    fi = DoLoad32(Addr, &Res);
    // We set the result register if it isnt a zero register
    if (!fi.error) {
      SetReg(GPRs, ResultReg, Res);
    }
  } else if (Size == 8) {
    u64 Res;
    fi = DoLoad64(Addr, &Res);
    // We set the result register if it isnt a zero register
    if (!fi.error) {
      SetReg(GPRs, ResultReg, Res);
    }
  } else {
    fi.error = -ENOSYS;
  }

  return fi;
}

static struct fault_info HandleAtomicStore(u32 Instr, u64* GPRs, s64 Offset) {
  u32 Size = 1 << (Instr >> 30);

  u32 DataReg = Instr & 0x1F;
  u32 AddressReg = (Instr >> 5) & 0b11111;

  u64 Addr = GetReg(GPRs, AddressReg) + Offset;
  struct fault_info fi = {0};

  if (Size == 2) {
    u16 Res;
    fi = DoCAS16(false, GetReg(GPRs, DataReg),
                 0, // Unused
                 Addr, CASNOPExpected16, CASSWAPDesired16, &Res);
  } else if (Size == 4) {
    u32 Res;
    fi = DoCAS32(false, GetReg(GPRs, DataReg),
                 0, // Unused
                 Addr, CASNOPExpected32, CASSWAPDesired32, &Res);
  } else if (Size == 8) {
    u64 Res;
    fi = DoCAS64(false, GetReg(GPRs, DataReg),
                 0, // Unused
                 Addr, CASNOPExpected64, CASSWAPDesired64, &Res);
  } else {
    fi.error = -ENOSYS;
  }

  return fi;
}


int do_unaligned_atomic_fixup(struct pt_regs* regs, u64* fault_addr) {
  u32* PC = (u32*)regs->pc;
  u64* GPRs = (u64*)regs->regs;
  u32 Instr;
  struct fault_info fi;

  if (get_user(Instr, (u32 __user*)(PC))) {
    return -ENOSYS;
  }
  u32 Size = (Instr & 0xC0000000) >> 30;
  u32 AddrReg = (Instr >> 5) & 0x1F;
  u32 DataReg = Instr & 0x1F;
  bool backpatch = test_thread_flag(TIF_UNALIGN_ATOMIC_BACKPATCH);

  if ((Instr & LDAXR_MASK) == LDAR_INST || // LDAR*
      (Instr & LDAXR_MASK) == LDAPR_INST ||
      (backpatch && (Instr & LDSTREGISTER_MASK) == LDR_INST)) { // LDAPR*
    if (backpatch) {
      u64 PatchAddr = (u64)&PC[0];
      u32 Expected = NOP_INST;
      fi = StoreCAS32(&Expected, DMB_LD, PatchAddr + 4);
      if (!fi.error || fi.error == -EAGAIN) {
        u32 LDR = LDR_INST;
        LDR |= Size << 30;
        LDR |= AddrReg << 5;
        LDR |= DataReg;
        StoreRelease32(LDR, PatchAddr);
        flush_icache_range(PatchAddr, PatchAddr + 8);
      }
    }
    fi = HandleAtomicLoad(Instr, GPRs, 0);
  } else if ((Instr & LDAXR_MASK) == STLR_INST ||
             (backpatch && (Instr & LDSTREGISTER_MASK) == STR_INST)) { // STLR*
    if (backpatch) {
      u64 PatchAddr = (u64)&PC[-1];
      u32 Expected = NOP_INST;
      fi = StoreCAS32(&Expected, DMB, PatchAddr);
      if (!fi.error || fi.error == -EAGAIN) {
        u32 STR = STR_INST;
        STR |= Size << 30;
        STR |= AddrReg << 5;
        STR |= DataReg;
        StoreRelease32(STR, PatchAddr + 4);
        flush_icache_range(PatchAddr, PatchAddr + 8);
      }
    }
    fi = HandleAtomicStore(Instr, GPRs, 0);
  } else if ((Instr & RCPC2_MASK) == LDAPUR_INST ||
             (backpatch && (Instr & LDSTUNSCALED_MASK) == LDUR_INST)) { // LDAPUR*
    if (backpatch) {
      u64 PatchAddr = (u64)&PC[0];
      u32 Expected = NOP_INST;
      fi = StoreCAS32(&Expected, DMB_LD, PatchAddr + 4);
      if (!fi.error || fi.error == -EAGAIN) {
        u32 LDUR = LDUR_INST;
        LDUR |= Size << 30;
        LDUR |= AddrReg << 5;
        LDUR |= DataReg;
        LDUR |= Instr & (0b111111111 << 12);
        StoreRelease32(LDUR, PatchAddr);
        flush_icache_range(PatchAddr, PatchAddr + 8);
      }
    }
    // Extract the 9-bit offset from the instruction
    s32 Offset = (s32)(Instr) << 11 >> 23;
    fi = HandleAtomicLoad(Instr, GPRs, Offset);
  } else if ((Instr & RCPC2_MASK) == STLUR_INST ||
             (backpatch && (Instr & LDSTUNSCALED_MASK) == STUR_INST)) { // STLUR*
    if (backpatch) {
      u32 Expected = NOP_INST;
      u64 PatchAddr = (u64)&PC[-1];
      fi = StoreCAS32(&Expected, DMB, PatchAddr);
      if (!fi.error || fi.error == -EAGAIN) {
        u32 STUR = STUR_INST;
        STUR |= Size << 30;
        STUR |= AddrReg << 5;
        STUR |= DataReg;
        STUR |= Instr & (0b111111111 << 12);
        StoreRelease32(STUR, PatchAddr + 4);
        flush_icache_range(PatchAddr, PatchAddr + 8);
      }
    }
    // Extract the 9-bit offset from the instruction
    s32 Offset = (s32)(Instr) << 11 >> 23;
    fi = HandleAtomicStore(Instr, GPRs, Offset);
  } else if ((Instr & CASPAL_MASK) == CASPAL_INST && ((Instr >> 30) & 1) == 0) { // CASPAL with Size == 0
    fi = HandleCASPAL(Instr, GPRs);
  } else if ((Instr & CASAL_MASK) == CASAL_INST) { // CASAL
    fi = HandleCASAL(GPRs, Instr);
  } else if ((Instr & ATOMIC_MEM_MASK) == ATOMIC_MEM_INST) { // Atomic memory op
    fi = HandleAtomicMemOp(Instr, GPRs);
  } else {
    fi.error = -ENOSYS;
  }

  if (fi.error) {
    *fault_addr = fi.addr;
    if (fi.error == -EFAULT) {
      int i;
      u8 tmp;
      for (i = 0; i < fi.size - 1 && !get_user(tmp, (u8 __user*)(fi.addr + i)); i++) {
        (*fault_addr)++;
      }
    }
    return fi.error;
  }

  arm64_skip_faulting_instruction(regs, 4);
  return 0;
}

// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2023 Daniel Berlin
 */

#ifndef BRCMFMAC_RATESPEC_H
#define BRCMFMAC_RATESPEC_H
/* Rate spec. definitions */
/* for BRCMF_RSPEC_ENCODING field >= BRCMF_RSPEC_ENCODING_HE, backward compatible */

/**< Legacy rate or MCS or MCS + NSS */
#define BRCMF_RSPEC_RATE_MASK 0x000000FFu
/**< Tx chain expansion beyond Nsts */
#define BRCMF_RSPEC_TXEXP_MASK 0x00000300u
#define BRCMF_RSPEC_TXEXP_SHIFT 8u
/* EHT GI indices */
#define BRCMF_RSPEC_EHT_GI_MASK 0x00000C00u
#define BRCMF_RSPEC_EHT_GI_SHIFT 10u
/* HE GI indices */
#define BRCMF_RSPEC_HE_GI_MASK 0x00000C00u
#define BRCMF_RSPEC_HE_GI_SHIFT 10u
/**< Range extension mask */
#define BRCMF_RSPEC_ER_MASK 0x0000C000u
#define BRCMF_RSPEC_ER_SHIFT 14u
/**< Range extension tone config */
#define BRCMF_RSPEC_ER_TONE_MASK 0x00004000u
#define BRCMF_RSPEC_ER_TONE_SHIFT 14u
/**< Range extension enable */
#define BRCMF_RSPEC_ER_ENAB_MASK 0x00008000u
#define BRCMF_RSPEC_ER_ENAB_SHIFT 15u
/**< Bandwidth */
#define BRCMF_RSPEC_BW_MASK 0x00070000u
#define BRCMF_RSPEC_BW_SHIFT 16u
/**< Dual Carrier Modulation */
#define BRCMF_RSPEC_DCM 0x00080000u
#define BRCMF_RSPEC_DCM_SHIFT 19u
/**< STBC expansion, Nsts = 2 * Nss */
#define BRCMF_RSPEC_STBC 0x00100000u
#define BRCMF_RSPEC_TXBF 0x00200000u
#define BRCMF_RSPEC_LDPC 0x00400000u
/* HT/VHT SGI indication */
#define BRCMF_RSPEC_SGI 0x00800000u
/**< DSSS short preable - Encoding 0 */
#define BRCMF_RSPEC_SHORT_PREAMBLE 0x00800000u
/**< Encoding of RSPEC_RATE field */
#define BRCMF_RSPEC_ENCODING_MASK 0x07000000u
#define BRCMF_RSPEC_ENCODING_SHIFT 24u
#define BRCMF_RSPEC_OVERRIDE_RATE 0x40000000u /**< override rate only */
#define BRCMF_RSPEC_OVERRIDE_MODE 0x80000000u /**< override both rate & mode */

/* ======== RSPEC_EHT_GI|RSPEC_SGI fields for EHT ======== */
/* 11be Draft 0.4 Table 36-35:Common field for non-OFDMA transmission.
 * Table 36-32 Common field for OFDMA transmission
 */
#define BRCMF_RSPEC_EHT_LTF_GI(rspec) \
	(((rspec) & BRCMF_RSPEC_EHT_GI_MASK) >> BRCMF_RSPEC_EHT_GI_SHIFT)
#define BRCMF_RSPEC_EHT_2x_LTF_GI_0_8us (0x0u)
#define BRCMF_RSPEC_EHT_2x_LTF_GI_1_6us (0x1u)
#define BRCMF_RSPEC_EHT_4x_LTF_GI_0_8us (0x2u)
#define BRCMF_RSPEC_EHT_4x_LTF_GI_3_2us (0x3u)
#define WL_EHT_GI_TO_RSPEC(gi)                             \
	((u32)(((gi) << BRCMF_RSPEC_EHT_GI_SHIFT) & \
		      BRCMF_RSPEC_EHT_GI_MASK))
#define WL_EHT_GI_TO_RSPEC_SET(rspec, gi) \
	((rspec & (~BRCMF_RSPEC_EHT_GI_MASK)) | WL_EHT_GI_TO_RSPEC(gi))

/* Macros for EHT LTF and GI */
#define EHT_IS_2X_LTF(gi)                             \
	(((gi) == BRCMF_RSPEC_EHT_2x_LTF_GI_0_8us) || \
	 ((gi) == BRCMF_RSPEC_EHT_2x_LTF_GI_1_6us))
#define EHT_IS_4X_LTF(gi)                             \
	(((gi) == BRCMF_RSPEC_EHT_4x_LTF_GI_0_8us) || \
	 ((gi) == BRCMF_RSPEC_EHT_4x_LTF_GI_3_2us))

#define EHT_IS_GI_0_8us(gi)                           \
	(((gi) == BRCMF_RSPEC_EHT_2x_LTF_GI_0_8us) || \
	 ((gi) == BRCMF_RSPEC_EHT_4x_LTF_GI_0_8us))
#define EHT_IS_GI_1_6us(gi) ((gi) == BRCMF_RSPEC_EHT_2x_LTF_GI_1_6us)
#define EHT_IS_GI_3_2us(gi) ((gi) == BRCMF_RSPEC_EHT_4x_LTF_GI_3_2us)

/* ======== RSPEC_HE_GI|RSPEC_SGI fields for HE ======== */

/* GI for HE */
#define BRCMF_RSPEC_HE_LTF_GI(rspec) \
	(((rspec) & BRCMF_RSPEC_HE_GI_MASK) >> BRCMF_RSPEC_HE_GI_SHIFT)
#define BRCMF_RSPEC_HE_1x_LTF_GI_0_8us (0x0u)
#define BRCMF_RSPEC_HE_2x_LTF_GI_0_8us (0x1u)
#define BRCMF_RSPEC_HE_2x_LTF_GI_1_6us (0x2u)
#define BRCMF_RSPEC_HE_4x_LTF_GI_3_2us (0x3u)
#define BRCMF_RSPEC_ISHEGI(rspec) \
	(RSPEC_HE_LTF_GI(rspec) > BRCMF_RSPEC_HE_1x_LTF_GI_0_8us)
#define HE_GI_TO_RSPEC(gi) \
	(((u32)(gi) << BRCMF_RSPEC_HE_GI_SHIFT) & BRCMF_RSPEC_HE_GI_MASK)
#define HE_GI_TO_RSPEC_SET(rspec, gi) \
	((rspec & (~BRCMF_RSPEC_HE_GI_MASK)) | HE_GI_TO_RSPEC(gi))

/* Macros for HE LTF and GI */
#define HE_IS_1X_LTF(gi) ((gi) == BRCMF_RSPEC_HE_1x_LTF_GI_0_8us)
#define HE_IS_2X_LTF(gi)                             \
	(((gi) == BRCMF_RSPEC_HE_2x_LTF_GI_0_8us) || \
	 ((gi) == BRCMF_RSPEC_HE_2x_LTF_GI_1_6us))
#define HE_IS_4X_LTF(gi) ((gi) == BRCMF_RSPEC_HE_4x_LTF_GI_3_2us)

#define HE_IS_GI_0_8us(gi)                           \
	(((gi) == BRCMF_RSPEC_HE_1x_LTF_GI_0_8us) || \
	 ((gi) == BRCMF_RSPEC_HE_2x_LTF_GI_0_8us))
#define HE_IS_GI_1_6us(gi) ((gi) == BRCMF_RSPEC_HE_2x_LTF_GI_1_6us)
#define HE_IS_GI_3_2us(gi) ((gi) == BRCMF_RSPEC_HE_4x_LTF_GI_3_2us)

/* RSPEC Macros for extracting and using HE-ER and DCM */
#define BRCMF_RSPEC_HE_DCM(rspec) \
	(((rspec) & BRCMF_RSPEC_DCM) >> BRCMF_RSPEC_DCM_SHIFT)
#define BRCMF_RSPEC_HE_ER(rspec) \
	(((rspec) & BRCMF_RSPEC_ER_MASK) >> BRCMF_RSPEC_ER_SHIFT)
#define BRCMF_RSPEC_HE_ER_ENAB(rspec) \
	(((rspec) & BRCMF_RSPEC_ER_ENAB_MASK) >> BRCMF_RSPEC_ER_ENAB_SHIFT)
#define BRCMF_RSPEC_HE_ER_TONE(rspec) \
	(((rspec) & BRCMF_RSPEC_ER_TONE_MASK) >> BRCMF_RSPEC_ER_TONE_SHIFT)
/* ======== RSPEC_RATE field ======== */

/* Encoding 0 - legacy rate */
/* DSSS, CCK, and OFDM rates in [500kbps] units */
#define BRCMF_RSPEC_LEGACY_RATE_MASK 0x0000007F
#define WLC_RATE_1M 2
#define WLC_RATE_2M 4
#define WLC_RATE_5M5 11
#define WLC_RATE_11M 22
#define WLC_RATE_6M 12
#define WLC_RATE_9M 18
#define WLC_RATE_12M 24
#define WLC_RATE_18M 36
#define WLC_RATE_24M 48
#define WLC_RATE_36M 72
#define WLC_RATE_48M 96
#define WLC_RATE_54M 108

/* Encoding 1 - HT MCS */
/**< HT MCS value mask in rspec */
#define BRCMF_RSPEC_HT_MCS_MASK 0x0000007F

/* Encoding >= 2 */
/* NSS & MCS values mask in rspec */
#define BRCMF_RSPEC_NSS_MCS_MASK 0x000000FF
/* mimo MCS value mask in rspec */
#define BRCMF_RSPEC_MCS_MASK 0x0000000F
/* mimo NSS value mask in rspec */
#define BRCMF_RSPEC_NSS_MASK 0x000000F0
/* mimo NSS value shift in rspec */
#define BRCMF_RSPEC_NSS_SHIFT 4

/* Encoding 2 - VHT MCS + NSS */
/**< VHT MCS value mask in rspec */
#define BRCMF_RSPEC_VHT_MCS_MASK BRCMF_RSPEC_MCS_MASK
/**< VHT Nss value mask in rspec */
#define BRCMF_RSPEC_VHT_NSS_MASK BRCMF_RSPEC_NSS_MASK
/**< VHT Nss value shift in rspec */
#define BRCMF_RSPEC_VHT_NSS_SHIFT BRCMF_RSPEC_NSS_SHIFT

/* Encoding 3 - HE MCS + NSS */
/**< HE MCS value mask in rspec */
#define BRCMF_RSPEC_HE_MCS_MASK BRCMF_RSPEC_MCS_MASK
/**< HE Nss value mask in rspec */
#define BRCMF_RSPEC_HE_NSS_MASK BRCMF_RSPEC_NSS_MASK
/**< HE Nss value shift in rpsec */
#define BRCMF_RSPEC_HE_NSS_SHIFT BRCMF_RSPEC_NSS_SHIFT

#define BRCMF_RSPEC_HE_NSS_UNSPECIFIED 0xf

/* Encoding 4 - EHT MCS + NSS */
/**< EHT MCS value mask in rspec */
#define BRCMF_RSPEC_EHT_MCS_MASK BRCMF_RSPEC_MCS_MASK
/**< EHT Nss value mask in rspec */
#define BRCMF_RSPEC_EHT_NSS_MASK BRCMF_RSPEC_NSS_MASK
/**< EHT Nss value shift in rpsec */
#define BRCMF_RSPEC_EHT_NSS_SHIFT BRCMF_RSPEC_NSS_SHIFT

/* ======== RSPEC_BW field ======== */

#define BRCMF_RSPEC_BW_UNSPECIFIED 0u
#define BRCMF_RSPEC_BW_20MHZ 0x00010000u
#define BRCMF_RSPEC_BW_40MHZ 0x00020000u
#define BRCMF_RSPEC_BW_80MHZ 0x00030000u
#define BRCMF_RSPEC_BW_160MHZ 0x00040000u
#define BRCMF_RSPEC_BW_320MHZ 0x00060000u

/* ======== RSPEC_ENCODING field ======== */

/* NOTE: Assuming the rate field is always NSS+MCS starting from VHT encoding!
 *       Modify/fix RSPEC_ISNSSMCS() macro if above condition changes any time.
 */
/**< Legacy rate is stored in RSPEC_RATE */
#define BRCMF_RSPEC_ENCODE_RATE 0x00000000u
/**< HT MCS is stored in RSPEC_RATE */
#define BRCMF_RSPEC_ENCODE_HT 0x01000000u
/**< VHT MCS and NSS are stored in RSPEC_RATE */
#define BRCMF_RSPEC_ENCODE_VHT 0x02000000u
/**< HE MCS and NSS are stored in RSPEC_RATE */
#define BRCMF_RSPEC_ENCODE_HE 0x03000000u
/**< EHT MCS and NSS are stored in RSPEC_RATE */
#define BRCMF_RSPEC_ENCODE_EHT 0x04000000u

/**
 * ===============================
 * Handy macros to parse rate spec
 * ===============================
 */
#define BRCMF_RSPEC_BW(rspec) ((rspec) & BRCMF_RSPEC_BW_MASK)
#define BRCMF_RSPEC_IS20MHZ(rspec) (RSPEC_BW(rspec) == BRCMF_RSPEC_BW_20MHZ)
#define BRCMF_RSPEC_IS40MHZ(rspec) (RSPEC_BW(rspec) == BRCMF_RSPEC_BW_40MHZ)
#define BRCMF_RSPEC_IS80MHZ(rspec) (RSPEC_BW(rspec) == BRCMF_RSPEC_BW_80MHZ)
#define BRCMF_RSPEC_IS160MHZ(rspec) (RSPEC_BW(rspec) == BRCMF_RSPEC_BW_160MHZ)
#if defined(WL_BW320MHZ)
#define BRCMF_RSPEC_IS320MHZ(rspec) (RSPEC_BW(rspec) == BRCMF_RSPEC_BW_320MHZ)
#else
#define BRCMF_RSPEC_IS320MHZ(rspec) (FALSE)
#endif /* WL_BW320MHZ */

#define BRCMF_RSPEC_BW_GE(rspec, rspec_bw) (RSPEC_BW(rspec) >= rspec_bw)
#define BRCMF_RSPEC_BW_LE(rspec, rspec_bw) (RSPEC_BW(rspec) <= rspec_bw)
#define BRCMF_RSPEC_BW_GT(rspec, rspec_bw) (!RSPEC_BW_LE(rspec, rspec_bw))
#define BRCMF_RSPEC_BW_LT(rspec, rspec_bw) (!RSPEC_BW_GE(rspec, rspec_bw))

#define BRCMF_RSPEC_ISSGI(rspec) (((rspec) & BRCMF_RSPEC_SGI) != 0)
#define BRCMF_RSPEC_ISLDPC(rspec) (((rspec) & BRCMF_RSPEC_LDPC) != 0)
#define BRCMF_RSPEC_ISSTBC(rspec) (((rspec) & BRCMF_RSPEC_STBC) != 0)
#define BRCMF_RSPEC_ISTXBF(rspec) (((rspec) & BRCMF_RSPEC_TXBF) != 0)

#define BRCMF_RSPEC_TXEXP(rspec) \
	(((rspec) & BRCMF_RSPEC_TXEXP_MASK) >> BRCMF_RSPEC_TXEXP_SHIFT)

#define BRCMF_RSPEC_ENCODE(rspec) \
	(((rspec) & BRCMF_RSPEC_ENCODING_MASK) >> BRCMF_RSPEC_ENCODING_SHIFT)
#define BRCMF_RSPEC_ISLEGACY(rspec) \
	(((rspec) & BRCMF_RSPEC_ENCODING_MASK) == BRCMF_RSPEC_ENCODE_RATE)

#define BRCMF_RSPEC_ISHT(rspec) \
	(((rspec) & BRCMF_RSPEC_ENCODING_MASK) == BRCMF_RSPEC_ENCODE_HT)
#define BRCMF_RSPEC_ISVHT(rspec) \
	(((rspec) & BRCMF_RSPEC_ENCODING_MASK) == BRCMF_RSPEC_ENCODE_VHT)
#define BRCMF_RSPEC_ISHE(rspec) \
	(((rspec) & BRCMF_RSPEC_ENCODING_MASK) == BRCMF_RSPEC_ENCODE_HE)
#define BRCMF_RSPEC_ISEHT(rspec) \
	(((rspec) & BRCMF_RSPEC_ENCODING_MASK) == BRCMF_RSPEC_ENCODE_EHT)

/* fast check if rate field is NSS+MCS format (starting from VHT ratespec) */
#define BRCMF_RSPEC_ISVHTEXT(rspec) \
	(((rspec) & BRCMF_RSPEC_ENCODING_MASK) >= BRCMF_RSPEC_ENCODE_VHT)
/* fast check if rate field is NSS+MCS format (starting from HE ratespec) */
#define BRCMF_RSPEC_ISHEEXT(rspec) \
	(((rspec) & BRCMF_RSPEC_ENCODING_MASK) >= BRCMF_RSPEC_ENCODE_HE)

#endif /* BRCMFMAC_RATESPEC_H */

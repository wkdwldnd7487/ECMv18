/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     EncAppCfg.h
    \brief    Handle encoder configuration parameters (header)
*/

#ifndef __ENCAPPCFG__
#define __ENCAPPCFG__

#include "CommonLib/CommonDef.h"
#include "EncoderLib/EncCfgParam.h"

#include <map>
template <class T1, class T2>
static inline std::istream& operator >> (std::istream &in, std::map<T1, T2> &map);

#include "Utilities/program_options_lite.h"

#include "EncoderLib/EncCfg.h"
#if EXTENSION_360_VIDEO
#include "AppEncHelper360/TExt360AppEncCfg.h"
#endif

#if JVET_O0756_CALCULATE_HDRMETRICS
#include "HDRLib/inc/DistortionMetric.H"
#endif
namespace po = df::program_options_lite;

#include <sstream>
#include <vector>
//! \ingroup EncoderApp
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder configuration class
class EncAppCfg
{
#if QP_SWITCHING_FOR_PARALLEL
public:
  template <class T>
  struct OptionalValue
  {
    bool bPresent;
    T    value;
    OptionalValue() : bPresent(false), value() { }
  };
#endif

protected:
  // file I/O
  std::string m_inputFileName;                                ///< source file name
  std::string m_bitstreamFileName;                            ///< output bitstream file
  std::string m_reconFileName;                                ///< output reconstruction file

  // Lambda modifiers
  double    m_adLambdaModifier[ MAX_TLAYER ];                 ///< Lambda modifier array for each temporal layer
  std::vector<double> m_adIntraLambdaModifier;                ///< Lambda modifier for Intra pictures, one for each temporal layer. If size>temporalLayer, then use [temporalLayer], else if size>0, use [size()-1], else use m_adLambdaModifier.
  double    m_dIntraQpFactor;                                 ///< Intra Q Factor. If negative, use a default equation: 0.57*(1.0 - Clip3( 0.0, 0.5, 0.05*(double)(isField ? (GopSize-1)/2 : GopSize-1) ))

  // source specification
  int       m_iFrameRate;                                     ///< source frame-rates (Hz)
  uint32_t      m_FrameSkip;                                      ///< number of skipped frames from the beginning
  uint32_t      m_temporalSubsampleRatio;                         ///< temporal subsample ratio, 2 means code every two frames
#if JVET_AA0146_WRAP_AROUND_FIX
  int       m_sourceWidth;                                   ///< source width in pixel
  int       m_sourceHeight;                                  ///< source height in pixel (when interlaced = field height)
#else
  int       m_iSourceWidth;                                   ///< source width in pixel
  int       m_iSourceHeight;                                  ///< source height in pixel (when interlaced = field height)
#endif
#if EXTENSION_360_VIDEO
  int       m_inputFileWidth;                                 ///< width of image in input file  (this is equivalent to sourceWidth,  if sourceWidth  is not subsequently altered due to padding)
  int       m_inputFileHeight;                                ///< height of image in input file (this is equivalent to sourceHeight, if sourceHeight is not subsequently altered due to padding)
#endif
  int       m_iSourceHeightOrg;                               ///< original source height in pixel (when interlaced = frame height)

  bool      m_isField;                                        ///< enable field coding
  bool      m_isTopFieldFirst;
  bool      m_bEfficientFieldIRAPEnabled;                     ///< enable an efficient field IRAP structure.
  bool      m_bHarmonizeGopFirstFieldCoupleEnabled;

  int       m_conformanceWindowMode;
  int       m_confWinLeft;
  int       m_confWinRight;
  int       m_confWinTop;
  int       m_confWinBottom;
  int       m_firstValidFrame;
  int       m_lastValidFrame;
  int       m_framesToBeEncoded;                              ///< number of encoded frames
#if JVET_AA0146_WRAP_AROUND_FIX
  int       m_sourcePadding[2];                               ///< number of padded pixels for width and height
#else
  int       m_aiPad[2];                                       ///< number of padded pixels for width and height
#endif
  bool      m_AccessUnitDelimiter;                            ///< add Access Unit Delimiter NAL units
  bool      m_enablePictureHeaderInSliceHeader;               ///< Enable Picture Header in Slice Header

  InputColourSpaceConversion m_inputColourSpaceConvert;       ///< colour space conversion to apply to input video
  bool      m_snrInternalColourSpace;                       ///< if true, then no colour space conversion is applied for snr calculation, otherwise inverse of input is applied.
  bool      m_outputInternalColourSpace;                    ///< if true, then no colour space conversion is applied for reconstructed video, otherwise inverse of input is applied.
  ChromaFormat m_InputChromaFormatIDC;

  bool      m_printMSEBasedSequencePSNR;
  bool      m_printHexPsnr;
  bool      m_printFrameMSE;
  bool      m_printSequenceMSE;
#if MSSIM_UNIFORM_METRICS_LOG
  bool      m_printMSSSIM;
#endif
  bool      m_cabacZeroWordPaddingEnabled;
  bool      m_bClipInputVideoToRec709Range;
  bool      m_bClipOutputVideoToRec709Range;
  bool      m_packedYUVMode;                                  ///< If true, output 10-bit and 12-bit YUV data as 5-byte and 3-byte (respectively) packed YUV data

#if JVET_S0179_CONDITIONAL_SIGNAL_GCI
  bool      m_gciPresentFlag;
#endif
  bool      m_bIntraOnlyConstraintFlag;
  uint32_t  m_maxBitDepthConstraintIdc;
  int       m_maxChromaFormatConstraintIdc;
#if !JVET_S0138_GCI_PTL
  bool      m_singleLayerConstraintFlag;
#endif
  bool      m_allLayersIndependentConstraintFlag;
  bool      m_noMrlConstraintFlag;
  bool      m_noIspConstraintFlag;
  bool      m_noMipConstraintFlag;
  bool      m_noLfnstConstraintFlag;
  bool      m_noMmvdConstraintFlag;
  bool      m_noSmvdConstraintFlag;
  bool      m_noProfConstraintFlag;
  bool      m_noPaletteConstraintFlag;
  bool      m_noActConstraintFlag;
  bool      m_noLmcsConstraintFlag;
#if JVET_S0050_GCI
  bool      m_noExplicitScaleListConstraintFlag;
  bool      m_noVirtualBoundaryConstraintFlag;
#endif
#if JVET_S0058_GCI
  bool      m_noMttConstraintFlag;
#endif
#if JVET_R0341_GCI
  bool      m_noChromaQpOffsetConstraintFlag;
#endif
  bool      m_noQtbttDualTreeIntraConstraintFlag;
#if JVET_S0066_GCI
  int       m_maxLog2CtuSizeConstraintIdc;
#endif
  bool      m_noPartitionConstraintsOverrideConstraintFlag;
  bool      m_noSaoConstraintFlag;
#if JVET_W0066_CCSAO
  bool      m_noCCSaoConstraintFlag;
#endif
  bool      m_noAlfConstraintFlag;
  bool      m_noCCAlfConstraintFlag;
#if JVET_S0058_GCI
  bool      m_noWeightedPredictionConstraintFlag;
#endif
  bool      m_noRefWraparoundConstraintFlag;
  bool      m_noTemporalMvpConstraintFlag;
  bool      m_noSbtmvpConstraintFlag;
  bool      m_noAmvrConstraintFlag;
  bool      m_noBdofConstraintFlag;
  bool      m_noDmvrConstraintFlag;
  bool      m_noCclmConstraintFlag;
  bool      m_noMtsConstraintFlag;
  bool      m_noSbtConstraintFlag;
  bool      m_noAffineMotionConstraintFlag;
  bool      m_noBcwConstraintFlag;
  bool      m_noIbcConstraintFlag;
#if ENABLE_DIMD
  bool      m_noDimdConstraintFlag;
#endif
#if JVET_W0123_TIMD_FUSION
  bool      m_noTimdConstraintFlag;
#endif
#if JVET_AB0155_SGPM
  bool      m_noSgpmConstraintFlag;
#endif
#if JVET_AD0082_TMRL_CONFIG
  bool      m_noTmrlConstraintFlag;
#endif
#if JVET_AG0058_EIP
  bool      m_noEipConstraintFlag;
#endif
#if JVET_AK0118_BF_FOR_INTRA_PRED
  bool      m_noIntraPredBfConstraintFlag;
#endif
#if ENABLE_OBMC
  bool      m_noObmcConstraintFlag;
#endif
  bool      m_noCiipConstraintFlag;
  bool      m_noGeoConstraintFlag;
  bool      m_noLadfConstraintFlag;
  bool      m_noTransformSkipConstraintFlag;
#if JVET_S0066_GCI
  bool      m_noLumaTransformSize64ConstraintFlag;
#endif
  bool      m_noBDPCMConstraintFlag;
  bool      m_noJointCbCrConstraintFlag;
  bool      m_noQpDeltaConstraintFlag;
  bool      m_noDepQuantConstraintFlag;
  bool      m_noSignDataHidingConstraintFlag;
  bool      m_noTrailConstraintFlag;
  bool      m_noStsaConstraintFlag;
  bool      m_noRaslConstraintFlag;
  bool      m_noRadlConstraintFlag;
  bool      m_noIdrConstraintFlag;
  bool      m_noCraConstraintFlag;
  bool      m_noGdrConstraintFlag;
  bool      m_noApsConstraintFlag;

  // profile/level
  Profile::Name m_profile;
  Level::Tier   m_levelTier;
  Level::Name   m_level;
#if JVET_S0138_GCI_PTL
  bool          m_frameOnlyConstraintFlag;
  bool          m_multiLayerEnabledFlag;
#endif
  std::vector<uint32_t>  m_subProfile;
  uint8_t      m_numSubProfile;

  uint32_t          m_bitDepthConstraint;
  ChromaFormat  m_chromaFormatConstraint;
  bool          m_onePictureOnlyConstraintFlag;
  bool          m_intraOnlyConstraintFlag;
  bool          m_nonPackedConstraintFlag;
  bool          m_nonProjectedConstraintFlag;
#if JVET_Q0114_ASPECT5_GCI_FLAG
  bool          m_noRprConstraintFlag;
#endif
  bool          m_noResChangeInClvsConstraintFlag;
  bool          m_oneTilePerPicConstraintFlag;
  bool          m_picHeaderInSliceHeaderConstraintFlag;
  bool          m_oneSlicePerPicConstraintFlag;
#if JVET_S0113_S0195_GCI
  bool          m_noIdrRplConstraintFlag;
  bool          m_noRectSliceConstraintFlag;
  bool          m_oneSlicePerSubpicConstraintFlag;
  bool          m_noSubpicInfoConstraintFlag;
#else
  bool          m_oneSubpicPerPicConstraintFlag;
#endif
#if !JVET_S0138_GCI_PTL
  bool          m_frameOnlyConstraintFlag;
#endif
  // coding structure
  int       m_iIntraPeriod;                                   ///< period of I-slice (random access period)
#if JVET_Z0118_GDR
  bool      m_gdrEnabled;
  int       m_gdrPocStart;
  int       m_gdrPeriod;
  int       m_gdrInterval;  
  bool      m_gdrNoHash;  
#endif
  int       m_iDecodingRefreshType;                           ///< random access type
  int       m_iGOPSize;                                       ///< GOP size of hierarchical structure
  int       m_drapPeriod;                                     ///< period of dependent RAP pictures
  bool      m_rewriteParamSets;                              ///< Flag to enable rewriting of parameter sets at random access points
  RPLEntry  m_RPLList0[MAX_GOP];                               ///< the RPL entries from the config file
  RPLEntry  m_RPLList1[MAX_GOP];                               ///< the RPL entries from the config file
  bool      m_idrRefParamList;                                ///< indicates if reference picture list syntax elements are present in slice headers of IDR pictures
  GOPEntry  m_GOPList[MAX_GOP];                               ///< the coding structure entries from the config file
  int       m_numReorderPics[MAX_TLAYER];                     ///< total number of reorder pictures
  int       m_maxDecPicBuffering[MAX_TLAYER];                 ///< total number of pictures in the decoded picture buffer
  bool      m_reconBasedCrossCPredictionEstimate;             ///< causes the alpha calculation in encoder search to be based on the decoded residual rather than the pre-transform encoder-side residual
  bool      m_useTransformSkip;                               ///< flag for enabling intra transform skipping
  bool      m_useTransformSkipFast;                           ///< flag for enabling fast intra transform skipping
  bool      m_useBDPCM;
  uint32_t      m_log2MaxTransformSkipBlockSize;                  ///< transform-skip maximum size (minimum of 2)
  bool      m_transformSkipRotationEnabledFlag;               ///< control flag for transform-skip/transquant-bypass residual rotation
  bool      m_transformSkipContextEnabledFlag;                ///< control flag for transform-skip/transquant-bypass single significance map context
  bool      m_persistentRiceAdaptationEnabledFlag;            ///< control flag for Golomb-Rice parameter adaptation over each slice
  bool      m_cabacBypassAlignmentEnabledFlag;
  bool      m_ISP;
  bool      m_useFastISP;                                    ///< flag for enabling fast methods for ISP
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
  bool      m_isRA;
  int       m_numQPOffset;
  int       m_qpOffsetList[MAX_GOP];
#endif

  // coding quality
#if QP_SWITCHING_FOR_PARALLEL
  OptionalValue<uint32_t> m_qpIncrementAtSourceFrame;             ///< Optional source frame number at which all subsequent frames are to use an increased internal QP.
#else
  double    m_fQP;                                            ///< QP value of key-picture (floating point)
#endif
  int       m_iQP;                                            ///< QP value of key-picture (integer)
  bool      m_useIdentityTableForNon420Chroma;
  ChromaQpMappingTableParams m_chromaQpMappingTableParams;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  int       m_intraQPOffset;                                  ///< QP offset for intra slice (integer)
  bool      m_lambdaFromQPEnable;                             ///< enable flag for QP:lambda fix
#endif
  std::string m_dQPFileName;                                  ///< QP offset for each slice (initialized from external file)
  int*      m_aidQP;                                          ///< array of slice QP values
  int       m_iMaxDeltaQP;                                    ///< max. |delta QP|
  uint32_t      m_uiDeltaQpRD;                                    ///< dQP range for multi-pass slice QP optimization
  int       m_cuQpDeltaSubdiv;                                ///< Maximum subdiv for CU luma Qp adjustment (0:default)
  int       m_cuChromaQpOffsetSubdiv;                         ///< If negative, then do not apply chroma qp offsets.
  bool      m_bFastDeltaQP;                                   ///< Fast Delta QP (false:default)

  int       m_cbQpOffset;                                     ///< Chroma Cb QP Offset (0:default)
  int       m_crQpOffset;                                     ///< Chroma Cr QP Offset (0:default)
  int       m_cbQpOffsetDualTree;                             ///< Chroma Cb QP Offset for dual tree (overwrite m_cbQpOffset for dual tree)
  int       m_crQpOffsetDualTree;                             ///< Chroma Cr QP Offset for dual tree (overwrite m_crQpOffset for dual tree)
  int       m_cbCrQpOffset;                                   ///< QP Offset for joint Cb-Cr mode
  int       m_cbCrQpOffsetDualTree;                           ///< QP Offset for joint Cb-Cr mode (overwrite m_cbCrQpOffset for dual tree)
#if ER_CHROMA_QP_WCG_PPS
  WCGChromaQPControl m_wcgChromaQpControl;                    ///< Wide-colour-gamut chroma QP control.
#endif
#if W0038_CQP_ADJ
  uint32_t      m_sliceChromaQpOffsetPeriodicity;                 ///< Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.
  int       m_sliceChromaQpOffsetIntraOrPeriodic[2/*Cb,Cr*/]; ///< Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.
#endif
#if SHARP_LUMA_DELTA_QP
  LumaLevelToDeltaQPMapping m_lumaLevelToDeltaQPMapping;      ///< mapping from luma level to Delta QP.
#endif
  SEIMasteringDisplay m_masteringDisplay;

  bool      m_bUseAdaptiveQP;                                 ///< Flag for enabling QP adaptation based on a psycho-visual model
  int       m_iQPAdaptationRange;                             ///< dQP range by QP adaptation
#if ENABLE_QPA
  bool      m_bUsePerceptQPA;                                 ///< Flag to enable perceptually motivated input-adaptive QP modification
  bool      m_bUseWPSNR;                                      ///< Flag to output perceptually weighted peak SNR (WPSNR) instead of PSNR
#endif
  int       m_maxTempLayer;                                   ///< Max temporal layer

  // coding unit (CU) definition
  unsigned  m_uiCTUSize;
  bool m_subPicInfoPresentFlag;
  unsigned m_numSubPics;
#if JVET_S0071_SAME_SIZE_SUBPIC_LAYOUT
  bool m_subPicSameSizeFlag;
#endif
  std::vector<uint32_t> m_subPicCtuTopLeftX;
  std::vector<uint32_t> m_subPicCtuTopLeftY;
  std::vector<uint32_t> m_subPicWidth;
  std::vector<uint32_t> m_subPicHeight;
  std::vector<bool>     m_subPicTreatedAsPicFlag;
  std::vector<bool>     m_loopFilterAcrossSubpicEnabledFlag;
  bool m_subPicIdMappingExplicitlySignalledFlag;
  bool m_subPicIdMappingInSpsFlag;
  unsigned m_subPicIdLen;
  std::vector<uint16_t> m_subPicId;
  bool      m_SplitConsOverrideEnabledFlag;
#if JVET_X0144_MAX_MTT_DEPTH_TID
  std::string m_sMaxMTTHierarchyDepthByTid;
  unsigned m_maxMTTHierarchyDepthByTid[MAX_TLAYER];
#endif
  unsigned  m_uiMinQT[3]; // 0: I slice luma; 1: P/B slice; 2: I slice chroma
  unsigned  m_uiMaxMTTHierarchyDepth;
  unsigned  m_uiMaxMTTHierarchyDepthI;
  unsigned  m_uiMaxMTTHierarchyDepthIChroma;
  unsigned  m_uiMaxBT[3];
  unsigned  m_uiMaxTT[3];
#if AHG7_MTS_TOOLOFF_CFG
  bool      m_MTSExt;
#endif
#if JVET_Y0152_TT_ENC_SPEEDUP
  int       m_ttFastSkip;
  double    m_ttFastSkipThr;
#endif
  bool      m_dualTree;
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  bool      m_interSliceSeparateTreeEnabled;
#endif
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  bool      m_intraLFNSTISlice;
  bool      m_intraLFNSTPBSlice;
  bool      m_interLFNST;
#else
  bool      m_LFNST;
#endif
  bool      m_useFastLFNST;
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  bool      m_useFastInterLFNST;
#endif
#if JVET_AI0050_INTER_MTSS
  bool      m_useInterMTSS;
#endif
#if JVET_AI0050_SBT_LFNST
  bool      m_useSbtLFNST;
#endif
#if AHG7_LN_TOOLOFF_CFG
  bool      m_NSPT;
  bool      m_LFNSTExt;
#endif
  bool      m_sbTmvpEnableFlag;
  bool      m_Affine;
  bool      m_AffineType;
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
  bool      m_useAltCost;
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
  bool      m_useExtAmvp;
#endif
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
  bool      m_useAffineTM;
#if JVET_AG0276_NLIC
  bool      m_useAffAltLMTM;
#endif
#if JVET_AH0119_SUBBLOCK_TM
  bool      m_useSbTmvpTM;
#endif
#endif
#if JVET_AG0135_AFFINE_CIIP
  bool      m_useCiipAffine;
#endif
#if AFFINE_MMVD
  bool      m_AffineMmvdMode;
#endif
#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM || MULTI_PASS_DMVR
  bool      m_DMVDMode;
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  bool      m_tmToolsEnableFlag;
#if TM_AMVP
  bool      m_tmAmvpMode;
#endif
#if TM_MRG
  bool      m_tmMrgMode;
#endif
#if JVET_W0097_GPM_MMVD_TM && TM_MRG
  bool      m_tmGPMMode;
#endif
#if JVET_Z0061_TM_OBMC && ENABLE_OBMC
  bool      m_tmOBMCMode;
#endif
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
  int       m_tmCIIPMode;
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
  bool      m_useTmvpNmvpReorder;
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  bool      m_useTMMMVD;
#endif
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  bool      m_altGPMSplitModeCode;
#endif
#if JVET_AL0134_SGPM_INTER
  bool      m_useSgpmInter;
#endif
  bool      m_PROF;
  bool      m_BIO;
#if JVET_W0090_ARMC_TM
  bool      m_AML;
#if JVET_AG0276_NLIC
  bool      m_altLM;
  bool      m_affAltLM;
#endif
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
  bool      m_mergeOppositeLic;
  bool      m_mergeTMOppositeLic;
  bool      m_mergeAffOppositeLic;
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  bool      m_armcRefinedMotion;
#endif
  int       m_LMChroma;
  bool      m_horCollocatedChromaFlag;
  bool      m_verCollocatedChromaFlag;
  int       m_MTS;                                            ///< XZ: Multiple Transform Set
  int       m_MTSIntraMaxCand;                                ///< XZ: Number of additional candidates to test
  int       m_MTSInterMaxCand;                                ///< XZ: Number of additional candidates to test
  int       m_MTSImplicit;
  bool      m_SBT;                                            ///< Sub-Block Transform for inter blocks
  int       m_SBTFast64WidthTh;
  bool      m_SMVD;
  bool      m_compositeRefEnabled;
  bool      m_bcw;
  bool      m_BcwFast;
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  bool      m_LadfEnabed;
  int       m_LadfNumIntervals;
  std::vector<int> m_LadfQpOffset;
  int       m_LadfIntervalLowerBound[MAX_LADF_INTERVALS];
#endif
#if JVET_AA0133_INTER_MTS_OPT
  int       m_interMTSMaxSize;
#endif
#if AHG7_MTS_TOOLOFF_CFG
  int       m_intraMTSMaxSize;
#endif
#if ENABLE_DIMD
  bool      m_dimd;
#endif
#if JVET_AL0108_BVG_DIMD
  bool      m_bvgDimd;
#endif
#if JVET_W0123_TIMD_FUSION
  bool      m_timd;
#if JVET_AJ0061_TIMD_MERGE
  bool      m_timdMrg;
#endif
#endif
#if JVET_AB0155_SGPM
  bool      m_sgpm;
#if JVET_AC0189_SGPM_NO_BLENDING
  bool      m_sgpmNoBlend;
#endif
#endif
#if JVET_AD0082_TMRL_CONFIG
  bool      m_tmrl;
#endif
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  bool      m_tmNoninterToolsEnableFlag;
#endif
#if JVET_AG0058_EIP
  bool      m_eip;
#endif
#if JVET_AK0118_BF_FOR_INTRA_PRED
  bool      m_intraPredBf;
#endif
#if JVET_AD0085_MPM_SORTING
  bool      m_mpmSorting;
#endif
#if JVET_AK0059_MDIP
  bool      m_mdip;
#endif
#if JVET_AH0136_CHROMA_REORDERING
  bool      m_chromaReordering;
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  int       m_cccm;
#endif
#if JVET_AD0188_CCP_MERGE
  bool      m_ccpMerge;
#endif
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
  int       m_maxNumCcpMergeAdjustCand;
#endif
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  bool      m_ddCcpFusion;
#endif
#if ENABLE_OBMC
  bool      m_OBMC;
#endif
  bool      m_ciip;
#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
  bool      m_ciipTimd;
#endif
  bool      m_Geo;
#if JVET_AJ0107_GPM_SHAPE_ADAPT
  bool      m_geoShapeAdapt;
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
  bool      m_geoInterIbc;
#endif
#if JVET_AK0101_REGRESSION_GPM_INTRA
  bool      m_geoBlendIntra;
#endif
  bool      m_HashME;
  bool      m_allowDisFracMMVD;
  bool      m_AffineAmvr;
  bool      m_AffineAmvrEncOpt;
  bool      m_AffineAmvp;
  bool      m_DMVR;
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  bool      m_affineParaRefinement;
#endif
  bool      m_MMVD;
  int       m_MmvdDisNum;
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION
  bool      m_mvdPred;
#endif
#if JVET_AC0104_IBC_BVD_PREDICTION
  bool      m_bvdPred;
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  bool      m_bvpCluster;
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
  bool      m_useARL;
#endif
  bool      m_rgbFormat;
  bool      m_useColorTrans;
  unsigned  m_PLTMode;
  bool      m_JointCbCrMode;
  bool      m_useChromaTS;
  unsigned  m_IBCMode;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  unsigned  m_IBCFracMode;
#endif
  unsigned  m_IBCLocalSearchRangeX;
  unsigned  m_IBCLocalSearchRangeY;
  unsigned  m_IBCHashSearch;
  unsigned  m_IBCHashSearchMaxCand;
  unsigned  m_IBCHashSearchRange4SmallBlk;
  unsigned  m_IBCFastMethod;
#if JVET_AF0057
  bool      m_dmvrEncSelect;
  int       m_dmvrEncSelectBaseQpTh;
  bool      m_dmvrEncSelectDisableHighestTemporalLayer;
#endif
#if JVET_AA0061_IBC_MBVD
  bool      m_ibcMbvd;
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
  bool      m_ibcMbvdAdSearch;
#endif
#endif
#if JVET_AC0112_IBC_CIIP
  bool     m_ibcCiip;
#endif
#if JVET_AC0112_IBC_GPM
  bool      m_ibcGpm;
#endif
#if JVET_AC0112_IBC_LIC
  bool      m_ibcLic;
#endif
#if JVET_AE0159_FIBC
  bool      m_ibcFilter;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  bool      m_ibcBiPred;
#endif
#if JVET_AE0094_IBC_NONADJACENT_SPATIAL_CANDIDATES
  bool      m_ibcNonAdjCand;
#endif
#if JVET_AG0136_INTRA_TMP_LIC
  bool      m_itmpLicExtension;
  bool      m_itmpLicMode;
#endif
#if JVET_AJ0057_HL_INTRA_METHOD_CONTROL
  int       m_intraToolControlMode;
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  bool      m_rribc;
  bool      m_tmibc;
  bool      m_ibcMerge;
#endif

  bool      m_wrapAround;
  unsigned  m_wrapAroundOffset;
#if JVET_AH0135_TEMPORAL_PARTITIONING
  bool      m_enableMaxMttIncrease;
#endif
#if MULTI_HYP_PRED
  int       m_numMHPCandsToTest;
  int       m_maxNumAddHyps;                                  ///< max. number of additional inter hypotheseis
  int       m_numAddHypWeights;                               ///< number of weights for additional inter hypotheseis
  int       m_maxNumAddHypRefFrames;                          ///< max. number of ref frames for additional inter hypotheseis
  int       m_addHypTries;                                    ///< max. number of tries for additional inter hypotheseis
#endif
#if JVET_V0130_INTRA_TMP
  bool      m_intraTMP;                                       ///< intra Template Matching 
  unsigned  m_intraTmpMaxSize;                               ///< max CU size for which intra TMP is allowed
#if JVET_AB0130_ITMP_SAMPLING
  bool      m_fastIntraTMP;                                   ///< fast IntraTMP RD search
#endif
#endif
#if JVET_AC0071_DBV
  bool m_intraDBV; ///< Direct Block Vector
#endif
#if JVET_AE0059_INTER_CCCM
  bool m_interCccm; ///< CCCM for inter prediction
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  bool m_interCcpMerge; ///< Cross-component prediction merge for inter prediction
  int  m_interCcpMergeFastMode;
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
  bool m_interCcpMergeZeroLumaCbf;
  int m_interCcpMergeZeroLumaCbfFastMode;
#endif
#endif

#if JVET_AE0100_BVGCCCM
  bool m_bvgCccm; ///< Block vector guided CCCM
#endif
#if JVET_V0094_BILATERAL_FILTER
  bool      m_BIF;                                            ///< bilateral filter
  unsigned  m_BIFStrength;                                    /// Bilateral filter strength
  int       m_BIFQPOffset;                                    /// Bilateral QP offset
#endif
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  bool      m_chromaBIF;
  unsigned  m_chromaBIFStrength;
  int       m_chromaBIFQPOffset;
#endif
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  unsigned  m_tempCabacInitMode;
#endif
#if JVET_AH0209_PDP
  bool      m_pdp;
#endif
#if JVET_AI0183_MVP_EXTENSION
  bool      m_scaledMvExtTmvp;
  bool      m_scaledMvExtBiTmvp;
#endif
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
  bool      m_sbTmvpMvExt;
#endif
#if JVET_AL0157_MERGE_CANDIDATE_EXTENSION
  bool      m_boundaryMvpExt;
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  unsigned  m_refineTmvpCfgIdx;
#endif
  
  // ADD_NEW_TOOL : (encoder app) add tool enabling flags and associated parameters here
  bool      m_virtualBoundariesEnabledFlag;
  bool      m_virtualBoundariesPresentFlag;
  unsigned  m_numVerVirtualBoundaries;
  unsigned  m_numHorVirtualBoundaries;
  std::vector<unsigned> m_virtualBoundariesPosX;
  std::vector<unsigned> m_virtualBoundariesPosY;
  bool      m_lmcsEnabled;
  uint32_t  m_reshapeSignalType;
  uint32_t  m_intraCMD;
  ReshapeCW m_reshapeCW;
  int       m_updateCtrl;
  int       m_adpOption;
  uint32_t  m_initialCW;
  int       m_CSoffset;
  bool      m_encDbOpt;
  unsigned  m_uiMaxCUWidth;                                   ///< max. CU width in pixel
  unsigned  m_uiMaxCUHeight;                                  ///< max. CU height in pixel
  unsigned m_log2MinCuSize;                                   ///< min. CU size log2
#if JVET_AJ0226_MTT_SKIP
  bool m_useMttSkip;
#endif

  bool      m_useFastLCTU;
  bool      m_usePbIntraFast;
  bool      m_useAMaxBT;
  bool      m_useFastMrg;
#if MERGE_ENC_OPT
  uint32_t  m_numFullRDMrg;
#endif
  bool      m_e0023FastEnc;
  bool      m_contentBasedFastQtbt;
  bool      m_useNonLinearAlfLuma;
  bool      m_useNonLinearAlfChroma;
  unsigned  m_maxNumAlfAlternativesChroma;
  bool      m_MRL;
  bool      m_MIP;
  bool      m_useFastMIP;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  int       m_fastLocalDualTreeMode;
#endif

  int       m_numSplitThreads;
  bool      m_forceSplitSequential;
  int       m_numWppThreads;
  int       m_numWppExtraLines;
  bool      m_ensureWppBitEqual;

  int       m_log2MaxTbSize;
  // coding tools (bit-depth)
  int       m_inputBitDepth   [MAX_NUM_CHANNEL_TYPE];         ///< bit-depth of input file
  int       m_outputBitDepth  [MAX_NUM_CHANNEL_TYPE];         ///< bit-depth of output file
  int       m_MSBExtendedBitDepth[MAX_NUM_CHANNEL_TYPE];      ///< bit-depth of input samples after MSB extension
  int       m_internalBitDepth[MAX_NUM_CHANNEL_TYPE];         ///< bit-depth codec operates at (input/output files will be converted)
  bool      m_extendedPrecisionProcessingFlag;
  bool      m_highPrecisionOffsetsEnabledFlag;

  //coding tools (chroma format)
  ChromaFormat m_chromaFormatIDC;


  // coding tool (SAO)
  bool      m_bUseSAO;
#if JVET_W0066_CCSAO
  bool      m_CCSAO;
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  bool      m_alfPrecision;
#endif
#if JVET_AH0057_CCALF_COEFF_PRECISION
  bool      m_ccalfPrecision;
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
  bool      m_alfLumaFixedFilterAdjust;
#endif
#if JVET_AK0121_LOOPFILTER_OFFSET_REFINEMENT
  bool      m_inloopOffsetRefineFlag;
  bool      m_inloopOffsetRefineFunc;
#endif
  bool      m_bTestSAODisableAtPictureLevel;
  double    m_saoEncodingRate;                                ///< When >0 SAO early picture termination is enabled for luma and chroma
  double    m_saoEncodingRateChroma;                          ///< The SAO early picture termination rate to use for chroma (when m_SaoEncodingRate is >0). If <=0, use results for luma.
  int       m_maxNumOffsetsPerPic;                            ///< SAO maximun number of offset per picture
  bool      m_saoCtuBoundary;                                 ///< SAO parameter estimation using non-deblocked pixels for CTU bottom and right boundary areas
  bool      m_saoGreedyMergeEnc;                              ///< SAO greedy merge encoding algorithm
  // coding tools (loop filter)
#if JVET_AB0171_ASYMMETRIC_DB_FOR_GDR
  bool      m_asymmetricILF;
#endif

  bool      m_bLoopFilterDisable;                             ///< flag for using deblocking filter
  bool      m_loopFilterOffsetInPPS;                         ///< offset for deblocking filter in 0 = slice header, 1 = PPS
#if DB_PARAM_TID
  std::vector<int> m_loopFilterBetaOffsetDiv2;                     ///< beta offset for deblocking filter
  std::vector<int> m_loopFilterTcOffsetDiv2;                       ///< tc offset for deblocking filter
#else
  int       m_loopFilterBetaOffsetDiv2;                     ///< beta offset for deblocking filter
  int       m_loopFilterTcOffsetDiv2;                       ///< tc offset for deblocking filter
#endif

  int       m_loopFilterCbBetaOffsetDiv2;                     ///< beta offset for Cb deblocking filter
  int       m_loopFilterCbTcOffsetDiv2;                       ///< tc offset for Cb deblocking filter
  int       m_loopFilterCrBetaOffsetDiv2;                     ///< beta offset for Cr deblocking filter
  int       m_loopFilterCrTcOffsetDiv2;                       ///< tc offset for Cr deblocking filter
#if W0038_DB_OPT
  int       m_deblockingFilterMetric;                         ///< blockiness metric in encoder
#else
  bool      m_DeblockingFilterMetric;                         ///< blockiness metric in encoder
#endif
  bool      m_enableIntraReferenceSmoothing;                  ///< flag for enabling(default)/disabling intra reference smoothing/filtering
#if JVET_AK0085_TM_BOUNDARY_PADDING
  bool      m_templateMatchingBoundaryPrediction;
#endif

  // coding tools (encoder-only parameters)
  bool      m_bUseASR;                                        ///< flag for using adaptive motion search range
  bool      m_bUseHADME;                                      ///< flag for using HAD in sub-pel ME
  bool      m_useRDOQ;                                       ///< flag for using RD optimized quantization
  bool      m_useRDOQTS;                                     ///< flag for using RD optimized quantization for transform skip
#if T0196_SELECTIVE_RDOQ
  bool      m_useSelectiveRDOQ;                               ///< flag for using selective RDOQ
#endif
  int       m_rdPenalty;                                      ///< RD-penalty for 32x32 TU for intra in non-intra slices (0: no RD-penalty, 1: RD-penalty, 2: maximum RD-penalty)
  bool      m_bDisableIntraPUsInInterSlices;                  ///< Flag for disabling intra predicted PUs in inter slices.
  MESearchMethod m_motionEstimationSearchMethod;
  bool      m_bRestrictMESampling;                            ///< Restrict sampling for the Selective ME
  int       m_iSearchRange;                                   ///< ME search range
  int       m_bipredSearchRange;                              ///< ME search range for bipred refinement
  int       m_minSearchWindow;                                ///< ME minimum search window size for the Adaptive Window ME
  bool      m_bClipForBiPredMeEnabled;                        ///< Enables clipping for Bi-Pred ME.
  bool      m_bFastMEAssumingSmootherMVEnabled;               ///< Enables fast ME assuming a smoother MV.
  FastInterSearchMode m_fastInterSearchMode;                  ///< Parameter that controls fast encoder settings
  bool      m_bUseEarlyCU;                                    ///< flag for using Early CU setting
  bool      m_useFastDecisionForMerge;                        ///< flag for using Fast Decision Merge RD-Cost
  bool      m_bUseCbfFastMode;                                ///< flag for using Cbf Fast PU Mode Decision
  bool      m_useEarlySkipDetection;                          ///< flag for using Early SKIP Detection
  bool      m_picPartitionFlag;                               ///< enable picture partitioning (0: single tile, single slice, 1: multiple tiles/slices can be used)
  bool      m_mixedLossyLossless;                             ///< enable mixed lossy/lossless coding
  std::vector<uint16_t> m_sliceLosslessArray;                 ///< Slice lossless array
  std::vector<uint32_t> m_tileColumnWidth;                    ///< tile column widths in units of CTUs (last column width will be repeated uniformly to cover any remaining picture width)
  std::vector<uint32_t> m_tileRowHeight;                      ///< tile row heights in units of CTUs (last row height will be repeated uniformly to cover any remaining picture height)
  bool      m_rasterSliceFlag;                                ///< indicates if using raster-scan or rectangular slices (0: rectangular, 1: raster-scan)
  std::vector<uint32_t> m_rectSlicePos;                       ///< rectangular slice positions (pairs of top-left CTU address followed by bottom-right CTU address)
  int       m_rectSliceFixedWidth;                            ///< fixed rectangular slice width in units of tiles (0: disable this feature and use RectSlicePositions instead)
  int       m_rectSliceFixedHeight;                           ///< fixed rectangular slice height in units of tiles (0: disable this feature and use RectSlicePositions instead)
  std::vector<uint32_t> m_rasterSliceSize;                    ///< raster-scan slice sizes in units of tiles (last size will be repeated uniformly to cover any remaining tiles in the picture)
  bool      m_disableLFCrossTileBoundaryFlag;                 ///< 0: filter across tile boundaries  1: do not filter across tile boundaries
  bool      m_disableLFCrossSliceBoundaryFlag;                ///< 0: filter across slice boundaries 1: do not filter across slice boundaries
  uint32_t  m_numSlicesInPic;                                 ///< derived number of rectangular slices in the picture (raster-scan slice specified at slice level)
  bool      m_tileIdxDeltaPresentFlag;                        ///< derived tile index delta present flag
  std::vector<RectSlice> m_rectSlices;                        ///< derived list of rectangular slice signalling parameters
  uint32_t  m_numTileCols;                                    ///< derived number of tile columns
  uint32_t  m_numTileRows;                                    ///< derived number of tile rows
  bool      m_singleSlicePerSubPicFlag;
  bool      m_entropyCodingSyncEnabledFlag;
  bool      m_entryPointPresentFlag;                          ///< flag for the presence of entry points

  bool      m_bFastUDIUseMPMEnabled;
  bool      m_bFastMEForGenBLowDelayEnabled;
  bool      m_bUseBLambdaForNonKeyLowDelayPictures;

  HashType  m_decodedPictureHashSEIType;                      ///< Checksum mode for decoded picture hash SEI message
#if JVET_R0294_SUBPIC_HASH
  HashType  m_subpicDecodedPictureHashType;
#endif
  bool      m_bufferingPeriodSEIEnabled;
  bool      m_pictureTimingSEIEnabled;
  bool      m_bpDeltasGOPStructure;
  bool      m_decodingUnitInfoSEIEnabled;
  bool      m_scalableNestingSEIEnabled;
  bool      m_frameFieldInfoSEIEnabled;
  bool      m_framePackingSEIEnabled;
  int       m_framePackingSEIType;
  int       m_framePackingSEIId;
  int       m_framePackingSEIQuincunx;
  int       m_framePackingSEIInterpretation;
  bool      m_parameterSetsInclusionIndicationSEIEnabled;
  int       m_selfContainedClvsFlag;
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  int       m_preferredTransferCharacteristics;
#endif
  // film grain characterstics sei
  bool      m_fgcSEIEnabled;
  bool      m_fgcSEICancelFlag;
  bool      m_fgcSEIPersistenceFlag;
  uint32_t  m_fgcSEIModelID;
  bool      m_fgcSEISepColourDescPresentFlag;
  uint32_t  m_fgcSEIBlendingModeID;
  uint32_t  m_fgcSEILog2ScaleFactor;
  bool      m_fgcSEICompModelPresent[MAX_NUM_COMPONENT];
  // content light level SEI
  bool      m_cllSEIEnabled;
  uint32_t  m_cllSEIMaxContentLevel;
  uint32_t  m_cllSEIMaxPicAvgLevel;
  // ambient viewing environment sei
  bool      m_aveSEIEnabled;
  uint32_t  m_aveSEIAmbientIlluminance;
  uint32_t  m_aveSEIAmbientLightX;
  uint32_t  m_aveSEIAmbientLightY;
  // content colour volume sei
  bool      m_ccvSEIEnabled;
  bool      m_ccvSEICancelFlag;
  bool      m_ccvSEIPersistenceFlag;
  bool      m_ccvSEIPrimariesPresentFlag;
  bool      m_ccvSEIMinLuminanceValuePresentFlag;
  bool      m_ccvSEIMaxLuminanceValuePresentFlag;
  bool      m_ccvSEIAvgLuminanceValuePresentFlag;
  double    m_ccvSEIPrimariesX[MAX_NUM_COMPONENT];
  double    m_ccvSEIPrimariesY[MAX_NUM_COMPONENT];
  double    m_ccvSEIMinLuminanceValue;
  double    m_ccvSEIMaxLuminanceValue;
  double    m_ccvSEIAvgLuminanceValue;

  bool      m_erpSEIEnabled;
  bool      m_erpSEICancelFlag;
  bool      m_erpSEIPersistenceFlag;
  bool      m_erpSEIGuardBandFlag;
  uint32_t  m_erpSEIGuardBandType;
  uint32_t  m_erpSEILeftGuardBandWidth;
  uint32_t  m_erpSEIRightGuardBandWidth;

  bool      m_sphereRotationSEIEnabled;
  bool      m_sphereRotationSEICancelFlag;
  bool      m_sphereRotationSEIPersistenceFlag;
  int       m_sphereRotationSEIYaw;
  int       m_sphereRotationSEIPitch;
  int       m_sphereRotationSEIRoll;

  bool      m_omniViewportSEIEnabled;
  uint32_t  m_omniViewportSEIId;
  bool      m_omniViewportSEICancelFlag;
  bool      m_omniViewportSEIPersistenceFlag;
  uint32_t  m_omniViewportSEICntMinus1;
  std::vector<int>      m_omniViewportSEIAzimuthCentre;
  std::vector<int>      m_omniViewportSEIElevationCentre;
  std::vector<int>      m_omniViewportSEITiltCentre;
  std::vector<uint32_t> m_omniViewportSEIHorRange;
  std::vector<uint32_t> m_omniViewportSEIVerRange;
  bool                  m_rwpSEIEnabled;
  bool                  m_rwpSEIRwpCancelFlag;
  bool                  m_rwpSEIRwpPersistenceFlag;
  bool                  m_rwpSEIConstituentPictureMatchingFlag;
  int                   m_rwpSEINumPackedRegions;
  int                   m_rwpSEIProjPictureWidth;
  int                   m_rwpSEIProjPictureHeight;
  int                   m_rwpSEIPackedPictureWidth;
  int                   m_rwpSEIPackedPictureHeight;
  std::vector<uint8_t>  m_rwpSEIRwpTransformType;
  std::vector<bool>     m_rwpSEIRwpGuardBandFlag;
  std::vector<uint32_t> m_rwpSEIProjRegionWidth;
  std::vector<uint32_t> m_rwpSEIProjRegionHeight;
  std::vector<uint32_t> m_rwpSEIRwpSEIProjRegionTop;
  std::vector<uint32_t> m_rwpSEIProjRegionLeft;
  std::vector<uint16_t> m_rwpSEIPackedRegionWidth;
  std::vector<uint16_t> m_rwpSEIPackedRegionHeight;
  std::vector<uint16_t> m_rwpSEIPackedRegionTop;
  std::vector<uint16_t> m_rwpSEIPackedRegionLeft;
  std::vector<uint8_t>  m_rwpSEIRwpLeftGuardBandWidth;
  std::vector<uint8_t>  m_rwpSEIRwpRightGuardBandWidth;
  std::vector<uint8_t>  m_rwpSEIRwpTopGuardBandHeight;
  std::vector<uint8_t>  m_rwpSEIRwpBottomGuardBandHeight;
  std::vector<bool>     m_rwpSEIRwpGuardBandNotUsedForPredFlag;
  std::vector<uint8_t>  m_rwpSEIRwpGuardBandType;

  bool                 m_gcmpSEIEnabled;
  bool                 m_gcmpSEICancelFlag;
  bool                 m_gcmpSEIPersistenceFlag;
  uint32_t             m_gcmpSEIPackingType;
  uint32_t             m_gcmpSEIMappingFunctionType;
  std::vector<uint8_t> m_gcmpSEIFaceIndex;
  std::vector<uint8_t> m_gcmpSEIFaceRotation;
  std::vector<double>  m_gcmpSEIFunctionCoeffU;
  std::vector<bool>    m_gcmpSEIFunctionUAffectedByVFlag;
  std::vector<double>  m_gcmpSEIFunctionCoeffV;
  std::vector<bool>    m_gcmpSEIFunctionVAffectedByUFlag;
  bool                 m_gcmpSEIGuardBandFlag;
  uint32_t             m_gcmpSEIGuardBandType;
  bool                 m_gcmpSEIGuardBandBoundaryExteriorFlag;
  uint32_t             m_gcmpSEIGuardBandSamplesMinus1;

  CfgSEISubpictureLevel m_cfgSubpictureLevelInfoSEI;

  bool                  m_sampleAspectRatioInfoSEIEnabled;
  bool                  m_sariCancelFlag;
  bool                  m_sariPersistenceFlag;
  int                   m_sariAspectRatioIdc;
  int                   m_sariSarWidth;
  int                   m_sariSarHeight;

  bool      m_MCTSEncConstraint;

  // weighted prediction
  bool      m_useWeightedPred;                    ///< Use of weighted prediction in P slices
  bool      m_useWeightedBiPred;                  ///< Use of bi-directional weighted prediction in B slices
  WeightedPredictionMethod m_weightedPredictionMethod;

  uint32_t      m_log2ParallelMergeLevel;                         ///< Parallel merge estimation region
  uint32_t      m_maxNumMergeCand;                                ///< Max number of merge candidates
#if JVET_AG0276_LIC_FLAG_SIGNALING
  uint32_t      m_maxNumOppositeLicMergeCand;                     ///< Max number of merge candidates with opposite LIC flag
  uint32_t      m_maxNumAffineOppositeLicMergeCand;               ///< Max number of affine merge candidates with opposite LIC flag
#endif
#if JVET_X0049_ADAPT_DMVR
  uint32_t      m_maxNumBMMergeCand;                                ///< Max number of BM merge candidates
#endif
  uint32_t      m_maxNumAffineMergeCand;                          ///< Max number of affine merge candidates
  uint32_t      m_maxNumGeoCand;
#if JVET_AG0164_AFFINE_GPM
  uint32_t      m_maxNumGpmAffCand;
#if JVET_AJ0274_GPM_AFFINE_TM
  uint32_t      m_maxNumGpmAffTmCand;
#endif
#endif
  uint32_t      m_maxNumIBCMergeCand;                             ///< Max number of IBC merge candidates
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
  uint32_t      m_maxNumMHPCand;
#endif

  bool      m_sliceLevelRpl;                                      ///< code reference picture lists in slice headers rather than picture header
  bool      m_sliceLevelDblk;                                     ///< code deblocking filter parameters in slice headers rather than picture header
  bool      m_sliceLevelSao;                                      ///< code SAO parameters in slice headers rather than picture header
  bool      m_sliceLevelAlf;                                      ///< code ALF parameters in slice headers rather than picture header
  bool      m_sliceLevelWp;                                       ///< code weighted prediction parameters in slice headers rather than picture header
  bool      m_sliceLevelDeltaQp;                                  ///< code delta in slice headers rather than picture header

  int       m_TMVPModeId;

#if TCQ_8STATES
	int       m_depQuantEnabledIdc;
#else
  bool      m_depQuantEnabledFlag;
#endif
  bool      m_signDataHidingEnabledFlag;
  bool      m_RCEnableRateControl;                ///< enable rate control or not
  int       m_RCTargetBitrate;                    ///< target bitrate when rate control is enabled
  int       m_RCKeepHierarchicalBit;              ///< 0: equal bit allocation; 1: fixed ratio bit allocation; 2: adaptive ratio bit allocation
  bool      m_RCLCULevelRC;                       ///< true: LCU level rate control; false: picture level rate control NOTE: code-tidy - rename to m_RCCtuLevelRC
  bool      m_RCUseLCUSeparateModel;              ///< use separate R-lambda model at LCU level                        NOTE: code-tidy - rename to m_RCUseCtuSeparateModel
  int       m_RCInitialQP;                        ///< inital QP for rate control
  bool      m_RCForceIntraQP;                     ///< force all intra picture to use initial QP or not
#if U0132_TARGET_BITS_SATURATION
  bool      m_RCCpbSaturationEnabled;             ///< enable target bits saturation to avoid CPB overflow and underflow
  uint32_t      m_RCCpbSize;                          ///< CPB size
  double    m_RCInitialCpbFullness;               ///< initial CPB fullness
#endif
  ScalingListMode m_useScalingListId;                         ///< using quantization matrix
  std::string m_scalingListFileName;                          ///< quantization matrix file name
  bool      m_disableScalingMatrixForLfnstBlks;
  bool      m_disableScalingMatrixForAlternativeColourSpace;
  bool      m_scalingMatrixDesignatedColourSpace;
  CostMode  m_costMode;                                       ///< Cost mode to use
  bool      m_TSRCdisableLL;                                  ///< disable TSRC for lossless

  bool      m_recalculateQPAccordingToLambda;                 ///< recalculate QP value according to the lambda value

  bool      m_DCIEnabled;                                     ///< enable Decoding Capability Information (DCI)
  bool      m_hrdParametersPresentFlag;                       ///< enable generation of HRD parameters
  bool      m_vuiParametersPresentFlag;                       ///< enable generation of VUI parameters
  bool      m_samePicTimingInAllOLS;                          ///< same picture timing SEI message is used in all OLS
  bool      m_aspectRatioInfoPresentFlag;                     ///< Signals whether aspect_ratio_idc is present
  int       m_aspectRatioIdc;                                 ///< aspect_ratio_idc
  int       m_sarWidth;                                       ///< horizontal size of the sample aspect ratio
  int       m_sarHeight;                                      ///< vertical size of the sample aspect ratio
  bool      m_colourDescriptionPresentFlag;                   ///< Signals whether colour_primaries, transfer_characteristics and matrix_coefficients are present
  int       m_colourPrimaries;                                ///< Indicates chromaticity coordinates of the source primaries
  int       m_transferCharacteristics;                        ///< Indicates the opto-electronic transfer characteristics of the source
  int       m_matrixCoefficients;                             ///< Describes the matrix coefficients used in deriving luma and chroma from RGB primaries
  bool      m_progressiveSourceFlag;                          ///< Indicates if the content is progressive
  bool      m_interlacedSourceFlag;                           ///< Indicates if the content is interlaced
  bool      m_chromaLocInfoPresentFlag;                       ///< Signals whether chroma_sample_loc_type_top_field and chroma_sample_loc_type_bottom_field are present
  int       m_chromaSampleLocTypeTopField;                    ///< Specifies the location of chroma samples for top field
  int       m_chromaSampleLocTypeBottomField;                 ///< Specifies the location of chroma samples for bottom field
  int       m_chromaSampleLocType;                            ///< Specifies the location of chroma samples for progressive content
  bool      m_overscanInfoPresentFlag;                        ///< Signals whether overscan_appropriate_flag is present
  bool      m_overscanAppropriateFlag;                        ///< Indicates whether conformant decoded pictures are suitable for display using overscan
  bool      m_videoFullRangeFlag;                             ///< Indicates the black level and range of luma and chroma signals
  int       m_ImvMode;                                        ///< imv mode
  int       m_Imv4PelFast;                                    ///< imv 4-Pel fast mode

  std::string m_summaryOutFilename;                           ///< filename to use for producing summary output file.
  std::string m_summaryPicFilenameBase;                       ///< Base filename to use for producing summary picture output files. The actual filenames used will have I.txt, P.txt and B.txt appended.
  uint32_t        m_summaryVerboseness;                           ///< Specifies the level of the verboseness of the text output.

  int         m_verbosity;

  std::string m_decodeBitstreams[2];                          ///< filename for decode bitstreams.
  int         m_debugCTU;
  int         m_switchPOC;                                    ///< dbg poc.
  int         m_switchDQP;                                    ///< switch DQP.
  int         m_fastForwardToPOC;                             ///< get to encoding the specified POC as soon as possible by skipping temporal layers irrelevant for the specified POC
  bool        m_stopAfterFFtoPOC;
  bool        m_bs2ModPOCAndType;
  bool        m_forceDecodeBitstream1;

#if NN_COMMON_SPS
  int         m_nnlfOption;
  bool        m_nnlf;
#endif
  
 #if NN_LF_UNIFIED
  NNLFUnifiedID    m_nnlfId;
  bool        m_nnlfUnified;
  uint32_t    m_nnlfBlockSize;
  uint32_t    m_nnlfInfSizeExt;
  uint32_t    m_nnlfMaxNumPrms;
#if JVET_AK0093_NON_NORMATIVE_TDO
  bool        m_nnlfTDO;
  std::vector<int> m_nnlfTDOParam;
#endif
#if JVET_AH0080_TRANS_INPUT
  bool        m_nnlfTransInput;
#endif
 #endif
#if NN_COMMON_SPS
  std::string m_nnlfModelName;                                ///<nnlf model
#endif  
  bool        m_alf;                                          ///< Adaptive Loop Filter
#if JVET_AL0153_ALF_CCCM
  bool        m_lfCccm;
#endif
#if FIXFILTER_CFG
  bool        m_alfFixedFilter;
#endif
  bool        m_ccalf;
  int         m_ccalfQpThreshold;

#if JVET_Q0114_ASPECT5_GCI_FLAG
  bool        m_rprEnabledFlag;
#endif

#if INTER_LIC
  bool        m_lic;
  bool        m_fastPicLevelLIC;
#if JVET_AG0276_LIC_SLOPE_ADJUST
  bool        m_licSlopeAdjust;
#endif
#endif

  double      m_scalingRatioHor;
  double      m_scalingRatioVer;
  bool        m_resChangeInClvsEnabled;
  double      m_fractionOfFrames;                             ///< encode a fraction of the frames as specified in FramesToBeEncoded
  int         m_switchPocPeriod;
  int         m_upscaledOutput;                               ////< Output upscaled (2), decoded cropped but in full resolution buffer (1) or decoded cropped (0, default) picture for RPR.
#if JVET_AC0096
  int         m_rprSwitchingResolutionOrderList[MAX_RPR_SWITCHING_ORDER_LIST_SIZE];
  int         m_rprSwitchingQPOffsetOrderList[MAX_RPR_SWITCHING_ORDER_LIST_SIZE];
  int         m_rprSwitchingListSize;
  bool        m_rprFunctionalityTestingEnabledFlag;
  bool        m_rprPopulatePPSatIntraFlag;
  int         m_rprSwitchingSegmentSize;
  double      m_rprSwitchingTime;
  double      m_scalingRatioHor2;
  double      m_scalingRatioVer2;
  double      m_scalingRatioHor3;
  double      m_scalingRatioVer3;
#endif
#if JVET_AG0116
  bool        m_gopBasedRPREnabledFlag;
  int         m_gopBasedRPRQPThreshold;
  double      m_psnrThresholdRPR;
  double      m_psnrThresholdRPR2;
  double      m_psnrThresholdRPR3;
  int         m_qpOffsetRPR;
  int         m_qpOffsetRPR2;
  int         m_qpOffsetRPR3;
  int         m_qpOffsetChromaRPR;
  int         m_qpOffsetChromaRPR2;
  int         m_qpOffsetChromaRPR3;
#endif
#if JVET_AB0082
  int         m_upscaleFilterForDisplay;
#endif
  bool        m_avoidIntraInDepLayer;

  bool                  m_gopBasedTemporalFilterEnabled;               ///< GOP-based Temporal Filter enable/disable
  int                   m_gopBasedTemporalFilterPastRefs;
  int                   m_gopBasedTemporalFilterFutureRefs;            ///< Enable/disable future frame references in the GOP-based Temporal Filter
  std::map<int, double> m_gopBasedTemporalFilterStrengths;             ///< Filter strength per frame for the GOP-based Temporal Filter
#if JVET_Y0240_BIM
  bool                  m_bimEnabled;
#endif

  int         m_maxLayers;
  int         m_targetOlsIdx;

  int         m_layerId[MAX_VPS_LAYERS];
  int         m_layerIdx;
  int         m_maxSublayers;
  bool        m_allLayersSameNumSublayersFlag;
  bool        m_allIndependentLayersFlag;
  std::string m_predDirectionArray;

  int         m_numRefLayers[MAX_VPS_LAYERS];
  std::string m_refLayerIdxStr[MAX_VPS_LAYERS];
  bool        m_eachLayerIsAnOlsFlag;
  int         m_olsModeIdc;
  int         m_numOutputLayerSets;
  std::string m_olsOutputLayerStr[MAX_VPS_LAYERS];

  int         m_numPtlsInVps;

  CfgVPSParameters m_cfgVPSParameters;
  Level::Name m_levelPtl[MAX_NUM_OLSS];
  int         m_olsPtlIdx[MAX_NUM_OLSS];
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  bool m_intraNN;
#endif

#if EXTENSION_360_VIDEO
  TExt360AppEncCfg m_ext360;
  friend class TExt360AppEncCfg;
  friend class TExt360AppEncTop;
#endif

#if JVET_O0756_CONFIG_HDRMETRICS || JVET_O0756_CALCULATE_HDRMETRICS
#if JVET_O0756_CALCULATE_HDRMETRICS
  double      m_whitePointDeltaE[hdrtoolslib::NB_REF_WHITE];
#else
  double      m_whitePointDeltaE[3];
#endif
  double      m_maxSampleValue;
  int         m_sampleRange;
  int         m_colorPrimaries;
  bool        m_enableTFunctionLUT;
  int         m_chromaLocation;
  int         m_chromaUPFilter;
  int         m_cropOffsetLeft;
  int         m_cropOffsetTop;
  int         m_cropOffsetRight;
  int         m_cropOffsetBottom;
  bool        m_calculateHdrMetrics;
#endif

#if SIGN_PREDICTION
  int m_numPredSign;
#if JVET_Y0141_SIGN_PRED_IMPROVE
  int m_log2SignPredArea;
#endif
#endif
#if DUMP_BEFORE_INLOOP
  bool        m_dumpBeforeInloop;
#endif
#if CONVERT_NUM_TU_SPLITS_TO_CFG
  int         m_maxNumTUs;
#endif

  // internal member functions
  bool  xCheckParameter ();                                   ///< check validity of configuration values
  void  xPrintParameter ();                                   ///< print configuration values
  void  xPrintUsage     ();                                   ///< print usage
  bool  xHasNonZeroTemporalID();                             ///< check presence of constant temporal ID in GOP structure
  bool  xHasLeadingPicture();                                 ///< check presence of leading pictures in GOP structure
  int   xAutoDetermineProfile();                              ///< auto determine the profile to use given the other configuration settings. Returns 1 if erred. Can select profile 'NONE'
public:
  EncAppCfg();
  virtual ~EncAppCfg();

public:
  void  create    ();                                         ///< create option handling class
  void  destroy   ();                                         ///< destroy option handling class
  bool  parseCfg  ( int argc, char* argv[] );                ///< parse configuration file to fill member variables

};// END CLASS DEFINITION EncAppCfg

//! \}

#endif // __ENCAPPCFG__


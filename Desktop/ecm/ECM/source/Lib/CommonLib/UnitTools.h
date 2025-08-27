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

/** \file     UnitTool.h
 *  \brief    defines operations for basic units
 */

#ifndef __UNITTOOLS__
#define __UNITTOOLS__

#include "Unit.h"
#include "UnitPartitioner.h"
#include "ContextModelling.h"
#include "InterPrediction.h"

// CS tools
namespace CS
{
  uint64_t getEstBits                   ( const CodingStructure &cs );
  UnitArea getArea                    ( const CodingStructure &cs, const UnitArea &area, const ChannelType chType );
  bool   isDualITree                  ( const CodingStructure &cs );
#if !MULTI_PASS_DMVR
  void   setRefinedMotionField(CodingStructure &cs);
#endif
#if JVET_AE0043_CCP_MERGE_TEMPORAL
  void   saveTemporalCcpModel(CodingStructure &cs); 
#endif
#if JVET_AG0058_EIP
  void   saveTemporalEipModel(CodingStructure &cs); 
#endif
}


// CU tools
namespace CU
{
#if JVET_AJ0085_SUBBLOCK_MERGE_MODE_EXTENSION
  bool hasAffineNb(const CodingUnit& cu);
  bool isAffineAllowed(const CodingUnit& cu);
  bool affineCtxInc(const CodingUnit& cu);
#endif

#if JVET_AG0276_NLIC
  bool isSecLicParaNeeded             (const CodingUnit &cu);
  bool isPredRefined                  (const CodingUnit &cu);
  bool isAllowSecLicPara              (const CodingUnit &cu);
  bool isTLCond                       (const CodingUnit &cu);
#endif
  bool isIntra                        (const CodingUnit &cu);
  bool isInter                        (const CodingUnit &cu);
  bool isIBC                          (const CodingUnit &cu);
  bool isPLT                          (const CodingUnit &cu);

  bool isSameCtu                      (const CodingUnit &cu, const CodingUnit &cu2);
  bool isSameSlice                    (const CodingUnit &cu, const CodingUnit &cu2);
  bool isSameTile                     (const CodingUnit &cu, const CodingUnit &cu2);
  bool isSameSliceAndTile             (const CodingUnit &cu, const CodingUnit &cu2);
  bool isSameSubPic                   (const CodingUnit &cu, const CodingUnit &cu2);
  bool isLastSubCUOfCtu               (const CodingUnit &cu);
  uint32_t getCtuAddr                     (const CodingUnit &cu);
#if JVET_AG0117_CABAC_SPATIAL_TUNING
  bool isOnCtuBottom                  (const CodingUnit& cu);
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  bool isPartitionerOnCtuBottom(const CodingUnit& cu, const Partitioner& partitioner);
#endif
#endif
#if JVET_V0130_INTRA_TMP
  Position getCtuXYAddr               (const CodingUnit& cu);
#endif
  int  predictQP                      (const CodingUnit& cu, const int prevQP );

  uint32_t getNumPUs                      (const CodingUnit& cu);
  void addPUs                         (      CodingUnit& cu);

  void saveMotionInHMVP               (const CodingUnit& cu, const bool isToBeDone );
#if JVET_AD0188_CCP_MERGE
  void saveModelsInHCCP               (const CodingUnit &cu);
#endif
#if JVET_AG0058_EIP
  void saveModelsInHEIP               (const CodingUnit &cu);
#endif
#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
  void saveCcInsideFilterFlagInCCP    (CodingUnit& cu);
#endif
#if JVET_AM0074_INTRA_MERGE
  bool checkNeighborIntraMergeCompatibility(const CodingUnit& cuNeighbour);
  bool allowIntraMergeMode(const CodingUnit &cu);
  bool isDifferentIntraBlk(const CodingUnit* cuNeighbour, IntraMergeCandidate* cands, int currIdx);
  bool areIntraParamsSame(IntraMergeCandidate* cands, int currIdx);
  bool isAValidNeighborForIntraMerge(const CodingUnit* cuNeighbour);
  bool setIntraMergeCandParamFromTimd(IntraMergeCandidate& ddIntraFusionCand, int timdMode, int timdModeSecondary, const int8_t timdFusionWeight[TIMD_FUSION_NUM], const int8_t timdLocDep[TIMD_FUSION_NUM]);
  Area getOverlappedArea(const Area& a1, const Area& a2);
  bool getItmpBasedIntraMergeCand(const CodingUnit* cuNeighbour, IntraMergeCandidate* ddIntraFusionCand, int ddIntrafusionCandIdx);
  bool setIntraMergeCandParam(const CodingUnit* cuNeighbours, IntraMergeCandidate* cands, int currIdx, int itmpIbcMode = 0);
#endif
  PartSplit getSplitAtDepth           (const CodingUnit& cu, const unsigned depth);
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  ModeType  getModeTypeAtDepth        (const CodingUnit& cu, const unsigned depth);
#endif
  uint32_t getNumNonZeroCoeffNonTsCorner8x8( const CodingUnit& cu, const bool lumaFlag = true, const bool chromaFlag = true );
  bool  isPredRegDiffFromTB(const CodingUnit& cu, const ComponentID compID);
  bool  isFirstTBInPredReg(const CodingUnit& cu, const ComponentID compID, const CompArea &area);
  bool  isMinWidthPredEnabledForBlkSize(const int w, const int h);
  void  adjustPredArea(CompArea &area);
  bool  isBcwIdxCoded                 (const CodingUnit& cu);
  uint8_t getValidBcwIdx              (const CodingUnit& cu);
  void  setBcwIdx                     (CodingUnit& cu, uint8_t uh);
  uint8_t deriveBcwIdx                (uint8_t bcwLO, uint8_t bcwL1);
  bool bdpcmAllowed                   (const CodingUnit& cu, const ComponentID compID);
  bool isMTSAllowed                   (const CodingUnit& cu, const ComponentID compID);
#if JVET_AK0217_INTRA_MTSS
  bool isMdirAllowed(const CodingUnit& cu);
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
  bool isLfnstAllowed                 (const CodingUnit &cu, const ComponentID compID);
#endif
#if INTER_LIC
  bool isLICFlagPresent               (const CodingUnit& cu);
#if JVET_AG0276_LIC_SLOPE_ADJUST
  bool isLicSlopeAllowed              (const CodingUnit& cu);
  bool licSlopeSizeTlCond             (const int bw, const int bh, const int layerId);
#endif
#endif
#if JVET_AC0130_NSPT
  bool  isNSPTAllowed                 ( const TransformUnit& tu, const ComponentID compID, int width, int height, bool isIntra );
#if AHG7_LN_TOOLOFF_CFG
  bool  isNSPTAllowed                 ( int width, int height );
#endif
  bool  nsptApplyCond                 ( const TransformUnit& tu, ComponentID compID, bool allowNSPT );
#endif

  bool      divideTuInRows            ( const CodingUnit &cu );
  PartSplit getISPType                ( const CodingUnit &cu,                         const ComponentID compID );
  bool      isISPLast                 ( const CodingUnit &cu, const CompArea &tuArea, const ComponentID compID );
  bool      isISPFirst                ( const CodingUnit &cu, const CompArea &tuArea, const ComponentID compID );
  bool      canUseISP                 ( const CodingUnit &cu,                         const ComponentID compID );
  bool      canUseISP                 ( const int width, const int height, const int maxTrSize = MAX_TB_SIZEY );
  bool      canUseLfnstWithISP        ( const CompArea& cuArea, const ISPType ispSplitType );
  bool      canUseLfnstWithISP        ( const CodingUnit& cu, const ChannelType chType );
#if JVET_W0119_LFNST_EXTENSION
  Size      getLfnstSize              ( const CodingUnit& cu, const ChannelType chType );
#endif
  uint32_t  getISPSplitDim            ( const int width, const int height, const PartSplit ispType );
  bool      allLumaCBFsAreZero        ( const CodingUnit& cu );
#if JVET_W0123_TIMD_FUSION
  TemplateType deriveTimdRefType      ( int iCurX, int iCurY, uint32_t uiCurWidth, uint32_t uiCurHeight, int iTemplateWidth, int iTemplateHeight, int& iRefX, int& iRefY, uint32_t& uiRefWidth, uint32_t& uiRefHeight );
#endif

  PUTraverser traversePUs             (      CodingUnit& cu);
  TUTraverser traverseTUs             (      CodingUnit& cu);
  cPUTraverser traversePUs            (const CodingUnit& cu);
  cTUTraverser traverseTUs            (const CodingUnit& cu);

  bool  hasSubCUNonZeroMVd            (const CodingUnit& cu);
  bool  hasSubCUNonZeroAffineMVd      ( const CodingUnit& cu );

  uint8_t getSbtInfo                  (uint8_t idx, uint8_t pos);
  uint8_t getSbtIdx                   (const uint8_t sbtInfo);
  uint8_t getSbtPos                   (const uint8_t sbtInfo);
  uint8_t getSbtMode                  (const uint8_t sbtIdx, const uint8_t sbtPos);
  uint8_t getSbtIdxFromSbtMode        (const uint8_t sbtMode);
  uint8_t getSbtPosFromSbtMode        (const uint8_t sbtMode);
  uint8_t targetSbtAllowed            (uint8_t idx, uint8_t sbtAllowed);
  uint8_t numSbtModeRdo               (uint8_t sbtAllowed);
  bool    isSbtMode                   (const uint8_t sbtInfo);
  bool    isSameSbtSize               (const uint8_t sbtInfo1, const uint8_t sbtInfo2);
#if JVET_AJ0274_REGRESSION_GPM_TM
  bool    checkGeoBlendTmAvail        (const CodingUnit& currCU, const CodingStructure* bestCS);
#endif
#if JVET_AK0101_REGRESSION_GPM_INTRA
  bool    checkGeoBlendIntraAvail(const CodingUnit& currCU, const CodingStructure* bestCS);
#endif
#if JVET_AI0050_SBT_LFNST
  void    getSBTPosAndSize            (const CodingUnit &cu, Position& pos, Size& size, uint8_t sbtMode);
#endif
  bool    getRprScaling               ( const SPS* sps, const PPS* curPPS, Picture* refPic, int& xScale, int& yScale );
  void    checkConformanceILRP        (Slice *slice);
#if JVET_AJ0146_TIMDSAD
  bool allowTimdSad(const CodingUnit& cu);
#endif
#if JVET_AB0157_TMRL
  bool allowTmrl(const CodingUnit& cu);
#endif
#if JVET_AK0059_MDIP
  bool allowMdip(const CodingUnit& cu);
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
  RefTemplateType getRefTemplateType(const CodingUnit& cu);
#endif
#if JVET_AC0094_REF_SAMPLES_OPT
  void getNbModesRemovedFirstLast(const bool &areAboveRightUnavail, const bool &areBelowLeftUnavail, const SizeType &height, const SizeType &width, int &nbRemovedFirst, int &nbRemovedLast);
  bool isIdxModeValid(const bool &areAboveRightUnavail, const bool &areBelowLeftUnavail, const SizeType &height, const SizeType &width, const SizeType &idx_mode_tested, const bool &isForcedValid);
#endif
#if JVET_AC0105_DIRECTIONAL_PLANAR
  bool isDirectionalPlanarAvailable(const CodingUnit &cu);
#endif
#if JVET_AE0059_INTER_CCCM
  bool interCccmSearchAllowed(const CodingUnit& cu);
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  void saveProCcpInfo                 (CodingUnit &cu);
  void saveProCcpInfoInter            (CodingUnit &cu, TransformUnit &tu);
  bool interCcpMergeSearchAllowed     (const CodingUnit& cu);
#endif
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  bool isGeoBlendAvailable(const CodingUnit& cu);
#if JVET_AK0101_REGRESSION_GPM_INTRA
  bool isGeoBlendIntraAvailable(const CodingUnit& cu);
#endif
#endif
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
  bool interCcpMergeZeroRootCbfAllowed(const CodingUnit& cu);
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  bool isIntraRegionRoot              (const CodingUnit &cu, const Partitioner& p);
#endif
}
// PU tools
namespace PU
{
#if JVET_AK0217_INTRA_MTSS
  bool     getTransposeFlag(uint32_t intraMode);
#endif
#if JVET_AJ0061_TIMD_MERGE
  int canTimdMergeImplicitDst7(const TransformUnit &tu);
  bool canTimdMerge(const PredictionUnit &pu);
  std::array<const CodingUnit *, TIMD_MERGE_MAX_NONADJACENT> timdMergeNonAdjacentNeighbours(const PredictionUnit &pu);
  bool hasTimdMergeCandidate(const PredictionUnit &pu);
#endif
#if (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  int  getItmpMergeCandidate      (const PredictionUnit& pu, std::vector<Mv>& pBvs);
#endif
  int  getItmpMergeCandidate      (const PredictionUnit& pu, std::vector<Mv>& pBvs
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AL0188_SGPM_FLIPAWARE_BV
    , std::vector<BvInfo>& pSgpmMvs
#else
    , std::vector<Mv>& pSgpmMvs
#endif
#endif
  );
  bool validItmpBv                (const PredictionUnit& pu, int tmpXdisp, int tmpYdisp
#if JVET_AL0108_BVG_DIMD
    , bool isAvailablePairFound = false
#endif
  );
  bool checkValidIntraTmpMergeCand(const PredictionUnit& pu, Mv Bv);
#if JVET_AH0055_INTRA_TMP_ARBVP
  bool checkBvAvailable(std::vector<Mv> &pBv, Mv curBv);
#endif
#if JVET_AL0188_SGPM_FLIPAWARE_BV
  bool checkBvInfoAvailable(std::vector<BvInfo>& pBv, BvInfo curBv);
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
  bool validIBCItmpMv(const PredictionUnit& pu, Mv curMv, int templateSize
#if JVET_AL0188_SGPM_FLIPAWARE_BV
    , int flipType = 0
#endif
  );
#endif
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  void  getSparseArBvMergeCandidate(const PredictionUnit& pu, std::vector<Mv>& pBvs, static_vector<TempLibFast, MTMP_NUM_SPARSE>& sparseMtmpCandList);
#endif
#endif
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  int  sortInOut(CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType, std::vector<Mv>& mergeCands);
  int  getArBvMergeCandidate(const PredictionUnit& pu, std::vector<Mv>& pBvs, const int numBvs);
#endif
#if JVET_AD0184_REMOVAL_OF_DIVISION_OPERATIONS
  int getMeanValue(int sum, int div);
#endif
  int  getLMSymbolList(const PredictionUnit &pu, int *modeList);
#if SECONDARY_MPM
  int getIntraMPMs(const PredictionUnit &pu, uint8_t *mpm, uint8_t* nonMpm
#if JVET_AC0094_REF_SAMPLES_OPT
                 , const bool &isForcedValid
#endif
#if JVET_AK0061_PDP_MPM
    , const bool& enableNonSortPDP = false
    , const bool& mpmSort = false
#endif
#if JVET_AD0085_MPM_SORTING
                 , IntraPrediction* pIntraPred = nullptr
#endif
                 , const ChannelType &channelType = CHANNEL_TYPE_LUMA
  );
#else
  int  getIntraMPMs(const PredictionUnit &pu, unsigned *mpm, const ChannelType &channelType = CHANNEL_TYPE_LUMA);
#endif
#if JVET_AK0061_PDP_MPM
  bool determinePDPTemp(const PredictionUnit& pu);
#endif

#if JVET_Y0065_GPM_INTRA
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  void getGeoIntraMPMs( const PredictionUnit &pu, uint8_t* mpm, uint8_t splitDir, uint8_t shape, bool doInit, bool doInitAL = true, bool doInitA = true, bool doInitL = true);
#endif
  void getGeoIntraMPMs( const PredictionUnit &pu, uint8_t* mpm, uint8_t splitDir, uint8_t shape );
#endif
#if JVET_AB0155_SGPM
  void getSgpmIntraMPMs(const PredictionUnit &pu, uint8_t *mpm, uint8_t splitDir, uint8_t shape);
#endif
  bool          isMIP                 (const PredictionUnit &pu, const ChannelType &chType = CHANNEL_TYPE_LUMA);
#if JVET_V0130_INTRA_TMP
  bool          isTmp(const PredictionUnit& pu, const ChannelType& chType = CHANNEL_TYPE_LUMA);
#endif
#if JVET_AL0108_BVG_DIMD
  bool hasBvgBv(const PredictionUnit *pu, const ChannelType &chType = CHANNEL_TYPE_LUMA);
  bool hasBvgBv(const PredictionUnit &pu, const ChannelType &chType = CHANNEL_TYPE_LUMA);
#endif
#if JVET_AG0058_EIP
  bool isEIP(const PredictionUnit& pu, const ChannelType& chType = CHANNEL_TYPE_LUMA);
#if JVET_AL0106_BV_EIP
  bool isBvEip(const PredictionUnit& pu, const ChannelType& chType = CHANNEL_TYPE_LUMA);
#endif
#endif
  bool          isDMChromaMIP         (const PredictionUnit &pu);
#if JVET_AB0155_SGPM
  bool isSgpm(const PredictionUnit &pu, const ChannelType &chType = CHANNEL_TYPE_LUMA);
  bool isDMChromaSgpm(const PredictionUnit &pu);
#if JVET_AJ0112_REGRESSION_SGPM
  bool isRegressionSgpm(const PredictionUnit &pu);
  bool isRegressionSgpmAllow(const PredictionUnit &pu);
#endif
#endif
#if JVET_AB0155_SGPM
  uint32_t getIntraDirLuma(const PredictionUnit &pu, const int partIdx = 0);
#else
  uint32_t      getIntraDirLuma       (const PredictionUnit &pu);
#endif
  void getIntraChromaCandModes(const PredictionUnit &pu, unsigned modeList[NUM_CHROMA_MODE]);

  const PredictionUnit &getCoLocatedLumaPU(const PredictionUnit &pu);
#if JVET_AB0155_SGPM
  uint32_t getFinalIntraMode              (const PredictionUnit &pu, const ChannelType &chType, const int partIdx = 0);
#else
  uint32_t getFinalIntraMode              (const PredictionUnit &pu, const ChannelType &chType);
#endif
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  void setTimdVimp(int vipm, int svipm, CodingUnit* cu);
  void setTimdVimp(int* timdDimdMode, int* timdSecondDimdMode, int& timdSadDimd, int& timdSadSecondDimd, CodingUnit* cu);
  void setTimdVimpToCu(int* timdDimdMode, int* timdSecondDimdMode, int timdSadDimd, int timdSadSecondDimd, CodingUnit* cu);
  void getTimdVimp(int& vipm, int& svipm, CodingUnit* cu);
  bool isBvTimdApplied(CodingUnit* cu);
#endif
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  void setDimdVimp(int& dimdDimdMode, int& dimdSecondDimdMode, int& obicDimdMode, int& obicSecondaryMode, CodingUnit* cu);
  void setDimdVimp(int vipm, int svipm, CodingUnit* cu);
  void setDimdVimpToCu(int dimdDimdMode, int dimdSecondDimdMode, int obicDimdMode, int obicSecondDimdMode, CodingUnit* cu);
  void getDimdVimp(int& vipm, int& svipm, CodingUnit* cu);
  bool isBvDimdApplied(CodingUnit* cu);
#endif
#if JVET_AC0130_NSPT
#if JVET_AK0217_INTRA_MTSS
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
  std::pair<uint32_t,uint32_t> getFinalIntraModeForTransform(bool& secondBucket, const TransformUnit& tu, const ComponentID compID);
#else
  uint32_t getFinalIntraModeForTransform(bool& secondBucket, const TransformUnit& tu, const ComponentID compID);
#endif
#else
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
  std::pair<uint32_t,uint32_t> getFinalIntraModeForTransform  ( const TransformUnit &tu, const ComponentID compID );
#else
  uint32_t getFinalIntraModeForTransform  ( const TransformUnit &tu, const ComponentID compID );
#endif
#endif
  uint32_t getNSPTIntraMode               ( int wideAngPredMode );
#endif
#if JVET_W0119_LFNST_EXTENSION
#if AHG7_LN_TOOLOFF_CFG
  int      getLFNSTMatrixDim          ( int width, int height, bool lfnstExtFlag = true );
#else
  int      getLFNSTMatrixDim          ( int width, int height );
#endif
#if JVET_AC0130_NSPT
  int      getNSPTMatrixDim           ( int width, int height );
#endif
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
#if JVET_AK0217_INTRA_MTSS
#if JVET_AL0215_NSPT_SET_FOR_INTRANN
  int      getNSPTBucket              ( const TransformUnit& tu, bool secondBucket, const ComponentID compID );
#else
  int      getNSPTBucket(const TransformUnit& tu, bool secondBucket);
#endif
#else
#if JVET_AL0215_NSPT_SET_FOR_INTRANN
  int      getNSPTBucket              ( const TransformUnit &tu, const ComponentID compID  );
#else
  int      getNSPTBucket              ( const TransformUnit &tu );
#endif
#endif
#endif
  bool     getUseLFNST8               ( int width, int height );
  uint8_t  getLFNSTIdx                ( int intraMode, int mtsMode = 0 );
  bool     getUseLFNST16              ( int width, int height );
#endif
#if JVET_AB0155_SGPM
  uint32_t getCoLocatedIntraLumaMode(const PredictionUnit &pu, const int partIdx = 0);
#else
  uint32_t getCoLocatedIntraLumaMode      (const PredictionUnit &pu);
#endif
#if JVET_AC0071_DBV
#if JVET_AH0136_CHROMA_REORDERING
  bool isDbvMode(int mode);
#endif
  bool dbvModeAvail(const PredictionUnit &pu);
  void deriveChromaBv(PredictionUnit &pu);
#if JVET_AA0070_RRIBC
#if JVET_AE0169_BIPREDICTIVE_IBC
  Mv adjustChromaBv(const PredictionUnit &lumaPU, const CompArea &lumaArea, RefPicList list = REF_PIC_LIST_0);
#else
  Mv adjustChromaBv(const PredictionUnit &lumaPU, const CompArea &lumaArea);
#endif
#if JVET_AL0188_SGPM_FLIPAWARE_BV
  Mv adjustLumaBv(const PredictionUnit& parentPU, const CompArea& lumaArea, Mv lumaMv);
#endif
#endif
  bool xCheckSimilarChromaBv(std::vector<Mv> &chromaBvList, const Mv chromaBv);
  bool checkIsChromaBvCandidateValid(const PredictionUnit &pu
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                   , const Mv mv
                                   , int filterIdx = 0
#else
                                   , const Mv chromaBv
#endif
                                   , bool isRefTemplate = false, bool isRefAbove = false);
#endif
#if JVET_AH0136_CHROMA_REORDERING
  bool checkIsChromaBvCandidateValidChromaTm(const PredictionUnit &pu, const Mv mv, const int tmpSize, int filterIdx = 0, bool isRefTemplate = false, bool isRefAbove = false);
#endif
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  uint32_t getCoLocatedIdxRepresentationPnn(const PredictionUnit& pu);
#endif
  int      getWideAngle                   ( const TransformUnit &tu, const uint32_t dirMode, const ComponentID compID );
#if MULTI_PASS_DMVR || JVET_W0097_GPM_MMVD_TM
  uint32_t getBDMVRMvdThreshold       (const PredictionUnit &pu);
#endif
#if TM_MRG || TM_AMVP || JVET_Z0084_IBC_TM
  uint32_t getTMMvdThreshold          (const PredictionUnit &pu);
#endif
#if JVET_AE0046_BI_GPM
  uint32_t getBiGpmThreshold(const PredictionUnit& pu);
#endif
#if TM_MRG
  int      reorderInterMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, int numCand, uint32_t mvdSimilarityThresh );
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  void refineTmvpCoverGetColocatedMVP           (InterPrediction* refineTmvpInterPred, const PredictionUnit &pu, bool refineTmvpInputC0Available, bool refineTmvpInputC1Available, RefPicList refineTmvpInputRefList,
                                         const Position & refineTmvpInputPosC0, const Position & refineTmvpInputPosC1, const int refineTmvpInputRefIdx, bool refineTmvpInputSbFlag, const int refineTmvpInputCol,
                                         Mv& refineTmvpOutputColoMv, int* refineTmvpOutputRefIdx, bool& refineTmvpOutputExistMv, int refineTmvpSbTmvpType = 0, bool refineTmvpIsSbTmvp = false);
  bool refineTmvpGetColocatedMVP                (bool refineTmvpIsSbTmvp, InterPrediction* refineTmvpInterPred, const PredictionUnit &pu, const RefPicList &eRefPicList,
                                         const Position &pos, Mv& rcMv, const int &refIdx, bool sbFlag, int col, int* targetRefIdx, int sbTmvpType);

  void getInterMergeCandidates        (InterPrediction* refineTmvpInterPred, const PredictionUnit &pu, MergeCtx& mrgCtx,
#else
  void getInterMergeCandidates        (const PredictionUnit &pu, MergeCtx& mrgCtx,
#endif
    int mmvdList,
    const int& mrgCandIdx = -1 
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
    , MergeCtx* mvpMrgCtx1 = NULL
    , MergeCtx* mvpMrgCtx2 = NULL
#endif
#if JVET_AE0046_BI_GPM
    , bool enableTh4Gpm = false
#endif
  );
#if JVET_AG0276_NLIC || JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  void     getAltMergeCandidates    (const PredictionUnit &pu, AltLMMergeCtx& cMrgCtx
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                                   , bool isLicInheritCand = false
#endif
  );
#if JVET_AG0276_LIC_FLAG_SIGNALING
  void     getAltBRMergeCandidates  (const PredictionUnit &pu, AltLMMergeCtx& cMrgCtx);
  bool     isValidAltMergeCandidate (const PredictionUnit &pu, bool isBRCand = false
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                                   , bool isLicInheritCand = false
#endif
  );
#else
  bool     isValidAltMergeCandidate (const PredictionUnit &pu
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                                   , bool isLicInheritCand = false
#endif
  );
#endif
  uint32_t getAltMergeMvdThreshold  (const PredictionUnit &pu);
#endif
#if JVET_AI0187_TMVP_FOR_CMVP
  void getBMCMVPMergeCandidates(const PredictionUnit &pu, MergeCtx& mergeCtx, MergeCtx& tmpMrgCtx, int maxNumMergeCand, int& cnt);
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  void getNonAdjacentMergeCandSubTMVP(const PredictionUnit &pu, MergeCtx& mvpMrgCtx, int col);
  void getInterMergeCandidatesSubTMVP(const PredictionUnit &pu, MergeCtx& mrgCtx, int col, const int& mrgCandIdx = -1
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
    , MergeCtx* mvpMrgCtx1 = NULL
#endif
  );
#endif
  void getTmvpMergeCand(const PredictionUnit &pu, MergeCtx& mvpMrgCtxz);
  void getNonAdjacentMergeCand        (const PredictionUnit &pu, MergeCtx& mvpMrgCtx);
#endif
  void getIBCMergeCandidates          (const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx = -1);
#if JVET_AA0070_RRIBC
  void rribcAdjustMotion(const PredictionUnit &pu, const Position *cPos, MotionInfo &miNeighbor);
#endif
#if  JVET_Y0058_IBC_LIST_MODIFY || JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  bool checkIsIBCCandidateValid(const PredictionUnit &pu
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                              ,       MotionInfo miNeighbor
                              , int  filterIdx = 0
#else
                              , const MotionInfo miNeighbor
#endif
                              , bool isRefTemplate = false, bool isRefAbove = false
  );
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AE0169_BIPREDICTIVE_IBC
  bool checkIsIBCCandidateValidBi(const PredictionUnit &pu, MotionInfo miNeighbor);
#endif
  uint32_t checkValidBvPU (const PredictionUnit& pu, ComponentID compID,                                Mv mv, bool ignoreFracMv = false, int filterIdx = 0);
  uint32_t checkValidBv   (const PredictionUnit& pu, ComponentID compID, int compWidth, int compHeight, Mv mv, bool ignoreFracMv = false, int filterIdx = 0
                         , bool isFinalMC = false // this flag is for non-normative SW speedup
                         , bool checkAllRefValid = false
  );
#endif
  
#if JVET_AK0185_TMVP_SELECTION
  bool collectTMVP(
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
    InterPrediction* refineTmvpInterPred,
#endif
    const PredictionUnit& pu,
    const std::vector<Position>& posList,
    const std::vector<bool>& availList,
    int iRefIdx,
    int col,

    MergeCtx& mrgCtx,
    int mrgCandIdx,
    int& cnt,

    const Slice& slice,
    int maxNumMergeCand,
    int mvdThreshold,

    MergeCtx* tmpMrgCtx,
    int* tmpMrgCtxcnt,

    int  tmvpFlag,
    bool useNullRefIdx,

    bool checkBiPredFromDifferentDirEqDistPoc = false,

    bool checkValidMergeMvCand = false,
    bool useAmvpMergeMode = false,
    int  amvpMergeCtxMergeDir = -1,
    int  amvpRefList = -1
  );

#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  void addTmvp2AMVP(InterPrediction* refineTmvpInterPred, PredictionUnit& pu, RefPicList eRefPicList, const std::vector<Position>& posList, const std::vector<bool>& availList, Mv& cColMv, const int refIdxCol, int colIdx, AMVPInfo* pInfo, bool oneTmvpFlag = false);
  void addTmvp2AffineAMVP(InterPrediction* refineTmvpInterPred, PredictionUnit& pu, RefPicList eRefPicList, const std::vector<Position>& posList, const std::vector<bool>& availList, Mv& cColMv, const int refIdxCol, AffineAMVPInfo& affiAMVPInfo, bool oneTmvpFlag = false);
#else
  void addTmvp2AMVP(const PredictionUnit& pu, RefPicList eRefPicList, const std::vector<Position>& posList, const std::vector<bool>& availList, Mv& cColMv, const int refIdxCol, int colIdx, AMVPInfo* pInfo, bool oneTmvpFlag = false);
  void addTmvp2AffineAMVP(const PredictionUnit& pu, RefPicList eRefPicList, const std::vector<Position>& posList, const std::vector<bool>& availList, Mv& cColMv, const int refIdxCol, AffineAMVPInfo& affiAMVPInfo, bool oneTmvpFlag = false);
#endif
#endif

#if JVET_AE0159_FIBC
  bool checkIsIBCFilterCandidateValid(const PredictionUnit &pu, const MotionInfo miNeighbor, int  filterIdx = 0, bool isRefTemplate = false, bool isRefAbove = false);
#endif
#if JVET_Y0058_IBC_LIST_MODIFY || JVET_Z0084_IBC_TM || JVET_AA0061_IBC_MBVD || JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  bool searchBv(const PredictionUnit& pu, int xPos, int yPos, int width, int height, int picWidth, int picHeight, int xBv, int yBv, int ctuSize
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
              , int xFilterTap = 0, int yFilterTap = 0, ComponentID compID = COMPONENT_Y
#endif
  );
#endif
#if JVET_AA0061_IBC_MBVD
  void getIbcMbvdMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, int numValidBv);
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
  void getIbcAdaptiveMbvdMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, int numValidBv);
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  int32_t getIbcMbvdEstBits      (const PredictionUnit &pu, int mmvdMergeCand, int mmvdMergeCand1 = -1);
#else
  int32_t getIbcMbvdEstBits      (const PredictionUnit &pu, unsigned int mmvdMergeCand);
#endif
#endif
  void getInterMMVDMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx = -1);
  int getDistScaleFactor(const int &currPOC, const int &currRefPOC, const int &colPOC, const int &colRefPOC);
  bool isDiffMER                      (const Position &pos1, const Position &pos2, const unsigned plevel);
  bool getColocatedMVP                (const PredictionUnit &pu, const RefPicList &eRefPicList, const Position &pos, Mv& rcMv, const int &refIdx, bool sbFlag
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    , int col = 0
#endif
    , int* targetRefIdx = nullptr
#endif
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
    , int sbTmvpType = 0
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  , RefineTmvpTmvpFirstPassParams* refineTmvpFirstPassParams = nullptr
#endif
  );
#if JVET_AI0197_AFFINE_TMVP
  bool getColocatedAffineCMVP(const PredictionUnit &pu, const RefPicList &eRefPicList, const Position &pos, Mv rcMv[3],
#if JVET_AK0095_ENHANCED_AFFINE_CANDIDATE
                              Mv rcMvAlt[3],
                              bool isAlt,
#endif
                              const int &refIdx, bool sbFlag
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                              ,
                              int col = 0
#endif
                              ,
                              int *targetRefIdx = nullptr
#endif
                              ,
                              EAffineModel *targetAffineType = nullptr);
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  inline void       getTemplateTop(const bool availableTop, const PredictionUnit &pu, CPelBuf pRecY, PelBuf pTemplateDest,
                                   Position offset, int Width, int templateSize);
  inline void       getTemplateLeft(const bool availableLeft, const PredictionUnit &pu, CPelBuf pRecY, PelBuf pTemplateDest,
                                    Position offset, int Height, int templateSize);
  inline void       getTemplateRefTop(const PredictionUnit &pu, CPelBuf pRecY, PelBuf pTempDest, Mv CandMv, int tempWidth,
                                      int tempHeight
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                    , InterPrediction *pcInter
#endif
  );
  inline void       getTemplateRefLeft(const PredictionUnit &pu, CPelBuf pRecY, PelBuf pTempDest, Mv CandMv, int tempWidth,
                                       int tempHeight
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                     , InterPrediction *pcInter
#endif
  );
  inline Distortion getTMCost(const PredictionUnit &pu, CPelBuf pRecY, Mv CandMv, bool availableTmTop,
                              bool availableTmLeft, InterPrediction *pcInter);
  inline void       getRribcBvpCand(PredictionUnit &pu, AMVPInfo *amvpInfo);
  inline void       clusterBvpCand(const int cbWidth, const int cbHeight, AMVPInfo *pInfo);
#endif
  void fillMvpCand                    (      PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AMVPInfo &amvpInfo
#if TM_AMVP || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                                     , InterPrediction* interPred = nullptr
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
                                     , InterPrediction* refineTmvpInterPred = nullptr
#endif
  );
#if (JVET_Z0084_IBC_TM && IBC_TM_AMVP) || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
#if JVET_AE0169_BIPREDICTIVE_IBC
  void fillIBCMvpCand                 (      PredictionUnit &pu, AMVPInfo &amvpInfo, MergeCtx& mergeCtx, InterPrediction* pcInter);
#else
  void fillIBCMvpCand                 (      PredictionUnit &pu, AMVPInfo &amvpInfo, InterPrediction* pcInter);
#endif
#else
  void fillIBCMvpCand                 (PredictionUnit &pu, AMVPInfo &amvpInfo);
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  void fillAffineMvpCand              (InterPrediction* refineTmvpInterPred, PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AffineAMVPInfo &affiAMVPInfo
#else
  void fillAffineMvpCand              (      PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AffineAMVPInfo &affiAMVPInfo
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
    , InterPrediction* interPred = nullptr
#endif
  );
  bool addMVPCandUnscaled(const PredictionUnit& pu, const RefPicList& eRefPicList, const int& iRefIdx, const Position& pos, const MvpDir& eDir, AMVPInfo& amvpInfo
#if JVET_AM0106_CHAIN_AMVP_MERGE
    , bool chase = false
#endif  
  );
  void xInheritedAffineMv             ( const PredictionUnit &pu, const PredictionUnit* puNeighbour, RefPicList eRefPicList, Mv rcMv[3] );
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
  void xCalcRMVFParameters(std::vector<RMVFInfo> &mvpInfoVec, int64_t dMatrix[2][4],
#if JVET_AA0107_RMVF_AFFINE_OVERFLOW_FIX || JVET_AB0189_RMVF_BITLENGTH_CONTROL
    int64_t sumbb[2][3][3], int64_t sumeb[2][3],
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    uint8_t shift,
#endif
#else
    int sumbb[2][3][3], int sumeb[2][3],
#endif
    uint16_t addedSize);
  void xReturnMvpVec(std::vector<RMVFInfo> mvp[NUM_REF_PIC_LIST_01][MAX_NUM_REF], const PredictionUnit &pu, const Position &pos
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
    , const MvpDir &eDir
#endif
  );

  void getRMVFAffineGuideCand(const PredictionUnit &pu, const PredictionUnit &abovePU, AffineMergeCtx &affMrgCtx, std::vector<RMVFInfo> mvp[NUM_REF_PIC_LIST_01][MAX_NUM_REF], int mrgCandIdx = -1
#if JVET_AG0276_NLIC
    , AltLMAffineMergeCtx* altAffineMergeCtx = NULL
#if JVET_AG0276_LIC_FLAG_SIGNALING
    , AltLMAffineMergeCtx* altBRAffineMergeCtx = NULL
#endif
#endif
  );
  Position convertNonAdjAffineBlkPos(const Position &pos, int curCtuX, int curCtuY);
  void collectNeiMotionInfo(std::vector<RMVFInfo> mvpInfoVec[NUM_REF_PIC_LIST_01][MAX_NUM_REF], const PredictionUnit &pu);
#endif
  bool addMergeHMVPCand               (const CodingStructure &cs, MergeCtx& mrgCtx, const int& mrgCandIdx, const uint32_t maxNumMergeCandMin1, int &cnt
    , const bool isAvailableA1, const MotionInfo miLeft, const bool isAvailableB1, const MotionInfo miAbove
#if !JVET_Z0075_IBC_HMVP_ENLARGE
    , const bool ibcFlag
    , const bool isGt4x4
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE || (JVET_Y0058_IBC_LIST_MODIFY && !JVET_Z0075_IBC_HMVP_ENLARGE)
    , const PredictionUnit &pu
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
#if !JVET_Y0128_NON_CTC
    , const int curPoc = 0
    , const int amvpPoc = 0
#endif
#endif
#if TM_MRG || (JVET_Z0084_IBC_TM && !JVET_Z0075_IBC_HMVP_ENLARGE) || JVET_AE0046_BI_GPM
    , const uint32_t mvdSimilarityThresh = 1
#endif
  );
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  void addMergeHMVPCandSubTMVP(const CodingStructure &cs, MergeCtx& mrgCtx, const int& mrgCandIdx, const uint32_t maxNumMergeCand, int &cnt
    , const bool isAvailableA1, const MotionInfo miLeft, const bool isAvailableB1, const MotionInfo miAbove
#if !JVET_Z0075_IBC_HMVP_ENLARGE
    , const bool ibcFlag
    , const bool isGt4x4
#endif
    , const PredictionUnit &pu, int col
#if TM_MRG || (JVET_Z0084_IBC_TM && !JVET_Z0075_IBC_HMVP_ENLARGE)
    , const uint32_t mvdSimilarityThresh = 1
#endif
  );
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
#if JVET_Y0128_NON_CTC
  bool checkIsValidMergeMvCand        (const PredictionUnit &pu, int8_t mergeRefIdx[ NUM_REF_PIC_LIST_01 ]);
#else
  bool checkIsValidMergeMvCand        (const CodingStructure &cs, const PredictionUnit &pu, const int curPoc, const int amvpPoc, int8_t mergeRefIdx[ NUM_REF_PIC_LIST_01 ]);
#endif
#endif
#if JVET_Z0075_IBC_HMVP_ENLARGE
  bool addIBCMergeHMVPCand               (const CodingStructure &cs, MergeCtx& mrgCtx, const int& mrgCandIdx, const uint32_t maxNumMergeCandMin1, int &cnt
#if JVET_Y0058_IBC_LIST_MODIFY
    , const PredictionUnit &pu
#endif
#if TM_MRG || JVET_Z0084_IBC_TM
    , const uint32_t mvdSimilarityThresh = 1
#endif
  );
#endif
  void addAMVPHMVPCand                (const PredictionUnit &pu, const RefPicList eRefPicList, const int currRefPOC, AMVPInfo &info);
#if JVET_Z0139_HIST_AFF
  void xGetAffineMvFromLUT(AffineMotionInfo* affHistInfo, int rParameters[4]);
  bool checkLastAffineMergeCandRedundancy(const PredictionUnit& pu, AffineMergeCtx& affMrgCtx);

  bool addOneAffineMergeHMVPCand(const PredictionUnit& pu, AffineMergeCtx& affMrgCtx, static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS>* lutAff, int affHMVPIdx, const MotionInfo& mvInfo, Position neiPosition, int iGBiIdx
#if INTER_LIC
                               ,       bool bICflag
#endif
  );
  bool addSpatialAffineMergeHMVPCand(const PredictionUnit& pu, AffineMergeCtx& affMrgCtx, static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS>* lutAff, int affHMVPIdx, const PredictionUnit* neiPUs[], Position neiPositions[], int iNeiNum, const int mrgCandIdx = -1);
  bool addSpatialAffineAMVPHMVPCand(PredictionUnit& pu, const RefPicList& eRefPicList, const int& refIdx, AffineAMVPInfo& affiAMVPInfo, static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS>* lutAff, int iHMVPlistIdx,
    int neiIdx[], int iNeiNum, int aiNeibeInherited[], bool bFoundOne);
  bool addMergeHMVPCandFromAffModel(const PredictionUnit& pu, MergeCtx& mrgCtx, const int& mrgCandIdx, int& cnt
#if TM_MRG || JVET_AE0046_BI_GPM
    ,const uint32_t mvdSimilarityThresh = 1
#endif
  );
  bool addOneMergeHMVPCandFromAffModel(const PredictionUnit& pu, MergeCtx& mrgCtx, int& cnt, static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS>* lutAff, int listIdx, const MotionInfo& mvInfo, Position neiPosition
    , int iGBiIdx
#if INTER_LIC
    , bool bICflag
#endif
#if TM_MRG || JVET_AE0046_BI_GPM
    , const uint32_t mvdSimilarityThresh = 1
#endif
  );

  bool addOneInheritedHMVPAffineMergeCand(const PredictionUnit& pu, AffineMergeCtx& affMrgCtx, static_vector<AffineInheritInfo, MAX_NUM_AFF_INHERIT_HMVP_CANDS>& lutAffInherit, int affHMVPIdx);
  void xGetAffineMvFromLUT(short affineParameters[4], int rParameters[4]);
  void deriveAffineParametersFromMVs(const PredictionUnit& pu, const Mv acMvTemp[3], int* affinePara, EAffineModel affModel);
  void deriveMVsFromAffineParameters(const PredictionUnit& pu, Mv rcMv[3], int* affinePara, const Mv& cBaseMv, const Position& cBasePos);
  void storeAffParas(int* affinePara);
  void deriveCenterMVFromAffineParameters(const PredictionUnit& pu, Mv& rcMv, int* affinePara, const Mv& cBaseMv, const Position& cBasePos);
#endif
#if JVET_Z0139_HIST_AFF || JVET_Z0139_NA_AFF
  bool checkLastAffineAMVPCandRedundancy(const PredictionUnit& pu, AffineAMVPInfo& affiAMVPInfo);
#endif
  bool addAffineMVPCandUnscaled       ( const PredictionUnit &pu, const RefPicList &refPicList, const int &refIdx, const Position &pos, const MvpDir &dir, AffineAMVPInfo &affiAmvpInfo 
#if JVET_Z0139_HIST_AFF
    , int aiNeibeInherited[5]
#endif
#if JVET_AG0164_AFFINE_GPM
    , int checkAffGPM = 0
#endif
  );
  bool isBipredRestriction            (const PredictionUnit &pu);
#if JVET_AE0046_BI_GPM
  void spanPuMv2DmvrBuffer            (const PredictionUnit &pu, Mv* bdmvrSubPuMv0, Mv* bdmvrSubPuMv1 );
#endif
#if MULTI_PASS_DMVR
  void spanMotionInfo                 (      PredictionUnit &pu, const MergeCtx &mrgCtx = MergeCtx(), 
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    int colIdx = 0,
#endif
    Mv* bdmvrSubPuMv0 = nullptr, Mv* bdmvrSubPuMv1 = nullptr, Mv* bdofSubPuMvOffset = nullptr );
#else
  void spanMotionInfo                 (      PredictionUnit &pu, const MergeCtx &mrgCtx = MergeCtx() );
#endif
#if JVET_W0123_TIMD_FUSION
#if MULTI_PASS_DMVR
  void spanMotionInfo2                (      PredictionUnit &pu, const MergeCtx &mrgCtx = MergeCtx(),
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    int colIdx = 0,
#endif
    Mv* bdmvrSubPuMv0 = nullptr, Mv* bdmvrSubPuMv1 = nullptr, Mv* bdofSubPuMvOffset = nullptr );
#else
  void spanMotionInfo2                (      PredictionUnit &pu, const MergeCtx &mrgCtx = MergeCtx() );
#endif
  void spanIpmInfoIntra               (      PredictionUnit &pu );
  void spanIpmInfoInter               (      PredictionUnit &pu, MotionBuf &mb, IpmBuf &ib );
#if JVET_AC0112_IBC_CIIP
  void spanIpmInfoIBC                 (      PredictionUnit &pu, IpmBuf &ib, int bvx, int bvy );
#endif
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  void spanSCCInfo                    (      PredictionUnit &pu);
#endif
#if JVET_AB0155_SGPM
  void spanIpmInfoSgpm                (      PredictionUnit &pu);
#endif
#if !JVET_Z0054_BLK_REF_PIC_REORDER
  void applyImv                       (      PredictionUnit &pu, MergeCtx &mrgCtx, InterPrediction *interPred = NULL );
#endif
#if JVET_Z0139_HIST_AFF
  bool getAffineControlPointCand(const PredictionUnit& pu, MotionInfo mi[4], bool isAvailable[4], int verIdx[4], int8_t bcwIdx, int modelIdx, int verNum, AffineMergeCtx& affMrgCtx);
#else
  void getAffineControlPointCand(const PredictionUnit &pu, MotionInfo mi[4], bool isAvailable[4], int verIdx[4], int8_t bcwIdx, int modelIdx, int verNum, AffineMergeCtx& affMrgCtx);
#endif
#if JVET_Z0139_NA_AFF
  bool xCPMVSimCheck(const PredictionUnit &pu, AffineMergeCtx &affMrgCtx, Mv curCpmv[2][3], unsigned char curdir, int8_t curRefIdx[2], EAffineModel curType, int bcwIdx, bool LICFlag);
  int  getMvDiffThresholdByWidthAndHeight(const PredictionUnit &pu, bool width);
  bool addNonAdjAffineConstructedCPMV(const PredictionUnit &pu, MotionInfo miNew[4], bool isAvaNew[4], Position pos[4], int8_t bcwId, AffineMergeCtx &affMrgCtx, int mrgCandIdx);
  bool addNonAdjCstAffineMVPCandUnscaled(const PredictionUnit &pu, const RefPicList &refPicList, const int &refIdx, AffineAMVPInfo &affiAmvpInfo);
  bool addNonAdjCstAffineMVPConstructedCPMV( const PredictionUnit &pu, MotionInfo miNew[3], bool isAvaNew[3], Position pos[3], const RefPicList &refPicList, const int &refIdx, AffineAMVPInfo &affiAmvpInfo);
  void getNonAdjCstMergeCand(const PredictionUnit &pu, AffineMergeCtx &affMrgCtx, const int mrgCandIdx = -1, bool isInitialized = false);
  int  getNonAdjAffParaDivFun(int num1, int num2);
#endif

#if JVET_AL0162_CONSTRUCTED_AFFINE_CMVP
  void getChainedNonAdjCstMergeCand(const PredictionUnit &pu, AffineMergeCtx &affMrgCtx, int list, const int mrgCandIdx = -1, bool isInitialized = false);
  void getChainedCstMergeCand(const PredictionUnit &pu, AffineMergeCtx &affMrgCtx, int list, const int mrgCandIdx = -1, bool isInitialized = false);
  void deriveChainedMVForAff(const PredictionUnit &pu, int list, MvField mvField, const MotionInfo miCascaded, int& interDir, int8_t* chainedRefIdx, Mv* chainedMv);
#endif

#if JVET_AL0160_SBSMVP
  int  getNeiAffNum(const PredictionUnit& pu);
  bool isInsertSubSmvp(const PredictionUnit& pu, AffineMergeCtx &affMrg, int affThres = 1);
  bool isSubMvAvail(const PredictionUnit& pu, Position neiPos, int sbSmvpType, MotionInfo& subMvInfo, MotionInfo& fistSubMvInfo, bool& allTheSame, bool isFirstSubMv = false);
  bool deriveOneSubMv(const PredictionUnit& pu, Position neiPos, int sbSmvpType, MotionInfo& subMvInfo);
  bool checkSimilarMotion(const MotionInfo& mvInfoA, const MotionInfo& mvInfoB, uint32_t mvdSimilarityThresh = 2);
  bool getOneSbSmvpCand(const PredictionUnit& pu, MergeCtx& mrgCtx, int sbSmvpType);
  void setMotion2Buf(const PredictionUnit& pu, MergeCtx& mrgCtx, MotionInfo mi, MotionBuf &mb, int bufX, int bufY, int bufW, int bufH);
#endif

#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  void getAffineMergeCand( InterPrediction* refineTmvpInterPred, RefineTmvpSbTmvpParams* refineTmvpSbTmvpParams, RefineTmvpSbTmvpSubParams* refineTmvpSbTmvpSubParams, const PredictionUnit &pu, AffineMergeCtx& affMrgCtx,
#else
  void getAffineMergeCand( const PredictionUnit &pu, AffineMergeCtx& affMrgCtx, 
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION   
    MergeCtx mrgCtx[2],
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION && JVET_W0090_ARMC_TM
    InterPrediction* m_pcInterSearch,
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
    AffineMergeCtx affineRMVFCtx,
    AffineMergeCtx affineRMVFOriCtx,
    uint16_t numCandtoAdd,
#endif
#if JVET_AK0095_ENHANCED_AFFINE_CANDIDATE
    AffineMergeCtx* tmvpMrgCtx = NULL,
#endif
#if AFFINE_MMVD
                           int mrgCandIdx = -1, bool isAfMmvd = false
#else
                           const int mrgCandIdx = -1
#endif
#if JVET_Z0139_NA_AFF && JVET_W0090_ARMC_TM
                         , bool isZeroCandIdx = false 
#endif
#if JVET_AG0164_AFFINE_GPM
                          , bool noSbTMVP = false
#endif
  );
#if JVET_AK0095_ENHANCED_AFFINE_CANDIDATE
  void getAffineTMVPMergeCand(const PredictionUnit &pu, AffineMergeCtx& affMrgCtx);
#endif
#if JVET_AG0276_NLIC
  void getAltLMAffineMergeCand(const PredictionUnit &pu, AltLMAffineMergeCtx& altLMAffMrgCtx);
#if JVET_AG0276_LIC_FLAG_SIGNALING
  void getAltLMBRAffineMergeCand(const PredictionUnit &pu, AltLMAffineMergeCtx& altLMAffMrgCtx);
#endif
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  bool isAffBMMergeFlagCoded(const PredictionUnit& pu);
  void getRMVFAffineCand(const PredictionUnit &pu, AffineMergeCtx& affineMergeRMVFCtx, AffineMergeCtx& affineMergeRMVFOriCtx, InterPrediction* m_pcInterSearch
  , uint16_t &numCand
#if JVET_AG0276_NLIC
    , AltLMAffineMergeCtx& altAffineMergeRMVFCtx
#if JVET_AG0276_LIC_FLAG_SIGNALING
    , AltLMAffineMergeCtx& altBRAffineMergeRMVFCtx
#endif
#endif
  );
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  void getBMAffineMergeCand(InterPrediction* refineTmvpInterPred, const PredictionUnit &pu, AffineMergeCtx& affineBMMergeCtx, AffineMergeCtx affineMergeRMVFCtx, int mrgCandIdx = -1);
#else
  void getBMAffineMergeCand(const PredictionUnit &pu, AffineMergeCtx& affineBMMergeCtx, AffineMergeCtx affineMergeRMVFCtx, int mrgCandIdx = -1);
#endif
  bool getBMNonAdjCstMergeCand(const PredictionUnit &pu, AffineMergeCtx &affMrgCtx, const int mrgCandIdx = -1);
#endif
#if AFFINE_MMVD
  void    getAfMmvdMvf                (const PredictionUnit &pu, const AffineMergeCtx& affineMergeCtx, MvField mvfMmvd[2][3], const uint16_t afMmvdBaseIdx, const uint16_t offsetStep, const uint16_t offsetDir);
  int32_t getAfMmvdEstBits            (const PredictionUnit &pu);
  uint8_t getMergeIdxFromAfMmvdBaseIdx(AffineMergeCtx& affMrgCtx, uint16_t afMmvdBaseIdx);
#endif
  void setAllAffineMvField            (      PredictionUnit &pu, MvField *mvField, RefPicList eRefList );
  void setAllAffineMv                 (      PredictionUnit &pu, Mv affLT, Mv affRT, Mv affLB, RefPicList eRefList, bool clipCPMVs = false );
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  void setAffineBdofRefinedMotion     (      PredictionUnit &pu, Mv* mvBufDecAffineBDOF);
  bool checkDoAffineBdofRefine        (const PredictionUnit &pu, InterPrediction *interPred);
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  void refineTmvpGetInterMergeSubPuMvpCand(InterPrediction* refineTmvpInterPred, const PredictionUnit &pu, MergeCtx& mrgCtx, RefineTmvpSbTmvpParams* refineTmvpSbTmvpParams, RefineTmvpSbTmvpSubParams* refineTmvpSbTmvpSubParams, int idxSbTmvpParams);
  bool getInterMergeSubPuMvpCand      (InterPrediction* refineTmvpInterPred, RefineTmvpSbTmvpParams* refineTmvpSbTmvpParams, RefineTmvpSbTmvpSubParams* refineTmvpSbTmvpSubParams, const PredictionUnit &pu, MergeCtx& mrgCtx, bool& LICFlag, const int count
#else
  bool getInterMergeSubPuMvpCand      (const PredictionUnit &pu, MergeCtx& mrgCtx, bool& LICFlag, const int count
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION && JVET_AF0163_TM_SUBBLOCK_REFINEMENT
    , bool isRefined
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
    , int subIdx, MergeCtx mergeCtxIn
    , int col = 0
#if JVET_AH0119_SUBBLOCK_TM
#if JVET_AI0183_MVP_EXTENSION
    , int fixRefIdx = 0
#else
    , bool fixRefIdx = false
#endif
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
    , int sbTmvpType = 0
#endif
#endif
#else
    , int mmvdList
#endif
  );
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
  void getTMVPCandOpt(const PredictionUnit &pu, RefPicList refList, int refIdx, MergeCtx &mrgCtx, MergeCtx mergeCtx, int col = 0);
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  void getAmvpSbTmvp(InterPrediction* refineTmvpInterPred, PredictionUnit &pu, MergeCtx& mrgCtx, const Mv mvShift, const bool useAmvpSbTmvpBuf = false, const Position bufTL = Position(0, 0), bool* tmvpBufValid = nullptr, MotionInfo* tmvpMotionBuf = nullptr);
#else
  void getAmvpSbTmvp(PredictionUnit &pu, MergeCtx& mrgCtx, const Mv mvShift, const bool useAmvpSbTmvpBuf = false, const Position bufTL = Position(0, 0), bool* tmvpBufValid = nullptr, MotionInfo* tmvpMotionBuf = nullptr);
#endif
  void clipColPos(int& posX, int& posY, const PredictionUnit& pu);
  void scalePositionInRef(PredictionUnit& pu, const PPS& pps, RefPicList refList, int refIdx, Position& PosY);
#endif
#if JVET_Y0128_NON_CTC
  bool isBiRefScaled(const CodingStructure& cs, const int refIdx0, const int refIdx1);
#endif
#if JVET_X0049_ADAPT_DMVR
  bool isBMMergeFlagCoded(const PredictionUnit& pu);
  bool isBiPredFromDifferentDirEqDistPoc(const PredictionUnit& pu, int refIdx0, int refIdx1);
  bool addBMMergeHMVPCand(const CodingStructure &cs, MergeCtx& mrgCtx, const int& mrgCandIdx, const uint32_t maxNumMergeCandMin1, int &cnt
    , const bool isAvailableA1, const MotionInfo miLeft, const bool isAvailableB1, const MotionInfo miAbove
#if JVET_AI0187_TMVP_FOR_CMVP
    , MergeCtx& tmpMrgCtx, int& tmpMrgIdx
#endif
#if !JVET_Z0075_IBC_HMVP_ENLARGE
    , const bool ibcFlag
    , const bool isGt4x4
#endif
#if TM_MRG
    , const uint32_t mvdSimilarityThresh = 1
#endif
  );
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  void getInterBMCandidates(InterPrediction* refineTmvpInterPred, const PredictionUnit &pu, MergeCtx& mrgCtx,
#else
  void getInterBMCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx,
#endif
#if JVET_AI0187_TMVP_FOR_CMVP
    MergeCtx& tmpMrgCtx1,
#endif
    const int& mrgCandIdx = -1
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
    , MergeCtx* mvpMrgCtx1 = NULL
    , MergeCtx* mvpMrgCtx2 = NULL
#endif
  );
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
#if JVET_AI0187_TMVP_FOR_CMVP
  void getTmvpBMCand(const PredictionUnit &pu, MergeCtx& mvpMrgCtx, MergeCtx& tmpMrgCtx);
  void getNonAdjacentBMCand(const PredictionUnit &pu, MergeCtx& mvpMrgCtx, MergeCtx& tmpMrgCtx);
#else
  void getTmvpBMCand(const PredictionUnit &pu, MergeCtx& mvpMrgCtx);
  void getNonAdjacentBMCand(const PredictionUnit &pu, MergeCtx& mvpMrgCtx);
#endif
#endif
#endif
  bool getInterMergeSubPuRecurCand(const PredictionUnit &pu, MergeCtx &mrgCtx, const int count);
  bool isBiPredFromDifferentDirEqDistPoc(const PredictionUnit &pu);
#if JVET_AG0067_DMVR_EXTENSIONS
  bool isBiPredFromDifferentDirGenDistPoc(const PredictionUnit &pu);
#endif

#if JVET_AJ0097_BDOF_LDB
  bool isBiPredFromSameDirUnEqDistPoc(const PredictionUnit& pu);
  bool isBiPredFromSameDirUnEqDistPoc(const PredictionUnit& pu, int refIdx0, int refIdx1);
  bool isMergeIndexBDOFCondition(const PredictionUnit& pu);
#if JVET_AL0081_BDOF_LDB_MV_REFINE
  uint8_t getLdbMvRefineMode(const PredictionUnit& pu);
#endif
#endif

  void restrictBiPredMergeCandsOne    (PredictionUnit &pu);
#if ENABLE_OBMC
#if JVET_AK0212_GPM_OBMC_MODIFICATION
  unsigned int getSameNeigMotion(PredictionUnit& pu, MotionInfo& mi, Position off, int  iDir, int& iLength, int iMaxLength,
                                 uint8_t checkLevel = 0, int offIdx = 0, std::vector<uint8_t>* nbEachLength = nullptr);
#else
  unsigned int getSameNeigMotion(PredictionUnit &pu, MotionInfo& mi, Position off, int  iDir, int& iLength, int iMaxLength);
#endif
  bool identicalMvOBMC(MotionInfo curMI, MotionInfo neighMI, bool bLD);
#if JVET_AK0212_GPM_OBMC_MODIFICATION
#if JVET_AL0081_BDOF_LDB_MV_REFINE
  bool getNeighborMotion(PredictionUnit& pu, MotionInfo& currMi, MotionInfo& mi, Position nbPos, const int mvTH = 1);
#else
  bool getNeighborMotion(PredictionUnit& pu, MotionInfo& currMi, MotionInfo& mi, Position nbPos);
#endif
#else
  bool getNeighborMotion(PredictionUnit &pu, MotionInfo& mi, Position off, Size unitSize, int iDir);
#endif
#endif
  bool isLMCMode                      (                          unsigned mode);
#if MMLM
  bool isMultiModeLM(unsigned mode);
#endif
  bool isLMCModeEnabled               (const PredictionUnit &pu, unsigned mode);
  bool isChromaIntraModeCrossCheckMode(const PredictionUnit &pu);
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  bool isArmcRefinedMotionEnabled(const PredictionUnit &pu, unsigned mode);
#endif

#if JVET_AE0046_BI_GPM
  void setGpmDirMode(PredictionUnit& pu);
#endif

#if JVET_AG0164_AFFINE_GPM
  void getGeoAffMergeCandidates(PredictionUnit& pu, AffineMergeCtx& gpmAffMrgCtx, InterPrediction* pcInterPred
#if !JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                              , AffineMergeCtx* affMergeCtx = NULL
#endif
  );
  bool isAffineGPMValid(const PredictionUnit& pu);
  bool isAffineGPMSizeValid(const PredictionUnit& pu);

  int  getAffGPMCtxOffset(const PredictionUnit& pu);
#if JVET_AJ0274_GPM_AFFINE_TM
  bool isAffineGpmTmValid(const PredictionUnit& pu);
#endif
#endif
#if JVET_W0097_GPM_MMVD_TM
#if TM_MRG
#if JVET_AE0046_BI_GPM
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  void getGeoMergeCandidates(PredictionUnit& pu, MergeCtx& GeoMrgCtx, InterPrediction* refineTmvpInterPred, MergeCtx* mergeCtx = NULL, bool is4GPM = false);
#else
  void getGeoMergeCandidates(PredictionUnit& pu, MergeCtx& GeoMrgCtx, MergeCtx* mergeCtx = NULL, bool is4GPM = false);
#endif
#else
  void getGeoMergeCandidates(PredictionUnit &pu, MergeCtx &GeoMrgCtx, MergeCtx* mergeCtx = NULL);
#endif
#else
#if JVET_AE0046_BI_GPM
  void getGeoMergeCandidates(const PredictionUnit& pu, MergeCtx& GeoMrgCtx, MergeCtx* mergeCtx = NULL, bool is4GPM = false);
#else
  void getGeoMergeCandidates(const PredictionUnit &pu, MergeCtx &GeoMrgCtx, MergeCtx* mergeCtx = NULL);
#endif
#endif
#else
  void getGeoMergeCandidates          (const PredictionUnit &pu, MergeCtx &GeoMrgCtx);
#endif
  void spanGeoMotionInfo              (      PredictionUnit &pu, MergeCtx &GeoMrgCtx, const uint8_t splitDir, const uint8_t candIdx0, const uint8_t candIdx1, const uint8_t* intraMPM
#if JVET_AG0164_AFFINE_GPM
    , AffineMergeCtx& geoAffMrgCtx
#endif
  );
#if JVET_W0097_GPM_MMVD_TM
#if TM_MRG
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
#if JVET_AE0046_BI_GPM
  void spanGeoMMVDMotionInfo(PredictionUnit& pu, MergeCtx& geoMrgCtx 
#if JVET_AG0164_AFFINE_GPM
     , AffineMergeCtx& geoAffMrgCtx
#if JVET_AJ0274_GPM_AFFINE_TM
     , AffineMergeCtx& geoAffTmMrgCtx
#endif
#endif
    , MergeCtx& geoTmMrgCtx0, MergeCtx& geoTmMrgCtx1, const uint8_t splitDir, const uint8_t mergeIdx0, const uint8_t mergeIdx1, const bool tmFlag0, const bool mmvdFlag0, const uint8_t mmvdIdx0, const bool tmFlag1, const bool mmvdFlag1, const uint8_t mmvdIdx1, const uint8_t bldIdx,const uint8_t *intraMPM,
#if JVET_AI0082_GPM_WITH_INTER_IBC
      const Mv* geoBvList,
#endif
    const bool dmvrPart0 = false, const bool dmvrPart1 = false, Mv* bdofSubPuMvOffsetPart0 = nullptr, Mv* bdofSubPuMvOffsetPart1 = nullptr);
#else
  void spanGeoMMVDMotionInfo(PredictionUnit &pu, MergeCtx &geoMrgCtx, MergeCtx &geoTmMrgCtx0, MergeCtx &geoTmMrgCtx1, const uint8_t splitDir, const uint8_t mergeIdx0, const uint8_t mergeIdx1, const bool tmFlag0, const bool mmvdFlag0, const uint8_t mmvdIdx0, const bool tmFlag1, const bool mmvdFlag1, const uint8_t mmvdIdx1, const uint8_t bldIdx, const uint8_t *intraMPM);
#endif
#else
  void spanGeoMMVDMotionInfo(PredictionUnit &pu, MergeCtx &geoMrgCtx, MergeCtx &geoTmMrgCtx0, MergeCtx &geoTmMrgCtx1, const uint8_t splitDir, const uint8_t mergeIdx0, const uint8_t mergeIdx1, const bool tmFlag0, const bool mmvdFlag0, const uint8_t mmvdIdx0, const bool tmFlag1, const bool mmvdFlag1, const uint8_t mmvdIdx1);
#endif
#else
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
#if JVET_AE0046_BI_GPM
  void spanGeoMMVDMotionInfo(PredictionUnit& pu, MergeCtx& GeoMrgCtx, const uint8_t splitDir, const uint8_t mergeIdx0, const uint8_t mergeIdx1, const bool mmvdFlag0, const uint8_t mmvdIdx0, const bool mmvdFlag1, const uint8_t mmvdIdx1, const uint8_t bldIdx,
    const bool dmvrPart0 = false, const bool dmvrPart1 = false, Mv* bdofSubPuMvOffsetPart0 = nullptr, Mv* bdofSubPuMvOffsetPart1 = nullptr);
#else
  void spanGeoMMVDMotionInfo(PredictionUnit &pu, MergeCtx &GeoMrgCtx, const uint8_t splitDir, const uint8_t mergeIdx0, const uint8_t mergeIdx1, const bool mmvdFlag0, const uint8_t mmvdIdx0, const bool mmvdFlag1, const uint8_t mmvdIdx1, const uint8_t bldIdx);
#endif
#else
  void spanGeoMMVDMotionInfo(PredictionUnit &pu, MergeCtx &GeoMrgCtx, const uint8_t splitDir, const uint8_t mergeIdx0, const uint8_t mergeIdx1, const bool mmvdFlag0, const uint8_t mmvdIdx0, const bool mmvdFlag1, const uint8_t mmvdIdx1);
#endif
#endif
#endif
#if JVET_AE0169_GPM_IBC_IBC
  void spanGeoIBCMotionInfo(PredictionUnit &pu, MergeCtx &geoMrgCtx);
#endif
  bool isAddNeighborMv  (const Mv& currMv, Mv* neighborMvs, int numNeighborMv);
  void getIbcMVPsEncOnly(PredictionUnit &pu, Mv* mvPred, int& nbPred);
#if JVET_AE0169_BIPREDICTIVE_IBC
  bool getDerivedBV(PredictionUnit &pu, const Mv& currentMv, Mv& derivedMv, Mv& derivedMvL1);
#else
  bool getDerivedBV(PredictionUnit &pu, const Mv& currentMv, Mv& derivedMv);
#endif
  bool checkDMVRCondition(const PredictionUnit& pu);
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  bool checkBDMVR4Affine(const PredictionUnit& pu);
  bool checkBDMVRCpmvRefinementPuUsage(const PredictionUnit& pu);
#endif
#if MULTI_PASS_DMVR
#if JVET_AE0046_BI_GPM
#if JVET_AG0135_AFFINE_CIIP
  bool checkBDMVRConditionCIIPAffine(const PredictionUnit& pu, bool disgardGpmFlag = false);
#endif
  bool checkBDMVRCondition(const PredictionUnit& pu, bool disgardGpmFlag = false);
#else
  bool checkBDMVRCondition(const PredictionUnit& pu);
#endif
#if JVET_AG0276_LIC_BDOF_BDMVR
  bool checkBDMVRCondition4Aff(const PredictionUnit& pu);
#endif
#endif
#if INTER_LIC && RPR_ENABLE
  bool checkRprLicCondition(const PredictionUnit& pu);
#endif
#if JVET_Y0128_NON_CTC || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_AMVP)
  bool checkTmEnableCondition(const SPS* sps, const PPS* pps, const Picture* refPic);
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
#if JVET_AG0164_AFFINE_GPM
  bool checkRprRefExistingInGpm(const PredictionUnit& pu, const MergeCtx& geoMrgCtx0, uint8_t candIdx0, const MergeCtx& geoMrgCtx1, uint8_t candIdx1, const AffineMergeCtx& affMergeCtx);
#endif
  bool checkRprRefExistingInGpm(const PredictionUnit& pu, const MergeCtx& geoMrgCtx0, uint8_t candIdx0, const MergeCtx& geoMrgCtx1, uint8_t candIdx1);
#endif
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
  bool checkAffineTMCondition(const PredictionUnit& pu);
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
  enum ExtAffineAmvpType
  {
    EXT_AFFINE_AMVP_TYPE_NONE = 0,
    EXT_AFFINE_AMVP_TYPE_NA_TEMP = 1,
    EXT_AFFINE_AMVP_TYPE_LISTS10 = (1 << 8) + (1 << 4) + 1,
    EXT_AFFINE_AMVP_TYPE_LISTS4 = (2 << 8) + (1 << 4) + 1,
    EXT_AFFINE_AMVP_TYPE_LISTS5 = (3 << 8) + (1 << 4) + 1,
    EXT_AFFINE_AMVP_TYPE_LISTS4_REORDER_FIRST =  (2 << 8) + (2 << 4) + 1
  };
  enum ExtRegularAmvpType
  {
    EXT_REGULAR_AMVP_TYPE_NONE = 0,
    EXT_REGULAR_AMVP_TYPE_NA_SPATIAL = (1 << 4),
    EXT_REGULAR_AMVP_TYPE_LISTS10 = (1 << 4) + 1
  };
  int checkExtAffineAmvpCondition(const PredictionUnit& pu);
  int checkExtRegularAmvpCondition(const PredictionUnit& pu);
#endif

#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  int getRoundMvComp(int x);
#endif

#if INTER_LIC
  void spanLicFlags(PredictionUnit &pu, const bool LICFlag);
#endif
#if MULTI_HYP_PRED
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  Mv   getMultiHypMVP(InterPrediction* refineTmvpInterPred, PredictionUnit &pu, const MultiHypPredictionData &mhData);
  AMVPInfo getMultiHypMVPCands(InterPrediction* refineTmvpInterPred, PredictionUnit &pu, const MultiHypPredictionData &mhData);
  AMVPInfo getMultiHypMVPCandsMerge(InterPrediction* refineTmvpInterPred, PredictionUnit &pu, const RefPicList eRefPicList, const int refIdx);
  AMVPInfo getMultiHypMVPCandsAMVP(InterPrediction* refineTmvpInterPred, PredictionUnit &pu, const RefPicList eRefPicList, const int refIdx);
#else
  Mv   getMultiHypMVP(PredictionUnit &pu, const MultiHypPredictionData &mhData);
  AMVPInfo getMultiHypMVPCands(PredictionUnit &pu, const MultiHypPredictionData &mhData);
  AMVPInfo getMultiHypMVPCandsMerge(PredictionUnit &pu, const RefPicList eRefPicList, const int refIdx);
  AMVPInfo getMultiHypMVPCandsAMVP(PredictionUnit &pu, const RefPicList eRefPicList, const int refIdx);
#endif
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
  bool useRefCombList(const PredictionUnit &pu);
  bool useRefPairList(const PredictionUnit &pu);
#endif
#if JVET_Z0050_CCLM_SLOPE
  bool hasCclmDeltaFlag(const PredictionUnit &pu, const int mode = -1);
#endif
#if JVET_AA0126_GLM
  bool hasGlmFlag      (const PredictionUnit &pu, const int mode = -1);
#endif
#if JVET_AA0057_CCCM || JVET_AB0092_GLM_WITH_LUMA || JVET_AC0119_LM_CHROMA_FUSION
  void getCccmRefLineNum  (const PredictionUnit& pu, const Area area, int& th, int& tv);
#endif
#if JVET_AA0057_CCCM
  bool cccmSingleModeAvail(const PredictionUnit& pu, int intraMode);
  bool cccmMultiModeAvail (const PredictionUnit& pu, int intraMode
#if JVET_AL0191_INTRA_CHROMA_ENCOPT_CCP_CONSTRAINTS
    , bool mmlmFusion = false
#endif
  );
#if JVET_AB0143_CCCM_TS
  bool isLeftCccmMode(const PredictionUnit& pu, int intraMode);
  bool isTopCccmMode(const PredictionUnit& pu, int intraMode);
#if JVET_AD0202_CCCM_MDF
  bool isMultiCccmWithMdf(const PredictionUnit& pu, int intraMode);
#endif
#endif
#endif
#if JVET_Z0050_DIMD_CHROMA_FUSION
  bool hasChromaFusionFlag(const PredictionUnit &pu, int intraMode);
#endif
#if JVET_AD0120_LBCCP
  bool hasCcInsideFilterFlag(const PredictionUnit &pu, int intraMode);
  bool isModetobeFiltered(int intraMode);
#endif
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
  bool hasCcpMergeAdjustFlag(const PredictionUnit& pu);
#endif
#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
  bool hasCCPMergeFusionFlag(const PredictionUnit& pu);
#endif
#if JVET_AJ0081_CHROMA_TMRL
  bool hasChromaTmrl(const PredictionUnit& pu);
#endif
#if JVET_AC0071_DBV
  bool hasChromaBvFlag(const PredictionUnit &pu);
#endif
#if JVET_AC0144_AFFINE_DMVR_REGRESSION
  void deriveAffineCandFromMvField(Position posLT, const int width, const int height, std::vector<RMVFInfo> mvInfoVec, Mv mvAffi[3]);
#endif
#if JVET_AD0085_MPM_SORTING
  bool allowMPMSorted(const PredictionUnit& pu);
#endif
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  bool  hasDecoderDerivedCCP(const PredictionUnit &pu);
#endif
#if JVET_AM0074_INTRA_MERGE
  int canIntraMergeImplicitDst7(const TransformUnit &tu);
#endif
#if JVET_AD0188_CCP_MERGE || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  void ccpParamsToCclmModel(const ComponentID compID, const CCPModelCandidate& params, CclmModel& cclmModel);
  void cclmModelToCcpParams(const ComponentID compId, CCPModelCandidate& params, const CclmModel& cclmModel);

#if JVET_AB0174_CCCM_DIV_FREE
  void cccmModelToCcpParams(CCPModelCandidate& params, const CccmModel cccmModelCb[2], const CccmModel cccmModelCr[2], const int yThres = 0, const int cccmLumaOffset = 0);
#else
  void cccmModelToCcpParams(CCPModelCandidate& params, const CccmModel cccmModelCb[2], const CccmModel cccmModelCr[2], const int yThres = 0);
#endif
  void ccpParamsToCccmModel(const CCPModelCandidate& params, CccmModel cccmModelCb[2], CccmModel cccmModelCr[2]);

#if JVET_AF0073_INTER_CCP_MERGE
#if JVET_AB0174_CCCM_DIV_FREE
  void cccmModelToCcpParams(CCPModelCandidate& params, const CccmModel& cccmModelCb, const CccmModel& cccmModelCr, const int cccmLumaOffset = 0);
#else
  void cccmModelToCcpParams(CCPModelCandidate& params, const CccmModel& cccmModelCb, const CccmModel& cccmModelCr);
#endif
  void ccpParamsToCccmModel(const CCPModelCandidate& params, CccmModel& cccmModelCb, CccmModel& cccmModelCr);
#endif

#if JVET_AB0092_GLM_WITH_LUMA
#if JVET_AB0174_CCCM_DIV_FREE
  void glmModelToCcpParams(const ComponentID compId, CCPModelCandidate& params, const CccmModel& glmModel, const int lumaOffset);
#else
  void glmModelToCcpParams(const ComponentID compId, CCPModelCandidate& params, const CccmModel& glmModel);
#endif
  void ccpParamsToGlmModel(const ComponentID compId, const CCPModelCandidate& params, CccmModel& glmModel);
#endif
  const PredictionUnit *getPUFromPos(const PredictionUnit &pu, const ChannelType &chType, const Position &refPos);
  bool  hasNonLocalCCP(const PredictionUnit &pu);
#if JVET_AF0073_INTER_CCP_MERGE
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
int   getCCPModelCandidateList(const PredictionUnit &pu, CCPModelCandidate candList[], bool isInterCcp = false, int validNum = 0, CCPModelCandidate interCcpMergeList[] = NULL, int selIdx = -1, CCPModelCandidate candList2[] = NULL, int candNum = 0);
#else
  int   getCCPModelCandidateList(const PredictionUnit &pu, CCPModelCandidate candList[], bool isInterCcp = false, int validNum = 0, CCPModelCandidate interCcpMergeList[] = NULL, int selIdx = -1);
#endif
#else
  int   getCCPModelCandidateList(const PredictionUnit &pu, CCPModelCandidate candList[], int selIdx = -1);
#endif
#endif

#if JVET_AE0100_BVGCCCM
  bool hasBvgCccmFlag(const PredictionUnit &pu);
  bool bvgCccmModeAvail(const PredictionUnit &pu);
  bool bvgCccmMultiModeAvail(const PredictionUnit& pu, int intraMode);
  void getBvgCccmCands(PredictionUnit &pu, bool &validBv);
  bool isBvgCccmCand(const PredictionUnit &pu, Mv &chromaBv, int& rrIbcType, int candIdx = 0);
  bool checkIsChromaBvCandidateValid(const PredictionUnit &pu, const Mv chromaBv, int &iWidth, int &iHeight, bool isRefTemplate = false, bool isRefAbove = false);
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
  bool isOppositeLIC(const PredictionUnit &pu);
  bool hasOppositeLICFlag(const PredictionUnit &pu);
#endif
#if JVET_AH0076_OBIC
  bool isObicAvail(const PredictionUnit &pu);
  bool checkAvailBlocks(const PredictionUnit &pu);
#endif
#if JVET_AL0108_BVG_DIMD
  bool isBvgDimdAvail(const PredictionUnit &pu);
#endif
}

// TU tools
namespace TU
{
  uint32_t getNumNonZeroCoeffsNonTSCorner8x8( const TransformUnit &tu, const bool bLuma = true, const bool bChroma = true );
  bool isNonTransformedResidualRotated(const TransformUnit &tu, const ComponentID &compID);
  bool getCbf                         (const TransformUnit &tu, const ComponentID &compID);
  bool getCbfAtDepth                  (const TransformUnit &tu, const ComponentID &compID, const unsigned &depth);
  void setCbfAtDepth                  (      TransformUnit &tu, const ComponentID &compID, const unsigned &depth, const bool &cbf);
  bool isTSAllowed                    (const TransformUnit &tu, const ComponentID  compID);

  bool needsSqrt2Scale                ( const TransformUnit &tu, const ComponentID &compID );
  bool needsBlockSizeTrafoScale       ( const TransformUnit &tu, const ComponentID &compID );
  TransformUnit* getPrevTU          ( const TransformUnit &tu, const ComponentID compID );
  bool           getPrevTuCbfAtDepth( const TransformUnit &tu, const ComponentID compID, const int trDepth );
  int            getICTMode         ( const TransformUnit &tu, int jointCbCr = -1 );
#if SIGN_PREDICTION
  bool getDelayedSignCoding(const TransformUnit &tu, const ComponentID compID);
  bool getUseSignPred(const TransformUnit &tu, const ComponentID compID);
  void     predBorderResi(const Position blkPos, const CPelBuf &recoBuf, const CPelBuf &predBuf, ComponentID compID,
                          uint32_t width, uint32_t height, Pel *predResiBorder, const Pel defaultVal);
  Position posSignHidingFirstCG(const TransformUnit &tu, ComponentID compID);
#endif
#if JVET_AE0059_INTER_CCCM
  bool interCccmAllowed(const TransformUnit& tu);
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  bool interCcpMergeAllowed(const TransformUnit& tu);
#endif
#if JVET_AL0181_ASBT
  bool isTranspostition( const TransformUnit& tu, ComponentID compID );
  bool isInterAsbt( const TransformUnit& tu );
  bool isInterAsbtH( const TransformUnit& tu );
  bool isInterAsbtW( const TransformUnit& tu );
#endif
}

uint32_t getCtuAddr        (const Position& pos, const PreCalcValues &pcv);
int  getNumModesMip   (const Size& block);
int getMipSizeId      (const Size& block);
bool allowLfnstWithMip(const Size& block);
#if JVET_V0130_INTRA_TMP
bool allowLfnstWithTmp();
#endif
#if JVET_AE0078_IBC_LIC_EXTENSION
int getLicDimBit      (const CodingUnit& cu, const ComponentID compID);
#endif

template<typename T, size_t N>
uint32_t updateCandList(T uiMode, double uiCost, static_vector<T, N>& candModeList, static_vector<double, N>& candCostList
  , size_t uiFastCandNum = N, int* iserttPos = nullptr)
{
  CHECK( std::min( uiFastCandNum, candModeList.size() ) != std::min( uiFastCandNum, candCostList.size() ), "Sizes do not match!" );
  CHECK( uiFastCandNum > candModeList.capacity(), "The vector is to small to hold all the candidates!" );

  size_t shift = 0;
  size_t currSize = std::min( uiFastCandNum, candCostList.size() );

  while( shift < uiFastCandNum && shift < currSize && uiCost < candCostList[currSize - 1 - shift] )
  {
    shift++;
  }

  if( candModeList.size() >= uiFastCandNum && shift != 0 )
  {
    for( size_t i = 1; i < shift && i < currSize; i++ ) // i < currSize condition is not needed, but it avoids out of bound compiler errors for old compilers
    {
      candModeList[currSize - i] = candModeList[currSize - 1 - i];
      candCostList[currSize - i] = candCostList[currSize - 1 - i];
    }
    candModeList[currSize - shift] = uiMode;
    candCostList[currSize - shift] = uiCost;
    if (iserttPos != nullptr)
    {
      *iserttPos = int(currSize - shift);
    }
    return 1;
  }
  else if( currSize < uiFastCandNum )
  {
    candModeList.insert( candModeList.end() - shift, uiMode );
    candCostList.insert( candCostList.end() - shift, uiCost );
    if (iserttPos != nullptr)
    {
      *iserttPos = int(candModeList.size() - shift - 1);
    }
    return 1;
  }
  if (iserttPos != nullptr)
  {
    *iserttPos = -1;
  }
  return 0;
}

#if JVET_AB0157_TMRL
template<typename T, size_t N>
uint32_t updateCandList(T uiMode, uint64_t uiCost, static_vector<T, N>& candModeList, static_vector<uint64_t, N>& candCostList
  , size_t uiFastCandNum = N, int* iserttPos = nullptr)
{
  CHECK(std::min(uiFastCandNum, candModeList.size()) != std::min(uiFastCandNum, candCostList.size()), "Sizes do not match!");
  CHECK(uiFastCandNum > candModeList.capacity(), "The vector is to small to hold all the candidates!");

  size_t i;
  size_t shift = 0;
  size_t currSize = std::min(uiFastCandNum, candCostList.size());

  while (shift < uiFastCandNum && shift < currSize && uiCost < candCostList[currSize - 1 - shift])
  {
    shift++;
  }

  if (candModeList.size() >= uiFastCandNum && shift != 0)
  {
    for (i = 1; i < shift; i++)
    {
      candModeList[currSize - i] = candModeList[currSize - 1 - i];
      candCostList[currSize - i] = candCostList[currSize - 1 - i];
    }
    candModeList[currSize - shift] = uiMode;
    candCostList[currSize - shift] = uiCost;
    if (iserttPos != nullptr)
    {
      *iserttPos = int(currSize - shift);
    }
    return 1;
  }
  else if (currSize < uiFastCandNum)
  {
    candModeList.insert(candModeList.end() - shift, uiMode);
    candCostList.insert(candCostList.end() - shift, uiCost);
    if (iserttPos != nullptr)
    {
      *iserttPos = int(candModeList.size() - shift - 1);
    }
    return 1;
  }
  if (iserttPos != nullptr)
  {
    *iserttPos = -1;
  }
  return 0;
}
#endif

#if JVET_AK0217_INTRA_MTSS
template<typename T, size_t N>
uint32_t updateCandListB2S(T uiMode, int uiCost, static_vector<T, N>& candModeList, static_vector<int, N>& candCostList
  , size_t uiFastCandNum = N, int* iserttPos = nullptr)
{
  CHECK(std::min(uiFastCandNum, candModeList.size()) != std::min(uiFastCandNum, candCostList.size()), "Sizes do not match!");
  CHECK(uiFastCandNum > candModeList.capacity(), "The vector is to small to hold all the candidates!");

  size_t i;
  size_t shift = 0;
  size_t currSize = std::min(uiFastCandNum, candCostList.size());

  while (shift < uiFastCandNum && shift < currSize && uiCost > candCostList[currSize - 1 - shift])
  {
    shift++;
  }

  if (candModeList.size() >= uiFastCandNum && shift != 0)
  {
    for (i = 1; i < shift; i++)
    {
      candModeList[currSize - i] = candModeList[currSize - 1 - i];
      candCostList[currSize - i] = candCostList[currSize - 1 - i];
    }
    candModeList[currSize - shift] = uiMode;
    candCostList[currSize - shift] = uiCost;
    if (iserttPos != nullptr)
    {
      *iserttPos = int(currSize - shift);
    }
    return 1;
  }
  else if (currSize < uiFastCandNum)
  {
    candModeList.insert(candModeList.end() - shift, uiMode);
    candCostList.insert(candCostList.end() - shift, uiCost);
    if (iserttPos != nullptr)
    {
      *iserttPos = int(candModeList.size() - shift - 1);
    }
    return 1;
  }
  if (iserttPos != nullptr)
  {
    *iserttPos = -1;
  }
  return 0;
}
#endif

#if JVET_W0097_GPM_MMVD_TM
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
template<size_t N>
void orderCandList(uint8_t uiMode, bool bNonMMVDListCand, int splitDir, double uiCost, uint8_t bldIdx, static_vector<uint8_t, N>& candModeList, static_vector<bool, N>& isNonMMVDListIdx, static_vector<int, N>&  candSplitDirList, static_vector<double, N>& candCostList, static_vector<uint8_t, N>&  geoBldList, size_t uiFastCandNum = N)
{
  CHECK(std::min(uiFastCandNum, candModeList.size()) != std::min(uiFastCandNum, candCostList.size()), "Sizes do not match!");
  CHECK(uiFastCandNum > candModeList.capacity(), "The vector is to small to hold all the candidates!");

  size_t i;
  size_t shift = 0;
  size_t currSize = std::min(uiFastCandNum, candCostList.size());

  while (shift < uiFastCandNum && shift < currSize && uiCost < candCostList[currSize - 1 - shift])
  {
    shift++;
  }

  if (candModeList.size() >= uiFastCandNum && shift != 0)
  {
    for (i = 1; i < shift; i++)
    {
      candModeList[currSize - i] = candModeList[currSize - 1 - i];
      isNonMMVDListIdx[currSize - i] = isNonMMVDListIdx[currSize - 1 - i];
      candSplitDirList[currSize - i] = candSplitDirList[currSize - 1 - i];
      candCostList[currSize - i] = candCostList[currSize - 1 - i];
      geoBldList[currSize - i] = geoBldList[currSize - 1 - i];
    }
    candModeList[currSize - shift] = uiMode;
    isNonMMVDListIdx[currSize - shift] = bNonMMVDListCand;
    candSplitDirList[currSize - shift] = splitDir;
    candCostList[currSize - shift] = uiCost;
    geoBldList[currSize - shift] = bldIdx;
    return;
  }
  else if (currSize < uiFastCandNum)
  {
    candModeList.insert(candModeList.end() - shift, uiMode);
    isNonMMVDListIdx.insert(isNonMMVDListIdx.end() - shift, bNonMMVDListCand);
    candSplitDirList.insert(candSplitDirList.end() - shift, splitDir);
    candCostList.insert(candCostList.end() - shift, uiCost);
    geoBldList.insert(geoBldList.end() - shift, bldIdx);
    return;
  }
  return;
}
#else
template<size_t N>
void orderCandList(uint8_t uiMode, bool bNonMMVDListCand, int splitDir, double uiCost, static_vector<uint8_t, N>& candModeList, static_vector<bool, N>& isNonMMVDListIdx, static_vector<int, N>&  candSplitDirList, static_vector<double, N>& candCostList, size_t uiFastCandNum = N)
{
  CHECK(std::min(uiFastCandNum, candModeList.size()) != std::min(uiFastCandNum, candCostList.size()), "Sizes do not match!");
  CHECK(uiFastCandNum > candModeList.capacity(), "The vector is to small to hold all the candidates!");

  size_t i;
  size_t shift = 0;
  size_t currSize = std::min(uiFastCandNum, candCostList.size());

  while (shift < uiFastCandNum && shift < currSize && uiCost < candCostList[currSize - 1 - shift])
  {
    shift++;
  }

  if (candModeList.size() >= uiFastCandNum && shift != 0)
  {
    for (i = 1; i < shift; i++)
    {
      candModeList[currSize - i] = candModeList[currSize - 1 - i];
      isNonMMVDListIdx[currSize - i] = isNonMMVDListIdx[currSize - 1 - i];
      candSplitDirList[currSize - i] = candSplitDirList[currSize - 1 - i];
      candCostList[currSize - i] = candCostList[currSize - 1 - i];
    }
    candModeList[currSize - shift] = uiMode;
    isNonMMVDListIdx[currSize - shift] = bNonMMVDListCand;
    candSplitDirList[currSize - shift] = splitDir;
    candCostList[currSize - shift] = uiCost;
    return;
  }
  else if (currSize < uiFastCandNum)
  {
    candModeList.insert(candModeList.end() - shift, uiMode);
    isNonMMVDListIdx.insert(isNonMMVDListIdx.end() - shift, bNonMMVDListCand);
    candSplitDirList.insert(candSplitDirList.end() - shift, splitDir);
    candCostList.insert(candCostList.end() - shift, uiCost);
    return;
  }
  return;
}
#endif

template<size_t N>
uint32_t updateGeoMMVDCandList(double uiCost, int splitDir, int mergeCand0, int mergeCand1, int mmvdCand0, int mmvdCand1,
  static_vector<double, N>& candCostList, static_vector<int, N>& geoSplitDirList, static_vector<int, N>& geoMergeCand0, static_vector<int, N>& geoMergeCand1, static_vector<int, N>& geoMmvdCand0, static_vector<int, N>& geoMmvdCand1,
  size_t uiFastCandNum)
{
  CHECK(std::min(uiFastCandNum, geoSplitDirList.size()) != std::min(uiFastCandNum, candCostList.size()), "Sizes do not match!");
  CHECK(uiFastCandNum > candCostList.capacity(), "The vector is to small to hold all the candidates!");

  size_t i;
  size_t shift = 0;
  size_t currSize = std::min(uiFastCandNum, candCostList.size());

  while (shift < uiFastCandNum && shift < currSize && uiCost < candCostList[currSize - 1 - shift])
  {
    shift++;
  }

  if (candCostList.size() >= uiFastCandNum && shift != 0)
  {
    for (i = 1; i < shift; i++)
    {
      geoSplitDirList[currSize - i] = geoSplitDirList[currSize - 1 - i];
      geoMergeCand0[currSize - i] = geoMergeCand0[currSize - 1 - i];
      geoMergeCand1[currSize - i] = geoMergeCand1[currSize - 1 - i];
      geoMmvdCand0[currSize - i] = geoMmvdCand0[currSize - 1 - i];
      geoMmvdCand1[currSize - i] = geoMmvdCand1[currSize - 1 - i];
      candCostList[currSize - i] = candCostList[currSize - 1 - i];
    }
    geoSplitDirList[currSize - shift] = splitDir;
    geoMergeCand0[currSize - shift] = mergeCand0;
    geoMergeCand1[currSize - shift] = mergeCand1;
    geoMmvdCand0[currSize - shift] = mmvdCand0;
    geoMmvdCand1[currSize - shift] = mmvdCand1;
    candCostList[currSize - shift] = uiCost;
    return 1;
  }
  else if (currSize < uiFastCandNum)
  {
    geoSplitDirList.insert(geoSplitDirList.end() - shift, splitDir);
    geoMergeCand0.insert(geoMergeCand0.end() - shift, mergeCand0);
    geoMergeCand1.insert(geoMergeCand1.end() - shift, mergeCand1);
    geoMmvdCand0.insert(geoMmvdCand0.end() - shift, mmvdCand0);
    geoMmvdCand1.insert(geoMmvdCand1.end() - shift, mmvdCand1);
    candCostList.insert(candCostList.end() - shift, uiCost);
    return 1;
  }
  return 0;
}

#if JVET_AC0112_IBC_GPM
template<size_t N>
uint32_t updateGeoIbcCandList(double uiCost, int splitDir, int mergeCand0, int mergeCand1, int mmvdCand0, int mmvdCand1,
  static_vector<double, N>& candCostList, static_vector<int, N>& geoSplitDirList, static_vector<int, N>& geoMergeCand0, static_vector<int, N>& geoMergeCand1, size_t uiFastCandNum)
{
  size_t i;
  size_t shift = 0;
  size_t currSize = std::min(uiFastCandNum, candCostList.size());

  while (shift < uiFastCandNum && shift < currSize && uiCost < candCostList[currSize - 1 - shift])
  {
    shift++;
  }

  if (candCostList.size() >= uiFastCandNum && shift != 0)
  {
    for (i = 1; i < shift; i++)
    {
      geoSplitDirList[currSize - i] = geoSplitDirList[currSize - 1 - i];
      geoMergeCand0[currSize - i] = geoMergeCand0[currSize - 1 - i];
      geoMergeCand1[currSize - i] = geoMergeCand1[currSize - 1 - i];
      candCostList[currSize - i] = candCostList[currSize - 1 - i];
    }
    geoSplitDirList[currSize - shift] = splitDir;
    geoMergeCand0[currSize - shift] = mergeCand0;
    geoMergeCand1[currSize - shift] = mergeCand1;
    candCostList[currSize - shift] = uiCost;
    return 1;
  }
  else if (currSize < uiFastCandNum)
  {
    geoSplitDirList.insert(geoSplitDirList.end() - shift, splitDir);
    geoMergeCand0.insert(geoMergeCand0.end() - shift, mergeCand0);
    geoMergeCand1.insert(geoMergeCand1.end() - shift, mergeCand1);
    candCostList.insert(candCostList.end() - shift, uiCost);
    return 1;
  }
  return 0;
}

template<size_t N>
void sortCandList(double uiCost, int mergeCand, static_vector<double, N>& candCostList, static_vector<int, N>& mergeCandList, int fastCandNum)
{
  size_t i;
  size_t shift = 0;
  size_t currSize = candCostList.size();
  CHECK(currSize > fastCandNum, "list overflow!");

  while (shift < currSize && uiCost < candCostList[currSize - 1 - shift])
  {
    shift++;
  }

  if (currSize == fastCandNum && shift != 0)
  {
    for (i = 1; i < shift; i++)
    {
      mergeCandList[currSize - i] = mergeCandList[currSize - 1 - i];
      candCostList[currSize - i] = candCostList[currSize - 1 - i];
    }
    mergeCandList[currSize - shift] = mergeCand;
    candCostList[currSize - shift] = uiCost;
  }
  else if (currSize < fastCandNum)
  {
    mergeCandList.insert(mergeCandList.end() - shift, mergeCand);
    candCostList.insert(candCostList.end() - shift, uiCost);
  }
}
#endif

template<size_t N>
void sortCandList(double uiCost, int mergeCand, int mmvdCand, static_vector<double, N>& candCostList, static_vector<int, N>& mergeCandList, static_vector<int, N>& mmvdCandList, int fastCandNum)
{
  size_t i;
  size_t shift = 0;
  size_t currSize = candCostList.size();
  CHECK(currSize > fastCandNum, "list overflow!");

  while (shift < currSize && uiCost < candCostList[currSize - 1 - shift])
  {
    shift++;
  }

  if (currSize == fastCandNum && shift != 0)
  {
    for (i = 1; i < shift; i++)
    {
      mergeCandList[currSize - i] = mergeCandList[currSize - 1 - i];
      mmvdCandList[currSize - i] = mmvdCandList[currSize - 1 - i];
      candCostList[currSize - i] = candCostList[currSize - 1 - i];
    }
    mergeCandList[currSize - shift] = mergeCand;
    mmvdCandList[currSize - shift] = mmvdCand;
    candCostList[currSize - shift] = uiCost;
  }
  else if (currSize < fastCandNum)
  {
    mergeCandList.insert(mergeCandList.end() - shift, mergeCand);
    mmvdCandList.insert(mmvdCandList.end() - shift, mmvdCand);
    candCostList.insert(candCostList.end() - shift, uiCost);
  }
}

#if JVET_Y0065_GPM_INTRA
template<size_t N>
void sortIntraCandList(double uiCost, int mergeCand, static_vector<double, N>& candCostList, static_vector<int, N>& intraCandList)
{
  size_t shift = 0;
  size_t currSize = candCostList.size();
#if JVET_AI0082_GPM_WITH_INTER_IBC
  CHECK(currSize >= GEO_MAX_NUM_INTRA_CANDS + GEO_MAX_NUM_IBC_CANDS, "list overflow!");
#else
  CHECK(currSize >= GEO_MAX_NUM_INTRA_CANDS, "list overflow!");
#endif

  while (shift < currSize && uiCost < candCostList[currSize - 1 - shift])
  {
    shift++;
  }

  intraCandList.insert(intraCandList.end() - shift, mergeCand);
  candCostList.insert(candCostList.end() - shift, uiCost);
}
#endif
#endif

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
bool storeContexts( const Slice* slice, const int ctuXPosInCtus, const int ctuYPosInCtus );
#endif

#if JVET_AC0144_AFFINE_DMVR_REGRESSION
int deriveAffineSubBlkSize(const int sz, const int minSbSz, const int deltaMvX, const int deltaMvY, const int shift);
#endif
#if JVET_AD0085_TMRL_EXTENSION
int getSpatialIpm(
#if JVET_AL0125_IMPROVEMENT_ON_MPM
  bool planarDisable,
#endif
  const PredictionUnit& pu, uint8_t* spatialIpm, const int maxCands
#if JVET_AC0094_REF_SAMPLES_OPT
                , const bool& isForcedValid
#endif
                , bool extPrecision = false
#if JVET_AK0061_PDP_MPM
  , const bool& pdpRefAvailable = false
#endif
#if JVET_AD0085_MPM_SORTING
#if JVET_AK0061_PDP_MPM
  , const bool& mpmSort = false
#endif

                , IntraPrediction* pIntraPred = nullptr
#endif
);
void fillMPMList(
#if JVET_AL0125_IMPROVEMENT_ON_MPM
  const bool &mpmSort,
  bool planarDisable,
#endif
  const PredictionUnit& pu, uint8_t* mpm, const int numToFill, const int numCand, bool extPrecision = false
#if JVET_AK0061_PDP_MPM
  , const bool& pdpRefAvailable = false
#endif
);
void fillNonMPMList(uint8_t* mpm, uint8_t* nonMpm
#if JVET_AK0061_PDP_MPM
  , const PredictionUnit& pu, const bool& pdpRefAvailable = false
#elif JVET_AK0059_MDIP
  , const PredictionUnit& pu
#endif

);
#endif
#if JVET_AK0061_PDP_MPM
int getPDPPredMode(const SizeType& width, const SizeType& height, const uint8_t& uiMode, const bool includedMode[]);
#endif

#if JVET_AG0058_EIP
Position getRecoLinesEIP(const CodingUnit& cu, const ComponentID compId);
bool     getAllowedEipMerge(const CodingUnit &cu, const ComponentID compId);
bool     getAllowedEip(const CodingUnit &cu, const ComponentID compId);
#if JVET_AJ0082_MM_EIP
int getAllowedCurEip(const CodingUnit& cu, const ComponentID compId, static_vector<EIPInfo, NUM_DERIVED_EIP>& eipInfoList, bool bMmEip = false);
#else 
int getAllowedCurEip(const CodingUnit& cu, const ComponentID compId, static_vector<EIPInfo, NUM_DERIVED_EIP>& eipInfoList);
#endif
#if JVET_AL0106_BV_EIP
bool     allowBvEip(const CodingUnit& cu);
#endif
#endif

#if JVET_AI0082_TEMPORAL_BV
bool getColocatedBVP(const PredictionUnit &pu, const Position &posIn, MotionInfo &rcMv, const int col);
void getTemporalBv(const PredictionUnit &pu, std::vector<MotionInfo>& temporalMiCandList);
#endif


#if JVET_AJ0203_DIMD_2X2_EDGE_OP
inline bool use2x2EdgeOperator(const Size& sz) { return sz.area() <= DIMD_SMALL_BLOCK_THR ? true : false; };
#endif 

#if JVET_AK0065_TALF
bool isBiTAlf(const int tAlfMode);
bool isMvTAlf(const int tAlfMode);
bool isFwdTAlf(const int tAlfMode);
#endif

#if JVET_AG0061_INTER_LFNST_NSPT
int buildHistogram(const Pel *pReco, int iStride, uint32_t uiHeight, uint32_t uiWidth, int *piHistogram, int direction, int bw, int bh
#if JVET_AJ0203_DIMD_2X2_EDGE_OP
                  , const int filterSizeIdx = 0// 0 - default, 1 - small
#endif  
#if  JVET_AK0217_INTRA_MTSS
                  , const bool subsampling = 0
#endif  
);
#endif
#if JVET_AJ0267_ADAPTIVE_HOG
void buildHistogramAdaptive(const Pel *pReco, int iStride, uint32_t uiHeight, uint32_t uiWidth, uint32_t* uiSizeExt, int *piHistogram, int direction,
                  const int cuHeight, const int cuWidth, int maxTemplateSize, bool* isExtraAvailable, uint64_t maxAmp
#if JVET_AJ0203_DIMD_2X2_EDGE_OP
                  , const int filterSizeIdx = 0// 0 - default, 1 - small
#endif  
);
#endif
#if JVET_AJ0161_OBMC_EXT_WITH_INTRA_PRED
void calcGradForOBMC(const PredictionUnit pu, const Pel* pReco, const int iStride, const int totalUnits, const int templateSize, const int blkSize, int* modeBuf, const int isAbove, const bool isExistFirst, const bool isExistLast);
#endif
#if JVET_AJ0249_NEURAL_NETWORK_BASED
bool isAllowedMultiple(const SizeType width, const SizeType height);
#endif
#if JVET_AK0059_MDIP
void buildExcludingMode(CodingUnit& cu, int *histogram, bool *includedMode);
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
int getTmpFusionIdx(const int tmpIdx);
int getTmpFusionNumber(const int tmpFusionIdx
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
                       , const bool isBvFus
#endif
                       );
#endif
#endif

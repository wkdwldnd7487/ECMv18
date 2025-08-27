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

/**
 \file     EncSampleAdaptiveOffset.h
 \brief    estimation part of sample adaptive offset class (header)
 */

#ifndef __ENCSAMPLEADAPTIVEOFFSET__
#define __ENCSAMPLEADAPTIVEOFFSET__

#include "CommonLib/SampleAdaptiveOffset.h"

#include "CABACWriter.h"

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

struct SAOStatData //data structure for SAO statistics
{
  int64_t diff[MAX_NUM_SAO_CLASSES];
  int64_t count[MAX_NUM_SAO_CLASSES];

  SAOStatData(){}
  ~SAOStatData(){}
  void reset()
  {
    ::memset(diff, 0, sizeof(int64_t)*MAX_NUM_SAO_CLASSES);
    ::memset(count, 0, sizeof(int64_t)*MAX_NUM_SAO_CLASSES);
  }
  const SAOStatData& operator=(const SAOStatData& src)
  {
    ::memcpy(diff, src.diff, sizeof(int64_t)*MAX_NUM_SAO_CLASSES);
    ::memcpy(count, src.count, sizeof(int64_t)*MAX_NUM_SAO_CLASSES);
    return *this;
  }
  const SAOStatData& operator+= (const SAOStatData& src)
  {
    for(int i=0; i< MAX_NUM_SAO_CLASSES; i++)
    {
      diff[i] += src.diff[i];
      count[i] += src.count[i];
    }
    return *this;
  }
};

#if JVET_W0066_CCSAO
struct CcSaoStatData
{
  int64_t  diff [MAX_CCSAO_CLASS_NUM];
  uint32_t count[MAX_CCSAO_CLASS_NUM];

  CcSaoStatData(){}
  ~CcSaoStatData(){}
  void reset()
  {
    ::memset(diff,  0, sizeof(int64_t)  * MAX_CCSAO_CLASS_NUM);
    ::memset(count, 0, sizeof(uint32_t) * MAX_CCSAO_CLASS_NUM);
  }
  const CcSaoStatData& operator=(const CcSaoStatData& src)
  {
    ::memcpy(diff,  src.diff,  sizeof(int64_t)  * MAX_CCSAO_CLASS_NUM);
    ::memcpy(count, src.count, sizeof(uint32_t) * MAX_CCSAO_CLASS_NUM);
    return *this;
  }
  const CcSaoStatData& operator+= (const CcSaoStatData& src)
  {
    for(int i = 0; i < MAX_CCSAO_CLASS_NUM; i++)
    {
      diff [i] += src.diff [i];
      count[i] += src.count[i];
    }
    return *this;
  }
};

struct CcSaoEncParam
{
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
#if JVET_AL0142_CCSAO_REUSE_CTU
  int      reusePrv;
#else
  bool     reusePrv;
#endif
  int      reusePrvId;
#endif
  uint8_t  setNum;
  bool     setEnabled [MAX_CCSAO_SET_NUM];
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
  uint8_t  setType    [MAX_CCSAO_SET_NUM];
#endif
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  uint16_t candPos    [MAX_CCSAO_SET_NUM][MAX_NUM_COMPONENT];
#else
  uint16_t candPos    [MAX_CCSAO_SET_NUM][MAX_NUM_LUMA_COMP];
#endif
  uint16_t bandNum    [MAX_CCSAO_SET_NUM][MAX_NUM_COMPONENT];
  short    offset     [MAX_CCSAO_SET_NUM][MAX_CCSAO_CLASS_NUM];
  uint8_t  mapIdxToIdc[MAX_CCSAO_SET_NUM + 1];

  CcSaoEncParam() {}
  ~CcSaoEncParam() {}
  void reset()
  {
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
    reusePrv   = false;
    reusePrvId = 0;
#endif
    setNum     = 0;
    ::memset(setEnabled,  false, sizeof(setEnabled));
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
    ::memset(setType,         0, sizeof(setType));
#endif
    ::memset(candPos,         0, sizeof(candPos));
    ::memset(bandNum,         0, sizeof(bandNum));
    ::memset(offset,          0, sizeof(offset));
    ::memset(mapIdxToIdc,     0, sizeof(mapIdxToIdc));
  }
  const CcSaoEncParam& operator= (const CcSaoEncParam& src)
  {
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
    reusePrv   = src.reusePrv;
    reusePrvId = src.reusePrvId;
#endif
    setNum     = src.setNum;
    ::memcpy(setEnabled,  src.setEnabled,  sizeof(setEnabled));
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
    ::memcpy(setType,     src.setType,     sizeof(setType));
#endif
    ::memcpy(candPos,     src.candPos,     sizeof(candPos));
    ::memcpy(bandNum,     src.bandNum,     sizeof(bandNum));
    ::memcpy(offset,      src.offset,      sizeof(offset));
    ::memcpy(mapIdxToIdc, src.mapIdxToIdc, sizeof(mapIdxToIdc));
    return *this;
  }
};
#endif

class EncSampleAdaptiveOffset : public SampleAdaptiveOffset
{
public:
  EncSampleAdaptiveOffset();
  virtual ~EncSampleAdaptiveOffset();

  //interface
  void createEncData(bool isPreDBFSamplesUsed, uint32_t numCTUsPic);
  void destroyEncData();
  void initCABACEstimator( CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice );
  void SAOProcess( CodingStructure& cs, bool* sliceEnabled, const double* lambdas,
#if ENABLE_QPA
                   const double lambdaChromaWeight,
#endif
                   const bool bTestSAODisableAtPictureLevel, const double saoEncodingRate, const double saoEncodingRateChroma, const bool isPreDBFSamplesUsed, bool isGreedyMergeEncoding
#if JVET_V0094_BILATERAL_FILTER
                                          , BIFCabacEst* bifCABACEstimator
#endif
                  );
#if JVET_W0066_CCSAO
  void CCSAOProcess(CodingStructure& cs, const double* lambdas, const int intraPeriod);
#endif
  void disabledRate( CodingStructure& cs, SAOBlkParam* reconParams, const double saoEncodingRate, const double saoEncodingRateChroma );
  void getPreDBFStatistics(CodingStructure& cs);
private: //methods

  void deriveLoopFilterBoundaryAvailibility(CodingStructure& cs, const Position &pos, bool& isLeftAvail, bool& isAboveAvail, bool& isAboveLeftAvail) const;
  void getStatistics(std::vector<SAOStatData**>& blkStats, PelUnitBuf& orgYuv, PelUnitBuf& srcYuv,
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                     PelUnitBuf& bifYuv,
#endif
                     CodingStructure& cs, bool isCalculatePreDeblockSamples = false);
  void decidePicParams(const Slice& slice, bool* sliceEnabled, const double saoEncodingRate, const double saoEncodingRateChroma);
  void decideBlkParams( CodingStructure& cs, bool* sliceEnabled, std::vector<SAOStatData**>& blkStats, PelUnitBuf& srcYuv, PelUnitBuf& resYuv, SAOBlkParam* reconParams, SAOBlkParam* codedParams, const bool bTestSAODisableAtPictureLevel,
#if ENABLE_QPA
                        const double chromaWeight,
#endif
                        const double saoEncodingRate, const double saoEncodingRateChroma, const bool isGreedymergeEncoding );
  void getBlkStats(const ComponentID compIdx, const int channelBitDepth, SAOStatData* statsDataTypes, Pel* srcBlk, Pel* orgBlk,
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                   Pel* bifBlk, int bifStride,
#endif
                   int srcStride, int orgStride, int width, int height, bool isLeftAvail,  bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail, bool isAboveRightAvail, bool isCalculatePreDeblockSamples
                 , bool isCtuCrossedByVirtualBoundaries, int horVirBndryPos[], int verVirBndryPos[], int numHorVirBndry, int numVerVirBndry
    );
  void deriveModeNewRDO(const BitDepths &bitDepths, int ctuRsAddr, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES], bool* sliceEnabled, std::vector<SAOStatData**>& blkStats, SAOBlkParam& modeParam, double& modeNormCost );
  void deriveModeMergeRDO(const BitDepths &bitDepths, int ctuRsAddr, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES], bool* sliceEnabled, std::vector<SAOStatData**>& blkStats, SAOBlkParam& modeParam, double& modeNormCost );
  int64_t getDistortion(const int channelBitDepth, int typeIdc, int typeAuxInfo, int* offsetVal, SAOStatData& statData);
  void deriveOffsets(ComponentID compIdx, const int channelBitDepth, int typeIdc, SAOStatData& statData, int* quantOffsets, int& typeAuxInfo);
#if JVET_AJ0237_INTERNAL_12BIT
  inline int64_t estSaoDist(int64_t count, int64_t offset, int64_t diffSum, int shift, int bdShift = 0);
#else
  inline int64_t estSaoDist(int64_t count, int64_t offset, int64_t diffSum, int shift);
#endif
  inline int estIterOffset(int typeIdx, double lambda, int offsetInput, int64_t count, int64_t diffSum, int shift, int bitIncrease, int64_t& bestDist, double& bestCost, int offsetTh );
  void addPreDBFStatistics(std::vector<SAOStatData**>& blkStats);
#if JVET_W0066_CCSAO
  void setupCcSaoLambdas(CodingStructure& cs, const double* lambdas);
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  void setupCcSaoSH(CodingStructure& cs, const CPelUnitBuf& orgYuv);
#endif
  void deriveCcSao(CodingStructure& cs, const ComponentID compID, const CPelUnitBuf& orgYuv, const CPelUnitBuf& srcYuv, const CPelUnitBuf& dstYuv);
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
  void setupInitCcSaoParam(CodingStructure &cs, const ComponentID compID, const int setNum
                         , int64_t *trainingDistortion[MAX_CCSAO_SET_NUM]
                         , CcSaoStatData *blkStats    [MAX_CCSAO_SET_NUM], CcSaoStatData frameStats    [MAX_CCSAO_SET_NUM]
                         , CcSaoStatData *blkStatsEdge[MAX_CCSAO_SET_NUM], CcSaoStatData frameStatsEdge[MAX_CCSAO_SET_NUM]
                         , CcSaoEncParam &initCcSaoParam, CcSaoEncParam &bestCcSaoParam
                         , uint8_t *initCcSaoControl, uint8_t *bestCcSaoControl);
#else
  void setupInitCcSaoParam(CodingStructure &cs, const ComponentID compID, const int setNum
                         , int64_t *trainingDistortion[MAX_CCSAO_SET_NUM]
                         , CcSaoStatData *blkStats    [MAX_CCSAO_SET_NUM], CcSaoStatData frameStats    [MAX_CCSAO_SET_NUM]
                         , CcSaoEncParam &initCcSaoParam, CcSaoEncParam &bestCcSaoParam
                         , uint8_t *initCcSaoControl, uint8_t *bestCcSaoControl);
#endif
  void setupTempCcSaoParam(CodingStructure& cs, const ComponentID compID, const int setNum
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
                         , const int edgeCmp
#endif
                         , const int candPosY, const int bandNumY, const int bandNumU, const int bandNumV
                         , CcSaoEncParam& tempCcSaoParam, CcSaoEncParam& initCcSaoParam
                         , uint8_t* tempCcSaoControl, uint8_t* initCcSaoControl
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
                         , int setType = CCSAO_SET_TYPE_BAND
#else
                         , int setType = 0
#endif
#endif
                          );
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  void setupTempCcSaoParamFromPrv(CodingStructure& cs, const ComponentID compID, const int prvId
#if JVET_AL0142_CCSAO_REUSE_CTU
                                , const int reuseMode
#endif
                                , CcSaoEncParam& tempCcSaoParam, CcSaoPrvParam& prvCcSaoParam
                                , uint8_t* tempCcSaoControl);
#endif
  void getCcSaoStatistics(CodingStructure& cs, const ComponentID compID
                        , const CPelUnitBuf& orgYuv, const CPelUnitBuf& srcYuv, const CPelUnitBuf& dstYuv
                        , CcSaoStatData* blkStats[MAX_CCSAO_SET_NUM], const CcSaoEncParam& ccSaoParam);
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  inline int getCcSaoEdgeStatIdx(const int ctuRsAddr, const int bandIdc
                               , const int edgeCmp, const int edgeIdc
                               , const int edgeDir, const int edgeThr
                                );
  void resetCcSaoEdgeStats(CcSaoStatData *blkStatsEdge);
#else
  int  calcEdgeStatIndex(const int ctuRsAddr, const int mode, const int type, const int th);
  void resetblkStatsEdgePre(CcSaoStatData *blkStatsEdge[MAX_CCSAO_SET_NUM - 1]);
#endif
#endif
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  void prepareCcSaoEdgeStats(CodingStructure &cs, const ComponentID compID
                           , const CPelUnitBuf &orgYuv, const CPelUnitBuf &srcYuv, const CPelUnitBuf &dstYuv
                           , CcSaoStatData *blkStats, const CcSaoEncParam &ccSaoParam);
#else
  void getCcSaoStatisticsEdgeNew(CodingStructure &cs, const CPelUnitBuf &orgYuv, const CPelUnitBuf &srcYuv,
                                 const CPelUnitBuf &dstYuv, CcSaoStatData *blkStats[MAX_CCSAO_SET_NUM],
                                 const CcSaoEncParam &ccSaoParam);
#endif

  void getCcSaoStatisticsEdge(CodingStructure &cs, const ComponentID compID
                            , const CPelUnitBuf &orgYuv, const CPelUnitBuf &srcYuv, const CPelUnitBuf &dstYuv
                            , CcSaoStatData *blkStats[MAX_CCSAO_SET_NUM]
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
                            , CcSaoStatData *blkStatsEdgePre
#else
                            , CcSaoStatData *blkStatsEdgePre[MAX_CCSAO_SET_NUM - 1]
#endif
                            , const CcSaoEncParam &ccSaoParam);

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  void getCcSaoBlkStatsEdgePre(CodingStructure &cs, const ComponentID compID, const ChromaFormat chromaFormat, const int bitDepth
                             , CcSaoStatData *blkStats, const int ctuRsAddr
#else
  void getCcSaoBlkStatsEdgeNew(const ComponentID compID, const ChromaFormat chromaFormat, const int bitDepth
                             , const int setIdx, CcSaoStatData *blkStats[N_C - 1], const int ctuRsAddr
                             , const uint16_t candPosY
                             , const uint16_t bandNumY, const uint16_t bandNumU, const uint16_t bandNumV
#endif
                             , const Pel *srcY, const Pel *srcU, const Pel *srcV
                             , const Pel *org, const Pel *dst
                             , const int srcStrideY, const int srcStrideU, const int srcStrideV, const int orgStride, const int dstStride, const int width, const int height
                             , bool isLeftAvail, bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail, bool isAboveRightAvail
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                             , bool isCtuCrossedByVirtualBoundaries, int horVirBndryPos[], int verVirBndryPos[], int numHorVirBndry, int numVerVirBndry
#endif
                              );
#endif
  void getCcSaoBlkStats(const ComponentID compID, const ChromaFormat chromaFormat, const int bitDepth
                      , const int setIdx, CcSaoStatData* blkStats[MAX_CCSAO_SET_NUM], const int ctuRsAddr
                      , const uint16_t candPosY
                      , const uint16_t bandNumY, const uint16_t bandNumU, const uint16_t bandNumV
                      , const Pel* srcY, const Pel* srcU, const Pel* srcV
                      , const Pel* org, const Pel* dst
                      , const int srcStrideY, const int srcStrideU, const int srcStrideV, const int orgStride, const int dstStride, const int width, const int height
                      , bool isLeftAvail, bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail, bool isAboveRightAvail
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                      , bool isCtuCrossedByVirtualBoundaries, int horVirBndryPos[], int verVirBndryPos[], int numHorVirBndry, int numVerVirBndry
#endif
                       );
  void getCcSaoFrameStats(const ComponentID compID, const int setIdx, const uint8_t* ccSaoControl
                        , CcSaoStatData* blkStats    [MAX_CCSAO_SET_NUM], CcSaoStatData frameStats    [MAX_CCSAO_SET_NUM]
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
                        , CcSaoStatData* blkStatsEdge[MAX_CCSAO_SET_NUM], CcSaoStatData frameStatsEdge[MAX_CCSAO_SET_NUM], const uint8_t setType
#endif
                         );
  void deriveCcSaoOffsets(const ComponentID compID, const int bitDepth, const int setIdx
                        , CcSaoStatData frameStats[MAX_CCSAO_SET_NUM]
                        , short offset[MAX_CCSAO_SET_NUM][MAX_CCSAO_CLASS_NUM]);
  inline int estCcSaoIterOffset(const double lambda, const int offsetInput, const int64_t count, const int64_t diffSum, const int shift, const int bitIncrease, int64_t& bestDist, double& bestCost, const int offsetTh);
#if JVET_AJ0237_INTERNAL_12BIT
  void getCcSaoDistortion(const ComponentID compID, const int setIdx, CcSaoStatData* blkStats[MAX_CCSAO_SET_NUM]
                        , short offset[MAX_CCSAO_SET_NUM][MAX_CCSAO_CLASS_NUM], int64_t* trainingDistortion[MAX_CCSAO_SET_NUM], const int shift);
#else
  void getCcSaoDistortion(const ComponentID compID, const int setIdx, CcSaoStatData* blkStats[MAX_CCSAO_SET_NUM]
                        , short offset[MAX_CCSAO_SET_NUM][MAX_CCSAO_CLASS_NUM], int64_t* trainingDistortion[MAX_CCSAO_SET_NUM]);
#endif
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER && !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  void getCcSaoDistortionEdge(const ComponentID compID, const int setIdx,
                              CcSaoStatData *blkStatsEdge[MAX_CCSAO_SET_NUM],
                              short          offset[MAX_CCSAO_SET_NUM][MAX_CCSAO_CLASS_NUM],
                              int64_t *      trainingDistortion[MAX_CCSAO_SET_NUM]);
#endif
  void deriveCcSaoRDO(CodingStructure& cs, const ComponentID compID, int64_t* trainingDistortion[MAX_CCSAO_SET_NUM]
                    , CcSaoStatData* blkStats    [MAX_CCSAO_SET_NUM], CcSaoStatData frameStats    [MAX_CCSAO_SET_NUM]
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
                    , CcSaoStatData *blkStatsEdge[MAX_CCSAO_SET_NUM], CcSaoStatData frameStatsEdge[MAX_CCSAO_SET_NUM]
#endif
                    , CcSaoEncParam& bestCcSaoParam, CcSaoEncParam& tempCcSaoParam
                    , uint8_t* bestCcSaoControl, uint8_t* tempCcSaoControl
                    , double& bestCost, double& tempCost);
  void determineCcSaoControlIdc(CodingStructure& cs, const ComponentID compID
                              , const int ctuWidthC, const int ctuHeightC, const int picWidthC, const int picHeightC
                              , CcSaoEncParam& ccSaoParam, uint8_t* ccSaoControl, int64_t* trainingDistorsion[MAX_CCSAO_SET_NUM]
                              , int64_t& curTotalDist, double& curTotalRate);
  int getCcSaoParamRate(const ComponentID compID, const CcSaoEncParam& ccSaoParam);
  int lengthUvlc(int uiCode);
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  int getCcSaoClassNumEnc(const int setIdx, const CcSaoEncParam& ccSaoParam);  // for CcSaoEncParam
  void setupCcSaoPrv(CodingStructure& cs);
#endif
#endif
private: //members
  //for RDO
  CABACWriter*           m_CABACEstimator;
  CtxCache*              m_ctxCache;
  double                 m_lambda[MAX_NUM_COMPONENT];

  //statistics
  std::vector<SAOStatData**>         m_statData; //[ctu][comp][classes]
  std::vector<SAOStatData**>         m_preDBFstatData;
  double                 m_saoDisabledRate[MAX_NUM_COMPONENT][MAX_TLAYER];
  int                    m_skipLinesR[MAX_NUM_COMPONENT][NUM_SAO_NEW_TYPES];
  int                    m_skipLinesB[MAX_NUM_COMPONENT][NUM_SAO_NEW_TYPES];

#if JVET_W0066_CCSAO
  bool                   m_createdEnc = false;
  int                    m_intraPeriod;
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  bool                   m_extChroma = false;
#endif
  CcSaoStatData*         m_ccSaoStatData     [MAX_CCSAO_SET_NUM];
  CcSaoStatData          m_ccSaoStatFrame    [MAX_CCSAO_SET_NUM];
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
  CcSaoStatData*         m_ccSaoStatDataEdge [MAX_CCSAO_SET_NUM];
  CcSaoStatData          m_ccSaoStatFrameEdge[MAX_CCSAO_SET_NUM];
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  CcSaoStatData*         m_ccSaoStatDataEdgePre;
#else
  CcSaoStatData*         m_ccSaoStatDataEdgeNew[N_C];
#endif
#endif
  CcSaoEncParam          m_bestCcSaoParam;
  CcSaoEncParam          m_tempCcSaoParam;
  CcSaoEncParam          m_initCcSaoParam;
  uint8_t*               m_bestCcSaoControl;
  uint8_t*               m_tempCcSaoControl;
  uint8_t*               m_initCcSaoControl;
  int64_t*               m_trainingDistortion[MAX_CCSAO_SET_NUM];
#endif
};


//! \}

#endif

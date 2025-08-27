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

/** \file     DecCu.h
    \brief    CU decoder class (header)
*/

#ifndef __DECCU__
#define __DECCU__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "CABACReader.h"

#include "CommonLib/TrQuant.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Reshape.h"
//! \ingroup DecoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// CU decoder class
class DecCu
{
public:
  DecCu();
  virtual ~DecCu();

  /// initialize access channels
  void  init              ( TrQuant* pcTrQuant, IntraPrediction* pcIntra, InterPrediction* pcInter );

  /// destroy internal buffers
  void  decompressCtu     ( CodingStructure& cs, const UnitArea& ctuArea );
  Reshape*          m_pcReshape;
  Reshape* getReshape     () { return m_pcReshape; }
  void initDecCuReshaper  ( Reshape* pcReshape, ChromaFormat chromaFormatIDC) ;
  void destoryDecCuReshaprBuf();

  /// reconstruct Ctu information
protected:
  void xIntraRecQT        ( CodingUnit&      cu, const ChannelType chType );
  void xIntraRecACTQT(CodingUnit&      cu);

  void xReconInter        ( CodingUnit&      cu );
  void xDecodeInterTexture( CodingUnit&      cu );
  void xReconIntraQT      ( CodingUnit&      cu );
#if !REMOVE_PCM
  void xFillPCMBuffer     ( CodingUnit&      cu );
#endif
  void xIntraRecBlk       ( TransformUnit&   tu, const ComponentID compID );
  void xIntraRecACTBlk(TransformUnit&   tu);
  void xDecodeInterTU     ( TransformUnit&   tu, const ComponentID compID );

  void xDeriveCUMV        ( CodingUnit&      cu );
  void xReconPLT          ( CodingUnit&      cu,       ComponentID compBegin, uint32_t numComp );
  PelStorage        *m_tmpStorageLCU;
private:
  TrQuant*          m_pcTrQuant;
  IntraPrediction*  m_pcIntraPred;
  InterPrediction*  m_pcInterPred;

  PelStorage        m_ciipBuffer;

#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
#if JVET_AG0098_AMVP_WITH_SBTMVP
  MotionInfo        m_subPuMiBuf[SUB_BUFFER_SIZE][(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
#else
  MotionInfo        m_subPuMiBuf[SUB_TMVP_NUM][(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
#endif
#else
  MotionInfo        m_subPuMiBuf[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
#endif

#if JVET_AL0160_SBSMVP
  MotionInfo        m_subSpatialPuMiBuf[NUM_SUB_SMVP][(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
#endif

#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  bool applyBDMVR4BM[BM_MRG_MAX_NUM_INIT_CANDS];
#endif
#if MULTI_PASS_DMVR
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC || JVET_AE0046_BI_GPM
  Mv                m_mvBufBDMVR[MRG_MAX_NUM_CANDS << 1][MAX_NUM_SUBCU_DMVR];
#else
  Mv                m_mvBufBDMVR[2][MAX_NUM_SUBCU_DMVR];
#endif
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
  MvField           m_mvFieldAmListDec[MAX_NUM_AMVP_CANDS_MAX_REF << 1];
#if JVET_AD0213_LIC_IMP
  bool              m_licAmListDec[MAX_NUM_AMVP_CANDS_MAX_REF << 1];
#endif
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  Mv                m_mvBufDecAffineBDOF[BDOF_SUBPU_MAX_NUM];
#endif

  MergeCtx          m_geoMrgCtx;
#if JVET_AG0164_AFFINE_GPM
  AffineMergeCtx    m_geoAffMrgCtx;
#if JVET_AJ0274_GPM_AFFINE_TM
  AffineMergeCtx    m_geoAffTmMrgCtx;
#endif
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
  Mv                m_geoBvList[GEO_MAX_NUM_IBC_CANDS];
#endif
#if JVET_W0097_GPM_MMVD_TM && TM_MRG
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  MergeCtx          m_geoTmMrgCtx[GEO_NUM_TM_MV_CAND];
#else
  MergeCtx          m_geoTmMrgCtx0, m_geoTmMrgCtx1;
#endif
#endif
#if JVET_AC0112_IBC_GPM
  MergeCtx          m_ibcMrgCtx;
#endif
#if JVET_AE0046_BI_GPM
protected:
  Mv                m_mvBufBDOF4GPM[MRG_MAX_NUM_CANDS][BDOF_SUBPU_MAX_NUM];
#endif
};

//! \}

#endif


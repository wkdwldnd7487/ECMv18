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

/** \file     TrQuant.h
    \brief    transform and quantization class (header)
*/

#ifndef __TRQUANT__
#define __TRQUANT__

#include <array>
#include "CommonDef.h"
#include "Unit.h"
#include "ChromaFormat.h"
#include "Contexts.h"
#include "ContextModelling.h"

#include "UnitPartitioner.h"
#include "Quant.h"

#include "DepQuant.h"
//! \ingroup CommonLib
//! \{

typedef void FwdTrans(const TCoeff*, TCoeff*, int, int, int, int);
typedef void InvTrans(const TCoeff*, TCoeff*, int, int, int, int, const TCoeff, const TCoeff);


// ====================================================================================================================
// Class definition
// ====================================================================================================================


/// transform and quantization class
class TrQuant
{
public:
  TrQuant();
  ~TrQuant();

  // initialize class
  void init      (
                    const Quant* otherQuant,
                    const uint32_t uiMaxTrSize,
                    const bool bUseRDOQ,
                    const bool bUseRDOQTS,
#if T0196_SELECTIVE_RDOQ
                    const bool useSelectiveRDOQ,
#endif
                    const bool bEnc
  );
  void getTrTypes(const TransformUnit tu, const ComponentID compID, int &trTypeHor, int &trTypeVer);

#if AHG7_LN_TOOLOFF_CFG
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  void fwdLfnstNxN( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize, bool lfnstExtFlag = true );
  void invLfnstNxN( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize, const int maxLog2TrDynamicRange, bool lfnstExtFlag = true );
#else
  void fwdLfnstNxN( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize, bool lfnstExtFlag = true );
  void invLfnstNxN( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize, bool lfnstExtFlag = true );
#endif
#else
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  void fwdLfnstNxN( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize );
  void invLfnstNxN( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize, const int maxLog2TrDynamicRange );
#else
  void fwdLfnstNxN( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize );
  void invLfnstNxN( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize );
#endif
#endif

#if JVET_AC0130_NSPT
#if INTRA_TRANS_ENC_OPT
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
  void( *m_computeFwdNspt ) ( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx, int bktIdx );
  void( *m_computeInvNspt ) ( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, const int maxLog2TrDynamicRange, int zeroOutSize, int nsptIdx, int bktIdx );
  static void computeFwdNspt( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx, int bktIdx );
  static void computeInvNspt( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, const int maxLog2TrDynamicRange, int zeroOutSize, int nsptIdx, int bktIdx );
#else
  void( *m_computeFwdNspt ) ( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx );
  void( *m_computeInvNspt ) ( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, const int maxLog2TrDynamicRange, int zeroOutSize, int nsptIdx );
  static void computeFwdNspt( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx );
  static void computeInvNspt( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, const int maxLog2TrDynamicRange, int zeroOutSize, int nsptIdx );
#endif
#else
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
  void( *m_computeFwdNspt ) ( int* src, int* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx = 0, int bktIdx = 0 );
  void( *m_computeInvNspt ) ( int* src, int* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx = 0, int bktIdx = 0 );
  static void computeFwdNspt( int* src, int* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx = 0, int bktIdx = 0 );
  static void computeInvNspt( int* src, int* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx = 0, int bktIdx = 0 );
#else
  void( *m_computeFwdNspt ) ( int* src, int* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx = 0 );
  void( *m_computeInvNspt ) ( int* src, int* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx = 0 );
  static void computeFwdNspt( int* src, int* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx = 0 );
  static void computeInvNspt( int* src, int* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx = 0 );
#endif
#endif
#else
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
  void computeFwdNspt( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx, int bktIdx );
  void computeInvNspt( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, const int maxLog2TrDynamicRange, int zeroOutSize, int nsptIdx, int bktIdx );
#else
  void computeFwdNspt( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx );
  void computeInvNspt( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, const int maxLog2TrDynamicRange, int zeroOutSize, int nsptIdx );
#endif
#else
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
  void computeFwdNspt( int* src, int* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx = 0, int bktIdx = 0 );
  void computeInvNspt( int* src, int* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx = 0, int bktIdx = 0 );
#else
  void computeFwdNspt( int* src, int* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx = 0 );
  void computeInvNspt( int* src, int* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx = 0 );
#endif
#endif
#endif
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
  static uint8_t getNsptKernelCluster( const uint32_t mode, const uint32_t width, const uint32_t height, int nsptIdx, int bktIdx );
  static const int8_t* getNsptMatrix( const uint32_t mode, const uint32_t width, const uint32_t height, int nsptIdx, int bktIdx );
#endif
#endif
  
  bool     getTransposeFlag ( uint32_t intraMode  );
#if JVET_Y0141_SIGN_PRED_IMPROVE
  int      getLfnstIdx(const TransformUnit &tu, ComponentID compID);
#endif
#if JVET_AK0217_INTRA_MTSS
  void   resetLfnstIntraModeIdx(const int lfnstIdx) { m_lfnstIntraModeIdx[lfnstIdx] = 0; }
  void   setLfnstIntraModeIdx(const int lfnstIdx, int modeIdx) { m_lfnstIntraModeIdx[lfnstIdx] = modeIdx; }
#endif
protected:

  void xFwdLfnst( const TransformUnit &tu, const ComponentID compID, const bool loadTr = false );
  void xInvLfnst( const TransformUnit &tu, const ComponentID compID );
#if JVET_AC0130_NSPT
  void xFwdNspt( const TransformUnit &tu, TCoeff* src, TCoeff* dst, const ComponentID compID, const int shift_1st, const int shift_2nd, int lfnstIdx );
  void xInvNspt( const TransformUnit &tu, const TCoeff* src, TCoeff* dst, const ComponentID compID, const int shift_1st, const int shift_2nd, int lfnstIdx );
#endif

public:

  void invTransformNxN  (TransformUnit &tu, const ComponentID &compID, PelBuf &pResi, const QpParam &cQPs);
  void transformNxN     ( TransformUnit& tu, const ComponentID& compID, const QpParam& cQP, std::vector<TrMode>* trModes, const int maxCand );
  void transformNxN     ( TransformUnit& tu, const ComponentID& compID, const QpParam& cQP, TCoeff& uiAbsSum, const Ctx& ctx, const bool loadTr = false );
#if JVET_W0103_INTRA_MTS
  uint64_t transformNxN(TransformUnit& tu);
#endif

  void transformSkipQuantOneSample(TransformUnit &tu, const ComponentID &compID, const TCoeff &resiDiff, TCoeff &coeff,    const uint32_t &uiPos, const QpParam &cQP, const bool bUseHalfRoundingPoint);
  void invTrSkipDeQuantOneSample  (TransformUnit &tu, const ComponentID &compID, const TCoeff &pcCoeff,  Pel &reconSample, const uint32_t &uiPos, const QpParam &cQP);

  void                        invTransformICT     ( const TransformUnit &tu, PelBuf &resCb, PelBuf &resCr );
  std::pair<int64_t,int64_t>  fwdTransformICT     ( const TransformUnit &tu, const PelBuf &resCb, const PelBuf &resCr, PelBuf& resC1, PelBuf& resC2, int jointCbCr = -1 );
  std::vector<int>            selectICTCandidates ( const TransformUnit &tu, CompStorage* resCb, CompStorage* resCr );

#if RDOQ_CHROMA_LAMBDA
  void   setLambdas  ( const double lambdas[MAX_NUM_COMPONENT] )   { m_quant->setLambdas( lambdas ); }
  void   selectLambda( const ComponentID compIdx )                 { m_quant->selectLambda( compIdx ); }
  void   getLambdas  ( double (&lambdas)[MAX_NUM_COMPONENT]) const { m_quant->getLambdas( lambdas ); }
#endif
  void   setLambda   ( const double dLambda )                      { m_quant->setLambda( dLambda ); }
  double getLambda   () const                                      { return m_quant->getLambda(); }

  DepQuant* getQuant() { return m_quant; }
  void   lambdaAdjustColorTrans(bool forward) { m_quant->lambdaAdjustColorTrans(forward); }
  void   resetStore() { m_quant->resetStore(); }
#if SIGN_PREDICTION
#if JVET_AL0181_ASBT
  void predCoeffSigns( TransformUnit &tu, const ComponentID compID, const bool reshapeChroma,const Pel * tmpPredResiBorder = NULL );
#else
  void predCoeffSigns( TransformUnit &tu, const ComponentID compID, const bool reshapeChroma );
#endif
#endif

#if ENABLE_SPLIT_PARALLELISM
  void    copyState( const TrQuant& other );
#endif

protected:
#if JVET_AK0217_INTRA_MTSS
  bool    secondBucket;
#endif
  TCoeff   m_tempCoeff[MAX_TB_SIZEY * MAX_TB_SIZEY];
#if SIGN_PREDICTION
  Pel m_tempSignPredResid[SIGN_PRED_MAX_BS * SIGN_PRED_MAX_BS * 2]{ 0 };
#if JVET_Y0141_SIGN_PRED_IMPROVE
  uint8_t  m_signsBuf[SIGN_PRED_FREQ_RANGE*SIGN_PRED_FREQ_RANGE];
#endif
#endif
#if JVET_AK0217_INTRA_MTSS
  int      m_lfnstIntraModeIdx[4];
#endif
private:
  DepQuant *m_quant;          //!< Quantizer
#if JVET_AG0061_INTER_LFNST_NSPT
#if JVET_AI0050_INTER_MTSS
  TCoeff   m_mtsCoeffs[NUM_TRAFO_MODES_MTS + 6][MAX_TB_SIZEY * MAX_TB_SIZEY];
#else
  TCoeff   m_mtsCoeffs[NUM_TRAFO_MODES_MTS + 3][MAX_TB_SIZEY * MAX_TB_SIZEY];
#endif
#else
  TCoeff   m_mtsCoeffs[NUM_TRAFO_MODES_MTS][MAX_TB_SIZEY * MAX_TB_SIZEY];
#endif
#if JVET_W0119_LFNST_EXTENSION
  TCoeff   m_tempInMatrix [ L16W_ZO ];
  TCoeff   m_tempOutMatrix[ L16W_ZO ];
#else
#if EXTENDED_LFNST
  TCoeff   m_tempInMatrix [ 64 ];
  TCoeff   m_tempOutMatrix[ 64 ];
#else
  TCoeff   m_tempInMatrix [ 48 ];
  TCoeff   m_tempOutMatrix[ 48 ];
#endif
#endif
#if JVET_AC0130_NSPT
  TCoeff   m_tempNsptCoeff[ MAX_TB_SIZEY * MAX_TB_SIZEY ];
  TCoeff   m_nsptTempInMatrix[ 256 ];
  TCoeff   m_nsptTempOutMatrix[ 256 ];
#endif
  static const int maxAbsIctMode = 3;
  void                      (*m_invICTMem[1+2*maxAbsIctMode])(PelBuf&,PelBuf&);
  std::pair<int64_t,int64_t>(*m_fwdICTMem[1+2*maxAbsIctMode])(const PelBuf&,const PelBuf&,PelBuf&,PelBuf&);
  void                      (**m_invICT)(PelBuf&,PelBuf&);
  std::pair<int64_t,int64_t>(**m_fwdICT)(const PelBuf&,const PelBuf&,PelBuf&,PelBuf&);

  std::array<std::array<FwdTrans*, g_numTransformMatrixSizes>, NUM_TRANS_TYPE> fastFwdTrans;
  std::array<std::array<InvTrans*, g_numTransformMatrixSizes>, NUM_TRANS_TYPE> fastInvTrans;

#if TRANSFORM_SIMD_OPT
  static std::array<std::array<const TMatrixCoeff*, g_numTransformMatrixSizes>, NUM_TRANS_TYPE> m_forwardTransformKernels;
  static std::array<std::array<const TMatrixCoeff*, g_numTransformMatrixSizes>, NUM_TRANS_TYPE> m_inverseTransformKernels;
#endif

  // forward Transform
  void xT               (const TransformUnit &tu, const ComponentID &compID, const CPelBuf &resi, CoeffBuf &dstCoeff, const int width, const int height);

  // skipping Transform
  void xTransformSkip   (const TransformUnit &tu, const ComponentID &compID, const CPelBuf &resi, TCoeff* psCoeff);

  // quantization
  void xQuant           (TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx);

  // dequantization
  void xDeQuant( const TransformUnit &tu,
                       CoeffBuf      &dstCoeff,
                 const ComponentID   &compID,
                 const QpParam       &cQP      );

  // inverse transform
  void xIT     ( const TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pCoeff, PelBuf &pResidual );

  // inverse skipping transform
  void xITransformSkip(
                 const CCoeffBuf     &plCoef,
                       PelBuf        &pResidual,
                 const TransformUnit &tu,
                 const ComponentID   &component);

  void xGetCoeffEnergy(
                       TransformUnit  &tu,
                 const ComponentID    &compID,
                 const CoeffBuf       &coeffs,
                       double*        diagRatio,
                       double*        horVerRatio );

#if INTRA_TRANS_ENC_OPT 
  void(*m_fwdLfnst)  (TCoeff* src, TCoeff*& dst, const int8_t*& trMat, const int trSize, const int zeroOutSize);
  static void forwardLfnst(TCoeff* src, TCoeff*& dst, const int8_t*& trMat, const int trSize, const int zeroOutSize);

  void(*m_invLfnst)  (TCoeff* src, TCoeff*  dst, const int8_t*  trMat, const int trSize, const int zeroOutSize, const TCoeff outputMinimum, const TCoeff outputMaximum);
  static void inverseLfnst(TCoeff* src, TCoeff*  dst, const int8_t*  trMat, const int trSize, const int zeroOutSize, const TCoeff outputMinimum, const TCoeff outputMaximum);
#endif
#if TRANSFORM_SIMD_OPT
  template <TransType transType, int transSize>
  static void fastForwardTransform_SIMD( const TCoeff *block, TCoeff *coeff, int shift, int line, int iSkipLine, int iSkipLine2 );

  template <TransType transType, int transSize>
  static void fastInverseTransform_SIMD( const TCoeff *coeff, TCoeff *block, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum );
#endif

#if TRANSFORM_SIMD_OPT
#ifdef TARGET_SIMD_X86
  void    initTrQuantX86();
  template <X86_VEXT vext>
  void    _initTrQuantX86();
#endif
#endif

};// END CLASS DEFINITION TrQuant

//! \}

#endif // __TRQUANT__

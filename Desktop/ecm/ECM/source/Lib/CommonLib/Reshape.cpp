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

/** \file     Reshape.cpp
    \brief    common reshaper class
*/
#include "Reshape.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <UnitTools.h>
 //! \ingroup CommonLib
 //! \{

 // ====================================================================================================================
 // Constructor / destructor / create / destroy
 // ====================================================================================================================

Reshape::Reshape()
: m_CTUFlag (false)
, m_recReshaped (false)
, m_reshape (true)
, m_chromaScale (1 << CSCALE_FP_PREC)
#if !LMCS_CHROMA_CALC_CU
, m_vpduX (-1)
, m_vpduY (-1)
#endif
{
}

Reshape::~Reshape()
{
}

void  Reshape::createDec(int bitDepth)
{
  m_lumaBD = bitDepth;
  m_reshapeLUTSize = 1 << m_lumaBD;
  m_initCW = m_reshapeLUTSize / PIC_CODE_CW_BINS;
  if (m_fwdLUT.empty())
    m_fwdLUT.resize(m_reshapeLUTSize, 0);
  if (m_invLUT.empty())
    m_invLUT.resize(m_reshapeLUTSize, 0);
  if (m_binCW.empty())
    m_binCW.resize(PIC_CODE_CW_BINS, 0);
  if (m_inputPivot.empty())
    m_inputPivot.resize(PIC_CODE_CW_BINS + 1, 0);
  if (m_fwdScaleCoef.empty())
    m_fwdScaleCoef.resize(PIC_CODE_CW_BINS, 1 << FP_PREC);
  if (m_invScaleCoef.empty())
    m_invScaleCoef.resize(PIC_CODE_CW_BINS, 1 << FP_PREC);
  if (m_reshapePivot.empty())
    m_reshapePivot.resize(PIC_CODE_CW_BINS + 1, 0);
  if (m_chromaAdjHelpLUT.empty())
    m_chromaAdjHelpLUT.resize(PIC_CODE_CW_BINS, 1<<CSCALE_FP_PREC);
}

void  Reshape::destroy()
{
}


/** compute chroma residuce scale for TU
* \param average luma pred of TU
* \return chroma residue scale
*/
int  Reshape::calculateChromaAdj(Pel avgLuma)
{
  int iAdj = m_chromaAdjHelpLUT[getPWLIdxInv(avgLuma)];
  return(iAdj);
}

/** compute chroma residuce scale for TU
* \param average luma pred of TU
* \return chroma residue scale
*/
int  Reshape::calculateChromaAdjVpduNei(TransformUnit &tu, const CompArea &areaY)
{
  CodingStructure &cs = *tu.cs;
  int xPos = areaY.lumaPos().x;
  int yPos = areaY.lumaPos().y;
#if LMCS_CHROMA_CALC_CU
  int numNeighborAbove = areaY.width, numNeighborLeft = areaY.height;
#else
  int ctuSize = cs.sps->getCTUSize();
  int numNeighbor = std::min(64, ctuSize);
  int numNeighborLog = floorLog2(numNeighbor);
  if (ctuSize == 128)
  {
    xPos = xPos / 64 * 64;
    yPos = yPos / 64 * 64;
  }
  else
  {
    xPos = xPos / ctuSize * ctuSize;
    yPos = yPos / ctuSize * ctuSize;
  }

#if JVET_Z0118_GDR
  if (isVPDUprocessed(xPos, yPos, (PictureType) tu.cs->picture->getCleanDirty()) && !cs.pcv->isEncoder)
#else
  if (isVPDUprocessed(xPos, yPos) && !cs.pcv->isEncoder)
#endif
  {
    return getChromaScale();
  }
  else
#endif
  {
#if !LMCS_CHROMA_CALC_CU
#if JVET_Z0118_GDR
    setVPDULoc(xPos, yPos, (PictureType) tu.cs->picture->getCleanDirty());
#else
    setVPDULoc(xPos, yPos);
#endif
#endif
    Position topLeft(xPos, yPos);
    CodingUnit *topLeftLuma;
    const CodingUnit *cuAbove, *cuLeft;
#if LMCS_CHROMA_CALC_CU
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    if (tu.cu->separateTree && cs.slice->getSliceType() == I_SLICE)
#else
    if (CS::isDualITree(cs) && cs.slice->getSliceType() == I_SLICE)
#endif
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      topLeftLuma = tu.cs->getLumaCU(topLeft,CHANNEL_TYPE_LUMA);
      cuAbove = tu.cs->getLumaCU(topLeft.offset(0, -1),CHANNEL_TYPE_LUMA);
      cuLeft  = tu.cs->getLumaCU(topLeft.offset(-1, 0),CHANNEL_TYPE_LUMA);
#else
      topLeftLuma = tu.cs->picture->cs->getCU(topLeft, CHANNEL_TYPE_LUMA);
      cuAbove = cs.picture->cs->getCURestricted(topLeft.offset(0, -1), *topLeftLuma, CHANNEL_TYPE_LUMA);
      cuLeft  = cs.picture->cs->getCURestricted(topLeft.offset(-1, 0), *topLeftLuma, CHANNEL_TYPE_LUMA);
#endif
    }
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    else if(tu.cu->separateTree)
    {
      topLeftLuma = cs.getLumaCU(topLeft,CHANNEL_TYPE_LUMA);
      cuAbove = tu.cs->getLumaCU(topLeft.offset(0, -1),CHANNEL_TYPE_LUMA);
      cuLeft  = tu.cs->getLumaCU(topLeft.offset(-1, 0),CHANNEL_TYPE_LUMA);
    }
#endif
    else
    {
      topLeftLuma = cs.getCU(topLeft, CHANNEL_TYPE_LUMA);
      cuAbove = cs.getCURestricted(topLeft.offset(0, -1), *topLeftLuma, CHANNEL_TYPE_LUMA);
      cuLeft  = cs.getCURestricted(topLeft.offset(-1, 0), *topLeftLuma, CHANNEL_TYPE_LUMA);
    }

    CompArea lumaArea = CompArea(COMPONENT_Y, tu.chromaFormat, topLeft, areaY, true);
#else
    if (CS::isDualITree(cs) && cs.slice->getSliceType() == I_SLICE)
    {
      topLeftLuma = tu.cs->picture->cs->getCU(topLeft, CHANNEL_TYPE_LUMA);
      cuAbove = cs.picture->cs->getCURestricted(topLeftLuma->lumaPos().offset(0, -1), *topLeftLuma, CHANNEL_TYPE_LUMA);
      cuLeft  = cs.picture->cs->getCURestricted(topLeftLuma->lumaPos().offset(-1, 0), *topLeftLuma, CHANNEL_TYPE_LUMA);
    }
    else
    {
      topLeftLuma = cs.getCU(topLeft, CHANNEL_TYPE_LUMA);
      cuAbove = cs.getCURestricted(topLeftLuma->lumaPos().offset(0, -1), *topLeftLuma, CHANNEL_TYPE_LUMA);
      cuLeft  = cs.getCURestricted(topLeftLuma->lumaPos().offset(-1, 0), *topLeftLuma, CHANNEL_TYPE_LUMA);
    }

    xPos = topLeftLuma->lumaPos().x;
    yPos = topLeftLuma->lumaPos().y;

    CompArea lumaArea = CompArea(COMPONENT_Y, tu.chromaFormat, topLeftLuma->lumaPos(), topLeftLuma->lumaSize(), true);
#endif
    PelBuf piRecoY = cs.picture->getRecoBuf(lumaArea);
    int strideY = piRecoY.stride;
    int chromaScale = (1 << CSCALE_FP_PREC);
    int lumaValue = -1;

    Pel* recSrc0 = piRecoY.bufAt(0, 0);
    const uint32_t picH = tu.cs->picture->lheight();
    const uint32_t picW = tu.cs->picture->lwidth();
    const Pel   valueDC = 1 << (tu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA) - 1);
    int32_t recLuma = 0;
    int pelnum = 0;
    if (cuLeft != nullptr)
    {
#if LMCS_CHROMA_CALC_CU
      for (int i = 0; i < numNeighborLeft; i++)
#else
      for (int i = 0; i < numNeighbor; i++)
#endif
      {
        int k = (yPos + i) >= picH ? (picH - yPos - 1) : i;
        recLuma += recSrc0[-1 + k * strideY];
        pelnum++;
      }
    }
    if (cuAbove != nullptr)
    {
#if LMCS_CHROMA_CALC_CU
      for (int i = 0; i < numNeighborAbove; i++)
#else
      for (int i = 0; i < numNeighbor; i++)
#endif
      {
        int k = (xPos + i) >= picW ? (picW - xPos - 1) : i;
        recLuma += recSrc0[-strideY + k];
        pelnum++;
      }
    }
#if LMCS_CHROMA_CALC_CU
    if (pelnum)
    {
      lumaValue = ClipPel((recLuma + (pelnum>>1)) / pelnum, tu.cs->slice->clpRng(COMPONENT_Y));
    }
#else
    if (pelnum == numNeighbor)
    {
      lumaValue = (recLuma + (1 << (numNeighborLog - 1))) >> numNeighborLog;
    }
    else if (pelnum == (numNeighbor << 1))
    {
      lumaValue = (recLuma + (1 << numNeighborLog)) >> (numNeighborLog + 1);
    }
#endif
    else
    {
      CHECK(pelnum != 0, "");
      lumaValue = valueDC;
    }
    chromaScale = calculateChromaAdj(lumaValue);
#if !LMCS_CHROMA_CALC_CU
    setChromaScale(chromaScale);
#endif
    return(chromaScale);
  }
}
/** find inx of PWL for inverse mapping
* \param average luma pred of TU
* \return idx of PWL for inverse mapping
*/
int Reshape::getPWLIdxInv(int lumaVal)
{
  int idxS = 0;
  for (idxS = m_sliceReshapeInfo.reshaperModelMinBinIdx; (idxS <= m_sliceReshapeInfo.reshaperModelMaxBinIdx); idxS++)
  {
    if (lumaVal < m_reshapePivot[idxS + 1])     break;
  }
  return std::min(idxS, PIC_CODE_CW_BINS-1);
}

/**
-copy Slice reshaper info structure
\param   tInfo describing the target Slice reshaper info structure
\param   sInfo describing the source Slice reshaper info structure
*/
void Reshape::copySliceReshaperInfo(SliceReshapeInfo& tInfo, SliceReshapeInfo& sInfo)
{
  tInfo.sliceReshaperModelPresentFlag = sInfo.sliceReshaperModelPresentFlag;
  if (sInfo.sliceReshaperModelPresentFlag)
  {
    tInfo.reshaperModelMaxBinIdx = sInfo.reshaperModelMaxBinIdx;
    tInfo.reshaperModelMinBinIdx = sInfo.reshaperModelMinBinIdx;
    memcpy(tInfo.reshaperModelBinCWDelta, sInfo.reshaperModelBinCWDelta, sizeof(int)*(PIC_CODE_CW_BINS));
    tInfo.maxNbitsNeededDeltaCW = sInfo.maxNbitsNeededDeltaCW;
    tInfo.chrResScalingOffset = sInfo.chrResScalingOffset;
  }
  tInfo.sliceReshaperEnableFlag = sInfo.sliceReshaperEnableFlag;
  if (sInfo.sliceReshaperEnableFlag)
    tInfo.enableChromaAdj = sInfo.enableChromaAdj;
  else
    tInfo.enableChromaAdj = 0;
}

/** Construct reshaper from syntax
* \param void
* \return void
*/
void Reshape::constructReshaper()
{
  int pwlFwdLUTsize = PIC_CODE_CW_BINS;
  int pwlFwdBinLen = m_reshapeLUTSize / PIC_CODE_CW_BINS;

  for (int i = 0; i < m_sliceReshapeInfo.reshaperModelMinBinIdx; i++)
    m_binCW[i] = 0;
  for (int i = m_sliceReshapeInfo.reshaperModelMaxBinIdx + 1; i < PIC_CODE_CW_BINS; i++)
    m_binCW[i] = 0;
  for (int i = m_sliceReshapeInfo.reshaperModelMinBinIdx; i <= m_sliceReshapeInfo.reshaperModelMaxBinIdx; i++)
    m_binCW[i] = (uint16_t)(m_sliceReshapeInfo.reshaperModelBinCWDelta[i] + (int)m_initCW);

  for (int i = 0; i < pwlFwdLUTsize; i++)
  {
    m_reshapePivot[i + 1] = m_reshapePivot[i] + m_binCW[i];
    m_inputPivot[i + 1] = m_inputPivot[i] + m_initCW;
    m_fwdScaleCoef[i] = ((int32_t)m_binCW[i] * (1 << FP_PREC) + (1 << (floorLog2(pwlFwdBinLen) - 1))) >> floorLog2(pwlFwdBinLen);
    if (m_binCW[i] == 0)
    {
      m_invScaleCoef[i] = 0;
      m_chromaAdjHelpLUT[i] = 1 << CSCALE_FP_PREC;
    }
    else
    {
      m_invScaleCoef[i] = (int32_t)(m_initCW * (1 << FP_PREC) / m_binCW[i]);
      m_chromaAdjHelpLUT[i] = (int32_t)(m_initCW * (1 << FP_PREC) / ( m_binCW[i] + m_sliceReshapeInfo.chrResScalingOffset ) );
    }
  }
  for (int lumaSample = 0; lumaSample < m_reshapeLUTSize; lumaSample++)
  {
    int idxY = lumaSample / m_initCW;
    int tempVal = m_reshapePivot[idxY] + ((m_fwdScaleCoef[idxY] * (lumaSample - m_inputPivot[idxY]) + (1 << (FP_PREC - 1))) >> FP_PREC);
    m_fwdLUT[lumaSample] = Clip3((Pel)0, (Pel)((1 << m_lumaBD) - 1), (Pel)(tempVal));

    int idxYInv = getPWLIdxInv(lumaSample);
    int invSample = m_inputPivot[idxYInv] + ((m_invScaleCoef[idxYInv] * (lumaSample - m_reshapePivot[idxYInv]) + (1 << (FP_PREC - 1))) >> FP_PREC);
    m_invLUT[lumaSample] = Clip3((Pel)0, (Pel)((1 << m_lumaBD) - 1), (Pel)(invSample));
  }
}



//
//! \}

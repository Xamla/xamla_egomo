extern "C" {
#include <TH/TH.h>

void reorderBits(THByteTensor *matrixInData, THByteTensor *matrixOutData)
{
  THByteTensor *matrixInData_ = THByteTensor_newContiguous(matrixInData);
  unsigned char *inData = THByteTensor_data(matrixInData_);

  int lengthByte = THByteTensor_size(matrixInData_, 0);
  int outLenghtBytes = 0;
  if(lengthByte%16 == 0) 
    outLenghtBytes=lengthByte;
  else
    outLenghtBytes = ((lengthByte/16)+1)*16;
  
  THByteTensor_resize1d(matrixOutData, outLenghtBytes);
  unsigned char *out = THByteTensor_data(matrixOutData);
  
  int nPixel=lengthByte/2;
  int stride=lengthByte/16;
  for(int i=0; i<lengthByte; i++)
    out[i]=0;

  int buffer[16];
  for(int i=0; i<16; i++)
    buffer[i]=0;
  long outPos=0;
  int bitpos=0;

  const unsigned short *in=reinterpret_cast<const unsigned short*>(inData);

  for(int i=0; i<nPixel; i++) {
    if(bitpos>=8) {
      for(int j=0; j<16; j++) {
        //std::cout << "loop " << outPos << ", j=" << j << ": " << std::bitset<8>(buffer[j]) << std::endl;
        out[j*stride + outPos] = buffer[j];
        buffer[j]=0;
      }
      outPos++;
      bitpos=0;
    }
    
    for(int j=0; j<16; j++) {
      buffer[15-j] |= ((in[i] >> j) & 1) << (7-bitpos);
    }
    
    bitpos++;
  }
  
  for(int j=0; j<16; j++)
     out[long(j*stride) + outPos] = buffer[j];

  THByteTensor_free(matrixInData_);
}


void restoreBitOrder(THByteTensor *matrixInData, THByteTensor *matrixOutData)
{
  THByteTensor *matrixInData_ = THByteTensor_newContiguous(matrixInData);
  unsigned char *inData = THByteTensor_data(matrixInData_);
  
  int lengthByte = THByteTensor_size(matrixInData_, 0);
  if(lengthByte%16 != 0)
    return;
  
  //int outLenghtBytes = THByteTensor_size(matrixInData_, 0);
  // TODO: if input size cannot be divided by 16, we have additional bytes. Remove these bytes (Problem: where to get the original size?)
  THByteTensor_resize1d(matrixOutData, lengthByte);
  unsigned char *out = THByteTensor_data(matrixOutData);
  
  int strideLengh=lengthByte/16;
  for(int i=0; i<lengthByte; i++)
    out[i]=0;
  
  for(int strideBlock = 0; strideBlock<8; strideBlock++) {
    for(int i=0; i<strideLengh; i++) {
      for(int j=0; j<8; j++) {
	out[i*16 + j*2 +1] |= ((inData[strideBlock*strideLengh + i] >> (7-j)) & 1) << (7-strideBlock);
      }
    }
  }
  for(int strideBlock = 8; strideBlock<16; strideBlock++) {
    for(int i=0; i<strideLengh; i++) {
         for(int j=0; j<8; j++) {
	   out[i*16 + j*2] |= ((inData[strideBlock*strideLengh + i] >> (7-j)) & 1) << (15-strideBlock);
         }
    }
   }
}
  
};

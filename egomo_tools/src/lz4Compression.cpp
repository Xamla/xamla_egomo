#include "lz4.h"
#include <iostream>
#include <cstring> // for std::memcpy

extern "C" {
#include <TH/TH.h>

int compressLZ4(THByteTensor *matrixInData, THByteTensor *matrixOutData)
{
     static const int BLOCK_BYTES = 1024 * 8;

  THByteTensor *matrixInData_ = THByteTensor_newContiguous(matrixInData);
  unsigned char *inputBuffer = THByteTensor_data(matrixInData_);
  
  int inputSize = THByteTensor_size(matrixInData_, 0);
  int outLenghtBytes = inputSize + inputSize*0.1;  // add some reserve in case of incompressible data
  
  THByteTensor_resize1d(matrixOutData, outLenghtBytes);
  unsigned char *outputBuffer = THByteTensor_data(matrixOutData);

     LZ4_stream_t lz4Stream_body;
     LZ4_stream_t* lz4Stream = &lz4Stream_body;

     int inpos=0;
     int outpos=0;
     LZ4_resetStream(lz4Stream);

     while(inputSize>0) {
     	int inpBytes;
     	if(inputSize>=BLOCK_BYTES)
     	  inpBytes=BLOCK_BYTES;
     	else
     	  inpBytes=inputSize;

     	char cmpBuf[LZ4_COMPRESSBOUND(BLOCK_BYTES)];
     	int cmpBytes = LZ4_compress_fast_continue(lz4Stream, (char*)&inputBuffer[inpos], cmpBuf, inpBytes, sizeof(cmpBuf), 1);
     	if(cmpBytes <= 0) {
     	  std::cout << "Error in compression function" << std::endl;
     	  break;
     	}

     	std::memcpy(&outputBuffer[outpos], reinterpret_cast<char *>(&cmpBytes), sizeof(cmpBytes));
     	outpos+=sizeof(cmpBytes);
     	std::memcpy(&outputBuffer[outpos], cmpBuf, (size_t) cmpBytes);
     	outpos+=cmpBytes;
     	inpos += inpBytes;
     	inputSize -= inpBytes;
     }

     outputBuffer[outpos] = 0;
     return outpos;
}


int decompressLZ4(THByteTensor *matrixInData, THByteTensor *matrixOutData, int outputSize)
{
  static const int BLOCK_BYTES = 1024 * 8;
  //std::cout << "Start decompression, expected output size: " << outputSize << std::endl;
  
  THByteTensor *matrixInData_ = THByteTensor_newContiguous(matrixInData);
  unsigned char *inputBuffer = THByteTensor_data(matrixInData_);
  
  THByteTensor_resize1d(matrixOutData, outputSize);
  unsigned char *outputBuffer = THByteTensor_data(matrixOutData);
  
  LZ4_streamDecode_t lz4StreamDecode_body;
  LZ4_streamDecode_t* lz4StreamDecode = &lz4StreamDecode_body;
  
  LZ4_setStreamDecode(lz4StreamDecode, NULL, 0);
  
  int inpos=0;
  int outpos=0;
  
  int decBytes = 0;
  while(1) {
    //std::cout << "inpos=" << inpos << ", outpos=" << outpos << std::endl;
    int  cmpBytes = 0;
    std::memcpy(reinterpret_cast<char *>(&cmpBytes), (&inputBuffer[inpos]), sizeof(cmpBytes));
    //std::cout << "cmpBytes=" << cmpBytes << std::endl;
    if(cmpBytes<=0)
      break;
    inpos+=sizeof(cmpBytes);
    
    decBytes = LZ4_decompress_safe_continue(lz4StreamDecode, (char*)&inputBuffer[inpos], (char*)&outputBuffer[outpos], 
					    cmpBytes, BLOCK_BYTES);
    //std::cout << "decByptes=" << decBytes << std::endl;
    inpos+=cmpBytes;
    outpos+=decBytes;

    //std::cout << std::endl;
  }
  // resize the tensor to the actual final data size
  THByteTensor_resize1d(matrixOutData, outpos);
  return outpos;
}
};

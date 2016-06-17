local ffi = require 'ffi'

local egomoTools = {}

egomoTools.compressLZ4 = {}

local compressLZ4_cdef = [[
void reorderBits(THByteTensor *matrixInData, THByteTensor *matrixOutData);
void restoreBitOrder(THByteTensor *matrixInData, THByteTensor *matrixOutData);
int compressLZ4(THByteTensor *matrixInData, THByteTensor *matrixOutData);
int decompressLZ4(THByteTensor *matrixInData, THByteTensor *matrixOutData, int uncompressedSize);
]]
ffi.cdef(compressLZ4_cdef)
egomoTools.lib = ffi.load(package.searchpath('libegomo-tools', package.cpath))


return egomoTools
